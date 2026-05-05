#include "cmd_cam.h"
#include "command.h"
#include "cmd_lcd.h"
//
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//
#include "pico/stdlib.h"
#include "hardware/i2c.h"
//
#include "cam.h"
#include "ov5640.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "sd_card.h"
#include "DEV_Config.h"

// Global variable definitions
bool cam_snap_pending = false;                  // True while an async cam snap operation is in progress
absolute_time_t cam_snap_deadline;             // Absolute time after which the frame-wait phase times out
char cam_snap_path[256];                       // Destination file path for the current cam snap operation
FIL cam_snap_file;                             // FatFS file object for the output file during cam snap
bool cam_snap_file_open = false;               // True when cam_snap_file is open and must be closed on finish/abort
uint32_t cam_snap_write_offset = 0;            // Byte offset into cam_ptr already written to the file
UINT cam_snap_total_written = 0;               // Cumulative bytes successfully written across all chunks so far
absolute_time_t cam_snap_next_step_time;       // Earliest time the state machine should execute its next step
cam_snap_state_t cam_snap_state = CAM_SNAP_IDLE; // Current phase of the cam snap state machine
const UINT cam_snap_chunk_size = 512;          // Bytes written per background-task tick (matches one SD block for efficiency)
bool cam_mirror_enabled = false;               // Tracks the current OV5640 horizontal mirror state for display

/**
 * @brief Allocate the camera frame buffer if not already done.
 *
 * The buffer holds two frames (cam_ful_size * 2 bytes) for double-buffering.
 * Allocation is deferred so commands that don't use the camera don't consume the RAM.
 * @return true if the buffer is ready, false if malloc failed.
 */
bool ensure_cam_buffer_allocated() {
    if (cam_ptr) return true;

    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
    if (!cam_ptr) {
        printf("malloc failed for frame buffer (%lu bytes)\n",
               (unsigned long)(cam_ful_size * 2));
        return false;
    }
    memset(cam_ptr, 0, cam_ful_size * 2);
    return true;
}

/**
 * @brief Write the camera frame buffer to a file as raw RGB565 bytes.
 *
 * Creates or overwrites the file at path.  Writes exactly cam_ful_size * 2 bytes
 * (little-endian row-major RGB565).
 * @return true only if both f_write and f_close succeeded without error.
 */
bool save_cam_buffer_to_file(const char *path) {
    FIL fil;
    FRESULT fr = f_open(&fil, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    UINT bw = 0;
    fr = f_write(&fil, cam_ptr, cam_ful_size * 2, &bw);
    if (FR_OK != fr) {
        printf("f_write error: %s (%d)\n", FRESULT_str(fr), fr);
    } else {
        printf("Wrote %u bytes to %s\n", bw, path);
    }

    FRESULT close_fr = f_close(&fil);
    if (FR_OK != close_fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(close_fr), close_fr);
    }

    return FR_OK == fr && FR_OK == close_fr;
}

/**
 * @brief Block until a camera frame is ready or the timeout expires, then free the camera on timeout.
 *
 * On timeout, frees the camera DMA resources and prints a message so the caller
 * doesn't need to handle cleanup.
 * @return true if a frame arrived within timeout_ms, false if it timed out.
 */
bool wait_for_cam_frame_with_timeout(uint32_t timeout_ms) {
    if (cam_wait_for_frame(timeout_ms)) {
        return true;
    }

    free_cam();
    printf("Camera capture timed out after %lu ms\n", (unsigned long)timeout_ms);
    return false;
}

/**
 * @brief Clean up after a cam snap operation: close the file, free the camera, and reset all state.
 *
 * Called both on success (after the last chunk write) and on error paths.
 *
 */
void finish_cam_snap(void) {
    if (cam_snap_file_open) {
        FRESULT close_fr = f_close(&cam_snap_file);
        if (FR_OK != close_fr) {
            printf("f_close error: %s (%d)\n", FRESULT_str(close_fr), close_fr);
        }
        cam_snap_file_open = false;
    }

    free_cam();
    buffer_ready = false;

    cam_snap_pending = false;
    cam_snap_state = CAM_SNAP_IDLE;
    cam_snap_path[0] = '\0';
    cam_snap_write_offset = 0;
    cam_snap_total_written = 0;
}

/**
 * @brief Read one OV5640 register (hex address) over I2C1 and print its value.
 */
void run_cam_rreg(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    uint16_t reg = (uint16_t)strtol(argv[0], NULL, 16);
    uint8_t val = OV5640_RD_Reg(i2c1, 0x3C, reg);
    printf("reg 0x%04X = 0x%02X (%u)\n", reg, val, val);
}


/**
 * @brief Set the camera XCLK frequency via PWM and print the requested vs actual kHz.
 */
void run_cam_xclk(const size_t argc, const char *argv[]){
    if (argc > 1) {
        printf("Usage: cam_xclk [freq_khz]  (default 24000)\n");
        return;
    }

    uint32_t requested_khz = 37000;
    if (argc == 1) {
        requested_khz = (uint32_t)atoi(argv[0]);
    }

    uint32_t actual_khz = set_pwm_freq_kHz(requested_khz, PIN_PWM);
    printf("XCLK requested: %lu kHz, actual: %lu kHz\n",
           (unsigned long)requested_khz,
           (unsigned long)actual_khz);
}

/**
 * @brief Initialise the i2c1 peripheral at 100 kHz and configure its SDA/SCL GPIO pins.
 */
void init_i2c(){
    i2c_inst_t * i2c = i2c1;
    // Initialize I2C port at 100 kHz
    i2c_init(i2c, 100 * 1000);

    // Initialize I2C pins
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);
}

/**
 * @brief Initialise the I2C1 bus for camera SCCB communication (calls init_i2c()).
 */
void run_cam_i2c_init(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;

    init_i2c();
}

/**
 * @brief Read and print the OV5640 chip ID from registers 0x300A/0x300B over I2C1.
 */
void run_cam_id(const size_t argc, const char *argv[]){
    if (!expect_argc(argc, argv, 0)) return;
    shared_enter_cam_prog();
    uint16_t reg;
    reg=OV5640_RD_Reg(i2c1,0x3C,0X300A);
	reg<<=8;
	reg|=OV5640_RD_Reg(i2c1,0x3C,0X300B);
    shared_leave_cam_prog();
    printf("ID: %d \r\n",reg);
}

/**
 * @brief Apply the original OV5640 default sensor init sequence via sccb_init() (resets size, PLL, color format).
 */
void run_cam_defaults(const size_t argc, const char *argv[]){
    if (!expect_argc(argc, argv, 0)) return;
    // Match the original Waveshare 02-CAM flow:
    // sccb_init() does the I2C setup, applies the default table with embedded
    // delays, and then programs size, image options, PLL and colorspace.
    shared_enter_cam_prog();
    sccb_init(I2C1_SDA, I2C1_SCL);
    shared_leave_cam_prog();
    printf("Camera default sensor init applied\n");
}

/**
 * @brief Change the camera output resolution: stops the pipeline, reallocates the frame buffer, and writes the new size to the OV5640.
 */
void run_cam_size(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 2)) return;

    uint32_t w = (uint32_t)atoi(argv[0]);
    uint32_t h = (uint32_t)atoi(argv[1]);

    // Take I2C1 ownership before writing camera registers (I2C1_SDA is shared with NRF CS).
    shared_enter_cam_prog();
    // Re-assert I2C pin mux each time in case a previous shared-pin state changed GPIO mode.
    init_i2c();

    // Stop any running capture pipeline so DMA doesn't write into the buffer we are about to free.
    free_cam();
    free(cam_ptr);
    cam_ptr = NULL;

    // Update the global resolution used by all cam.c / command.cpp callers.
    cam_width    = w;
    cam_height   = h;
    cam_ful_size = w * h;

    // Write the new output window to the OV5640 (X_OUTPUT_SIZE_H encodes both W and H in one call).
    uint8_t wr = OV5640_WR_Reg_2(i2c1, 0x3C, X_OUTPUT_SIZE_H, w, h);
    if (wr == 0xFF) {
        shared_leave_cam_prog();
        printf("cam size failed: SCCB write timed out (check CAM_SDA/CAM_SCL wiring or shared-pin state)\n");
        return;
    }

    // Allocate the new double-sized buffer (cam_ful_size pixels × 2 bytes/pixel × 2 for double-buffering).
    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
    if (!cam_ptr) {
        shared_leave_cam_prog();
        printf("malloc failed for %lu x %lu\n", w, h);
        return;
    }

    shared_leave_cam_prog();
    printf("Size set to %lu x %lu\n", w, h);
    printf("Run cam dma then cam start to restart capture, or run cam snap directly\n");
}

/**
 * @brief Write the vertical flip register (TIMING_TC_REG20) of the OV5640 (0=normal, 6=flipped).
 */
void run_cam_flip(const size_t argc, const char* argv[]){
    if(!expect_argc(argc,argv,1)) return;
    uint16_t flip = (uint16_t) atoi(argv[0]);
    shared_enter_cam_prog();
    OV5640_WR_Reg(i2c1, 0x3C, TIMING_TC_REG20, flip);
    shared_leave_cam_prog();
    printf("Flip set to %u\n", flip);
}

/**
 * @brief Write the horizontal mirror register (TIMING_TC_REG21) of the OV5640 (0=normal, 6=mirrored).
 */
void run_cam_mirror(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,1)) return;
    uint16_t mirror = (uint16_t) atoi(argv[0]);
    shared_enter_cam_prog();
    OV5640_WR_Reg(i2c1, 0x3C, TIMING_TC_REG21, mirror);
    shared_leave_cam_prog();
    printf("Mirror set to %u\n", mirror);
}

/**
 * @brief Write the SC_PLL_CONTRL_2 multiplier register to adjust the OV5640 pixel clock speed.
 */
void run_cam_pll(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    uint8_t multiplier = (uint8_t)atoi(argv[0]);
    shared_enter_cam_prog();
    OV5640_WR_Reg(i2c1, 0x3C, SC_PLL_CONTRL_2, multiplier);
    shared_leave_cam_prog();
    printf("PLL multiplier set to %u\n", multiplier);
}

/**
 * @brief Set the OV5640 output color format to either rgb565 or yuv422 by writing FORMAT_CTRL/FORMAT_CTRL00.
 */
void run_cam_format(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    shared_enter_cam_prog();
    if (strcmp(argv[0], "rgb565") == 0) {
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL,  0x01);
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL00, 0x61);
        shared_leave_cam_prog();
        printf("Format set to RGB565\n");
    } else if (strcmp(argv[0], "yuv422") == 0) {
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL,  0x00);
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL00, 0x00);
        shared_leave_cam_prog();
        printf("Format set to YUV422\n");
    } else {
        shared_leave_cam_prog();
        printf("Unknown format: %s (use rgb565 or yuv422)\n", argv[0]);
    }
}

/**
 * @brief Allocate (or reallocate) the raw camera frame buffer to cam_ful_size * 2 bytes.
 */
void run_cam_alloc(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    if(cam_ptr) free(cam_ptr);
    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
}

/**
 * @brief Enable IRQ-driven capture and configure the DMA channel and buffer for the camera pipeline.
 */
void run_cam_dma(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    cam_set_use_irq(true);
    config_cam_buffer();
}

/**
 * @brief Enable continuous capture mode, enable IRQ, and start the camera PIO/DMA pipeline.
 */
void run_cam_start(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    cam_set_continuous(true);
    cam_set_use_irq(true);
    start_cam();
}

/**
 * @brief Write one OV5640 register (hex address and value) over I2C1 and confirm the write.
 */
void run_cam_wreg(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,2)) return;
    uint16_t reg = (uint16_t)strtol(argv[0], NULL, 16);
    uint8_t val = (uint8_t)strtol(argv[1], NULL, 16);
    shared_enter_cam_prog();
    OV5640_WR_Reg(i2c1,0x3C,reg,val);
    shared_leave_cam_prog();
    printf("reg 0x%04X = 0x%02X (%u)\n", reg, val, val);
}

/**
 * @brief Stop the camera DMA pipeline and release its resources (calls free_cam()).
 */
void run_cam_stop(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return ;
    free_cam();
}

/**
 * @brief Print a summary of camera state: resolution, buffer pointers, stream flags, FPS, and mirror status.
 */
void run_cam_status(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    printf("Resolution:      %lux%lu  (%lu pixels)\n",
           (unsigned long)cam_width, (unsigned long)cam_height, (unsigned long)cam_ful_size);
    printf("Frame buffer:    %s  (ptr=%p)\n",
           cam_ptr ? "allocated" : "not allocated", (void *)cam_ptr);
    printf("Double buffer:   %s  (ptr1=%p)\n",
           cam_ptr1 ? "active" : "inactive", (void *)cam_ptr1);
    printf("Display ptr:     %p\n", (void *)cam_display_ptr);
    printf("Buffer ready:    %s\n", buffer_ready ? "yes" : "no");
    printf("Stream active:   %s\n", lcd_cam_stream_active ? "yes" : "no");
    printf("Mirror:          %s\n", cam_mirror_enabled ? "on" : "off");
    printf("Capture FPS:     %lu\n", (unsigned long)cam_get_capture_fps());
    printf("Capture total:   %lu frames\n", (unsigned long)cam_get_capture_frames_total());
    printf("Display FPS:     %lu\n", (unsigned long)lcd_display_fps);
    printf("Display total:   %lu frames\n", (unsigned long)lcd_display_frames_total);
}

/**
 * @brief Capture one frame synchronously (blocking wait up to 2 s) and save it to the given file path.
 */
void run_cam_capture(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    // wait for a fresh complete frame
    buffer_ready = false;
    printf("Waiting for frame...\n");
    if (!wait_for_cam_frame_with_timeout(2000)) return;
    buffer_ready = false;  // clear before we use the buffer

    save_cam_buffer_to_file(argv[0]);
}

/**
 * @brief Arm an asynchronous single-frame capture: configure the camera for one-shot mode, then hand off to
 *        the background state machine which opens the file, writes chunks, and closes it without blocking the shell.
 */
void run_cam_snap(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;
    if (!ensure_cam_buffer_allocated()) return;
    if (lcd_load_pending || lcd_cam_snap_pending || lcd_cam_stream_active) {
        printf("Another camera or LCD operation is already in progress.\n");
        return;
    }
    if (cam_snap_pending) {
        printf("A camera snapshot is already in progress\n");
        return;
    }

    strlcpy(cam_snap_path, argv[0], sizeof(cam_snap_path));

    printf("Capturing frame...\n");
    fflush(stdout);
    stdio_flush();

    // Defensive reset: after heavy stream mode switching, DMA/PIO state may still be
    // mid-transition for one loop tick. Force a clean one-shot baseline here so cam_snap
    // cannot inherit a stale busy channel/SM state and appear to hang at "armed".
    cam_ptr1 = NULL;
    free_cam();

    // Verify VSYNC (GP8) is actually toggling before arming the PIO.
    // If the camera is not free-running, cam_wait_for_frame will hang forever.
    {
        const uint vsync_pin = CAM_BASE_PIN + 8;
        gpio_init(vsync_pin);
        gpio_set_dir(vsync_pin, GPIO_IN);
        int transitions = 0;
        bool prev = gpio_get(vsync_pin);
        uint64_t deadline_us = time_us_64() + 100000;  // 100 ms ≈ 3 frames at 30 fps
        while (time_us_64() < deadline_us) {
            bool cur = gpio_get(vsync_pin);
            if (cur != prev) { transitions++; prev = cur; }
        }
        // Restore pin to PIO0 so picampinos_program_init can use it.
        gpio_set_function(vsync_pin, GPIO_FUNC_PIO0);
        printf("VSYNC check (GP%u): %d transitions in 100 ms\n", vsync_pin, transitions);
        if (transitions == 0) {
            printf("VSYNC missing — re-initialising camera sensor...\n");
            shared_enter_cam_prog();
            sccb_init(I2C1_SDA, I2C1_SCL);
            shared_leave_cam_prog();
            // Re-check after reinit — give the sensor one frame to stabilise.
            sleep_ms(50);
            gpio_init(vsync_pin);
            gpio_set_dir(vsync_pin, GPIO_IN);
            transitions = 0;
            prev = gpio_get(vsync_pin);
            deadline_us = time_us_64() + 100000;
            while (time_us_64() < deadline_us) {
                bool cur = gpio_get(vsync_pin);
                if (cur != prev) { transitions++; prev = cur; }
            }
            gpio_set_function(vsync_pin, GPIO_FUNC_PIO0);
            if (transitions == 0) {
                printf("ERROR: camera still not responding after reinit\n");
                print_prompt();
                return;
            }
            printf("Camera recovered — VSYNC restored (%d transitions)\n", transitions);
        }
    }

    // One-shot mode: disable continuous capture and IRQ so the DMA stops after a single frame.
    cam_set_continuous(false);
    cam_set_use_irq(false);
    buffer_ready = false;
    config_cam_buffer();
    start_cam();  // Begin PIO + DMA capture; process_background_tasks() will observe buffer_ready

    // Arm the background state machine.
    cam_snap_deadline = make_timeout_time_ms(2000);
    cam_snap_next_step_time = get_absolute_time();
    cam_snap_pending = true;
    cam_snap_state = CAM_SNAP_WAIT_FRAME;
    cam_snap_write_offset = 0;
    cam_snap_total_written = 0;
}
