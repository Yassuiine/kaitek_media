#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
//
#include "pico/stdlib.h"
#include "ff.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Asynchronous snapshot-to-file phases (`cam snap`).
 */
typedef enum {
    CAM_SNAP_IDLE = 0,
    CAM_SNAP_WAIT_FRAME,
    CAM_SNAP_OPEN_FILE,
    CAM_SNAP_WRITE_FILE,
    CAM_SNAP_CLOSE_FILE,
} cam_snap_state_t;

// Globals (extern declarations)
extern bool cam_snap_pending;
extern absolute_time_t cam_snap_deadline;
extern char cam_snap_path[256];
extern FIL cam_snap_file;
extern bool cam_snap_file_open;
extern uint32_t cam_snap_write_offset;
extern UINT cam_snap_total_written;
extern absolute_time_t cam_snap_next_step_time;
extern cam_snap_state_t cam_snap_state;
extern const UINT cam_snap_chunk_size;
extern bool cam_mirror_enabled;

// Function declarations
bool ensure_cam_buffer_allocated(void);
bool save_cam_buffer_to_file(const char *path);
bool wait_for_cam_frame_with_timeout(uint32_t timeout_ms);
void finish_cam_snap(void);

void run_cam_rreg(size_t argc, const char *argv[]);
void run_cam_xclk(size_t argc, const char *argv[]);
void init_i2c(void);
void run_cam_i2c_init(size_t argc, const char *argv[]);
void run_cam_id(size_t argc, const char *argv[]);
void run_cam_defaults(size_t argc, const char *argv[]);
void run_cam_size(size_t argc, const char *argv[]);
void run_cam_flip(size_t argc, const char *argv[]);
void run_cam_mirror(size_t argc, const char *argv[]);
void run_cam_pll(size_t argc, const char *argv[]);
void run_cam_format(size_t argc, const char *argv[]);
void run_cam_alloc(size_t argc, const char *argv[]);
void run_cam_dma(size_t argc, const char *argv[]);
void run_cam_start(size_t argc, const char *argv[]);
void run_cam_wreg(size_t argc, const char *argv[]);
void run_cam_stop(size_t argc, const char *argv[]);
void run_cam_status(size_t argc, const char *argv[]);
void run_cam_capture(size_t argc, const char *argv[]);
void run_cam_snap(size_t argc, const char *argv[]);

#ifdef __cplusplus
}
#endif
