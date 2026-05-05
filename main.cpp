#include <stdio.h>
#include <stdlib.h>
//
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
//
#include "command.h"
#include "cmd_sd.h"
#include "cmd_nrf.h"
#include "crash.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_rtc.h"
#include "tests.h"
#include "sd_card.h"
//
#include "diskio.h" /* Declarations of disk functions */

#ifndef USE_PRINTF
#error This program is useless without standard input and output.
#endif

static volatile bool card_det_int_pend;
static volatile uint card_det_int_gpio;

static void process_card_detect_int() {
    card_det_int_pend = false;
    for (size_t i = 0; i < sd_get_num(); ++i) {
        sd_card_t *sd_card_p = sd_get_by_num(i);
        if (!sd_card_p)
            continue;
        if (sd_card_p->card_detect_gpio == card_det_int_gpio) {
            if (sd_card_p->state.mounted) {
                DBG_PRINTF("(Card Detect Interrupt: unmounting %s)\n", sd_get_drive_prefix(sd_card_p));
                FRESULT fr = f_unmount(sd_get_drive_prefix(sd_card_p));
                if (FR_OK == fr) {
                    sd_card_p->state.mounted = false;
                } else {
                    printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
                }
            }
            sd_card_p->state.m_Status |= STA_NOINIT;  // in case medium is removed
            sd_card_detect(sd_card_p);
        }
    }
}

// If the card is physically removed, unmount the filesystem:
static void card_detect_callback(uint gpio, uint32_t events) {
    (void)events;
    // This is actually an interrupt service routine!
    card_det_int_gpio = gpio;
    card_det_int_pend = true;
}

int main() {
    crash_handler_init();
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 1);  // specify that the stream should be unbuffered

    printf("\033[2J\033[H");  // Clear Screen

    time_init();

    // Check fault capture from RAM:
    crash_info_t const *const pCrashInfo = crash_handler_get_info();
    if (pCrashInfo) {
        printf("*** Fault Capture Analysis (RAM): ***\n");
        int n = 0;
        do {
            char buf[256] = {0};
            n = dump_crash_info(pCrashInfo, n, buf, sizeof(buf));
            if (buf[0]) printf("\t%s", buf);
        } while (n != 0);
    }
    printf("Command Line Interface -- type \"help\" for help\n");
    printf("\nKaitek> ");
    fflush(stdout);

    // Implicitly called by disk_initialize,
    // but called here to set up the GPIOs
    // before enabling the card detect interrupt:
    sd_init_driver();

    for (size_t i = 0; i < sd_get_num(); ++i) {
        sd_card_t *sd_card_p = sd_get_by_num(i);
        if (!sd_card_p) 
            continue;
        if (sd_card_p->use_card_detect) {
            // Set up an interrupt on Card Detect to detect removal of the card
            // when it happens:
            gpio_set_irq_enabled_with_callback(
                sd_card_p->card_detect_gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                true, &card_detect_callback);
        }
    }

    for (;;) {  // Super Loop
        if (logger_enabled &&
            absolute_time_diff_us(get_absolute_time(), next_log_time) < 0) {
            if (!process_logger()) logger_enabled = false;
            next_log_time = delayed_by_ms(next_log_time, period);
        }
        if (card_det_int_pend)
            process_card_detect_int();
        // Drain any pending input first so the shell stays responsive even
        // while background tasks like camera file saves are active.
        for (;;) {
            int cRxedChar = getchar_timeout_us(0);
            if (PICO_ERROR_TIMEOUT == cRxedChar) {
                break;
            }
            process_stdio(cRxedChar);
        }
        process_background_tasks();
        // When no time-sensitive streaming is running, yield the CPU rather than
        // busy-spinning.  best_effort_wfe_or_timeout() enters WFE and returns on
        // the next interrupt (USB SOF, timer, GPIO) or after 1 ms at most — so
        // latency for input and background tasks stays under 1 ms while power
        // consumption drops by ~60 % in idle and ~20 % during shell use.
        if (!lcd_cam_stream_active && !nrf_fft_stream_is_active()) {
            best_effort_wfe_or_timeout(make_timeout_time_ms(1));
        }
    }
    return 0;
}
