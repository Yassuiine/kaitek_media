#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
//
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// Globals (extern declarations)
extern bool logger_enabled;
extern const uint32_t period;
extern absolute_time_t next_log_time;

// Function declarations
const char *chk_dflt_log_drv(size_t argc, const char *argv[]);
void run_date(size_t argc, const char *argv[]);
void run_setrtc(size_t argc, const char *argv[]);
void run_info(size_t argc, const char *argv[]);
void run_format(size_t argc, const char *argv[]);
void run_mount(size_t argc, const char *argv[]);
void run_unmount(size_t argc, const char *argv[]);
void run_chdrive(size_t argc, const char *argv[]);
void run_cd(size_t argc, const char *argv[]);
void run_mkdir(size_t argc, const char *argv[]);
void run_ls(size_t argc, const char *argv[]);
void run_pwd(size_t argc, const char *argv[]);
void run_cat(size_t argc, const char *argv[]);
void run_cp(size_t argc, const char *argv[]);
void run_mv(size_t argc, const char *argv[]);
void run_rm(size_t argc, const char *argv[]);
void del_node(const char *path);
void run_del_node(size_t argc, const char *argv[]);
void run_lliot(size_t argc, const char *argv[]);
void run_big_file_test(size_t argc, const char *argv[]);
void run_simple(size_t argc, const char *argv[]);
void run_bench(size_t argc, const char *argv[]);
void run_cdef(size_t argc, const char *argv[]);
void run_swcwdt(size_t argc, const char *argv[]);
void run_loop_swcwdt(size_t argc, const char *argv[]);
void run_start_logger(size_t argc, const char *argv[]);
void run_stop_logger(size_t argc, const char *argv[]);

#ifdef __cplusplus
}
#endif
