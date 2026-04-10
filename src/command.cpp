#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
//
#include "hardware/adc.h"
#include "hardware/clocks.h" 
#include "pico/aon_timer.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
//
#include "f_util.h"
#include "crash.h"
#include "hw_config.h"
#include "my_debug.h"
#include "my_rtc.h"
#include "sd_card.h"
#include "tests.h"
#include "cam.h"
#include "ov5640.h"
//
#include "diskio.h" /* Declarations of disk functions */
//
#include "command.h"

static char *saveptr;  // For strtok_r
static volatile bool die;
static char cmd_buffer[256];
static size_t cmd_ix;
static bool cam_snap_pending;
static absolute_time_t cam_snap_deadline;
static char cam_snap_path[256];
static FIL cam_snap_file;
static bool cam_snap_file_open;
static uint32_t cam_snap_write_offset;
static UINT cam_snap_total_written;
static absolute_time_t cam_snap_next_step_time;
static bool last_input_was_cr;

enum cam_snap_state_t {
    CAM_SNAP_IDLE = 0,
    CAM_SNAP_WAIT_FRAME,
    CAM_SNAP_OPEN_FILE,
    CAM_SNAP_WRITE_FILE,
    CAM_SNAP_CLOSE_FILE,
};

static cam_snap_state_t cam_snap_state;
static const UINT cam_snap_chunk_size = 512;

bool logger_enabled;
const uint32_t period = 1000;
absolute_time_t next_log_time;

#pragma GCC diagnostic ignored "-Wunused-function"
#ifdef NDEBUG 
#  pragma GCC diagnostic ignored "-Wunused-variable"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

static void missing_argument_msg() {
    printf("Missing argument\n");
}
static void extra_argument_msg(const char *s) {
    printf("Unexpected argument: %s\n", s);
}
static bool expect_argc(const size_t argc, const char *argv[], const size_t expected) {
    if (argc < expected) {
        missing_argument_msg();
        return false;
    }
    if (argc > expected) {
        extra_argument_msg(argv[expected]);
        return false;
    }
    return true;
}
const char *chk_dflt_log_drv(const size_t argc, const char *argv[]) {
    if (argc > 1) {
        extra_argument_msg(argv[1]);
        return NULL;
    }
    if (!argc) {
        if (1 == sd_get_num()) {
            return sd_get_drive_prefix(sd_get_by_num(0));
        } else {
            printf("Missing argument: Specify logical drive\n");
            return NULL;
        }        
    }
    return argv[0];
}
static void run_date(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    char buf[128] = {0};
    time_t epoch_secs = time(NULL);
    if (epoch_secs < 1) {
        printf("RTC not running\n");
        return;
    }
    struct tm *ptm = localtime(&epoch_secs);
    assert(ptm);
    size_t n = strftime(buf, sizeof(buf), "%c", ptm);
    assert(n);
    printf("%s\n", buf);
    strftime(buf, sizeof(buf), "%j",
             ptm);  // The day of the year as a decimal number (range
                    // 001 to 366).
    printf("Day of year: %s\n", buf);
}
static void run_setrtc(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 6)) return;

    int date = atoi(argv[0]);
    int month = atoi(argv[1]);
    int year = atoi(argv[2]) + 2000;
    int hour = atoi(argv[3]);
    int min = atoi(argv[4]);
    int sec = atoi(argv[5]);

    struct tm t = {
        // tm_sec	int	seconds after the minute	0-61*
        .tm_sec = sec,
        // tm_min	int	minutes after the hour	0-59
        .tm_min = min,
        // tm_hour	int	hours since midnight	0-23
        .tm_hour = hour,
        // tm_mday	int	day of the month	1-31
        .tm_mday = date,
        // tm_mon	int	months since January	0-11
        .tm_mon = month - 1,
        // tm_year	int	years since 1900
        .tm_year = year - 1900,
        // tm_wday	int	days since Sunday	0-6
        .tm_wday = 0,
        // tm_yday	int	days since January 1	0-365
        .tm_yday = 0,
        // tm_isdst	int	Daylight Saving Time flag
        .tm_isdst = 0
    };
    /* The values of the members tm_wday and tm_yday of timeptr are ignored, and the values of
       the other members are interpreted even if out of their valid ranges */
    time_t epoch_secs = mktime(&t);
    if (-1 == epoch_secs) {
        printf("The passed in datetime was invalid\n");
        return;
    }
    struct timespec ts = {.tv_sec = epoch_secs, .tv_nsec = 0};
    aon_timer_set_time(&ts);
}
static char const *fs_type_string(int fs_type) {
    switch (fs_type) {
        case FS_FAT12:
            return "FAT12";
        case FS_FAT16:
            return "FAT16";
        case FS_FAT32:
            return "FAT32";
        case FS_EXFAT:
            return "EXFAT";
        default:
            return "Unknown";
    }
}
static const char *sd_if_string(sd_if_t type) {
    switch (type) {
        case SD_IF_SPI:
            return "SPI";
        case SD_IF_SDIO:
            return "SDIO";
        default:
            return "NONE";
    }
}
static const char *sd_card_type_string(card_type_t type) {
    switch (type) {
        case SDCARD_NONE:
            return "None";
        case SDCARD_V1:
            return "SDv1";
        case SDCARD_V2:
            return "SDv2";
        case SDCARD_V2HC:
            return "SDv2HC";
        default:
            return "Unknown";
    }
}
static void run_info(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;
    sd_card_t *sd_card_p = sd_get_by_drive_prefix(arg);
    if (!sd_card_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }
    int ds = sd_card_p->init(sd_card_p);
    if (STA_NODISK & ds || STA_NOINIT & ds) {
        printf("SD card initialization failed\n");
        return;
    }
    printf("Drive %s interface=%s card_type=%s status=0x%02x sectors=%lu\n",
           arg, sd_if_string(sd_card_p->type), sd_card_type_string(sd_card_p->state.card_type),
           sd_card_p->state.m_Status, (unsigned long)sd_card_p->state.sectors);
    // Card IDendtification register. 128 buts wide.
    cidDmp(sd_card_p, printf);
    // Card-Specific Data register. 128 bits wide.
    csdDmp(sd_card_p, printf);
    
    // SD Status
    size_t au_size_bytes;
    bool ok = sd_allocation_unit(sd_card_p, &au_size_bytes);
    if (ok)
        printf("\nSD card Allocation Unit (AU_SIZE) or \"segment\": %zu bytes (%zu sectors)\n", 
            au_size_bytes, au_size_bytes / sd_block_size);
    
    if (!sd_card_p->state.mounted) {
        printf("Drive \"%s\" is not mounted\n", argv[0]);
        return;
    }

    /* Get volume information and free clusters of drive */
    FATFS *fs_p = &sd_card_p->state.fatfs;
    if (!fs_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }
    DWORD fre_clust, fre_sect, tot_sect;
    FRESULT fr = f_getfree(arg, &fre_clust, &fs_p);
    if (FR_OK != fr) {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    /* Get total sectors and free sectors */
    tot_sect = (fs_p->n_fatent - 2) * fs_p->csize;
    fre_sect = fre_clust * fs_p->csize;
    /* Print the free space (assuming 512 bytes/sector) */
    printf("\n%10lu KiB (%lu MiB) total drive space.\n%10lu KiB (%lu MiB) available.\n",
           tot_sect / 2, tot_sect / 2 / 1024,
           fre_sect / 2, fre_sect / 2 / 1024);

#if FF_USE_LABEL
    // Report label:
    TCHAR buf[34] = {};/* [OUT] Volume label */
    DWORD vsn;         /* [OUT] Volume serial number */
    fr = f_getlabel(arg, buf, &vsn);
    if (FR_OK != fr) {
        printf("f_getlabel error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    } else {
        printf("\nVolume label: %s\nVolume serial number: %lu\n", buf, vsn);
    }
#endif

    // Report format
    printf("\nFilesystem type: %s\n", fs_type_string(fs_p->fs_type));

    // Report Partition Starting Offset
    // uint64_t offs = fs_p->volbase;
    // printf("Partition Starting Offset: %llu sectors (%llu bytes)\n",
    //         offs, offs * sd_block_size);
	printf("Volume base sector: %llu\n", fs_p->volbase);		
	printf("FAT base sector: %llu\n", fs_p->fatbase);		
	printf("Root directory base sector (FAT12/16) or cluster (FAT32/exFAT): %llu\n", fs_p->dirbase);		 
	printf("Data base sector: %llu\n", fs_p->database);		

    // Report cluster size ("allocation unit")
    printf("FAT Cluster size (\"allocation unit\"): %d sectors (%llu bytes)\n",
           fs_p->csize,
           (uint64_t)sd_card_p->state.fatfs.csize * FF_MAX_SS);
}
static void run_format(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;
    sd_card_t *sd_card_p = sd_get_by_drive_prefix(arg);
    if (!sd_card_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }
    int ds = sd_card_p->init(sd_card_p);
    if (STA_NODISK & ds || STA_NOINIT & ds) {
        printf("SD card initialization failed, status=0x%02x\n", ds);
        return;
    }
    printf("Drive %s interface=%s card_type=%s status=0x%02x sectors=%lu\n",
           arg, sd_if_string(sd_card_p->type), sd_card_type_string(sd_card_p->state.card_type),
           sd_card_p->state.m_Status, (unsigned long)sd_card_p->state.sectors);
    
    /* I haven't been able to find a way to obtain the layout produced
    by the SD Association's "SD Memory Card Formatter"
    (https://www.sdcard.org/downloads/formatter/).

    SD Memory Card Formatter:
    Volume base sector: 8192
    FAT base sector: 8790
    Root directory base sector (FAT12/16) or cluster (FAT32/exFAT): 2
    Data base sector: 16384
    FAT Cluster size ("allocation unit"): 64 sectors (32768 bytes)

    f_mkfs:
    Volume base sector: 63
    FAT base sector: 594
    Root directory base sector (FAT12/16) or cluster (FAT32/exFAT): 2
    Data base sector: 8192
    FAT Cluster size ("allocation unit"): 64 sectors (32768 bytes)    
    */

    /* Attempt to align partition to SD card segment (AU) */
    size_t au_size_bytes;
    bool ok = sd_allocation_unit(sd_card_p, &au_size_bytes);
    if (!ok || !au_size_bytes)
        au_size_bytes = 4194304; // Default to 4 MiB
    UINT n_align = au_size_bytes / sd_block_size;

    MKFS_PARM opt = {
        FM_ANY,  /* Format option (FM_FAT, FM_FAT32, FM_EXFAT and FM_SFD) */
        2,       /* Number of FATs */
        n_align, /* Data area alignment (sector) */
        0,       /* Number of root directory entries */
        0        /* Cluster size (byte) */
    };
    /* Format the drive */
    printf("Formatting %s (this may take a while)...\n", arg);
    FRESULT fr = f_mkfs(arg, &opt, 0, 32 * 1024);  // 32 KB: ~32x fewer disk_write calls vs FF_MAX_SS*2
    if (FR_OK != fr) printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);

    /* This only works if the drive is mounted: */
#if FF_USE_LABEL
    TCHAR label[32];
    snprintf(label, sizeof(label), "%sFatFS", arg);
    fr = f_setlabel(label);
#endif
}
static void run_mount(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;
    sd_card_t *sd_card_p = sd_get_by_drive_prefix(arg);
    if (!sd_card_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }
    int ds = sd_card_p->init(sd_card_p);
    if (STA_NODISK & ds || STA_NOINIT & ds) {
        printf("SD card initialization failed, status=0x%02x\n", ds);
        return;
    }
    printf("Drive %s interface=%s card_type=%s status=0x%02x sectors=%lu\n",
           arg, sd_if_string(sd_card_p->type), sd_card_type_string(sd_card_p->state.card_type),
           sd_card_p->state.m_Status, (unsigned long)sd_card_p->state.sectors);
    fflush(stdout);
    stdio_flush();
    printf("f_mount: begin\n");
    fflush(stdout);
    stdio_flush();
    FATFS *fs_p = &sd_card_p->state.fatfs;
    FRESULT fr = f_mount(fs_p, arg, 1);
    printf("f_mount: returned %s (%d)\n", FRESULT_str(fr), fr);
    fflush(stdout);
    stdio_flush();
    if (FR_OK != fr) {
        return;
    }
    sd_card_p->state.mounted = true;
}
static void run_unmount(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;

    sd_card_t *sd_card_p = sd_get_by_drive_prefix(arg);
    if (!sd_card_p) {
        printf("Unknown logical drive id: \"%s\"\n", arg);
        return;
    }    
    FRESULT fr = f_unmount(arg);
    if (FR_OK != fr) {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_p->state.mounted = false;
    sd_card_p->state.m_Status |= STA_NOINIT;  // in case medium is removed
}
static void run_chdrive(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;

    FRESULT fr = f_chdrive(arg);
    if (FR_OK != fr) printf("f_chdrive error: %s (%d)\n", FRESULT_str(fr), fr);
}
static void run_cd(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    FRESULT fr = f_chdir(argv[0]);
    if (FR_OK != fr) printf("f_chdir error: %s (%d)\n", FRESULT_str(fr), fr);
}
static void run_mkdir(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    FRESULT fr = f_mkdir(argv[0]);
    if (FR_OK != fr) printf("f_mkdir error: %s (%d)\n", FRESULT_str(fr), fr);
}
static void run_ls(const size_t argc, const char *argv[]) {
    if (argc > 1) {
        extra_argument_msg(argv[1]);
        return;
    }
    if (argc)
        ls(argv[0]);
    else
        ls("");
}
static void run_pwd(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    char buf[512];
    FRESULT fr = f_getcwd(buf, sizeof buf);
    if (FR_OK != fr)
        printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
    else
        printf("%s", buf);
}
static void run_cat(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    FIL fil;
    FRESULT fr = f_open(&fil, argv[0], FA_READ);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil)) {
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr) printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
}
static void run_cp(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 2)) return;

    FIL fsrc, fdst;    /* File objects */
    FRESULT fr;        /* FatFs function common result code */
    UINT br, bw;       /* File read/write count */

    /* Open source file on the drive 1 */
    fr = f_open(&fsrc, argv[0], FA_READ);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    /* Create destination file on the drive 0 */
    fr = f_open(&fdst, argv[1], FA_WRITE | FA_CREATE_ALWAYS);
    if (FR_OK != fr) {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        f_close(&fsrc);        
        return;
    }
    /* Copy source to destination */
    FSIZE_t buffer_sz = f_size(&fsrc);
    if (buffer_sz > 32768)
        buffer_sz = 32768;
    /* File copy buffer */
    BYTE *buffer = (BYTE *)malloc(buffer_sz);
    if (!buffer) {
        printf("malloc(%llu) failed\n", buffer_sz);
        f_close(&fdst);
        f_close(&fsrc);
        return;
    }
    for (;;) {
        fr = f_read(&fsrc, buffer, buffer_sz, &br); /* Read a chunk of data from the source file */
        if (FR_OK != fr) printf("f_read error: %s (%d)\n", FRESULT_str(fr), fr);
        if (br == 0) break;                   /* error or eof */
        fr = f_write(&fdst, buffer, br, &bw); /* Write it to the destination file */
        if (FR_OK != fr) printf("f_write error: %s (%d)\n", FRESULT_str(fr), fr);
        if (bw < br) break; /* error or disk full */
    }
    free(buffer);
    /* Close open files */
    fr = f_close(&fsrc);
    if (FR_OK != fr) printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    fr = f_close(&fdst);
    if (FR_OK != fr) printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
}
static void run_mv(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 2)) return;

    FRESULT fr = f_rename(argv[0], argv[1]);
    if (FR_OK != fr) printf("f_rename error: %s (%d)\n", FRESULT_str(fr), fr);
}
static void run_lliot(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;

    size_t pnum = 0;
    pnum = strtoul(arg, NULL, 0);
    lliot(pnum);
}
static void run_big_file_test(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 3)) return;

    const char *pcPathName = argv[0];
    size_t size = strtoul(argv[1], 0, 0);
    uint32_t seed = atoi(argv[2]);
    big_file_test(pcPathName, size, seed);
}
static void del_node(const char *path) {
    FILINFO fno;
    char buff[256];
    /* Directory to be deleted */
    strlcpy(buff, path, sizeof(buff));
    /* Delete the directory */
    FRESULT fr = delete_node(buff, sizeof buff / sizeof buff[0], &fno);
    /* Check the result */
    if (fr) {
        printf("Failed to delete the directory %s. ", path);
        printf("%s error: %s (%d)\n", __func__, FRESULT_str(fr), fr);
    }
}
static void run_del_node(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;
    del_node(argv[0]);
}
static void run_rm(const size_t argc, const char *argv[]) {
    if (argc < 1) {
        missing_argument_msg();
        return;
    }
    if (argc > 2) {
        extra_argument_msg(argv[2]);
        return;
    }
    if (2 == argc) {
        if (0 == strcmp("-r", argv[0])) {
            del_node(argv[1]);
        } else if (0 == strcmp("-d", argv[0])) {
            FRESULT fr = f_unlink(argv[1]);
            if (FR_OK != fr) printf("f_unlink error: %s (%d)\n", FRESULT_str(fr), fr);
        } else {
            EMSG_PRINTF("Unknown option: %s\n", argv[0]);
        }
    } else {
        FRESULT fr = f_unlink(argv[0]);
        if (FR_OK != fr) printf("f_unlink error: %s (%d)\n", FRESULT_str(fr), fr);
    }
}
static void run_simple(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    simple();
}
static void run_bench(const size_t argc, const char *argv[]) {
    const char *arg = chk_dflt_log_drv(argc, argv);
    if (!arg)
        return;

    bench(arg);
}
static void run_cdef(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    f_mkdir("/cdef");  // fake mountpoint
    vCreateAndVerifyExampleFiles("/cdef");
}
static void run_swcwdt(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    vStdioWithCWDTest("/cdef");
}
static void run_loop_swcwdt(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    die = false;
    do {
        f_chdir("/");
        del_node("/cdef");
        f_mkdir("/cdef");  // fake mountpoint
        vCreateAndVerifyExampleFiles("/cdef");
        vStdioWithCWDTest("/cdef");
    } while (!die);
}
static void run_start_logger(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
    adc_init();
    adc_set_temp_sensor_enabled(true);
    logger_enabled = true;
    next_log_time = delayed_by_ms(get_absolute_time(), period);
}
static void run_stop_logger(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    logger_enabled = false;
}
static void run_mem_stats(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;
 
    // extern ptrdiff_t __StackTop, __StackBottom, __StackOneTop, __StackOneBottom;
    // printf("__StackTop - __StackBottom = %zu\n", __StackTop - __StackBottom);
    // printf("__StackOneTop - __StackOneBottom = %zu\n", __StackOneTop - __StackOneBottom);

    malloc_stats();
}

/* Derived from pico-examples/clocks/hello_48MHz/hello_48MHz.c */
static void run_measure_freqs(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
#if PICO_RP2040
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
#endif

    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\treported  = %lukHz\n", f_clk_sys, clock_get_hz(clk_sys) / KHZ);
    printf("clk_peri = %dkHz\treported  = %lukHz\n", f_clk_peri, clock_get_hz(clk_peri) / KHZ);
    printf("clk_usb  = %dkHz\treported  = %lukHz\n", f_clk_usb, clock_get_hz(clk_usb) / KHZ);
    printf("clk_adc  = %dkHz\treported  = %lukHz\n", f_clk_adc, clock_get_hz(clk_adc) / KHZ);
#if PICO_RP2040
    printf("clk_rtc  = %dkHz\treported  = %lukHz\n", f_clk_rtc, clock_get_hz(clk_rtc) / KHZ);
#endif

    // Can't measure clk_ref / xosc as it is the ref
}
static void run_set_sys_clock_48mhz(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    set_sys_clock_48mhz();
    setup_default_uart();
}
static void run_set_sys_clock_khz(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    int khz = atoi(argv[0]);

    bool configured = set_sys_clock_khz(khz, false);
    if (!configured) {
        printf("Not possible. Clock not configured.\n");
        return;
    }
    /*
    By default, when reconfiguring the system clock PLL settings after runtime initialization,
    the peripheral clock is switched to the 48MHz USB clock to ensure continuity of peripheral operation.
    There seems to be a problem with running the SPI 2.4 times faster than the system clock,
    even at the same SPI baud rate.
    Anyway, for now, reconfiguring the peripheral clock to the system clock at its new frequency works OK.
    */
    bool ok = clock_configure(clk_peri,
                              0,
                              CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                              clock_get_hz(clk_sys),
                              clock_get_hz(clk_sys));
    assert(ok);

    setup_default_uart();
}
static void set(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    int gp = atoi(argv[0]);

    gpio_init(gp);
    gpio_set_dir(gp, GPIO_OUT);
    gpio_put(gp, 1);
}
static void clr(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    int gp = atoi(argv[0]);

    gpio_init(gp);
    gpio_set_dir(gp, GPIO_OUT);
    gpio_put(gp, 0);
}
static void run_test(const size_t argc, const char *argv[]) {    
    if (!expect_argc(argc, argv, 0)) return;

    extern bool my_test();
    my_test();
}

static void run_help(const size_t argc, const char *argv[]);

static void print_prompt(void) {
    printf("Kaitek> ");
    stdio_flush();
}

static bool ensure_cam_buffer_allocated() {
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

static bool save_cam_buffer_to_file(const char *path) {
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

static bool wait_for_cam_frame_with_timeout(uint32_t timeout_ms) {
    if (cam_wait_for_frame(timeout_ms)) {
        return true;
    }

    free_cam();
    printf("Camera capture timed out after %lu ms\n", (unsigned long)timeout_ms);
    return false;
}

static void finish_cam_snap(void) {
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

//added
static void run_cam_rreg(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    uint16_t reg = (uint16_t)strtol(argv[0], NULL, 16);
    uint8_t val = OV5640_RD_Reg(i2c1, 0x3C, reg);
    printf("reg 0x%04X = 0x%02X (%u)\n", reg, val, val);
}


//added
static void run_cam_xclk(const size_t argc, const char *argv[]){
    if(argc == 1){
        set_pwm_freq_kHz((size_t)atoi(argv[0]), PIN_PWM);
    }
    else{
        set_pwm_freq_kHz(24000, PIN_PWM);
    }
}

//added
static void init_i2c(){
    i2c_inst_t * i2c = i2c1;
    // Initialize I2C port at 100 kHz
    i2c_init(i2c, 100 * 1000);

    // Initialize I2C pins
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);
}

//added
static void run_cam_i2c_init(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    
    init_i2c();
}

//added
static void run_cam_id(const size_t argc, const char *argv[]){
    if (!expect_argc(argc, argv, 0)) return;
    uint16_t reg;
    reg=OV5640_RD_Reg(i2c1,0x3C,0X300A);
	reg<<=8;
	reg|=OV5640_RD_Reg(i2c1,0x3C,0X300B);
    printf("ID: %d \r\n",reg);
}

//added
static void run_cam_defaults(const size_t argc, const char *argv[]){
    if (!expect_argc(argc, argv, 0)) return;
    // Match the original Waveshare 02-CAM flow:
    // sccb_init() does the I2C setup, applies the default table with embedded
    // delays, and then programs size, image options, PLL and colorspace.
    sccb_init(I2C1_SDA, I2C1_SCL);
    printf("Camera default sensor init applied\n");
}

//added
static void run_cam_size(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 2)) return;

    uint32_t w = (uint32_t)atoi(argv[0]);
    uint32_t h = (uint32_t)atoi(argv[1]);

    // stop pipeline and free old buffer
    free_cam();
    free(cam_ptr);
    cam_ptr = NULL;

    // update globals — cam.c will use these from now on
    cam_width    = w;
    cam_height   = h;
    cam_ful_size = w * h;

    // tell sensor new output size
    OV5640_WR_Reg_2(i2c1, 0x3C, X_OUTPUT_SIZE_H, w, h);

    // reallocate buffer
    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
    if (!cam_ptr) {
        printf("malloc failed for %lu x %lu\n", w, h);
        return;
    }

    printf("Size set to %lu x %lu\n", w, h);
    printf("Run cam_dma then cam_start to restart capture\n");
}

//added
static void run_cam_flip(const size_t argc, const char* argv[]){
    if(!expect_argc(argc,argv,1)) return;
    uint16_t flip = (uint16_t) atoi(argv[0]);
    OV5640_WR_Reg(i2c1, 0x3C, TIMING_TC_REG20, flip);
    printf("Flip set to %u\n", flip);
}

//added
static void run_cam_mirror(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,1)) return;
    uint16_t mirror = (uint16_t) atoi(argv[0]);
    OV5640_WR_Reg(i2c1, 0x3C, TIMING_TC_REG21, mirror);
    printf("Mirror set to %u\n", mirror);
}

//added
static void run_cam_pll(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    uint8_t multiplier = (uint8_t)atoi(argv[0]);
    OV5640_WR_Reg(i2c1, 0x3C, SC_PLL_CONTRL_2, multiplier);
    printf("PLL multiplier set to %u\n", multiplier);
}

//added
static void run_cam_format(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    if (strcmp(argv[0], "rgb565") == 0) {
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL,  0x01);
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL00, 0x61);
        printf("Format set to RGB565\n");
    } else if (strcmp(argv[0], "yuv422") == 0) {
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL,  0x00);
        OV5640_WR_Reg(i2c1, 0x3C, FORMAT_CTRL00, 0x00);
        printf("Format set to YUV422\n");
    } else {
        printf("Unknown format: %s (use rgb565 or yuv422)\n", argv[0]);
    }
}

//added
static void run_cam_alloc(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    if(cam_ptr) free(cam_ptr);
    cam_ptr = (uint8_t *)malloc(cam_ful_size * 2);
}

//added
static void run_cam_dma(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    cam_set_use_irq(true);
    config_cam_buffer();
}

//added
static void run_cam_start(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return;
    cam_set_continuous(true);
    cam_set_use_irq(true);
    start_cam();
}

//added
static void run_cam_wreg(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,2)) return;
    uint16_t reg = (uint16_t)strtol(argv[0], NULL, 16);
    uint8_t val = (uint8_t)strtol(argv[1], NULL, 16);
    OV5640_WR_Reg(i2c1,0x3C,reg,val);
    printf("reg 0x%04X = 0x%02X (%u)\n", reg, val, val);
}

//added
static void run_cam_stop(const size_t argc, const char *argv[]){
    if(!expect_argc(argc,argv,0)) return ;
    free_cam();
}

//added
static void run_cam_capture(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;

    // wait for a fresh complete frame
    buffer_ready = false;
    printf("Waiting for frame...\n");
    if (!wait_for_cam_frame_with_timeout(2000)) return;
    buffer_ready = false;  // clear before we use the buffer

    save_cam_buffer_to_file(argv[0]);
}

//added
static void run_cam_snap(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 1)) return;
    if (!ensure_cam_buffer_allocated()) return;
    if (cam_snap_pending) {
        printf("A camera snapshot is already in progress\n");
        return;
    }

    strlcpy(cam_snap_path, argv[0], sizeof(cam_snap_path));

    printf("Capturing frame...\n");
    fflush(stdout);
    stdio_flush();

    cam_set_continuous(false);
    cam_set_use_irq(false);
    buffer_ready = false;
    config_cam_buffer();
    printf("cam_snap: before start_cam\n");
    fflush(stdout);
    stdio_flush();
    start_cam();
    printf("cam_snap: after start_cam\n");
    fflush(stdout);
    stdio_flush();

    cam_snap_deadline = make_timeout_time_ms(2000);
    cam_snap_next_step_time = get_absolute_time();
    cam_snap_pending = true;
    cam_snap_state = CAM_SNAP_WAIT_FRAME;
    cam_snap_write_offset = 0;
    cam_snap_total_written = 0;
    printf("cam_snap: armed\n");
}


typedef void (*p_fn_t)(const size_t argc, const char *argv[]);
typedef struct {
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc,
     "setrtc <DD> <MM> <YY> <hh> <mm> <ss>:\n"
     " Set Real Time Clock\n"
     " Parameters: new date (DD MM YY) new time in 24-hour format "
     "(hh mm ss)\n"
     "\te.g.:setrtc 16 3 21 0 4 0"},
    {"date", run_date, "date:\n Print current date and time"},
    {"format", run_format,
     "format [<drive#:>]:\n"
     " Creates an FAT/exFAT volume on the logical drive.\n"
     "\te.g.: format 0:"},
    {"mount", run_mount,
     "mount [<drive#:>]:\n"
     " Register the work area of the volume\n"
     "\te.g.: mount 0:"},
    {"unmount", run_unmount,
     "unmount <drive#:>:\n"
     " Unregister the work area of the volume"},
    {"chdrive", run_chdrive,
     "chdrive <drive#:>:\n"
     " Changes the current directory of the logical drive.\n"
     " <path> Specifies the directory to be set as current directory.\n"
     "\te.g.: chdrive 1:"},
    {"info", run_info, 
    "info [<drive#:>]:\n"
      " Print information about an SD card"},
    {"cd", run_cd,
     "cd <path>:\n"
     " Changes the current directory of the logical drive.\n"
     " <path> Specifies the directory to be set as current directory.\n"
     "\te.g.: cd /dir1"},
    {"mkdir", run_mkdir,
     "mkdir <path>:\n"
     " Make a new directory.\n"
     " <path> Specifies the name of the directory to be created.\n"
     "\te.g.: mkdir /dir1"},
    // {"del_node", run_del_node,
    //  "del_node <path>:\n"
    //  "  Remove directory and all of its contents.\n"
    //  "  <path> Specifies the name of the directory to be deleted.\n"
    //  "\te.g.: del_node /dir1"},
    {"rm", run_rm,
     "rm [options] <pathname>:\n"
     " Removes (deletes) a file or directory\n"
     " <pathname> Specifies the path to the file or directory to be removed\n"
     " Options:\n"
     " -d Remove an empty directory\n"
     " -r Recursively remove a directory and its contents"},
    {"cp", run_cp,
     "cp <source file> <dest file>:\n"
     " Copies <source file> to <dest file>"},
    {"mv", run_mv,
     "mv <source file> <dest file>:\n"
     " Moves (renames) <source file> to <dest file>"},
    {"pwd", run_pwd,
     "pwd:\n"
     " Print Working Directory"},
    {"ls", run_ls, "ls [pathname]:\n List directory"},
    // {"dir", run_ls, "dir:\n List directory"},
    {"cat", run_cat, "cat <filename>:\n Type file contents"},
    {"simple", run_simple, "simple:\n Run simple FS tests"},
    {"lliot", run_lliot,
     "lliot <physical drive#>:\n !DESTRUCTIVE! Low Level I/O Driver Test\n"
     "The SD card will need to be reformatted after this test.\n"
     "\te.g.: lliot 1"},
    {"bench", run_bench, "bench <drive#:>:\n A simple binary write/read benchmark"},
    {"big_file_test", run_big_file_test,
     "big_file_test <pathname> <size in MiB> <seed>:\n"
     " Writes random data to file <pathname>.\n"
     " Specify <size in MiB> in units of mebibytes (2^20, or 1024*1024 bytes)\n"
     "\te.g.: big_file_test 0:/bf 1 1\n"
     "\tor: big_file_test 1:big3G-3 3072 3"},
    {"bft", run_big_file_test,"bft: Alias for big_file_test"},
    {"cdef", run_cdef,
     "cdef:\n Create Disk and Example Files\n"
     " Expects card to be already formatted and mounted"},
    {"swcwdt", run_swcwdt,
     "swcwdt:\n Stdio With CWD Test\n"
     "Expects card to be already formatted and mounted.\n"
     "Note: run cdef first!"},
    {"loop_swcwdt", run_loop_swcwdt,
     "loop_swcwdt:\n Run Create Disk and Example Files and Stdio With CWD "
     "Test in a loop.\n"
     "Expects card to be already formatted and mounted.\n"
     "Note: Hit Enter key to quit."},
    {"start_logger", run_start_logger,
     "start_logger:\n"
     " Start Data Log Demo"},
    {"stop_logger", run_stop_logger,
     "stop_logger:\n"
     " Stop Data Log Demo"},
    {"mem-stats", run_mem_stats,
     "mem-stats:\n"
     " Print memory statistics"},
    {"cam_xclk", run_cam_xclk,
     "cam_xclk [<freq_khz>]:\n"
     " Set camera XCLK frequency. Default 24000\n"
     "\te.g.: cam_xclk 24000"},
    {"cam_i2c", run_cam_i2c_init,
     "cam_i2c:\n"
     " Initialise I2C1 bus for camera SCCB"},
    {"cam_id", run_cam_id,
     "cam_id:\n"
     " Read and print OV5640 chip ID"},
    {"cam_defaults", run_cam_defaults,
     "cam_defaults:\n"
     " Apply the original OV5640 default sensor init sequence"},
    {"cam_size", run_cam_size,
     "cam_size <w> <h>:\n"
     " Set camera output resolution\n"
     "\te.g.: cam_size 240 320"},
    {"cam_flip", run_cam_flip,
     "cam_flip <val>:\n"
     " Set vertical flip (0=normal, 6=flipped)\n"
     "\te.g.: cam_flip 0"},
    {"cam_mirror", run_cam_mirror,
     "cam_mirror <val>:\n"
     " Set horizontal mirror (0=normal, 6=mirrored)\n"
     "\te.g.: cam_mirror 0"},
    {"cam_pll", run_cam_pll,
     "cam_pll <multiplier>:\n"
     " Set PLL multiplier (4-127 any, 128-252 even only)\n"
     "\te.g.: cam_pll 11"},
    {"cam_format", run_cam_format,
     "cam_format <rgb565|yuv422>:\n"
     " Set camera output color format\n"
     "\te.g.: cam_format rgb565"},
    {"cam_alloc", run_cam_alloc,
     "cam_alloc:\n"
     " Allocate frame buffer in RAM"},
    {"cam_dma", run_cam_dma,
     "cam_dma:\n"
     " Configure DMA channel and IRQ handler"},
    {"cam_start", run_cam_start,
     "cam_start:\n"
     " Load PIO program and start capture"},
    {"cam_rreg", run_cam_rreg,
     "cam_rreg <reg>:\n"
     " Read one OV5640 register (hex address)\n"
     "\te.g.: cam_rreg 501f"},
    {"cam_wreg", run_cam_wreg,
     "cam_wreg <reg> <val>:\n"
     " Write one OV5640 register (hex address and value)\n"
     "\te.g.: cam_wreg 501f 01"},
    {"cam_capture", run_cam_capture,
     "cam_capture <filename>:\n"
     " Wait for a complete frame and save to SD\n"
     "\te.g.: cam_capture 0:/photo.bin"},
    {"cam_snap", run_cam_snap,
     "cam_snap <filename>:\n"
     " Capture one frame, restore UART, and save to SD\n"
     "\te.g.: cam_snap 0:/photo.bin"},
    {"cam_stop", run_cam_stop,
     "cam_stop:\n"
     " Stop camera DMA and pipeline"},

    // // Clocks testing:
    // {"set_sys_clock_48mhz", run_set_sys_clock_48mhz,
    //  "set_sys_clock_48mhz:\n"
    //  " Set the system clock to 48MHz"},
    // {"set_sys_clock_khz", run_set_sys_clock_khz,
    //  "set_sys_clock_khz <khz>:\n"
    //  " Set the system clock system clock frequency in khz."},
    // {"measure_freqs", run_measure_freqs,
    //  "measure_freqs:\n"
    //  " Count the RP2040 clock frequencies and report."},
    // {"clr", clr, "clr <gpio #>: clear a GPIO"},
    // {"set", set, "set <gpio #>: set a GPIO"},
    // {"test", run_test, "test:\n"
    //  " Development test"},
    {"help", run_help,
     "help:\n"
     " Shows this command help."}
};
static void run_help(const size_t argc, const char *argv[]) {
    if (!expect_argc(argc, argv, 0)) return;

    for (size_t i = 0; i < count_of(cmds); ++i) {
        printf("%s\n\n", cmds[i].help);
    }
    fflush(stdout);
    stdio_flush();
}

// Break command
static void chars_available_callback(void *ptr) {
    (void)ptr;   
    int cRxedChar = getchar_timeout_us(0);
    switch (cRxedChar) {
    case 3: // Ctrl-C
        SYSTEM_RESET();
        break;
    case 27: // Esc
        __BKPT();
        break;
    case '\r':
        die = true;
    }
}

static void process_cmd(size_t cmd_sz, char *cmd) {
    assert(cmd);
    assert(cmd[0]);
    char *cmdn = strtok_r(cmd, " ", &saveptr);
    if (cmdn) {
        assert(cmdn < cmd + cmd_sz);

        /* Breaking with Unix tradition of argv[0] being command name,
        argv[0] is first argument after command name */

        size_t argc = 0;
        const char *argv[10] = {0}; // Arbitrary limit of 10 arguments
        const char *arg_p;
        do {
            arg_p = strtok_r(NULL, " ", &saveptr);
            if (arg_p) {
                assert(arg_p < cmd + cmd_sz);
                if (argc >= count_of(argv)) {
                    extra_argument_msg(arg_p);
                    return;
                }
                argv[argc++] = arg_p;
            }
        } while (arg_p);

        size_t i;
        for (i = 0; i < count_of(cmds); ++i) {
            if (0 == strcmp(cmds[i].command, cmdn)) {
                
                // get notified when there are input characters available
                stdio_set_chars_available_callback(chars_available_callback, NULL);
                // run the command
                (*cmds[i].function)(argc, argv);
                stdio_set_chars_available_callback(NULL, NULL);

                break;
            }
        }
        if (count_of(cmds) == i) printf("Command \"%s\" not found\n", cmdn);
    }
}

void process_stdio(int cRxedChar) {
    if (!(0 < cRxedChar &&  cRxedChar <= 0x7F))
        return; // Not dealing with multibyte characters
    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != 127)
        return;
    if (cRxedChar == '\n' && last_input_was_cr) {
        last_input_was_cr = false;
        return;
    }
    if (cRxedChar == '\r' || cRxedChar == '\n') {
        last_input_was_cr = (cRxedChar == '\r');
        printf("%c", cRxedChar);  // echo Enter
        stdio_flush();
        /* Just to space the output from the input. */
        printf("%c", '\n');
        stdio_flush();

        if (!cmd_buffer[0]) {  // Empty input
            print_prompt();
            return;
        }

        /* Process the input string received prior to the newline. */
        process_cmd(sizeof cmd_buffer, cmd_buffer);

        /* Reset everything for next cmd */
        cmd_ix = 0;
        memset(cmd_buffer, 0, sizeof cmd_buffer);
        printf("\n");
        print_prompt();
    } else {  // Not newline
        last_input_was_cr = false;
        if (cRxedChar == '\b' || cRxedChar == (char)127) {
            /* Backspace was pressed.  Erase the last character
             in the string - if any. */
            if (cmd_ix > 0) {
                cmd_ix--;
                cmd_buffer[cmd_ix] = '\0';
                printf("\b \b");
                stdio_flush();
            }
        } else {
            /* A character was entered.  Add it to the string
             entered so far.  When a \n is entered the complete
             string will be passed to the command interpreter. */
            if (cmd_ix < sizeof cmd_buffer - 1) {
                cmd_buffer[cmd_ix] = cRxedChar;
                cmd_ix++;
                printf("%c", cRxedChar);  // echo
                stdio_flush();
            }
        }
    }
}

bool command_input_in_progress(void) {
    return cmd_ix > 0;
}

void process_background_tasks(void) {
    if (!cam_snap_pending) {
        return;
    }

    if (!time_reached(cam_snap_next_step_time)) {
        return;
    }

    switch (cam_snap_state) {
    case CAM_SNAP_WAIT_FRAME:
        if (cam_wait_for_frame(0)) {
            cam_snap_state = CAM_SNAP_OPEN_FILE;
            cam_snap_next_step_time = delayed_by_ms(get_absolute_time(), 1);
            return;
        }
        if (time_reached(cam_snap_deadline)) {
            finish_cam_snap();
            printf("\nCamera capture timed out after 2000 ms\n");
            print_prompt();
        }
        return;

    case CAM_SNAP_OPEN_FILE: {
        FRESULT fr = f_open(&cam_snap_file, cam_snap_path, FA_WRITE | FA_CREATE_ALWAYS);
        if (FR_OK != fr) {
            printf("\nf_open error: %s (%d)\n", FRESULT_str(fr), fr);
            finish_cam_snap();
            print_prompt();
            return;
        }
        cam_snap_file_open = true;
        cam_snap_state = CAM_SNAP_WRITE_FILE;
        cam_snap_next_step_time = delayed_by_ms(get_absolute_time(), 1);
        return;
    }

    case CAM_SNAP_WRITE_FILE: {
        uint32_t total_bytes = cam_ful_size * 2;
        if (cam_snap_write_offset >= total_bytes) {
            cam_snap_state = CAM_SNAP_CLOSE_FILE;
            return;
        }

        UINT chunk = cam_snap_chunk_size;
        uint32_t remaining = total_bytes - cam_snap_write_offset;
        if (remaining < chunk) {
            chunk = (UINT)remaining;
        }

        UINT bw = 0;
        FRESULT fr = f_write(&cam_snap_file, cam_ptr + cam_snap_write_offset, chunk, &bw);
        if (FR_OK != fr || bw != chunk) {
            if (FR_OK != fr) {
                printf("\nf_write error: %s (%d)\n", FRESULT_str(fr), fr);
            } else {
                printf("\nShort write: %u of %u bytes\n", bw, chunk);
            }
            finish_cam_snap();
            print_prompt();
            return;
        }

        cam_snap_write_offset += bw;
        cam_snap_total_written += bw;
        cam_snap_next_step_time = delayed_by_ms(get_absolute_time(), 1);
        return;
    }

    case CAM_SNAP_CLOSE_FILE: {
        char completed_path[sizeof(cam_snap_path)];
        strlcpy(completed_path, cam_snap_path, sizeof(completed_path));
        UINT completed_written = cam_snap_total_written;
        finish_cam_snap();
        printf("\nWrote %u bytes to %s\n", completed_written, completed_path);
        print_prompt();
        return;
    }

    case CAM_SNAP_IDLE:
    default:
        return;
    }
}
