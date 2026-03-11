#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "xprintf.h"
#include "ff.h"
#include "hx_drv_gpio.h"
#include "hx_drv_scu.h"
#include "sdlog.h"

/* -----------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------- */
static FATFS  g_fs;
static char   g_session_dir[16];   /* e.g. "SESSION_0003" */
static FIL    g_log_fil;
static int    g_sdlog_ready = 0;

/* GPIO callbacks required by fatfs/port/mmc_spi/mmc_we2_spi.c */
void SSPI_CS_GPIO_Output_Level(bool setLevelHigh)
{
    hx_drv_gpio_set_out_value(GPIO16, (GPIO_OUT_LEVEL_E)setLevelHigh);
}

void SSPI_CS_GPIO_Pinmux(bool setGpioFn)
{
    if (setGpioFn)
        hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_GPIO16, 0);
    else
        hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_SPI_M_CS_1, 0);
}

void SSPI_CS_GPIO_Dir(bool setDirOut)
{
    if (setDirOut)
        hx_drv_gpio_set_output(GPIO16, GPIO_OUT_HIGH);
    else
        hx_drv_gpio_set_input(GPIO16);
}

/* Write a JPEG from SRAM to the specified full path on the SD card */
static void sdlog_write_image(uint32_t addr, uint32_t sz, const char *path)
{
    FIL fil;
    FRESULT res;
    UINT bw;

    res = f_open(&fil, path, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK) {
        SCB_InvalidateDCache_by_Addr((void *)addr, sz);
        res = f_write(&fil, (void *)addr, sz, &bw);
        if (res) { xprintf("[SDLOG] f_write(%s) res=%d\r\n", path, res); }
        f_close(&fil);
    } else {
        xprintf("[SDLOG] f_open(%s) res=%d\r\n", path, res);
    }
}

/* -----------------------------------------------------------------------
 * sdlog_session_init
 * -------------------------------------------------------------------- */
void sdlog_session_init(void)
{
    FRESULT res;
    FILINFO fno;
    char path[64];
    uint32_t idx = 0;

    /* Configure SPI master pin mux for SD card (PB2=MOSI, PB3=MISO, PB4=CLK, PB5=CS) */
    hx_drv_scu_set_PB2_pinmux(SCU_PB2_PINMUX_SPI_M_DO_1, 1);
    hx_drv_scu_set_PB3_pinmux(SCU_PB3_PINMUX_SPI_M_DI_1, 1);
    hx_drv_scu_set_PB4_pinmux(SCU_PB4_PINMUX_SPI_M_SCLK_1, 1);
    hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_SPI_M_CS_1, 1);

    /* Mount SD card */
    res = f_mount(&g_fs, "", 1);
    if (res != FR_OK) {
        xprintf("[SDLOG] f_mount failed: %d — SD logging disabled\r\n", res);
        return;
    }
    xprintf("[SDLOG] SD card mounted\r\n");

    /* Find the next available SESSION_XXXX directory */
    while (1) {
        xsprintf(g_session_dir, "SESSION_%04lu", idx);
        res = f_stat(g_session_dir, &fno);
        if (res == FR_OK) {
            idx++;
        } else {
            break;
        }
    }

    /* Create SESSION_XXXX */
    res = f_mkdir(g_session_dir);
    if (res && res != FR_EXIST) {
        xprintf("[SDLOG] f_mkdir(%s) failed: %d\r\n", g_session_dir, res);
        return;
    }

    /* Create SESSION_XXXX/ALL */
    xsprintf(path, "%s/ALL", g_session_dir);
    res = f_mkdir(path);
    if (res && res != FR_EXIST) {
        xprintf("[SDLOG] f_mkdir(%s) failed: %d\r\n", path, res);
        return;
    }

    /* Create SESSION_XXXX/DETECT */
    xsprintf(path, "%s/DETECT", g_session_dir);
    res = f_mkdir(path);
    if (res && res != FR_EXIST) {
        xprintf("[SDLOG] f_mkdir(%s) failed: %d\r\n", path, res);
        return;
    }

    /* Open / create session.log */
    xsprintf(path, "%s/session.log", g_session_dir);
    res = f_open(&g_log_fil, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        xprintf("[SDLOG] f_open(session.log) failed: %d\r\n", res);
        return;
    }

    g_sdlog_ready = 1;
    sdlog_write("[BOOT] Session %s started\r\n", g_session_dir);
    xprintf("[SDLOG] Session %s initialised\r\n", g_session_dir);
}

/* -----------------------------------------------------------------------
 * sdlog_save_all
 * -------------------------------------------------------------------- */
void sdlog_save_all(uint32_t addr, uint32_t sz, const char *fname)
{
    if (!g_sdlog_ready) return;
    char path[80];
    xsprintf(path, "%s/ALL/%s", g_session_dir, fname);
    sdlog_write_image(addr, sz, path);
}

/* -----------------------------------------------------------------------
 * sdlog_save_detect
 * -------------------------------------------------------------------- */
void sdlog_save_detect(uint32_t addr, uint32_t sz, const char *fname)
{
    if (!g_sdlog_ready) return;
    char path[80];
    xsprintf(path, "%s/DETECT/%s", g_session_dir, fname);
    sdlog_write_image(addr, sz, path);
}

/* -----------------------------------------------------------------------
 * sdlog_write  (printf-style, flushed after each call)
 * -------------------------------------------------------------------- */
void sdlog_write(const char *fmt, ...)
{
    if (!g_sdlog_ready) return;

    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    UINT bw;
    f_write(&g_log_fil, buf, strlen(buf), &bw);
    f_sync(&g_log_fil);
}
