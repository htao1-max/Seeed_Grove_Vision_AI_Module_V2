#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "xprintf.h"
#include "ff.h"
#include "hx_drv_gpio.h"
#include "hx_drv_scu.h"
#include "system_WE2_ARMCM55.h"
#include "sdlog.h"
#include "i2c_cmd.h"   /* TLM_SAMPLE_BYTES */

/* -----------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------- */
static FATFS  g_fs;
static char   g_session_dir[16];   /* e.g. "SESSION_0003" */
static FIL    g_log_fil;
static int    g_sdlog_ready = 0;

/* Telemetry CSV state */
static FIL      g_tlm_fil;
static int      g_tlm_ready    = 0;
static uint32_t g_tlm_sync_ctr = 0;

/* ms since boot, derived from the bare-metal SysTick countdown + overflow
 * counter (SystemGetTick) and SystemCoreClock. */
static uint32_t sdlog_now_ms(void)
{
    uint32_t tick, loops;
    SystemGetTick(&tick, &loops);
    const uint64_t period = (uint64_t)SysTick_LOAD_RELOAD_Msk + 1ULL;
    uint64_t cycles = (uint64_t)loops * period + (period - (uint64_t)tick);
    uint32_t cyc_per_ms = SystemCoreClock / 1000U;
    if (cyc_per_ms == 0U) return 0U;
    return (uint32_t)(cycles / cyc_per_ms);
}

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

    if (sz == 0 || addr == 0) {
        xprintf("[SDLOG] skip %s (addr=0x%x sz=%u)\r\n", path, addr, sz);
        return;
    }

    SCB_CleanInvalidateDCache_by_Addr((void *)addr, sz);

    res = f_open(&fil, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res == FR_OK) {
        res = f_write(&fil, (void *)addr, sz, &bw);
        if (res != FR_OK) {
            xprintf("[SDLOG] f_write(%s) res=%d bw=%u\r\n", path, res, bw);
        }
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
    xprintf("\r\n****************************************************\r\n");
    xprintf("****************************************************\r\n");
    xprintf("***       [SDLOG] SD card mounted                ***\r\n");
    xprintf("****************************************************\r\n");
    xprintf("****************************************************\r\n");

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
    xprintf("\r\n****************************************************\r\n");
    xprintf("****************************************************\r\n");
    xprintf("***       [SDLOG] Session %s initialised    ***\r\n", g_session_dir);
    xprintf("****************************************************\r\n");
    xprintf("****************************************************\r\n");
}

/* -----------------------------------------------------------------------
 * sdlog_tlm_init — open telemetry.csv inside the active session dir and
 * write the CSV header row. Must run after sdlog_session_init.
 * -------------------------------------------------------------------- */
void sdlog_tlm_init(void)
{
    if (!g_sdlog_ready) {
        xprintf("[SDLOG_TLM] sdlog not ready — skipping telemetry.csv\r\n");
        return;
    }

    char path[80];
    xsprintf(path, "%s/telemetry.csv", g_session_dir);

    FRESULT res = f_open(&g_tlm_fil, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        xprintf("[SDLOG_TLM] f_open(%s) res=%d — telemetry disabled\r\n", path, res);
        return;
    }

    static const char header[] =
        "stm32_tick_ms,qw,qx,qy,qz,temp_c,vbat,"
        "vm1,vm2,vm3,vm4,im1,im2,im3,im4,depth,himax_recv_ms\r\n";
    UINT bw;
    res = f_write(&g_tlm_fil, header, (UINT)(sizeof(header) - 1), &bw);
    if (res != FR_OK) {
        xprintf("[SDLOG_TLM] header f_write res=%d — telemetry disabled\r\n", res);
        f_close(&g_tlm_fil);
        return;
    }
    f_sync(&g_tlm_fil);

    g_tlm_ready = 1;
    g_tlm_sync_ctr = 0;
    xprintf("[SDLOG_TLM] telemetry.csv ready in %s\r\n", g_session_dir);
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
 * sdlog_save_detect_txt  (write annotation text file to DETECT/)
 * -------------------------------------------------------------------- */
void sdlog_save_detect_txt(const char *fname, const char *content)
{
    if (!g_sdlog_ready) return;

    char path[80];
    FIL fil;
    UINT bw;

    xsprintf(path, "%s/DETECT/%s", g_session_dir, fname);
    FRESULT res = f_open(&fil, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res == FR_OK) {
        f_write(&fil, content, strlen(content), &bw);
        f_close(&fil);
    } else {
        xprintf("[SDLOG] f_open(%s) res=%d\r\n", path, res);
    }
}

/* -----------------------------------------------------------------------
 * sdlog_write  (printf-style, flushed after each call)
 * -------------------------------------------------------------------- */
void sdlog_write(const char *fmt, ...)
{
    if (!g_sdlog_ready) return;

    char buf[280];
    int pfx = snprintf(buf, sizeof(buf), "[%010lu ms] ",
                       (unsigned long)sdlog_now_ms());
    if (pfx < 0) pfx = 0;

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf + pfx, sizeof(buf) - (size_t)pfx, fmt, args);
    va_end(args);

    UINT bw;
    f_write(&g_log_fil, buf, strlen(buf), &bw);
    f_sync(&g_log_fil);
}

/* -----------------------------------------------------------------------
 * Deferred-log SPSC ring
 *
 * Producer: i2c_customer_handler (i2ccomm RX event, highest priority — can
 *           preempt the main scenario loop). Only memcpy + head advance.
 * Consumer: sdlog_log_drain(), called once per frame from the scenario loop.
 *
 * Word-sized volatile head/tail indices are atomic on Cortex-M55, so no
 * locking is needed for a single-producer / single-consumer ring.
 * -------------------------------------------------------------------- */
#define SDLOG_RING_SLOTS 8
#define SDLOG_RING_MSG   208  /* msg capacity per slot */
#define SDLOG_RING_TAG   16

typedef struct {
    uint32_t ts_ms;                     /* ms since boot, captured at enqueue */
    char     tag[SDLOG_RING_TAG];
    char     msg[SDLOG_RING_MSG];
} sdlog_ring_entry_t;

static sdlog_ring_entry_t g_ring[SDLOG_RING_SLOTS];
static volatile uint32_t  g_ring_head = 0;   /* producer writes */
static volatile uint32_t  g_ring_tail = 0;   /* consumer reads */
static volatile uint32_t  g_ring_dropped = 0;

void sdlog_log_enqueue(const char *tag, const char *msg)
{
    if (tag == NULL || msg == NULL) return;

    uint32_t head = g_ring_head;
    uint32_t tail = g_ring_tail;
    if ((head - tail) >= SDLOG_RING_SLOTS) {
        g_ring_dropped++;
        return;
    }

    sdlog_ring_entry_t *e = &g_ring[head % SDLOG_RING_SLOTS];

    e->ts_ms = sdlog_now_ms();

    size_t tl = strnlen(tag, SDLOG_RING_TAG - 1);
    memcpy(e->tag, tag, tl);
    e->tag[tl] = '\0';

    size_t ml = strnlen(msg, SDLOG_RING_MSG - 1);
    memcpy(e->msg, msg, ml);
    e->msg[ml] = '\0';

    __DMB();
    g_ring_head = head + 1;
}

void sdlog_log_drain(void)
{
    if (!g_sdlog_ready) return;

    while (g_ring_tail != g_ring_head) {
        sdlog_ring_entry_t *e = &g_ring[g_ring_tail % SDLOG_RING_SLOTS];

        char line[SDLOG_RING_TAG + SDLOG_RING_MSG + 32];
        xsprintf(line, "[%010lu ms] [%s] %s\r\n",
                 (unsigned long)e->ts_ms, e->tag, e->msg);
        UINT bw;
        f_write(&g_log_fil, line, (UINT)strlen(line), &bw);

        __DMB();
        g_ring_tail++;
    }

    f_sync(&g_log_fil);

    if (g_ring_dropped) {
        uint32_t d = g_ring_dropped;
        g_ring_dropped = 0;
        char warn[96];
        xsprintf(warn, "[%010lu ms] [SDLOG] dropped %lu log entries (ring full)\r\n",
                 (unsigned long)sdlog_now_ms(), (unsigned long)d);
        UINT bw;
        f_write(&g_log_fil, warn, (UINT)strlen(warn), &bw);
        f_sync(&g_log_fil);
    }
}

/* -----------------------------------------------------------------------
 * Telemetry SPSC ring + drain
 *
 * Producer: i2c_customer_handler (i2ccomm RX event). Only memcpy +
 *           head advance.
 * Consumer: sdlog_tlm_drain(), called once per scenario loop iteration.
 * -------------------------------------------------------------------- */
#define SDLOG_TLM_RING_SLOTS  4
#define SDLOG_TLM_BLOB_MAX    (4U * 64U)  /* TLM_BATCH_SIZE * TLM_SAMPLE_BYTES */

typedef struct {
    uint16_t plen;
    uint8_t  blob[SDLOG_TLM_BLOB_MAX];
} sdlog_tlm_entry_t;

static sdlog_tlm_entry_t g_tlm_ring[SDLOG_TLM_RING_SLOTS];
static volatile uint32_t g_tlm_head = 0;   /* producer */
static volatile uint32_t g_tlm_tail = 0;   /* consumer */
static volatile uint32_t g_tlm_dropped = 0;

void sdlog_tlm_enqueue(const uint8_t *payload, uint16_t plen)
{
    if (payload == NULL) return;
    if (plen == 0 || plen > SDLOG_TLM_BLOB_MAX || (plen % TLM_SAMPLE_BYTES) != 0) {
        return;  /* drop malformed; logged at dispatcher */
    }

    uint32_t head = g_tlm_head;
    uint32_t tail = g_tlm_tail;
    if ((head - tail) >= SDLOG_TLM_RING_SLOTS) {
        g_tlm_dropped++;
        return;
    }

    sdlog_tlm_entry_t *e = &g_tlm_ring[head % SDLOG_TLM_RING_SLOTS];
    e->plen = plen;
    memcpy(e->blob, payload, plen);

    __DMB();
    g_tlm_head = head + 1;
}

/* Read 4 little-endian bytes as float (no host-order assumption). */
static float tlm_unpack_f32_le(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0]
               | ((uint32_t)p[1] << 8)
               | ((uint32_t)p[2] << 16)
               | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static uint32_t tlm_unpack_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* Format a float as ",<sign><whole>.<6 fractional digits>" into buf.
 * Used because xprintf/snprintf in this firmware do NOT support %f
 * (the EI porting layer's ei_printf_float helper exists for this same
 * reason). Returns chars written (excluding NUL); never writes more
 * than (n - 1) bytes. Saturates to ±999999 to keep the helper simple
 * — telemetry values (volts, amps, deg C, quaternion, depth in m) are
 * comfortably below that bound. */
static int tlm_csv_append_f6(char *buf, int n, float f)
{
    if (n < 16) return 0;

    int neg = 0;
    if (f < 0.0f) { neg = 1; f = -f; }
    if (f > 999999.0f) f = 999999.0f;
    if (f != f /* NaN */) f = 0.0f;

    uint32_t whole = (uint32_t)f;
    uint32_t frac  = (uint32_t)((f - (float)whole) * 1000000.0f + 0.5f);
    if (frac >= 1000000U) { frac = 0; whole++; }

    return snprintf(buf, (size_t)n, ",%s%lu.%06lu",
                    neg ? "-" : "",
                    (unsigned long)whole,
                    (unsigned long)frac);
}

void sdlog_tlm_drain(void)
{
    if (!g_tlm_ready) return;

    int wrote_any = 0;

    while (g_tlm_tail != g_tlm_head) {
        wrote_any = 1;
        sdlog_tlm_entry_t *e = &g_tlm_ring[g_tlm_tail % SDLOG_TLM_RING_SLOTS];

        for (uint16_t off = 0; off < e->plen; off += TLM_SAMPLE_BYTES) {
            const uint8_t *p = &e->blob[off];
            float qw    = tlm_unpack_f32_le(p +  0);
            float qx    = tlm_unpack_f32_le(p +  4);
            float qy    = tlm_unpack_f32_le(p +  8);
            float qz    = tlm_unpack_f32_le(p + 12);
            float temp  = tlm_unpack_f32_le(p + 16);
            float vbat  = tlm_unpack_f32_le(p + 20);
            float vm1   = tlm_unpack_f32_le(p + 24);
            float vm2   = tlm_unpack_f32_le(p + 28);
            float vm3   = tlm_unpack_f32_le(p + 32);
            float vm4   = tlm_unpack_f32_le(p + 36);
            float im1   = tlm_unpack_f32_le(p + 40);
            float im2   = tlm_unpack_f32_le(p + 44);
            float im3   = tlm_unpack_f32_le(p + 48);
            float im4   = tlm_unpack_f32_le(p + 52);
            float depth = tlm_unpack_f32_le(p + 56);
            uint32_t stm32_tick = tlm_unpack_u32_le(p + 60);

            uint32_t himax_recv_ms = sdlog_now_ms();

            /* Build the CSV row by hand: integer fields use snprintf
             * (%lu works); float fields go through tlm_csv_append_f6
             * because the firmware's printf has no %f support. */
            char line[300];
            int pos = 0;
            int rem = (int)sizeof(line);

            int n = snprintf(line + pos, (size_t)rem, "%lu",
                             (unsigned long)stm32_tick);
            if (n < 0 || n >= rem) continue;
            pos += n; rem -= n;

            const float fields[] = {
                qw, qx, qy, qz, temp, vbat,
                vm1, vm2, vm3, vm4, im1, im2, im3, im4, depth
            };
            for (size_t i = 0; i < sizeof(fields)/sizeof(fields[0]); i++) {
                n = tlm_csv_append_f6(line + pos, rem, fields[i]);
                if (n <= 0 || n >= rem) { pos = -1; break; }
                pos += n; rem -= n;
            }
            if (pos < 0) continue;

            n = snprintf(line + pos, (size_t)rem, ",%lu\r\n",
                         (unsigned long)himax_recv_ms);
            if (n < 0 || n >= rem) continue;
            pos += n;

            UINT bw;
            f_write(&g_tlm_fil, line, (UINT)pos, &bw);

            if (++g_tlm_sync_ctr >= 60U) {
                f_sync(&g_tlm_fil);
                g_tlm_sync_ctr = 0;
            }
        }

        __DMB();
        g_tlm_tail++;
    }

    /* Trailing sync: when we drained the ring empty, flush any residue
     * that the sync-every-60 counter didn't cover. At sustained 60 Hz
     * the ring rarely empties (drains pop 1 frame, more arrive), so
     * the every-60 counter still dominates. At low rates / end-of-run
     * this guarantees data hits the SD before idle. */
    if (wrote_any && g_tlm_tail == g_tlm_head) {
        f_sync(&g_tlm_fil);
        g_tlm_sync_ctr = 0;
    }

    if (g_tlm_dropped) {
        uint32_t d = g_tlm_dropped;
        g_tlm_dropped = 0;
        xprintf("[SDLOG_TLM] dropped %lu telemetry frames (ring full)\r\n",
                (unsigned long)d);
    }
}
