#include <stdint.h>
#include <string.h>
#include "xprintf.h"
#include "i2c_comm.h"
#include "evt_i2ccomm.h"
#include "i2c_cmd.h"
#include "sdlog.h"
#include "common_config.h"

/* -----------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------- */
volatile uint8_t g_recording_active  = 0;
volatile float   g_detect_threshold  = DETECT_CONF_THRESHOLD;

/* -----------------------------------------------------------------------
 * i2c_customer_handler — called by evt_i2ccomm when feature is in
 * I2CCOMM_FEATURE_CUSTOMER_MIN..MAX range. Receives a pointer to the
 * frame bytes (a FIFO slot owned by evt_i2ccomm). Must NOT re-arm the
 * I2C HW — that is now done in ISR context immediately after enqueue.
 * -------------------------------------------------------------------- */
static void i2c_customer_handler(const uint8_t *frame)
{
    int retval;
    unsigned char feature = frame[I2CFMT_FEATURE_OFFSET];
    unsigned char cmd     = frame[I2CFMT_COMMAND_OFFSET];

#ifdef I2C_CMD_VERBOSE_DEBUG
    /* Dump raw received bytes for debugging. Off by default — at
     * 30 Hz × 4-batch the drain loop bursts up to 16 frames at once,
     * and ~50 chars/frame of UART output at 115200 baud blocks the
     * scenario loop long enough to starve JPEG recording. */
    {
        uint16_t plen = ((uint16_t)frame[I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                      |  (uint16_t)frame[I2CFMT_PAYLOADLEN_LSB_OFFSET];
        uint16_t dump_len = 4 + plen + 2;
        if (dump_len > 16) dump_len = 16;
        xprintf("[I2C_CMD] raw %u bytes:", dump_len);
        for (uint16_t i = 0; i < dump_len; i++)
            xprintf(" %02x", frame[i]);
        xprintf("\r\n");
    }
#endif

    /* Validate checksum (silent on common-case failure to keep the
     * drain hot path short). */
    retval = hx_lib_i2ccomm_validate_checksum((unsigned char *)frame);
    if (retval != I2CCOMM_NO_ERROR) {
#ifdef I2C_CMD_VERBOSE_DEBUG
        xprintf("[I2C_CMD] checksum error (retval=%d), skipping check for debug\r\n", retval);
#endif
        /* Continue anyway for debugging — remove this bypass once CRC is fixed */
    }

    if (feature == I2C_FEATURE_RECORDER && cmd == I2C_CMD_RECORD_START) {
        /* Read optional threshold override from payload byte 0 */
        uint16_t payload_len = ((uint16_t)frame[I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                             |  (uint16_t)frame[I2CFMT_PAYLOADLEN_LSB_OFFSET];

        if (payload_len >= 1) {
            uint8_t raw = frame[I2CFMT_PAYLOAD_OFFSET];
            if (raw <= 100) {
                g_detect_threshold = (float)raw / 100.0f;
            }
        }

        g_recording_active = 1;
        xprintf("\r\n****************************************************\r\n");
        xprintf("****************************************************\r\n");
        xprintf("***  [I2C_CMD] Recording started (thresh=%.2f)  ***\r\n", g_detect_threshold);
        xprintf("****************************************************\r\n");
        xprintf("****************************************************\r\n");
        sdlog_write("[I2C] Recording started (threshold=%.2f)\r\n", g_detect_threshold);
    } else if (feature == I2C_FEATURE_LOG && cmd == I2C_CMD_LOG_WRITE) {
        uint16_t plen = ((uint16_t)frame[I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                      |  (uint16_t)frame[I2CFMT_PAYLOADLEN_LSB_OFFSET];

        if (plen < 2 || plen > I2CCOMM_MAX_PAYLOAD_SIZE) {
            xprintf("[I2C_LOG] bad plen=%u\r\n", (unsigned)plen);
        } else {
            const char *buf = (const char *)&frame[I2CFMT_PAYLOAD_OFFSET];
            size_t tag_len = strnlen(buf, 16);
            if (tag_len >= 16 || (tag_len + 1) >= plen) {
                xprintf("[I2C_LOG] malformed payload (tag_len=%u, plen=%u)\r\n",
                        (unsigned)tag_len, (unsigned)plen);
            } else {
                const char *tag = buf;
                const char *msg = buf + tag_len + 1;
                size_t msg_max = (size_t)plen - (tag_len + 1);
                size_t msg_len = strnlen(msg, msg_max);
                if (msg_len >= msg_max) {
                    xprintf("[I2C_LOG] msg not NUL-terminated (msg_max=%u)\r\n",
                            (unsigned)msg_max);
                } else {
                    /* Defer SD write to main loop — SPSC ring enqueue is safe
                     * from this (highest-priority) event context. Calling
                     * f_write here would race the scenario-loop JPEG saves on
                     * the shared FatFs/SPI bus. */
                    sdlog_log_enqueue(tag, msg);
#ifdef I2C_CMD_VERBOSE_DEBUG
                    xprintf("[I2C_LOG] [%s] %s\r\n", tag, msg);
#endif
                }
            }
        }
    } else if (feature == I2C_FEATURE_TLM && cmd == I2C_CMD_TLM_WRITE) {
        uint16_t plen = ((uint16_t)frame[I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                      |  (uint16_t)frame[I2CFMT_PAYLOADLEN_LSB_OFFSET];

        if (plen == 0 || plen > I2CCOMM_MAX_PAYLOAD_SIZE
            || (plen % TLM_SAMPLE_BYTES) != 0) {
            xprintf("[I2C_TLM] bad plen=%u\r\n", (unsigned)plen);
        } else {
            const uint8_t *payload = &frame[I2CFMT_PAYLOAD_OFFSET];
            sdlog_tlm_enqueue(payload, plen);
#ifdef I2C_CMD_VERBOSE_DEBUG
            xprintf("[I2C_TLM] enqueued %u bytes (%u samples)\r\n",
                    (unsigned)plen, (unsigned)(plen / TLM_SAMPLE_BYTES));
#endif
        }
    } else {
        xprintf("[I2C_CMD] Unknown customer cmd: feature=0x%02x cmd=0x%02x\r\n", feature, cmd);
    }

    /* No re-arm here — evt_i2ccomm.c re-arms HW from the ISR after
     * enqueueing into the FIFO. Re-arming here would race the next
     * iteration's drain. */
}

/* -----------------------------------------------------------------------
 * i2c_cmd_init
 *
 * NOTE: evt_i2ccomm_init() is called by event_handler_init() (inside
 * app_start_state()).  Here we only register the customer callback so
 * that it is in place when I2C events start firing.
 * -------------------------------------------------------------------- */
void i2c_cmd_init(void)
{
    i2ccomm_cmd_customer_register_cb(USE_DW_IIC_SLV_0, i2c_customer_handler);
    xprintf("\r\n****************************************************\r\n");
    xprintf("****************************************************\r\n");
    xprintf("***  [I2C_CMD] Handler registered (addr=0x28)   ***\r\n");
    xprintf("****************************************************\r\n");
    xprintf("****************************************************\r\n");
}
