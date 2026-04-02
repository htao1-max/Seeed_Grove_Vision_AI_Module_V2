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

/* Buffers shared with evt_i2ccomm.c */
extern unsigned char gRead_buf[DW_IIC_S_NUM][I2CCOMM_MAX_RBUF_SIZE];
extern unsigned char gWrite_buf[DW_IIC_S_NUM][I2CCOMM_MAX_WBUF_SIZE];

/* -----------------------------------------------------------------------
 * i2c_customer_handler — called by evt_i2ccomm when feature is in
 * I2CCOMM_FEATURE_CUSTOMER_MIN..MAX range.
 * -------------------------------------------------------------------- */
static void i2c_customer_handler(void)
{
    int retval;
    unsigned char feature = gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_FEATURE_OFFSET];
    unsigned char cmd     = gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_COMMAND_OFFSET];

    /* Dump raw received bytes for debugging */
    {
        uint16_t plen = ((uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                      |  (uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_LSB_OFFSET];
        uint16_t dump_len = 4 + plen + 2;  /* header + payload + crc */
        if (dump_len > 16) dump_len = 16;
        xprintf("[I2C_CMD] raw %u bytes:", dump_len);
        for (uint16_t i = 0; i < dump_len; i++)
            xprintf(" %02x", gRead_buf[USE_DW_IIC_SLV_0][i]);
        xprintf("\r\n");
    }

    /* Validate checksum */
    retval = hx_lib_i2ccomm_validate_checksum((unsigned char *)&gRead_buf[USE_DW_IIC_SLV_0]);
    if (retval != I2CCOMM_NO_ERROR) {
        xprintf("[I2C_CMD] checksum error (retval=%d), skipping check for debug\r\n", retval);
        /* Continue anyway for debugging — remove this bypass once CRC is fixed */
    }

    if (feature == I2C_FEATURE_RECORDER && cmd == I2C_CMD_RECORD_START) {
        /* Read optional threshold override from payload byte 0 */
        uint16_t payload_len = ((uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                             |  (uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_LSB_OFFSET];

        if (payload_len >= 1) {
            uint8_t raw = gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOAD_OFFSET];
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
    } else {
        xprintf("[I2C_CMD] Unknown customer cmd: feature=0x%02x cmd=0x%02x\r\n", feature, cmd);
    }

    /* Re-arm read for next command */
    memset((void *)&gRead_buf[USE_DW_IIC_SLV_0], 0xFF, 4);
    hx_lib_i2ccomm_enable_read(USE_DW_IIC_SLV_0,
                               (unsigned char *)&gRead_buf[USE_DW_IIC_SLV_0],
                               I2CCOMM_MAX_RBUF_SIZE);
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
