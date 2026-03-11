#ifndef TFLM_YOLOV8_OD_SDLOG_I2C_CMD_H_
#define TFLM_YOLOV8_OD_SDLOG_I2C_CMD_H_

#include <stdint.h>

/* Customer feature / command codes */
#define I2C_FEATURE_RECORDER    0x80
#define I2C_CMD_RECORD_START    0x01

/**
 * g_recording_active — set to 1 by I2C handler when the host sends
 * feature=0x80, cmd=0x01.  Checked every frame in the datapath callback.
 */
extern volatile uint8_t g_recording_active;

/**
 * g_detect_threshold — detection confidence threshold for DETECT/ saves.
 * Defaults to DETECT_CONF_THRESHOLD; can be overridden by I2C payload.
 */
extern volatile float g_detect_threshold;

/**
 * i2c_cmd_init() — arm the I2C slave listener.
 * Must be called after sdlog_session_init() but before entering the event loop.
 */
void i2c_cmd_init(void);

#endif /* TFLM_YOLOV8_OD_SDLOG_I2C_CMD_H_ */
