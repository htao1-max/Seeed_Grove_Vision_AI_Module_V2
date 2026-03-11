#ifndef TFLM_YOLOV8_OD_SDLOG_SDLOG_H_
#define TFLM_YOLOV8_OD_SDLOG_SDLOG_H_

#include <stdint.h>

/**
 * sdlog_session_init() — call once at boot.
 * Mounts the SD card, scans for the next available SESSION_XXXX folder,
 * creates SESSION_XXXX/ALL/ and SESSION_XXXX/DETECT/, then opens
 * SESSION_XXXX/session.log for writing.
 */
void sdlog_session_init(void);

/**
 * sdlog_save_all() — write JPEG from SRAM to SESSION_XXXX/ALL/<fname>.
 */
void sdlog_save_all(uint32_t addr, uint32_t sz, const char *fname);

/**
 * sdlog_save_detect() — write JPEG from SRAM to SESSION_XXXX/DETECT/<fname>.
 */
void sdlog_save_detect(uint32_t addr, uint32_t sz, const char *fname);

/**
 * sdlog_write() — printf-style append to SESSION_XXXX/session.log.
 * Flushes (f_sync) after every write so entries survive a power loss.
 */
void sdlog_write(const char *fmt, ...);

#endif /* TFLM_YOLOV8_OD_SDLOG_SDLOG_H_ */
