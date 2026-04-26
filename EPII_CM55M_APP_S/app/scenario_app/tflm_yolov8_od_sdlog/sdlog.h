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
 * sdlog_save_detect_txt() — write a text annotation file to SESSION_XXXX/DETECT/<fname>.
 * Used to save YOLO bounding box results alongside detected images.
 */
void sdlog_save_detect_txt(const char *fname, const char *content);

/**
 * sdlog_write() — printf-style append to SESSION_XXXX/session.log.
 * Flushes (f_sync) after every write so entries survive a power loss.
 */
void sdlog_write(const char *fmt, ...);

/**
 * sdlog_log_enqueue() — SPSC ring enqueue for log entries arriving from the
 * i2ccomm RX event context (can preempt the main scenario loop). Only copies
 * into a RAM ring; no SD / SPI / FatFs access here.
 */
void sdlog_log_enqueue(const char *tag, const char *msg);

/**
 * sdlog_log_drain() — called from the main scenario loop. Pops every pending
 * entry from the ring and writes it to session.log. Must NOT be called from
 * interrupt / event-callback context.
 */
void sdlog_log_drain(void);

/**
 * sdlog_tlm_init() — open SESSION_XXXX/telemetry.csv (or top-level
 * telemetry.csv depending on convention) and write the CSV header row.
 * Must be called after sdlog_session_init() so the SD card and session
 * directory already exist.
 */
void sdlog_tlm_init(void);

/**
 * sdlog_tlm_enqueue() — SPSC ring enqueue called from the i2ccomm RX
 * event context. Copies one I2C frame's payload (TLM_BATCH_SIZE *
 * TLM_SAMPLE_BYTES bytes) into a RAM ring; no SD / SPI / FatFs access.
 */
void sdlog_tlm_enqueue(const uint8_t *payload, uint16_t plen);

/**
 * sdlog_tlm_drain() — called from the main scenario loop. Pops every
 * pending frame, unpacks each 44-byte sample, stamps a Himax-side
 * timestamp, and writes one CSV row per sample. f_syncs every 60
 * samples (~once per second at 60 Hz). Must NOT be called from
 * interrupt / event-callback context.
 */
void sdlog_tlm_drain(void);

#endif /* TFLM_YOLOV8_OD_SDLOG_SDLOG_H_ */
