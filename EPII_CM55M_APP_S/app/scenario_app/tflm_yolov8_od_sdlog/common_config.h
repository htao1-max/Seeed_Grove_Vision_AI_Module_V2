#ifndef SCENARIO_TFLM_YOLOV8_OD_SDLOG_COMMON_CONFIG_H_
#define SCENARIO_TFLM_YOLOV8_OD_SDLOG_COMMON_CONFIG_H_

#define FRAME_CHECK_DEBUG   1
#define EN_ALGO             1
#define SPI_SEN_PIC_CLK     (12000000)

#define DBG_APP_LOG 0

/* Minimum confidence to save a copy in DETECT/ folder */
#define DETECT_CONF_THRESHOLD   0.50f

//current FW image is 409600 bytes => 0x64000. set  0~0x171000 as FW area
#define FW_IMG_SZ                           0x3A171000

//0x3AB7B000
#define YOLOV8_OBJECT_DETECTION_FLASH_ADDR  0x3AB7B000

#endif /* SCENARIO_TFLM_YOLOV8_OD_SDLOG_COMMON_CONFIG_H_ */
