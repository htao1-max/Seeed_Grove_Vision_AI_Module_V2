#ifndef SCENARIO_TFLM_YOLOV8_OD_SDLOG_CVAPP_YOLOV8N_OB_H_
#define SCENARIO_TFLM_YOLOV8_OD_SDLOG_CVAPP_YOLOV8N_OB_H_

#include "spi_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

int cv_yolov8n_ob_init(bool security_enable, bool privilege_enable, uint32_t model_addr);
int cv_yolov8n_ob_run(struct_yolov8_ob_algoResult *algoresult_yolov8n_ob);
int cv_yolov8n_ob_deinit();

/* Result accessors for sdlog module */
int  cvapp_get_result_count(void);
void cvapp_get_result(int idx, float *conf, uint16_t *class_idx);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_TFLM_YOLOV8_OD_SDLOG_CVAPP_YOLOV8N_OB_H_ */
