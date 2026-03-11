#ifndef SCENARIO_TFLM_YOLOV8_OD_SDLOG_H_
#define SCENARIO_TFLM_YOLOV8_OD_SDLOG_H_

#define APP_BLOCK_FUNC() do{ \
    __asm volatile("b    .");\
    }while(0)

typedef enum
{
    APP_STATE_ALLON_YOLOV8N_OB,
} APP_STATE_E;

int tflm_yolov8_od_sdlog_app(void);
void model_change(void);
void SetPSPDNoVid_24M(void);
void SetPSPDNoVid(void);

#endif /* SCENARIO_TFLM_YOLOV8_OD_SDLOG_H_ */
