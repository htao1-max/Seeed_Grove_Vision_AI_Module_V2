/*
 * cvapp_yolov8n_ob.cpp — YOLOv8n object detection CV app for tflm_yolov8_od_sdlog.
 * Derived from tflm_yolov8_od/cvapp_yolov8n_ob.cpp with added result export API.
 */

#include <cstdio>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "WE2_device.h"
#include "board.h"
#include "cvapp_yolov8n_ob.h"
#include "cisdp_sensor.h"

#include "WE2_core.h"

#include "ethosu_driver.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
#if TFLM2209_U55TAG2205
#include "tensorflow/lite/micro/micro_error_reporter.h"
#endif
#include "img_proc_helium.h"
#include "yolo_postprocessing.h"

#include "xprintf.h"
#include "spi_master_protocol.h"
#include "cisdp_cfg.h"
#include "memory_manage.h"
#include <send_result.h>

#define CHANGE_YOLOV8_OB_OUPUT_SHAPE 1

#define INPUT_IMAGE_CHANNELS 3

#if 1
#define YOLOV8_OB_INPUT_TENSOR_WIDTH   192
#define YOLOV8_OB_INPUT_TENSOR_HEIGHT  192
#define YOLOV8_OB_INPUT_TENSOR_CHANNEL INPUT_IMAGE_CHANNELS
#else
#define YOLOV8_OB_INPUT_TENSOR_WIDTH   224
#define YOLOV8_OB_INPUT_TENSOR_HEIGHT  224
#define YOLOV8_OB_INPUT_TENSOR_CHANNEL INPUT_IMAGE_CHANNELS
#endif

#define YOLOV8N_OB_DBG_APP_LOG 0

// #define EACH_STEP_TICK
#define TOTAL_STEP_TICK
#define YOLOV8_POST_EACH_STEP_TICK 0
uint32_t systick_1, systick_2;
uint32_t loop_cnt_1, loop_cnt_2;
#define CPU_CLK 0xffffff+1
static uint32_t capture_image_tick = 0;
#ifdef TRUSTZONE_SEC
#define U55_BASE    BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#ifndef TRUSTZONE
#define U55_BASE    BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#define U55_BASE    BASE_ADDR_APB_U55_CTRL
#endif
#endif

/* -----------------------------------------------------------------------
 * Module-static result storage (exported via cvapp_get_result*)
 * -------------------------------------------------------------------- */
#define MAX_SDLOG_RESULTS MAX_TRACKED_YOLOV8_ALGO_RES

typedef struct {
    float    conf;
    uint16_t class_idx;
} sdlog_result_t;

static sdlog_result_t g_last_results[MAX_SDLOG_RESULTS];
static int            g_last_result_count = 0;

extern "C" int cvapp_get_result_count(void)
{
    return g_last_result_count;
}

extern "C" void cvapp_get_result(int idx, float *conf, uint16_t *class_idx)
{
    if (idx < 0 || idx >= g_last_result_count) {
        *conf = 0.0f;
        *class_idx = 0;
        return;
    }
    *conf      = g_last_results[idx].conf;
    *class_idx = g_last_results[idx].class_idx;
}

using namespace std;

namespace {

constexpr int tensor_arena_size = 1053*1024;

static uint32_t tensor_arena=0;

struct ethosu_driver ethosu_drv; /* Default Ethos-U device driver */
tflite::MicroInterpreter *yolov8n_ob_int_ptr=nullptr;
TfLiteTensor *yolov8n_ob_input, *yolov8n_ob_output, *yolov8n_ob_output2;
};

#if YOLOV8N_OB_DBG_APP_LOG
std::string coco_classes[] = {"person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};
#endif

static void _arm_npu_irq_handler(void)
{
    ethosu_irq_handler(&ethosu_drv);
}

static void _arm_npu_irq_init(void)
{
    const IRQn_Type ethosu_irqnum = (IRQn_Type)U55_IRQn;
    EPII_NVIC_SetVector(ethosu_irqnum, (uint32_t)_arm_npu_irq_handler);
    NVIC_EnableIRQ(ethosu_irqnum);
}

static int _arm_npu_init(bool security_enable, bool privilege_enable)
{
    int err = 0;
    _arm_npu_irq_init();
#if TFLM2209_U55TAG2205
    const void * ethosu_base_address = (void *)(U55_BASE);
#else
    void * const ethosu_base_address = (void *)(U55_BASE);
#endif
    if (0 != (err = ethosu_init(
                        &ethosu_drv,
                        ethosu_base_address,
                        NULL,
                        0,
                        security_enable,
                        privilege_enable))) {
        xprintf("failed to initalise Ethos-U device\n");
        return err;
    }
    xprintf("Ethos-U55 device initialised\n");
    return 0;
}


int cv_yolov8n_ob_init(bool security_enable, bool privilege_enable, uint32_t model_addr) {
    int ercode = 0;

    tensor_arena = mm_reserve_align(tensor_arena_size,0x20);
    xprintf("TA[%x]\r\n",tensor_arena);

    if(_arm_npu_init(security_enable, privilege_enable)!=0)
        return -1;

    if(model_addr != 0) {
        static const tflite::Model*yolov8n_ob_model = tflite::GetModel((const void *)model_addr);

        if (yolov8n_ob_model->version() != TFLITE_SCHEMA_VERSION) {
            xprintf(
                "[ERROR] yolov8n_ob_model's schema version %d is not equal "
                "to supported version %d\n",
                yolov8n_ob_model->version(), TFLITE_SCHEMA_VERSION);
            return -1;
        }
        else {
            xprintf("yolov8n_ob model's schema version %d\n", yolov8n_ob_model->version());
        }
#if TFLM2209_U55TAG2205
        static tflite::MicroErrorReporter yolov8n_ob_micro_error_reporter;
#endif
        static tflite::MicroMutableOpResolver<2> yolov8n_ob_op_resolver;
        yolov8n_ob_op_resolver.AddTranspose();
        if (kTfLiteOk != yolov8n_ob_op_resolver.AddEthosU()){
            xprintf("Failed to add Arm NPU support to op resolver.");
            return false;
        }
#if TFLM2209_U55TAG2205
        static tflite::MicroInterpreter yolov8n_ob_static_interpreter(yolov8n_ob_model, yolov8n_ob_op_resolver,
                (uint8_t*)tensor_arena, tensor_arena_size, &yolov8n_ob_micro_error_reporter);
#else
        static tflite::MicroInterpreter yolov8n_ob_static_interpreter(yolov8n_ob_model, yolov8n_ob_op_resolver,
                (uint8_t*)tensor_arena, tensor_arena_size);
#endif
        if(yolov8n_ob_static_interpreter.AllocateTensors()!= kTfLiteOk) {
            return false;
        }
        yolov8n_ob_int_ptr = &yolov8n_ob_static_interpreter;
        yolov8n_ob_input = yolov8n_ob_static_interpreter.input(0);
        yolov8n_ob_output = yolov8n_ob_static_interpreter.output(0);
#if CHANGE_YOLOV8_OB_OUPUT_SHAPE
        yolov8n_ob_output2 = yolov8n_ob_static_interpreter.output(1);
#endif
    }

    xprintf("initial done\n");
    return ercode;
}


typedef struct detection_cls_yolov8{
    box bbox;
    float confidence;
    float index;
} detection_cls_yolov8;

static bool yolov8_det_comparator(detection_cls_yolov8 &pa, detection_cls_yolov8 &pb)
{
    return pa.confidence > pb.confidence;
}

static void yolov8_NMSBoxes(std::vector<box> &boxes,std::vector<float> &confidences,float modelScoreThreshold,float modelNMSThreshold,std::vector<int>& nms_result)
{
    detection_cls_yolov8 yolov8_bbox;
    std::vector<detection_cls_yolov8> yolov8_bboxes{};
    for(int i = 0; i < (int)boxes.size(); i++)
    {
        yolov8_bbox.bbox = boxes[i];
        yolov8_bbox.confidence = confidences[i];
        yolov8_bbox.index = i;
        yolov8_bboxes.push_back(yolov8_bbox);
    }
    sort(yolov8_bboxes.begin(), yolov8_bboxes.end(), yolov8_det_comparator);
    int updated_size = yolov8_bboxes.size();
    for(int k = 0; k < updated_size; k++)
    {
        if(yolov8_bboxes[k].confidence < modelScoreThreshold)
            continue;
        nms_result.push_back(yolov8_bboxes[k].index);
        for(int j = k + 1; j < updated_size; j++)
        {
            float iou = box_iou(yolov8_bboxes[k].bbox, yolov8_bboxes[j].bbox);
            if(iou > modelNMSThreshold)
            {
                yolov8_bboxes.erase(yolov8_bboxes.begin() + j);
                updated_size = yolov8_bboxes.size();
                j = j - 1;
            }
        }
    }
}


#if CHANGE_YOLOV8_OB_OUPUT_SHAPE
static void yolov8_ob_post_processing(tflite::MicroInterpreter* static_interpreter,float modelScoreThreshold, float modelNMSThreshold, struct_yolov8_ob_algoResult *alg, std::forward_list<el_box_t> &el_algo)
{
    uint32_t img_w = app_get_raw_width();
    uint32_t img_h = app_get_raw_height();
    TfLiteTensor* output   = static_interpreter->output(0);
    TfLiteTensor* output_2 = static_interpreter->output(1);

    int input_w = YOLOV8_OB_INPUT_TENSOR_WIDTH;
    int input_h = YOLOV8_OB_INPUT_TENSOR_HEIGHT;

    std::vector<uint16_t> class_idxs;
    std::vector<float>    confidences;
    std::vector<box>      boxes;

    float output_scale     = ((TfLiteAffineQuantization*)(output->quantization.params))->scale->data[0];
    int   output_zeropoint = ((TfLiteAffineQuantization*)(output->quantization.params))->zero_point->data[0];
    float output_2_scale     = ((TfLiteAffineQuantization*)(output_2->quantization.params))->scale->data[0];
    int   output_2_zeropoint = ((TfLiteAffineQuantization*)(output_2->quantization.params))->zero_point->data[0];

    for(int dims_cnt_2 = 0; dims_cnt_2 < output->dims->data[2]; dims_cnt_2++)
    {
        float outputs_bbox_data[4];
        float maxScore = (-1);
        uint16_t maxClassIndex = 0;
        for(int dims_cnt_1 = 0; dims_cnt_1 < output->dims->data[1]; dims_cnt_1++)
        {
            int value = output->data.int8[ dims_cnt_2 + dims_cnt_1 * output->dims->data[2]];
            float deq_value = ((float)value - (float)output_zeropoint) * output_scale;
            if(dims_cnt_1%2)
                deq_value *= (float)input_h;
            else
                deq_value *= (float)input_w;
            outputs_bbox_data[dims_cnt_1] = deq_value;
        }
        for(int output_2_dims_cnt_1 = 0; output_2_dims_cnt_1 < output_2->dims->data[2]; output_2_dims_cnt_1++)
        {
            int value_2 = output_2->data.int8[ output_2_dims_cnt_1 + dims_cnt_2 * output_2->dims->data[2]];
            float deq_value_2 = ((float)value_2 - (float)output_2_zeropoint) * output_2_scale;
            if(maxScore < deq_value_2)
            {
                maxScore = deq_value_2;
                maxClassIndex = output_2_dims_cnt_1;
            }
        }
        if (maxScore >= modelScoreThreshold)
        {
            box bbox;
            bbox.x = (outputs_bbox_data[0] - (0.5f * outputs_bbox_data[2]));
            bbox.y = (outputs_bbox_data[1] - (0.5f * outputs_bbox_data[3]));
            bbox.w = outputs_bbox_data[2];
            bbox.h = outputs_bbox_data[3];
            boxes.push_back(bbox);
            class_idxs.push_back(maxClassIndex);
            confidences.push_back(maxScore);
        }
    }

    std::vector<int> nms_result;
    yolov8_NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    /* Reset module result store */
    g_last_result_count = 0;

    for (int i = 0; i < (int)nms_result.size(); i++)
    {
        if(!(MAX_TRACKED_YOLOV8_ALGO_RES-i)) break;
        int idx = nms_result[i];

        float scale_factor_w = (float)img_w / (float)YOLOV8_OB_INPUT_TENSOR_WIDTH;
        float scale_factor_h = (float)img_h / (float)YOLOV8_OB_INPUT_TENSOR_HEIGHT;
        alg->obr[i].confidence = confidences[idx];
        alg->obr[i].bbox.x      = (uint32_t)(boxes[idx].x * scale_factor_w);
        alg->obr[i].bbox.y      = (uint32_t)(boxes[idx].y * scale_factor_h);
        alg->obr[i].bbox.width  = (uint32_t)(boxes[idx].w * scale_factor_w);
        alg->obr[i].bbox.height = (uint32_t)(boxes[idx].h * scale_factor_h);
        alg->obr[i].class_idx   = class_idxs[idx];

        el_box_t temp_el_box;
        temp_el_box.score  = confidences[idx] * 100;
        temp_el_box.target = class_idxs[idx];
        temp_el_box.x = (uint32_t)(boxes[idx].x * scale_factor_w);
        temp_el_box.y = (uint32_t)(boxes[idx].y * scale_factor_h);
        temp_el_box.w = (uint32_t)(boxes[idx].w * scale_factor_w);
        temp_el_box.h = (uint32_t)(boxes[idx].h * scale_factor_h);
        el_algo.emplace_front(temp_el_box);

        /* Store result for sdlog */
        if (g_last_result_count < MAX_SDLOG_RESULTS) {
            g_last_results[g_last_result_count].conf      = confidences[idx];
            g_last_results[g_last_result_count].class_idx = class_idxs[idx];
            g_last_result_count++;
        }
    }
}
#endif


int cv_yolov8n_ob_run(struct_yolov8_ob_algoResult *algoresult_yolov8n_ob) {
    int ercode = 0;
    float w_scale;
    float h_scale;
    uint32_t img_w = app_get_raw_width();
    uint32_t img_h = app_get_raw_height();
    uint32_t ch = app_get_raw_channels();
    uint32_t raw_addr = app_get_raw_addr();
    std::forward_list<el_box_t> el_algo;

    if(yolov8n_ob_int_ptr != nullptr) {
#ifdef TOTAL_STEP_TICK
        SystemGetTick(&systick_1, &loop_cnt_1);
#endif
        w_scale = (float)(img_w - 1) / (YOLOV8_OB_INPUT_TENSOR_WIDTH - 1);
        h_scale = (float)(img_h - 1) / (YOLOV8_OB_INPUT_TENSOR_HEIGHT - 1);

        hx_lib_image_resize_BGR8U3C_to_RGB24_helium((uint8_t*)raw_addr, (uint8_t*)yolov8n_ob_input->data.data,
                        img_w, img_h, ch,
                        YOLOV8_OB_INPUT_TENSOR_WIDTH, YOLOV8_OB_INPUT_TENSOR_HEIGHT, w_scale, h_scale);

        for (int i = 0; i < yolov8n_ob_input->bytes; ++i) {
            *((int8_t *)yolov8n_ob_input->data.data+i) = *((int8_t *)yolov8n_ob_input->data.data+i) - 128;
        }

        TfLiteStatus invoke_status = yolov8n_ob_int_ptr->Invoke();
        if(invoke_status != kTfLiteOk)
        {
            xprintf("yolov8 object detect invoke fail\n");
            return -1;
        }

        yolov8_ob_post_processing(yolov8n_ob_int_ptr, 0.25, 0.45, algoresult_yolov8n_ob, el_algo);

#ifdef TOTAL_STEP_TICK
        SystemGetTick(&systick_2, &loop_cnt_2);
#endif
    }

#ifdef UART_SEND_ALOGO_RESEULT
    algoresult_yolov8n_ob->algo_tick = (loop_cnt_2-loop_cnt_1)*CPU_CLK+(systick_1-systick_2) + capture_image_tick;
    uint32_t judge_case_data;
    uint32_t g_trans_type;
    hx_drv_swreg_aon_get_appused1(&judge_case_data);
    g_trans_type = (judge_case_data>>16);
    if( g_trans_type == 0 || g_trans_type == 2)
    {
        hx_InvalidateDCache_by_Addr((volatile void *)app_get_jpeg_addr(), sizeof(uint8_t) *app_get_jpeg_sz());

        el_img_t temp_el_jpg_img = el_img_t{};
        temp_el_jpg_img.data   = (uint8_t *)app_get_jpeg_addr();
        temp_el_jpg_img.size   = app_get_jpeg_sz();
        temp_el_jpg_img.width  = app_get_raw_width();
        temp_el_jpg_img.height = app_get_raw_height();
        temp_el_jpg_img.format = EL_PIXEL_FORMAT_JPEG;
        temp_el_jpg_img.rotate = EL_PIXEL_ROTATE_0;

        send_device_id();
        event_reply(concat_strings(", ", algo_tick_2_json_str(algoresult_yolov8n_ob->algo_tick),", ", box_results_2_json_str(el_algo), ", ", img_2_json_str(&temp_el_jpg_img)));
    }
    set_model_change_by_uart();
#endif

    SystemGetTick(&systick_1, &loop_cnt_1);
    sensordplib_retrigger_capture();
    SystemGetTick(&systick_2, &loop_cnt_2);
    capture_image_tick = (loop_cnt_2-loop_cnt_1)*CPU_CLK+(systick_1-systick_2);
    return ercode;
}

int cv_yolov8n_ob_deinit()
{
    return 0;
}
