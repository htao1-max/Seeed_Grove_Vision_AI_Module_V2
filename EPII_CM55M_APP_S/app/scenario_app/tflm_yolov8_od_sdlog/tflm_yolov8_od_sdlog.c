#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "powermode_export.h"

#define WATCH_DOG_TIMEOUT_TH    (500) //ms

#ifdef TRUSTZONE_SEC
#ifdef FREERTOS
#else
#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif
#include "arm_cmse.h"
#endif
#endif

#include "WE2_device.h"

#include "spi_master_protocol.h"
#include "hx_drv_spi.h"
#include "spi_eeprom_comm.h"
#include "board.h"
#include "xprintf.h"
#include "tflm_yolov8_od_sdlog.h"
#include "evt_i2ccomm.h"
#include "WE2_core.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#ifdef IP_sensorctrl
#include "hx_drv_sensorctrl.h"
#endif
#ifdef IP_xdma
#include "hx_drv_xdma.h"
#include "sensor_dp_lib.h"
#endif
#ifdef IP_cdm
#include "hx_drv_cdm.h"
#endif
#ifdef IP_gpio
#include "hx_drv_gpio.h"
#endif
#include "hx_drv_pmu_export.h"
#include "hx_drv_pmu.h"
#include "powermode.h"
#include "BITOPS.h"

#include "common_config.h"
#include "cisdp_sensor.h"
#include "event_handler.h"
#include "cvapp_yolov8n_ob.h"
#include "memory_manage.h"
#include "hx_drv_watchdog.h"

/* SD logging and I2C command modules */
#include "sdlog.h"
#include "i2c_cmd.h"

#ifdef EPII_FPGA
#define DBG_APP_LOG     (1)
#else
#define DBG_APP_LOG     (0)
#endif
#if DBG_APP_LOG
    #define dbg_app_log(fmt, ...)   xprintf(fmt, ##__VA_ARGS__)
#else
    #define dbg_app_log(fmt, ...)
#endif

#define TOTAL_STEP_TICK 1
#define TOTAL_STEP_TICK_DBG_LOG 0

#if TOTAL_STEP_TICK
#define CPU_CLK 0xffffff+1
#endif

#define GROVE_VISION_AI_II

/* Frame / detection counters for SD logging */
static volatile uint32_t g_frame_count  = 0;
static volatile uint32_t g_detect_count = 0;

static uint8_t  g_xdma_abnormal, g_md_detect, g_cdm_fifoerror, g_wdt1_timeout, g_wdt2_timeout, g_wdt3_timeout;
static uint8_t  g_hxautoi2c_error, g_inp1bitparer_abnormal;
static uint32_t g_dp_event;
static uint8_t  g_frame_ready;
static uint32_t g_cur_jpegenc_frame;
static uint8_t  g_time;
static uint8_t  g_spi_master_initial_status;
static uint32_t g_use_case;
/*volatile*/ uint32_t jpeg_addr, jpeg_sz;
struct_algoResult algoresult;
struct_fm_algoResult  algoresult_fm;
struct_hp_algoResult  algoresult_pl;
struct_fl_fr_enroll_algoResult  algoresult_fl_fr_enroll;
struct_fl_fr_algoResult  algoresult_fl_fr_infer;
struct_yolov8_ob_algoResult algoresult_yolov8n_ob;
struct_yolov8_ob_algoResult algoresult_yolofastest_ob;
struct_yolov8_pose_algoResult algoresult_yolov8_pose;
static uint32_t g_trans_type;
static uint32_t judge_case_data;
void app_start_state(APP_STATE_E state);
void model_change(void);
void pinmux_init(void);

#ifdef GROVE_VISION_AI_II
void spi_m_pinmux_cfg(SCU_PINMUX_CFG_T *pinmux_cfg)
{
    pinmux_cfg->pin_pb2  = SCU_PB2_PINMUX_SPI_M_DO_1;
    pinmux_cfg->pin_pb3  = SCU_PB3_PINMUX_SPI_M_DI_1;
    pinmux_cfg->pin_pb4  = SCU_PB4_PINMUX_SPI_M_SCLK_1;
    pinmux_cfg->pin_pb11 = SCU_PB11_PINMUX_SPI_M_CS;
}
#else
void spi_m_pinmux_cfg(SCU_PINMUX_CFG_T *pinmux_cfg)
{
    pinmux_cfg->pin_pb2 = SCU_PB2_PINMUX_SPI_M_DO_1;
    pinmux_cfg->pin_pb3 = SCU_PB3_PINMUX_SPI_M_DI_1;
    pinmux_cfg->pin_pb4 = SCU_PB4_PINMUX_SPI_M_SCLK_1;
    pinmux_cfg->pin_pb5 = SCU_PB5_PINMUX_SPI_M_CS_1;
}
#endif

void pinmux_init(void)
{
    SCU_PINMUX_CFG_T pinmux_cfg;
    hx_drv_scu_get_all_pinmux_cfg(&pinmux_cfg);
    
    pinmux_cfg.pin_pa2 = SCU_PA2_PINMUX_SB_I2C_S_SCL_0;
    pinmux_cfg.pin_pa3 = SCU_PA3_PINMUX_SB_I2C_S_SDA_0;
    
    spi_m_pinmux_cfg(&pinmux_cfg);
    hx_drv_scu_set_all_pinmux_cfg(&pinmux_cfg, 1);
}

static void dp_var_int(void)
{
    g_xdma_abnormal = 0;
    g_md_detect = 0;
    g_cdm_fifoerror = 0;
    g_wdt1_timeout = 0;
    g_wdt2_timeout = 0;
    g_wdt3_timeout = 0;
    g_inp1bitparer_abnormal = 0;
    g_dp_event = 0;
    g_frame_ready = 0;
    g_time = 0;
    g_cur_jpegenc_frame = 0;
    g_hxautoi2c_error = 0;
    g_spi_master_initial_status = 0;
}

void WDG_Reset_ISR_CB(uint32_t event)
{
    uint32_t read_data;
    read_data = hx_drv_watchdog_value(WATCHDOG_ID_0);
    xprintf("read_data=%d not reset\n", read_data);
    xprintf("CLI_WDG_Reset_ISR_CB event=%d\n", event);
}

void SetAlarmPMU(void) {
    uint32_t id;
    TIMER_CFG_T timer_cfg;
#ifdef EPII_FPGA
    timer_cfg.period = 1000;
#else
    timer_cfg.period = 1000;
#endif
    timer_cfg.mode  = TIMER_MODE_ONESHOT;
    timer_cfg.ctrl  = TIMER_CTRL_PMU;
    timer_cfg.state = TIMER_STATE_PMU;
    id = 1;
    hx_drv_timer_hw_start(id, &timer_cfg, NULL);
}

void SetPSPDNoVid(void)
{
    PM_PD_NOVIDPRE_CFG_T cfg;
    uint8_t speed, reset, precap, nframeend_ctrl, trigger, retention;
    uint32_t pmu_pad_pa01_mask, pmu_rtc_mask, support_debugdump;
    uint32_t pmu_pad_pa23_mask, pmu_i2cw_mask, pmu_timer_mask, pmu_cmp_mask, pmu_ts_mask;
    uint32_t dcdcpin, freq, cm55mdiv, cm55sdiv, pmu_anti_mask;
    SCU_LSC_CLK_CFG_T lsc_cfg;
    SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;
    PM_CFG_PWR_MODE_E mode;

    speed = SCU_PLL_FREQ_ENABLE;
    reset = 0;
    nframeend_ctrl = 0;
    retention = 0;
    precap = 0;
    pmu_pad_pa01_mask = 0;
    pmu_rtc_mask = 0;
    pmu_pad_pa23_mask = 0;
    pmu_i2cw_mask = 0;
    pmu_timer_mask = 0;
    pmu_cmp_mask = 0;
    pmu_ts_mask = 0;
    trigger = 1;
    support_debugdump = 0;
    dcdcpin = 0;
    freq = 400000000;
    cm55mdiv = SCU_HSCCLKDIV_1;
    cm55sdiv = SCU_LSCCLKDIV_4;
    pmu_anti_mask = 0;

    mode = PM_MODE_PS_NOVID_PREROLLING;
    hx_lib_pm_get_defcfg_bymode(&cfg, mode);

    cfg.bootromspeed.bootromclkfreq = speed;
    cfg.bootromspeed.pll_freq = freq;
    cfg.bootromspeed.cm55m_div = cm55mdiv;
    cfg.bootromspeed.cm55s_div = cm55sdiv;
    cfg.cm55s_reset = reset;
    cfg.pmu_pad_pa01_mask = pmu_pad_pa01_mask;
    cfg.pmu_rtc_mask = pmu_rtc_mask;
    cfg.pmu_pad_pa23_mask = pmu_pad_pa23_mask;
    cfg.pmu_i2cw_mask = pmu_i2cw_mask;
    cfg.pmu_timer_mask = pmu_timer_mask;
    cfg.pmu_cmp_mask = pmu_cmp_mask;
    cfg.pmu_ts_mask = pmu_ts_mask;
    cfg.pmu_anti_mask = pmu_anti_mask;
    cfg.support_debugdump = support_debugdump;
    cfg.nframeend_ctrl = nframeend_ctrl;
    cfg.tcm_retention = retention;
    cfg.hscsram_retention[0] = retention;
    cfg.hscsram_retention[1] = retention;
    cfg.hscsram_retention[2] = retention;
    cfg.hscsram_retention[3] = retention;
    cfg.lscsram_retention = retention;
    cfg.skip_bootflow.sec_mem_flag = retention;
    cfg.skip_bootflow.first_bl_flag = retention;
    cfg.skip_bootflow.cm55m_s_app_flag = retention;
    cfg.skip_bootflow.cm55m_ns_app_flag = retention;
    cfg.skip_bootflow.cm55s_s_app_flag = retention;
    cfg.skip_bootflow.cm55s_ns_app_flag = retention;
    cfg.skip_bootflow.cm55m_model_flag = retention;
    cfg.skip_bootflow.cm55s_model_flag = retention;
    cfg.skip_bootflow.cm55m_appcfg_flag = retention;
    cfg.skip_bootflow.cm55s_appcfg_flag = retention;
    cfg.skip_bootflow.cm55m_s_app_rwdata_flag = retention;
    cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = retention;
    cfg.skip_bootflow.cm55s_s_app_rwdata_flag = retention;
    cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = retention;
    cfg.skip_bootflow.secure_debug_flag = retention;
    cfg.support_bootwithcap = precap;
    cfg.pmu_dcdc_outpin = dcdcpin;
    cfg.ioret = PM_CFG_PD_IORET_ON;
    cfg.mipi_lane_en = PMU_MIPI_LANE_ALL_DISABLE;
    cfg.sensor_type = PM_SENSOR_TIMING_FVLDLVLD_SHIFT;
    cfg.simo_pd_onoff = PM_SIMO_PD_ONOFF_ON;

    hx_lib_pm_cfg_set(&cfg, NULL, PM_MODE_PS_NOVID_PREROLLING);
    SetAlarmPMU();

    hsc_cfg.hscclk.hscclksrc = SCU_HSCCLKSRC_XTAL24M;
    hsc_cfg.hscclk.hscclkdiv = SCU_HSCCLKDIV_1;
    hsc_cfg.hscd12clksrc = SCU_HSCD12CLKSRC_HSC;
    hsc_cfg.i3chcdiv = SCU_HSCI3CHCLKDIV_1;
    hsc_cfg.sdiodiv = SCU_HSCSDIOCLKDIV_1;
    lsc_cfg.lscclksrc = SCU_LSCCLKSRC_XTAL24M;
    lsc_cfg.lscclkdiv = SCU_LSCCLKDIV_1;

    if(trigger == 1)
        hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYAPP);
}

void SetPSPDNoVid_24M(void)
{
    PM_PD_NOVIDPRE_CFG_T cfg;
    uint8_t speed, reset, precap, nframeend_ctrl, trigger, retention;
    uint32_t pmu_pad_pa01_mask, pmu_rtc_mask, support_debugdump;
    uint32_t pmu_pad_pa23_mask, pmu_i2cw_mask, pmu_timer_mask, pmu_cmp_mask, pmu_ts_mask;
    uint32_t dcdcpin, freq, cm55mdiv, cm55sdiv, pmu_anti_mask;
    SCU_LSC_CLK_CFG_T lsc_cfg;
    SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;
    PM_CFG_PWR_MODE_E mode;

    speed = SCU_PLL_FREQ_DISABLE;
    reset = 1;
    nframeend_ctrl = 0;
    retention = 0;
    precap = 0;
    pmu_pad_pa01_mask = 0;
    pmu_rtc_mask = 0;
    pmu_pad_pa23_mask = 0;
    pmu_i2cw_mask = 0;
    pmu_timer_mask = 0;
    pmu_cmp_mask = 0;
    pmu_ts_mask = 0;
    trigger = 1;
    support_debugdump = 0;
    dcdcpin = 0;
    freq = 0;
    cm55mdiv = SCU_HSCCLKDIV_1;
    cm55sdiv = SCU_LSCCLKDIV_1;
    pmu_anti_mask = 0;

    mode = PM_MODE_PS_NOVID_PREROLLING;
    hx_lib_pm_get_defcfg_bymode(&cfg, mode);

    cfg.bootromspeed.bootromclkfreq = speed;
    cfg.bootromspeed.pll_freq = freq;
    cfg.bootromspeed.cm55m_div = cm55mdiv;
    cfg.bootromspeed.cm55s_div = cm55sdiv;
    cfg.cm55s_reset = reset;
    cfg.pmu_pad_pa01_mask = pmu_pad_pa01_mask;
    cfg.pmu_rtc_mask = pmu_rtc_mask;
    cfg.pmu_pad_pa23_mask = pmu_pad_pa23_mask;
    cfg.pmu_i2cw_mask = pmu_i2cw_mask;
    cfg.pmu_timer_mask = pmu_timer_mask;
    cfg.pmu_cmp_mask = pmu_cmp_mask;
    cfg.pmu_ts_mask = pmu_ts_mask;
    cfg.pmu_anti_mask = pmu_anti_mask;
    cfg.support_debugdump = support_debugdump;
    cfg.nframeend_ctrl = nframeend_ctrl;
    cfg.tcm_retention = retention;
    cfg.hscsram_retention[0] = retention;
    cfg.hscsram_retention[1] = retention;
    cfg.hscsram_retention[2] = retention;
    cfg.hscsram_retention[3] = retention;
    cfg.lscsram_retention = retention;
    cfg.skip_bootflow.sec_mem_flag = retention;
    cfg.skip_bootflow.first_bl_flag = retention;
    cfg.skip_bootflow.cm55m_s_app_flag = retention;
    cfg.skip_bootflow.cm55m_ns_app_flag = retention;
    cfg.skip_bootflow.cm55s_s_app_flag = retention;
    cfg.skip_bootflow.cm55s_ns_app_flag = retention;
    cfg.skip_bootflow.cm55m_model_flag = retention;
    cfg.skip_bootflow.cm55s_model_flag = retention;
    cfg.skip_bootflow.cm55m_appcfg_flag = retention;
    cfg.skip_bootflow.cm55s_appcfg_flag = retention;
    cfg.skip_bootflow.cm55m_s_app_rwdata_flag = retention;
    cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = retention;
    cfg.skip_bootflow.cm55s_s_app_rwdata_flag = retention;
    cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = retention;
    cfg.skip_bootflow.secure_debug_flag = retention;
    cfg.support_bootwithcap = precap;
    cfg.pmu_dcdc_outpin = dcdcpin;
    cfg.ioret = PM_CFG_PD_IORET_ON;
    cfg.mipi_lane_en = PMU_MIPI_LANE_ALL_DISABLE;
    cfg.sensor_type = PM_SENSOR_TIMING_FVLDLVLD_SHIFT;
    cfg.simo_pd_onoff = PM_SIMO_PD_ONOFF_ON;

    hx_lib_pm_cfg_set(&cfg, NULL, PM_MODE_PS_NOVID_PREROLLING);
    SetAlarmPMU();

    hsc_cfg.hscclk.hscclksrc = SCU_HSCCLKSRC_XTAL24M;
    hsc_cfg.hscclk.hscclkdiv = SCU_HSCCLKDIV_1;
    hsc_cfg.hscd12clksrc = SCU_HSCD12CLKSRC_HSC;
    hsc_cfg.i3chcdiv = SCU_HSCI3CHCLKDIV_1;
    hsc_cfg.sdiodiv = SCU_HSCSDIOCLKDIV_1;
    lsc_cfg.lscclksrc = SCU_LSCCLKSRC_XTAL24M;
    lsc_cfg.lscclkdiv = SCU_LSCCLKDIV_1;

    if(trigger == 1)
        hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYAPP);
}


/* -----------------------------------------------------------------------
 * Datapath callback — fired once per captured frame
 * -------------------------------------------------------------------- */
static void dp_app_cv_yolov8n_ob_eventhdl_cb(EVT_INDEX_E event)
{
    uint16_t err;
    int32_t read_status;

    g_dp_event = event;

    switch(event)
    {
    case EVT_INDEX_1BITPARSER_ERR:
        hx_drv_inp1bitparser_get_errstatus(&err);
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_1BITPARSER_ERR err=0x%x\r\n", err);
        hx_drv_inp1bitparser_clear_int();
        hx_drv_inp1bitparser_set_enable(0);
        g_inp1bitparer_abnormal = 1;
        break;
    case EVT_INDEX_EDM_WDT1_TIMEOUT:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_EDM_WDT1_TIMEOUT\r\n");
        g_wdt1_timeout = 1;
        break;
    case EVT_INDEX_EDM_WDT2_TIMEOUT:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_EDM_WDT2_TIMEOUT\r\n");
        g_wdt2_timeout = 1;
        break;
    case EVT_INDEX_EDM_WDT3_TIMEOUT:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_EDM_WDT3_TIMEOUT\r\n");
        g_wdt3_timeout = 1;
        break;
    case EVT_INDEX_CDM_FIFO_ERR:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_CDM_FIFO_ERR\r\n");
        g_cdm_fifoerror = 1;
        break;
    case EVT_INDEX_XDMA_WDMA1_ABNORMAL:
    case EVT_INDEX_XDMA_WDMA2_ABNORMAL:
    case EVT_INDEX_XDMA_WDMA3_ABNORMAL:
    case EVT_INDEX_XDMA_RDMA_ABNORMAL:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_XDMA_WDMA123_ABNORMAL or RDMA_ABNORMAL\r\n");
        g_xdma_abnormal = 1;
        break;
    case EVT_INDEX_CDM_MOTION:
        dbg_printf(DBG_LESS_INFO, "Motion Detect\n");
        g_md_detect = 1;
        break;
    case EVT_INDEX_XDMA_FRAME_READY:
        g_cur_jpegenc_frame++;
        g_frame_ready = 1;
        dbg_printf(DBG_LESS_INFO, "SENSORDPLIB_STATUS_XDMA_FRAME_READY %d\n", g_cur_jpegenc_frame);
        break;
    case EVT_INDEX_SENSOR_RTC_FIRE:
        g_time++;
        break;
    case EVT_INDEX_HXAUTOI2C_ERR:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_HXAUTOI2C_ERR\r\n");
        g_hxautoi2c_error = 1;
        break;
    default:
        dbg_printf(DBG_LESS_INFO, "Other Event %d\n", event);
        break;
    }

    if(g_frame_ready == 1)
    {
        g_frame_ready = 0;

        hx_drv_swreg_aon_get_appused1(&judge_case_data);

        if( ((judge_case_data&0xff) != g_use_case) || ((judge_case_data>>16) != g_trans_type) ) {
            model_change();
#ifdef CPU_24MHZ_VERSION
            SetPSPDNoVid_24M();
#else
            SetPSPDNoVid();
#endif
        }

        cisdp_get_jpginfo(&jpeg_sz, &jpeg_addr);

#ifdef EN_ALGO
#ifdef UART_SEND_ALOGO_RESEULT
        hx_drv_swreg_aon_get_appused1(&judge_case_data);
        g_trans_type = (judge_case_data>>16);
        if( g_trans_type == 0 )
        {
            cv_yolov8n_ob_run(&algoresult_yolov8n_ob);
        }
        else if( g_trans_type == 1 || g_trans_type == 2)
        {
#if TOTAL_STEP_TICK
            uint32_t systick_1, systick_2;
            uint32_t loop_cnt_1, loop_cnt_2;
            SystemGetTick(&systick_1, &loop_cnt_1);
#endif
            cv_yolov8n_ob_run(&algoresult_yolov8n_ob);
#if TOTAL_STEP_TICK
            SystemGetTick(&systick_2, &loop_cnt_2);
            if(g_trans_type == 1)
                algoresult_yolov8n_ob.algo_tick = (loop_cnt_2-loop_cnt_1)*CPU_CLK+(systick_1-systick_2);
#endif
        }
#else
#if TOTAL_STEP_TICK
        uint32_t systick_1, systick_2;
        uint32_t loop_cnt_1, loop_cnt_2;
        SystemGetTick(&systick_1, &loop_cnt_1);
#endif
        cv_yolov8n_ob_run(&algoresult_yolov8n_ob);
#if TOTAL_STEP_TICK
        SystemGetTick(&systick_2, &loop_cnt_2);
        algoresult_yolov8n_ob.algo_tick = (loop_cnt_2-loop_cnt_1)*CPU_CLK+(systick_1-systick_2);
#endif
#endif /* UART_SEND_ALOGO_RESEULT */

        /* ---- SD card logging ---- */
        if (g_recording_active) {
            char fname[48];

            if (g_frame_count == 0) {
                xprintf("[SDLOG] First frame: jpeg_addr=0x%x jpeg_sz=%u\r\n", jpeg_addr, jpeg_sz);
            }

            /* 1. Always save to ALL/ */
            xsprintf(fname, "img_%04lu.jpg", g_frame_count);
            sdlog_save_all(jpeg_addr, jpeg_sz, fname);
            sdlog_write("[ALL] %s (frame %lu)\r\n", fname, g_frame_count);

            /* 2. Save to DETECT/ if any object is above threshold.
             * Cap logged/written detections at DET_LOG_MAX. cvapp results
             * are already NMS-sorted by descending confidence, so the
             * first DET_LOG_MAX entries are the top-N. annot[] sized for
             * DET_LOG_MAX lines at ~50 B/line worst case. */
            #define DET_LOG_MAX 10
            int n_total = cvapp_get_result_count();
            int n = (n_total > DET_LOG_MAX) ? DET_LOG_MAX : n_total;
            if (n_total > DET_LOG_MAX) {
                xprintf("[SDLOG][ERR] frame %lu: %d detections exceeds cap %d; keeping top %d\r\n",
                        g_frame_count, n_total, DET_LOG_MAX, DET_LOG_MAX);
                sdlog_write("[ERR] frame %lu: %d detections > cap %d; kept top %d\r\n",
                            g_frame_count, n_total, DET_LOG_MAX, DET_LOG_MAX);
            }
            int triggered = 0;
            char annot[768];
            size_t annot_len = 0;

            /* Build annotation buffer with top-N detections (also flags
             * `triggered` if any pass threshold). Must scan all n_total
             * for the trigger so a low-conf top-1 doesn't suppress an
             * above-threshold detection at index 11+. */
            for (int i = 0; i < n_total; i++) {
                float conf;
                uint16_t cls;
                cvapp_get_result(i, &conf, &cls);
                if (conf >= g_detect_threshold && !triggered)
                    triggered = 1;
                if (i >= n) continue;  /* over the top-N cap */
                /* newlib-nano strips %f; format conf via integer split */
                unsigned conf_x100 = (unsigned)(conf * 100.0f + 0.5f);
                int w = snprintf(annot + annot_len, sizeof(annot) - annot_len,
                    "%u %u.%02u %lu %lu %lu %lu\n",
                    (unsigned)cls, conf_x100 / 100, conf_x100 % 100,
                    algoresult_yolov8n_ob.obr[i].bbox.x,
                    algoresult_yolov8n_ob.obr[i].bbox.y,
                    algoresult_yolov8n_ob.obr[i].bbox.width,
                    algoresult_yolov8n_ob.obr[i].bbox.height);
                if (w < 0) continue;
                if ((size_t)w >= sizeof(annot) - annot_len)
                    annot_len = sizeof(annot) - 1;
                else
                    annot_len += (size_t)w;
            }

            /* If at least one detection exceeded threshold, save image + annotation.
             *
             * Gate [DETECT] log line + counter bump on jpeg save success so
             * session.log entries always match files on disk. Order:
             *   jpeg (durable on f_close) → txt (best-effort) → [DETECT] line
             *   (f_sync'd) → counter++.
             * On jpeg failure: emit [ERR_DETECT], do not bump counter (number
             * is retried next time). On txt-only failure: emit [WARN_DETECT]
             * but still emit [DETECT] and bump counter (jpeg is the source
             * of truth for "this detection happened"). */
            if (triggered) {
                xsprintf(fname, "det_%04lu.jpg", g_detect_count);
                if (!sdlog_save_detect(jpeg_addr, jpeg_sz, fname)) {
                    sdlog_write("[ERR_DETECT] frame %lu: jpeg save failed (would-be %s)\r\n",
                                g_frame_count, fname);
                } else {
                    char txt_fname[48];
                    xsprintf(txt_fname, "det_%04lu.txt", g_detect_count);
                    if (!sdlog_save_detect_txt(txt_fname, annot)) {
                        sdlog_write("[WARN_DETECT] frame %lu: txt save failed for %s (jpeg ok)\r\n",
                                    g_frame_count, fname);
                    }

                    /* Log top-N detections for this frame to session.log */
                    sdlog_write("[DETECT] %s (%d objects, frame %lu)\r\n",
                                fname, n, g_frame_count);
                    for (int i = 0; i < n; i++) {
                        float conf;
                        uint16_t cls;
                        cvapp_get_result(i, &conf, &cls);
                        unsigned conf_x100 = (unsigned)(conf * 100.0f + 0.5f);
                        sdlog_write("  obj[%d] class=%u conf=%u.%02u bbox=(%lu,%lu,%lu,%lu)\r\n",
                                    i, (unsigned)cls,
                                    conf_x100 / 100, conf_x100 % 100,
                                    algoresult_yolov8n_ob.obr[i].bbox.x,
                                    algoresult_yolov8n_ob.obr[i].bbox.y,
                                    algoresult_yolov8n_ob.obr[i].bbox.width,
                                    algoresult_yolov8n_ob.obr[i].bbox.height);
                    }
                    g_detect_count++;
                }
            }
            #undef DET_LOG_MAX
            g_frame_count++;
        }

        /* Drain any deferred log entries from the i2ccomm RX event. Runs on
         * the main scenario thread — single writer to session.log. */
        sdlog_log_drain();
        sdlog_tlm_drain();

        /* Surface I2C mailbox FIFO drops on change. Bumping means the
         * worst-case scenario-loop stall exceeded the 16-slot mailbox
         * depth — bump EVT_I2CCOMM_RX_FIFO_SLOTS and/or
         * SDLOG_TLM_RING_SLOTS. Pattern matches sdlog.c's
         * g_tlm_dropped print-on-change. */
        {
            static uint32_t s_last_iic_dropped = 0;
            uint32_t cur = evt_i2ccomm_get_rx_dropped(USE_DW_IIC_SLV_0);
            if (cur != s_last_iic_dropped) {
                xprintf("[I2CCOMM] dropped %lu rx frames (mailbox FIFO full)\r\n",
                        (unsigned long)cur);
                s_last_iic_dropped = cur;
            }
        }

        /* Clear algo results for next frame */
        for (int i = 0; i < MAX_TRACKED_YOLOV8_ALGO_RES; ++i) {
            algoresult_yolov8n_ob.obr[i].bbox.x      = 0;
            algoresult_yolov8n_ob.obr[i].bbox.y      = 0;
            algoresult_yolov8n_ob.obr[i].bbox.width  = 0;
            algoresult_yolov8n_ob.obr[i].bbox.height = 0;
            algoresult_yolov8n_ob.obr[i].confidence  = 0;
            algoresult_yolov8n_ob.obr[i].class_idx   = 0;
        }
#endif /* EN_ALGO */
    }

    if(g_md_detect == 1)
        g_md_detect = 0;

    if(g_inp1bitparer_abnormal == 1 || g_wdt1_timeout == 1 || g_wdt2_timeout == 1 || g_wdt3_timeout == 1
            || g_cdm_fifoerror == 1 || g_xdma_abnormal == 1 || g_hxautoi2c_error == 1)
    {
        cisdp_sensor_stop();
    }
}

void app_start_state(APP_STATE_E state)
{
    if(cisdp_sensor_init() < 0)
    {
        xprintf("\r\nCIS Init fail\r\n");
        APP_BLOCK_FUNC();
    }

    dp_var_int();
#ifdef UART_SEND_ALOGO_RESEULT
    hx_drv_swreg_aon_get_appused1(&judge_case_data);
    g_trans_type = (judge_case_data>>16);
    if( g_trans_type == 0 || g_trans_type == 2)
    {
        if(state == APP_STATE_ALLON_YOLOV8N_OB)
        {
            if(cisdp_dp_init(true, SENSORDPLIB_PATH_INT_INP_HW5X5_JPEG, dp_app_cv_yolov8n_ob_eventhdl_cb, 4, APP_DP_RES_RGB640x480_INP_SUBSAMPLE_2X) < 0)
            {
                xprintf("\r\nDATAPATH Init fail\r\n");
                APP_BLOCK_FUNC();
            }
        }
    }
    else if ( g_trans_type == 1 )
    {
        if(state == APP_STATE_ALLON_YOLOV8N_OB)
        {
            if(cisdp_dp_init(true, SENSORDPLIB_PATH_INT_INP_HW5X5_JPEG, dp_app_cv_yolov8n_ob_eventhdl_cb, 4, APP_DP_RES_RGB640x480_INP_SUBSAMPLE_2X) < 0)
            {
                xprintf("\r\nDATAPATH Init fail\r\n");
                APP_BLOCK_FUNC();
            }
        }
    }
#else
    if(state == APP_STATE_ALLON_YOLOV8N_OB)
    {
        if(cisdp_dp_init(true, SENSORDPLIB_PATH_INT_INP_HW5X5_JPEG, dp_app_cv_yolov8n_ob_eventhdl_cb, 4, APP_DP_RES_RGB640x480_INP_SUBSAMPLE_2X) < 0)
        {
            xprintf("\r\nDATAPATH Init fail\r\n");
            APP_BLOCK_FUNC();
        }
    }
#endif
    event_handler_init();
    cisdp_sensor_start();

    /* Mount SD / open session log only after the camera is fully up and
     * streaming. sdlog_session_init() remuxes PB2/PB3 from I2C_M (camera
     * control bus) to SPI_M (SD card) — done here, after the last CIS
     * register write (OV5647_stream_on in cisdp_sensor_start()), so the
     * camera's I2C writes during init/start still reach the sensor. Must
     * stay before event_handler_start(), which never returns. */
    sdlog_session_init();
    sdlog_tlm_init();

#if SDLOG_TESTING_AUTO_RECORD
    g_recording_active = 1;
    xprintf("[TESTING] Auto-start recording (SDLOG_TESTING_AUTO_RECORD=1)\r\n");
    sdlog_write("[TESTING] Auto-start recording (SDLOG_TESTING_AUTO_RECORD=1)\r\n");
#endif

    event_handler_start();
}

void model_change(void) {
    event_handler_stop();
    event_handler_deinit();
    cisdp_sensor_stop();
#ifdef EN_ALGO
    if(g_use_case == 0)
        cv_yolov8n_ob_deinit();
#endif
}

/* -----------------------------------------------------------------------
 * Application entry point
 * -------------------------------------------------------------------- */
int tflm_yolov8_od_sdlog_app(void) {

    uint32_t wakeup_event;
    uint32_t wakeup_event1;
    uint32_t freq = 0;

    hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT, &wakeup_event);
    hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT1, &wakeup_event1);

    hx_drv_swreg_aon_get_pllfreq(&freq);
    xprintf("wakeup_event=0x%x,WakeupEvt1=0x%x, freq=%d\n", wakeup_event, wakeup_event1, freq);

    pinmux_init();

    if((wakeup_event == PMU_WAKEUP_NONE) && (wakeup_event1 == PMU_WAKEUPEVENT1_NONE)) {
        /* cold boot */
    } else {
        hx_lib_pm_ctrl_fromPMUtoCPU(NULL);
    }

    hx_lib_spi_eeprom_open(USE_DW_SPI_MST_Q);
    hx_lib_spi_eeprom_enable_XIP(USE_DW_SPI_MST_Q, true, FLASH_QUAD, true);

    hx_drv_swreg_aon_get_appused1(&judge_case_data);
    g_trans_type = (judge_case_data>>16);
    g_use_case   = (judge_case_data&0xff);

#ifndef CPU_24MHZ_VERSION
    xprintf("ori_clk src info, 0x56100030=%x\n", EPII_get_memory(0x56100030));
    xprintf("ori_clk src info, 0x56100034=%x\n", EPII_get_memory(0x56100034));
    xprintf("ori_clk src info, 0x56100038=%x\n", EPII_get_memory(0x56100038));
    EPII_set_memory(0x56100030, 0x4037);
    EPII_set_memory(0x56100034, 0x0);
    EPII_set_memory(0x56100038, 0xc1b8);
    xprintf("clk src info, 0x56100030=%x\n", EPII_get_memory(0x56100030));
    xprintf("clk src info, 0x56100034=%x\n", EPII_get_memory(0x56100034));
    xprintf("clk src info, 0x56100038=%x\n", EPII_get_memory(0x56100038));
#endif

#ifdef __GNU__
    xprintf("__GNUC \n");
    extern char __mm_start_addr__;
    xprintf("__mm_start_addr__ address: %x\r\n", &__mm_start_addr__);
    mm_set_initial((int)(&__mm_start_addr__), 0x00200000-((int)(&__mm_start_addr__)-0x34000000));
#else
    static uint8_t mm_start_addr __attribute__((section(".bss.mm_start_addr")));
    xprintf("mm_start_addr address: %x \r\n", &mm_start_addr);
    mm_set_initial((int)(&mm_start_addr), 0x00200000-((int)(&mm_start_addr)-0x34000000));
#endif

    /* ---- Step 2: arm I2C slave callback (before events start) ---- */
    i2c_cmd_init();

    /* ---- Step 3: SD mount happens inside app_start_state(), after the
     * camera is initialized and streaming — see the comment there. It
     * can't happen here because PB2/PB3 are shared between I2C_M (camera
     * control) and SPI_M (SD card), and cisdp_sensor_init()/_start() need
     * them as I2C_M. ---- */

    if(g_use_case == 0) {
        xprintf("YOLOv8n object detection (sdlog)\n");
#ifdef EN_ALGO
        cv_yolov8n_ob_init(true, true, YOLOV8_OBJECT_DETECTION_FLASH_ADDR);
#endif
        /* NOTE: app_start_state -> event_handler_start -> hx_eventloop_start
         * is an infinite loop — nothing after this call executes. */
        app_start_state(APP_STATE_ALLON_YOLOV8N_OB);
    }

    return 0;
}
