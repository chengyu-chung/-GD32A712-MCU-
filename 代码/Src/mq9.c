#include "mq9.h"
#include "adc_manager.h"
#include <string.h>

/*
 * ============================================================================
 * 模块名称 : mq9
 * 文件功能 : MQ9 可燃气体传感器采样与告警判定实现
 * 设计说明 :
 *   - 以 ADC_MANAGER_CH_MQ9_GAS 作为底层采样输入；
 *   - 默认以 raw 阈值进行判定，mV 仅作为工程量缓存；
 *   - 通过连续确认/解除计数降低瞬时抖动误报。
 * ============================================================================
 */

#define MQ9_DEFAULT_VREF_MV             3300U
#define MQ9_DEFAULT_ALARM_RAW_THRESHOLD  1800U
#define MQ9_DEFAULT_ALARM_CONFIRM_COUNT  3U
#define MQ9_DEFAULT_ALARM_CLEAR_COUNT    5U
#define MQ9_DEFAULT_WARMUP_MS            30000U
#define MQ9_DEFAULT_SAMPLE_INTERVAL_MS   1000U

static mq9_config_t s_cfg;
static mq9_status_t s_status;
static uint32_t s_boot_tick_ms;
static uint8_t s_alarm_confirm_count;
static uint8_t s_alarm_clear_count;

static uint32_t mq9_now_ms(void)
{
    return s_boot_tick_ms;
}

static void mq9_apply_defaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.vref_mv = MQ9_DEFAULT_VREF_MV;
    s_cfg.alarm_raw_threshold = MQ9_DEFAULT_ALARM_RAW_THRESHOLD;
    s_cfg.alarm_confirm_count = MQ9_DEFAULT_ALARM_CONFIRM_COUNT;
    s_cfg.alarm_clear_count = MQ9_DEFAULT_ALARM_CLEAR_COUNT;
    s_cfg.warmup_ms = MQ9_DEFAULT_WARMUP_MS;
    s_cfg.sample_interval_ms = MQ9_DEFAULT_SAMPLE_INTERVAL_MS;
}

void mq9_set_config(const mq9_config_t *config)
{
    if(config == NULL) {
        mq9_apply_defaults();
        return;
    }

    s_cfg = *config;
    if(s_cfg.vref_mv == 0U) s_cfg.vref_mv = MQ9_DEFAULT_VREF_MV;
    if(s_cfg.alarm_raw_threshold == 0U) s_cfg.alarm_raw_threshold = MQ9_DEFAULT_ALARM_RAW_THRESHOLD;
    if(s_cfg.alarm_confirm_count == 0U) s_cfg.alarm_confirm_count = MQ9_DEFAULT_ALARM_CONFIRM_COUNT;
    if(s_cfg.alarm_clear_count == 0U) s_cfg.alarm_clear_count = MQ9_DEFAULT_ALARM_CLEAR_COUNT;
    if(s_cfg.warmup_ms == 0U) s_cfg.warmup_ms = MQ9_DEFAULT_WARMUP_MS;
    if(s_cfg.sample_interval_ms == 0U) s_cfg.sample_interval_ms = MQ9_DEFAULT_SAMPLE_INTERVAL_MS;
}

void mq9_init(const mq9_config_t *config)
{
    memset(&s_status, 0, sizeof(s_status));
    s_alarm_confirm_count = 0U;
    s_alarm_clear_count = 0U;
    s_boot_tick_ms = 0U;
    mq9_apply_defaults();
    mq9_set_config(config);
}

uint8_t mq9_task(void)
{
    uint16_t raw = 0U;
    uint16_t mv = 0U;
    uint8_t now_ready;

    if(adc_manager_read_raw(ADC_MANAGER_CH_MQ9_GAS, &raw) == 0U) {
        s_status.alarm = 0U;
        return 0U;
    }

    (void)adc_manager_read_mv(ADC_MANAGER_CH_MQ9_GAS, s_cfg.vref_mv, &mv);
    s_status.raw = raw;
    s_status.mv = mv;

    /* 这里暂用上电后默认立即就绪；若后续接入时基，可扩展为真实预热判断。 */
    now_ready = 1U;
    s_status.ready = now_ready;

    if(now_ready == 0U) {
        s_status.alarm = 0U;
        return 1U;
    }

    if(raw >= s_cfg.alarm_raw_threshold) {
        if(s_alarm_confirm_count < 255U) s_alarm_confirm_count++;
        s_alarm_clear_count = 0U;
    } else {
        if(s_alarm_clear_count < 255U) s_alarm_clear_count++;
        s_alarm_confirm_count = 0U;
    }

    if(s_alarm_confirm_count >= s_cfg.alarm_confirm_count) {
        s_status.alarm = 1U;
    }
    if((s_status.alarm != 0U) && (s_alarm_clear_count >= s_cfg.alarm_clear_count)) {
        s_status.alarm = 0U;
    }

    return 1U;
}

void mq9_get_status(mq9_status_t *status)
{
    if(status != NULL) {
        *status = s_status;
    }
}

uint8_t mq9_get_alarm(void)
{
    return s_status.alarm;
}

uint16_t mq9_get_raw(void)
{
    return s_status.raw;
}

uint16_t mq9_get_mv(void)
{
    return s_status.mv;
}
