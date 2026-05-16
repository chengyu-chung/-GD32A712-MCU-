#include "temperature_collect.h"
#include <string.h>

/*
 * ============================================================================
 * 模块名称 : temperature_collect
 * 文件功能 : 多路温度采集与统计封装
 * 设计目标 :
 *   1) 统一调用底层温度读取回调；
 *   2) 统计平均值、最大值、最小值；
 *   3) 对外提供一份稳定的温度采样结果快照。
 *
 * 单位约定 :
 *   - 温度值统一使用 0.1°C（十倍摄氏度）；
 *   - 采样通道数由 TEMPERATURE_COLLECT_CHANNEL_COUNT 决定。
 * ============================================================================
 */

static temperature_collect_config_t s_cfg;
static temperature_collect_status_t s_status;
static uint8_t s_enabled;

/*
 * 载入安全默认配置。
 * 当前实现采用清零方式，避免未初始化字段影响采样逻辑。
 */
static void temperature_collect_apply_safe_defaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
}

/*
 * 初始化温度采集模块。
 * 执行内容：
 *   1) 清空状态；
 *   2) 装载默认配置；
 *   3) 拷贝上层配置；
 *   4) 使能采样功能。
 */
void temperature_collect_init(const temperature_collect_config_t *config)
{
    memset(&s_status, 0, sizeof(s_status));
    temperature_collect_apply_safe_defaults();

    if(config != NULL) {
        s_cfg = *config;
    }

    s_enabled = 1U;
    s_status.valid = 0U;
    s_status.sample_count = 0U;
    s_status.valid_channel_count = 0U;
    s_status.average_tenths_c = 0;
    s_status.max_tenths_c = 0;
    s_status.min_tenths_c = 0;
}

/*
 * 使能/关闭温度采集功能。
 * 关闭后采样函数会直接返回安全默认值。
 */
void temperature_collect_set_enabled(uint8_t enable)
{
    s_enabled = (enable != 0U) ? 1U : 0U;
}

/*
 * 执行一次完整温度采样。
 * 处理流程：
 *   1) 遍历所有温度通道；
 *   2) 调用底层回调读取每路温度；
 *   3) 统计有效通道数量、平均值、最大值、最小值；
 *   4) 将结果写入内部状态快照。
 */
void temperature_collect_sample_once(void)
{
    uint8_t i;
    uint8_t ok_count = 0U;
    int32_t sum = 0;
    int16_t temp = 0;
    int16_t min_temp = 32767;
    int16_t max_temp = -32768;

    s_status.sample_count = TEMPERATURE_COLLECT_CHANNEL_COUNT;
    s_status.valid_channel_count = 0U;

    if((s_enabled == 0U) || (s_cfg.read_temperature == NULL)) {
        s_status.valid = 0U;
        s_status.average_tenths_c = 0;
        s_status.max_tenths_c = 0;
        s_status.min_tenths_c = 0;
        memset(s_status.raw_tenths_c, 0, sizeof(s_status.raw_tenths_c));
        return;
    }

    for(i = 0U; i < TEMPERATURE_COLLECT_CHANNEL_COUNT; i++) {
        if(s_cfg.read_temperature(i, &temp, s_cfg.user_ctx) != 0U) {
            s_status.raw_tenths_c[i] = temp;
            sum += temp;
            if(temp < min_temp) {
                min_temp = temp;
            }
            if(temp > max_temp) {
                max_temp = temp;
            }
            ok_count++;
        } else {
            s_status.raw_tenths_c[i] = 0;
        }
    }

    s_status.valid_channel_count = ok_count;
    s_status.valid = (ok_count == TEMPERATURE_COLLECT_CHANNEL_COUNT) ? 1U : 0U;

    if(ok_count != 0U) {
        s_status.average_tenths_c = (int16_t)(sum / (int32_t)ok_count);
        s_status.max_tenths_c = max_temp;
        s_status.min_tenths_c = min_temp;
    } else {
        s_status.average_tenths_c = 0;
        s_status.max_tenths_c = 0;
        s_status.min_tenths_c = 0;
    }
}

/*
 * 获取温度采样状态快照。
 * 上层可直接复制一份结果用于显示、上报或诊断。
 */
void temperature_collect_get_status(temperature_collect_status_t *status)
{
    if(status != NULL) {
        *status = s_status;
    }
}
