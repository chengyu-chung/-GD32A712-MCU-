#include "fault_manager.h"
#include "adc_manager.h"
#include <string.h>

/*
 * ============================================================================
 * 模块名称 : fault_manager
 * 文件功能 : 故障检测与故障码管理实现
 * 设计目标 :
 *   1) 统一执行器过流、传感器异常、环境传感器异常判定；
 *   2) 提供故障锁存、计数防抖与位图输出；
 *   3) 让 battery_thermal 专注于状态机与控制策略。
 * ============================================================================
 */

#ifndef FAULT_TEMP_MAX_COUNT
#define FAULT_TEMP_MAX_COUNT 3U
#endif
#ifndef FAULT_CURRENT_MIN_RAW
#define FAULT_CURRENT_MIN_RAW 50U
#endif
#ifndef FAULT_CURRENT_MAX_RAW
#define FAULT_CURRENT_MAX_RAW 3800U
#endif
#ifndef FAULT_CURRENT_SAMPLE_COUNT
#define FAULT_CURRENT_SAMPLE_COUNT 3U
#endif
#ifndef FAULT_SENSOR_STABLE_LIMIT
#define FAULT_SENSOR_STABLE_LIMIT 8U
#endif
#ifndef FAULT_SENSOR_STABLE_DELTA
#define FAULT_SENSOR_STABLE_DELTA 0
#endif

#define FAULT_TEMP_SENSOR_BIT      (1U << 0)
#define FAULT_FAN_CURRENT_BIT      (1U << 1)
#define FAULT_PUMP_CURRENT_BIT     (1U << 2)
#define FAULT_COOLER_CURRENT_BIT   (1U << 3)
#define FAULT_GAS_CURRENT_BIT      (1U << 4)
#define FAULT_COMM_BIT             (1U << 5)
#define FAULT_GAS_SENSOR_BIT       (1U << 6)
#define FAULT_PRESSURE_SENSOR_BIT  (1U << 7)

static fault_manager_status_t s_status;
static uint8_t s_temp_fail_count[TEMPERATURE_COLLECT_CHANNEL_COUNT];
static uint8_t s_temp_last_valid[TEMPERATURE_COLLECT_CHANNEL_COUNT];
static int16_t s_temp_last_value[TEMPERATURE_COLLECT_CHANNEL_COUNT];
static uint8_t s_temp_stable_count[TEMPERATURE_COLLECT_CHANNEL_COUNT];
static uint8_t s_fan_current_fault_count;
static uint8_t s_pump_current_fault_count;
static uint8_t s_cooler_current_fault_count;
static uint8_t s_gas_current_fault_count;
static uint8_t s_pressure_fail_count;
static uint8_t s_pressure_confirm_count;
static uint8_t s_pressure_clear_count;
static uint8_t s_pressure_alarm_latched;

static uint8_t fault_eval_current(uint16_t raw)
{
    return ((raw <= FAULT_CURRENT_MIN_RAW) || (raw >= FAULT_CURRENT_MAX_RAW)) ? 1U : 0U;
}

void fault_manager_init(void)
{
    fault_manager_reset();
}

void fault_manager_reset(void)
{
    memset(&s_status, 0, sizeof(s_status));
    memset(s_temp_fail_count, 0, sizeof(s_temp_fail_count));
    memset(s_temp_last_valid, 0, sizeof(s_temp_last_valid));
    memset(s_temp_last_value, 0, sizeof(s_temp_last_value));
    memset(s_temp_stable_count, 0, sizeof(s_temp_stable_count));
    s_fan_current_fault_count = 0U;
    s_pump_current_fault_count = 0U;
    s_cooler_current_fault_count = 0U;
    s_gas_current_fault_count = 0U;
    s_pressure_fail_count = 0U;
    s_pressure_confirm_count = 0U;
    s_pressure_clear_count = 0U;
    s_pressure_alarm_latched = 0U;
}

void fault_manager_update_sensor_fault(uint8_t channel, uint8_t read_ok, int16_t value)
{
    if(channel >= TEMPERATURE_COLLECT_CHANNEL_COUNT) {
        return;
    }

    if(read_ok == 0U) {
        if(s_temp_fail_count[channel] < 255U) s_temp_fail_count[channel]++;
        if(s_temp_fail_count[channel] >= FAULT_TEMP_MAX_COUNT) {
            s_status.temp_sensor_fault = 1U;
        }
        s_temp_last_valid[channel] = 0U;
        return;
    }

    s_temp_fail_count[channel] = 0U;
    if(s_temp_last_valid[channel] != 0U) {
        int16_t diff = (value > s_temp_last_value[channel]) ? (value - s_temp_last_value[channel]) : (s_temp_last_value[channel] - value);
        if(diff <= FAULT_SENSOR_STABLE_DELTA) {
            if(s_temp_stable_count[channel] < 255U) s_temp_stable_count[channel]++;
        } else {
            s_temp_stable_count[channel] = 0U;
        }
        if(s_temp_stable_count[channel] >= FAULT_SENSOR_STABLE_LIMIT) {
            s_status.temp_sensor_fault = 1U;
        }
    }
    s_temp_last_valid[channel] = 1U;
    s_temp_last_value[channel] = value;
}

void fault_manager_update_env_fault(uint8_t gas_alarm, uint8_t gas_ok, uint8_t pressure_ok, uint8_t pressure_alarm, int32_t pressure_pa)
{
    (void)pressure_pa;

    s_status.gas_sensor_fault = (gas_ok == 0U) ? 1U : 0U;
    s_status.pressure_sensor_fault = (pressure_ok == 0U) ? 1U : 0U;
    (void)gas_alarm;

    if(pressure_alarm != 0U) {
        if(s_pressure_confirm_count < 255U) s_pressure_confirm_count++;
        s_pressure_clear_count = 0U;
    } else {
        if(s_pressure_clear_count < 255U) s_pressure_clear_count++;
        s_pressure_confirm_count = 0U;
    }

    if(s_pressure_confirm_count >= 3U) {
        s_pressure_alarm_latched = 1U;
    }
    if((s_pressure_alarm_latched != 0U) && (s_pressure_clear_count >= 5U)) {
        s_pressure_alarm_latched = 0U;
    }
}

void fault_manager_update_current_faults(uint16_t fan_raw, uint16_t pump_raw, uint16_t cooler_raw, uint16_t gas_raw)
{
    if(fault_eval_current(fan_raw) != 0U) {
        if(s_fan_current_fault_count < 255U) s_fan_current_fault_count++;
    } else {
        s_fan_current_fault_count = 0U;
    }

    if(fault_eval_current(pump_raw) != 0U) {
        if(s_pump_current_fault_count < 255U) s_pump_current_fault_count++;
    } else {
        s_pump_current_fault_count = 0U;
    }

    if(fault_eval_current(cooler_raw) != 0U) {
        if(s_cooler_current_fault_count < 255U) s_cooler_current_fault_count++;
    } else {
        s_cooler_current_fault_count = 0U;
    }

    if(fault_eval_current(gas_raw) != 0U) {
        if(s_gas_current_fault_count < 255U) s_gas_current_fault_count++;
    } else {
        s_gas_current_fault_count = 0U;
    }

    s_status.fan_current_fault = (s_fan_current_fault_count >= FAULT_CURRENT_SAMPLE_COUNT) ? 1U : 0U;
    s_status.pump_current_fault = (s_pump_current_fault_count >= FAULT_CURRENT_SAMPLE_COUNT) ? 1U : 0U;
    s_status.cooler_current_fault = (s_cooler_current_fault_count >= FAULT_CURRENT_SAMPLE_COUNT) ? 1U : 0U;
    s_status.gate_current_fault = (s_gas_current_fault_count >= FAULT_CURRENT_SAMPLE_COUNT) ? 1U : 0U;
}

void fault_manager_update_from_thermal_status(const battery_thermal_status_t *status)
{
    if(status == NULL) {
        return;
    }

    s_status.temp_sensor_fault = status->temp_sensor_fault;
    s_status.fan_current_fault = status->fan_current_fault;
    s_status.pump_current_fault = status->pump_current_fault;
    s_status.cooler_current_fault = status->cooler_current_fault;
    s_status.gate_current_fault = status->gate_current_fault;
    s_status.gas_sensor_fault = status->gas_sensor_fault;
    s_status.pressure_sensor_fault = status->pressure_sensor_fault;
    s_status.comm_fault = 0U;
}

void fault_manager_set_comm_fault(uint8_t fault)
{
    s_status.comm_fault = (fault != 0U) ? 1U : 0U;
}

void fault_manager_set_comm_timeout(uint8_t timeout)
{
    if(timeout != 0U) {
        s_status.comm_fault = 1U;
    }
}

uint16_t fault_manager_get_bitmap(void)
{
    uint16_t bitmap = 0U;

    if(s_status.temp_sensor_fault != 0U) bitmap |= FAULT_TEMP_SENSOR_BIT;
    if(s_status.fan_current_fault != 0U) bitmap |= FAULT_FAN_CURRENT_BIT;
    if(s_status.pump_current_fault != 0U) bitmap |= FAULT_PUMP_CURRENT_BIT;
    if(s_status.cooler_current_fault != 0U) bitmap |= FAULT_COOLER_CURRENT_BIT;
    if(s_status.gate_current_fault != 0U) bitmap |= FAULT_GAS_CURRENT_BIT;
    if(s_status.comm_fault != 0U) bitmap |= FAULT_COMM_BIT;
    if(s_status.gas_sensor_fault != 0U) bitmap |= FAULT_GAS_SENSOR_BIT;
    if(s_status.pressure_sensor_fault != 0U) bitmap |= FAULT_PRESSURE_SENSOR_BIT;

    return bitmap;
}

void fault_manager_get_status(fault_manager_status_t *status)
{
    if(status != NULL) {
        *status = s_status;
    }
}
