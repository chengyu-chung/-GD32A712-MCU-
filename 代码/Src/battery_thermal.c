#include "battery_thermal.h"
#include "adc_manager.h"
#include "fault_manager.h"
#include <string.h>

/*
 * ============================================================================
 * 模块名称 : battery_thermal
 * 文件功能 : 电池热管理核心状态机实现
 * 核心功能 :
 *   1) 多路温度采样与统计（avg/max/min）；
 *   2) 温度阈值驱动状态迁移（NORMAL/MONITOR/EMERGENCY/ESCAPE）；
 *   3) 环境告警（气体/压力）与执行器电流故障联合判定；
 *   4) 输出运行状态快照并回调上层。
 *
 * 单位约定 :
 *   - 温度阈值、温度值：°C；
 *   - 时间相关参数：ms；
 *   - 电流采样值：ADC raw（12bit，0~4095）。
 * ============================================================================
 */

/* --------------------------- 默认配置阈值（可外部覆写） --------------------------- */
#ifndef BTM_DEFAULT_SENSOR_COUNT
#define BTM_DEFAULT_SENSOR_COUNT 4U                 /* 默认温度通道数（个） */
#endif
#ifndef BTM_DEFAULT_NORMAL_TO_MONITOR_C
#define BTM_DEFAULT_NORMAL_TO_MONITOR_C 40          /* NORMAL->MONITOR（°C） */
#endif
#ifndef BTM_DEFAULT_MONITOR_TO_EMERGENCY_C
#define BTM_DEFAULT_MONITOR_TO_EMERGENCY_C 60       /* MONITOR->EMERGENCY（°C） */
#endif
#ifndef BTM_DEFAULT_EMERGENCY_TO_ESCAPE_C
#define BTM_DEFAULT_EMERGENCY_TO_ESCAPE_C 70        /* EMERGENCY->ESCAPE（°C） */
#endif
#ifndef BTM_DEFAULT_MONITOR_TO_NORMAL_C
#define BTM_DEFAULT_MONITOR_TO_NORMAL_C 35          /* MONITOR->NORMAL（°C） */
#endif
#ifndef BTM_DEFAULT_EMERGENCY_TO_MONITOR_C
#define BTM_DEFAULT_EMERGENCY_TO_MONITOR_C 55       /* EMERGENCY->MONITOR（°C） */
#endif
#ifndef BTM_DEFAULT_ESCAPE_CONFIRM_MS
#define BTM_DEFAULT_ESCAPE_CONFIRM_MS 10000U        /* ESCAPE 确认保持时长（ms） */
#endif
#ifndef BTM_SAMPLE_INTERVAL_NORMAL_MS
#define BTM_SAMPLE_INTERVAL_NORMAL_MS 10000U        /* NORMAL 采样间隔（ms） */
#endif
#ifndef BTM_SAMPLE_INTERVAL_MONITOR_MS
#define BTM_SAMPLE_INTERVAL_MONITOR_MS 5000U        /* MONITOR 采样间隔（ms） */
#endif
#ifndef BTM_SAMPLE_INTERVAL_EMERGENCY_MS
#define BTM_SAMPLE_INTERVAL_EMERGENCY_MS 2000U      /* EMERGENCY 采样间隔（ms） */
#endif
#ifndef BTM_SAMPLE_INTERVAL_ESCAPE_MS
#define BTM_SAMPLE_INTERVAL_ESCAPE_MS 1000U         /* ESCAPE 采样间隔（ms） */
#endif
#ifndef BTM_DEFAULT_MONITOR_RECOVER_MS
#define BTM_DEFAULT_MONITOR_RECOVER_MS 10000U       /* MONITOR 恢复保持时长（ms） */
#endif
#ifndef BTM_DEFAULT_EMERGENCY_RECOVER_MS
#define BTM_DEFAULT_EMERGENCY_RECOVER_MS 20000U     /* EMERGENCY 恢复保持时长（ms） */
#endif
#ifndef BTM_DEFAULT_ENV_EVERY_N_SAMPLES
#define BTM_DEFAULT_ENV_EVERY_N_SAMPLES 2U          /* 每 N 次温采样做一次环境检查 */
#endif
#ifndef BTM_DEFAULT_MQ2_WARMUP_MS
#define BTM_DEFAULT_MQ2_WARMUP_MS 30000U            /* MQ2 预热时间（ms） */
#endif
#ifndef BTM_DEFAULT_BMP280_READY_MS
#define BTM_DEFAULT_BMP280_READY_MS 100U            /* BMP280 启动就绪时间（ms） */
#endif
#ifndef BTM_SENSOR_FAULT_MAX_COUNT
#define BTM_SENSOR_FAULT_MAX_COUNT 3U               /* 连续失败达到该值判温度传感器故障 */
#endif
#ifndef BTM_SENSOR_STABLE_LIMIT
#define BTM_SENSOR_STABLE_LIMIT 8U                  /* 连续不变化次数阈值 */
#endif
#ifndef BTM_SENSOR_STABLE_DELTA
#define BTM_SENSOR_STABLE_DELTA 0                   /* 认为“无变化”的温差阈值（°C） */
#endif

/*
 * 执行器电流故障判定已迁移到 fault_manager 模块。
 * battery_thermal 只保留状态机职责，不再直接维护电流故障计数或阈值。
 */

/* ------------------------------ 模块静态变量区 ------------------------------ */
static battery_thermal_config_t s_cfg;              /* 当前生效配置 */
static battery_thermal_status_t s_status;           /* 当前运行状态快照 */

/* 运行控制镜像位（对外 set_xxx_enable 写入） */
static uint8_t s_run_enable;
static uint8_t s_heat_enable;
static uint8_t s_report_enable;

/* 采样与状态恢复时序 */
static uint32_t s_last_sample_tick_ms;              /* 上次采样时间戳 */
static uint32_t s_below_recover_start_ms;           /* 进入恢复窗口起始时间戳 */
static uint8_t s_monitor_recover_arm;               /* MONITOR 恢复计时使能位 */
static uint8_t s_emergency_recover_arm;             /* EMERGENCY 恢复计时使能位 */
static uint8_t s_escape_confirm_arm;                /* ESCAPE 进入确认计时使能位 */
static uint32_t s_escape_confirm_start_ms;          /* ESCAPE 进入确认起始时间戳 */
static uint32_t s_boot_tick_ms;                     /* 模块启动时间戳 */

/* 故障判定由 fault_manager 统一维护。 */

/* 应用安全默认配置。 */
static void btm_apply_safe_defaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.thresholds.normal_to_monitor_c = BTM_DEFAULT_NORMAL_TO_MONITOR_C;
    s_cfg.thresholds.monitor_to_emergency_c = BTM_DEFAULT_MONITOR_TO_EMERGENCY_C;
    s_cfg.thresholds.emergency_to_escape_c = BTM_DEFAULT_EMERGENCY_TO_ESCAPE_C;
    s_cfg.thresholds.monitor_to_normal_c = BTM_DEFAULT_MONITOR_TO_NORMAL_C;
    s_cfg.thresholds.emergency_to_monitor_c = BTM_DEFAULT_EMERGENCY_TO_MONITOR_C;
    s_cfg.sensor_count = BTM_DEFAULT_SENSOR_COUNT;
    s_cfg.env_check_every_n_temp_samples = BTM_DEFAULT_ENV_EVERY_N_SAMPLES;
    s_cfg.monitor_recover_ms = BTM_DEFAULT_MONITOR_RECOVER_MS;
    s_cfg.emergency_recover_ms = BTM_DEFAULT_EMERGENCY_RECOVER_MS;
    s_cfg.mq9_warmup_ms = BTM_DEFAULT_MQ2_WARMUP_MS;
    s_cfg.bmp280_ready_ms = BTM_DEFAULT_BMP280_READY_MS;
}

/* int32 转 int16 饱和裁剪。 */
static int16_t btm_clamp_int16(int32_t value)
{
    if(value > 32767) return 32767;
    if(value < -32768) return -32768;
    return (int16_t)value;
}

/* 获取当前系统 tick（ms）。 */
static uint32_t btm_now_ms(void)
{
    if(s_cfg.get_tick_ms != NULL) {
        return s_cfg.get_tick_ms(s_cfg.user_ctx);
    }
    return 0U;
}

/* 将内部运行控制镜像同步到状态快照。 */
static void btm_sync_runtime_flags(void)
{
    s_status.run_enable = s_run_enable;
    s_status.heat_enable = s_heat_enable;
    s_status.thermal_report_enable = s_report_enable;
}

/* 获取不同状态的采样间隔（ms）。 */
static uint32_t btm_state_interval_ms(battery_thermal_state_t state)
{
    switch(state) {
    case MONITOR: return BTM_SAMPLE_INTERVAL_MONITOR_MS;
    case EMERGENCY: return BTM_SAMPLE_INTERVAL_EMERGENCY_MS;
    case ESCAPE: return BTM_SAMPLE_INTERVAL_ESCAPE_MS;
    case NORMAL:
    default: return BTM_SAMPLE_INTERVAL_NORMAL_MS;
    }
}

static void btm_state_enter(battery_thermal_state_t state)
{
    if(s_cfg.on_state_enter != NULL) s_cfg.on_state_enter(state, s_cfg.user_ctx);
}

static void btm_state_exit(battery_thermal_state_t state)
{
    if(s_cfg.on_state_exit != NULL) s_cfg.on_state_exit(state, s_cfg.user_ctx);
}

static void btm_state_run(battery_thermal_state_t state)
{
    if(s_cfg.on_state_run != NULL) s_cfg.on_state_run(state, &s_status, s_cfg.user_ctx);
}

/* 读取单路温度，未注册回调时返回默认 25°C。 */
static uint8_t btm_read_temp(uint8_t channel, int16_t *temp)
{
    if(s_cfg.read_temperature != NULL) return s_cfg.read_temperature(channel, temp, s_cfg.user_ctx);
    if(temp != NULL) *temp = 250;
    return 1U;
}

/* 统一状态切换入口：含 enter/exit 回调与恢复计时复位。 */
static void btm_enter_state(battery_thermal_state_t next)
{
    if(next != s_status.state) {
        btm_state_exit(s_status.state);
        s_status.state = next;
        s_status.state_changed = 1U;
        s_below_recover_start_ms = 0U;
        s_monitor_recover_arm = 0U;
        s_emergency_recover_arm = 0U;
        s_escape_confirm_arm = 0U;
        s_escape_confirm_start_ms = 0U;
        btm_state_enter(s_status.state);
    } else {
        s_status.state_changed = 0U;
    }
}

/*
 * 基于平均温度进行升降级判定。
 * 策略：
 *   - 升级优先（escape > emergency > monitor）；
 *   - 降级采用“阈值+保持时长”双条件防抖。
 */
static void btm_eval_temp_state_up_down(int16_t avg_temp, uint32_t now_ms)
{
    battery_thermal_state_t cur = s_status.state;

    if(cur == ESCAPE) {
        s_status.state_changed = 0U;
        return;
    }

    if(cur != EMERGENCY) {
        if(avg_temp >= s_cfg.thresholds.emergency_to_escape_c) {
            if(s_escape_confirm_arm == 0U) {
                s_escape_confirm_arm = 1U;
                s_escape_confirm_start_ms = now_ms;
            } else if((now_ms - s_escape_confirm_start_ms) >= BTM_DEFAULT_ESCAPE_CONFIRM_MS) {
                btm_enter_state(ESCAPE);
                return;
            }
        } else {
            s_escape_confirm_arm = 0U;
            s_escape_confirm_start_ms = 0U;
        }
    }

    if(avg_temp >= s_cfg.thresholds.monitor_to_emergency_c) {
        btm_enter_state(EMERGENCY);
        return;
    }
    if(avg_temp >= s_cfg.thresholds.normal_to_monitor_c) {
        btm_enter_state(MONITOR);
        return;
    }

    if(cur == MONITOR) {
        if(avg_temp < s_cfg.thresholds.monitor_to_normal_c) {
            if(s_monitor_recover_arm == 0U) {
                s_monitor_recover_arm = 1U;
                s_below_recover_start_ms = now_ms;
            } else if((now_ms - s_below_recover_start_ms) >= s_cfg.monitor_recover_ms) {
                btm_enter_state(NORMAL);
            }
        } else {
            s_monitor_recover_arm = 0U;
            s_below_recover_start_ms = 0U;
        }
    } else if(cur == EMERGENCY) {
        if(avg_temp < s_cfg.thresholds.emergency_to_monitor_c) {
            if(s_emergency_recover_arm == 0U) {
                s_emergency_recover_arm = 1U;
                s_below_recover_start_ms = now_ms;
            } else if((now_ms - s_below_recover_start_ms) >= s_cfg.emergency_recover_ms) {
                btm_enter_state(MONITOR);
            }
        } else {
            s_emergency_recover_arm = 0U;
            s_below_recover_start_ms = 0U;
        }
    } else {
        s_status.state_changed = 0U;
    }
}

/* 更新环境传感器就绪位（预热/启动窗口）。 */
static void btm_update_env_ready(uint32_t now_ms)
{
    uint32_t up_ms = now_ms - s_boot_tick_ms;
    s_status.gas_ready = (up_ms >= s_cfg.mq9_warmup_ms) ? 1U : 0U;
    s_status.pressure_ready = (up_ms >= s_cfg.bmp280_ready_ms) ? 1U : 0U;
}

/* 温度传感器故障判据已迁移到 fault_manager。 */
static void btm_update_sensor_faults(uint8_t channel, uint8_t read_ok, int16_t value)
{
    fault_manager_update_sensor_fault(channel, read_ok, value);
}

/* 将 fault_manager 的故障结果同步到热管理状态快照。 */
static void btm_sync_fault_status(void)
{
    fault_manager_status_t fault_status;

    fault_manager_get_status(&fault_status);
    s_status.temp_sensor_fault = fault_status.temp_sensor_fault;
    s_status.fan_current_fault = fault_status.fan_current_fault;
    s_status.pump_current_fault = fault_status.pump_current_fault;
    s_status.cooler_current_fault = fault_status.cooler_current_fault;
    s_status.gate_current_fault = fault_status.gate_current_fault;
    s_status.gas_sensor_fault = fault_status.gas_sensor_fault;
    s_status.pressure_sensor_fault = fault_status.pressure_sensor_fault;
}

/*
 * 环境检查到期逻辑：按 N 次温采样触发，且需满足传感器就绪。
 * 当前版本把气体/压力/电流故障判定统一交给 fault_manager。
 */
static void btm_env_check_if_due(uint32_t now_ms)
{
    uint8_t due;
    uint8_t gas_alarm = 0U;
    uint8_t gas_ok = 1U;
    uint8_t pressure_ok = 1U;
    uint8_t pressure_alarm = 0U;
    int32_t pressure_pa = 0;

    btm_update_env_ready(now_ms);

    due = ((s_cfg.env_check_every_n_temp_samples != 0U) &&
           ((s_status.temp_sample_count % s_cfg.env_check_every_n_temp_samples) == 0U)) ? 1U : 0U;

    if((due == 0U) && (s_status.env_check_pending == 0U)) return;

    if((s_status.gas_ready == 0U) || (s_status.pressure_ready == 0U)) {
        s_status.env_check_pending = 1U;
        return;
    }

    s_status.env_check_pending = 0U;

    if(s_cfg.read_gas_alarm != NULL) {
        gas_alarm = s_cfg.read_gas_alarm(s_cfg.user_ctx);
    } else {
        gas_ok = 0U;
    }
    if(s_cfg.read_pressure_alarm != NULL) {
        pressure_ok = s_cfg.read_pressure_alarm(s_cfg.user_ctx);
    } else {
        pressure_ok = 0U;
    }

    s_status.gas_alarm = gas_alarm;

    if(s_cfg.read_pressure_value != NULL) {
        if(s_cfg.read_pressure_value(&pressure_pa, s_cfg.user_ctx) != 0U) {
            pressure_alarm = ((pressure_pa <= s_cfg.pressure_low_alarm_pa) ||
                              (pressure_pa >= s_cfg.pressure_high_alarm_pa)) ? 1U : 0U;
        }
    }

    fault_manager_update_env_fault(gas_alarm, gas_ok, pressure_ok, pressure_alarm, pressure_pa);
    btm_sync_fault_status();
    s_status.pressure_alarm = pressure_alarm;

    if((s_status.gas_alarm != 0U) || (pressure_alarm != 0U)) {
        if(s_status.state == EMERGENCY) {
            /* EMERGENCY 保持，不额外切换。 */
        } else if(s_status.state != ESCAPE) {
            btm_enter_state(EMERGENCY);
        }
    }
}

void battery_thermal_set_config(const battery_thermal_config_t *config)
{
    if(config == NULL) {
        btm_apply_safe_defaults();
        return;
    }

    s_cfg = *config;
    if(s_cfg.sensor_count == 0U) s_cfg.sensor_count = BTM_DEFAULT_SENSOR_COUNT;
    if(s_cfg.sensor_count > TEMPERATURE_COLLECT_CHANNEL_COUNT) s_cfg.sensor_count = TEMPERATURE_COLLECT_CHANNEL_COUNT;
    if(s_cfg.env_check_every_n_temp_samples == 0U) s_cfg.env_check_every_n_temp_samples = BTM_DEFAULT_ENV_EVERY_N_SAMPLES;
    if(s_cfg.monitor_recover_ms == 0U) s_cfg.monitor_recover_ms = BTM_DEFAULT_MONITOR_RECOVER_MS;
    if(s_cfg.emergency_recover_ms == 0U) s_cfg.emergency_recover_ms = BTM_DEFAULT_EMERGENCY_RECOVER_MS;
    if(s_cfg.mq9_warmup_ms == 0U) s_cfg.mq9_warmup_ms = BTM_DEFAULT_MQ2_WARMUP_MS;
    if(s_cfg.bmp280_ready_ms == 0U) s_cfg.bmp280_ready_ms = BTM_DEFAULT_BMP280_READY_MS;
}

void battery_thermal_init(const battery_thermal_config_t *config)
{
    memset(&s_status, 0, sizeof(s_status));
    btm_apply_safe_defaults();
    battery_thermal_set_config(config);
    fault_manager_init();

    s_status.state = NORMAL;
    s_run_enable = 0U;
    s_heat_enable = 0U;
    s_report_enable = 1U;
    s_last_sample_tick_ms = 0U;
    s_below_recover_start_ms = 0U;
    s_monitor_recover_arm = 0U;
    s_emergency_recover_arm = 0U;
    s_boot_tick_ms = btm_now_ms();

    s_status.next_sample_interval_ms = BTM_SAMPLE_INTERVAL_NORMAL_MS;
    s_status.temp_sample_count = 0U;
    s_status.env_check_pending = 1U;
    s_status.gas_ready = 0U;
    s_status.pressure_ready = 0U;
    s_status.gas_alarm = 0U;
    s_status.pressure_alarm = 0U;
    s_escape_confirm_arm = 0U;
    s_escape_confirm_start_ms = 0U;
    btm_sync_fault_status();
    /* 电流故障状态由 fault_manager 统一维护，热管理层只保留镜像读取。 */

    btm_sync_runtime_flags();
    btm_state_enter(s_status.state);
}

void battery_thermal_get_status(battery_thermal_status_t *status)
{
    if(status != NULL) *status = s_status;
}

/**
 * @brief 设置电池热管理运行使能状态
 *
 * @param enable 使能标志。非零值表示启用，零值表示禁用
 */
void set_run_enable(uint8_t enable)
{
    // 标准化输入参数并更新内部运行使能状态
    s_run_enable = (enable != 0U) ? 1U : 0U;
    // 同步运行时标志以确保状态一致性
    btm_sync_runtime_flags();
}

void set_heat_enable(uint8_t enable)
{
    s_heat_enable = (enable != 0U) ? 1U : 0U;
    btm_sync_runtime_flags();
}

void set_report_enable(uint8_t enable)
{
    s_report_enable = (enable != 0U) ? 1U : 0U;
    btm_sync_runtime_flags();
}

battery_thermal_state_t battery_thermal_get_state(void)
{
    return s_status.state;
}

void force_state(battery_thermal_state_t state)
{
    if(state > ESCAPE) return;
    btm_enter_state(state);
}

/*
 * 周期任务主入口。
 * 执行步骤：
 *   1) 依据 run_enable 与采样周期判断是否执行本轮采样；
 *   2) 采集温度并统计；
 *   3) 温度状态评估 + 环境检查 + 执行器电流故障评估；
 *   4) 更新时间戳并执行 on_state_run 回调。
 */
void battery_thermal_task(void)
{
    uint32_t now_ms;
    uint32_t interval_ms;
    uint8_t i;
    int32_t sum = 0;
    int16_t temp = 0;
    int16_t min_temp = 32767;
    int16_t max_temp = -32768;
    uint8_t valid_count = 0U;

    s_status.temperature_count = s_cfg.sensor_count;
    btm_sync_runtime_flags();

    if(s_run_enable == 0U) {
        s_status.temperature_valid = 0U;
        s_status.temp_sensor_valid_count = 0U;
        s_status.average_temperature_c = 0;
        s_status.max_temperature_c = 0;
        s_status.min_temperature_c = 0;
        s_status.next_sample_interval_ms = BTM_SAMPLE_INTERVAL_NORMAL_MS;
        s_last_sample_tick_ms = btm_now_ms();
        fault_manager_update_from_thermal_status(&s_status);
        btm_state_run(s_status.state);
        return;
    }

    now_ms = btm_now_ms();
    interval_ms = btm_state_interval_ms(s_status.state);
    s_status.next_sample_interval_ms = interval_ms;

    if(!((s_last_sample_tick_ms == 0U) || ((now_ms - s_last_sample_tick_ms) >= interval_ms))) {
        btm_state_run(s_status.state);
        return;
    }

    memset(s_status.temperatures_c, 0, sizeof(s_status.temperatures_c));
    s_status.temp_sensor_fault = 0U;

    for(i = 0U; i < s_cfg.sensor_count; i++) {
        uint8_t read_ok = btm_read_temp(i, &temp);
        if(read_ok != 0U) {
            s_status.temperatures_c[i] = temp;
            sum += temp;
            if(temp < min_temp) min_temp = temp;
            if(temp > max_temp) max_temp = temp;
            valid_count++;
        }
        btm_update_sensor_faults(i, read_ok, temp);
    }

    s_status.temp_sensor_valid_count = valid_count;
    s_status.temperature_valid = (valid_count == s_cfg.sensor_count) ? 1U : 0U;

    if(valid_count != 0U) {
        s_status.average_temperature_c = btm_clamp_int16(sum / (int32_t)valid_count);
        s_status.max_temperature_c = max_temp;
        s_status.min_temperature_c = min_temp;
        s_status.temp_sample_count++;
        btm_eval_temp_state_up_down(s_status.average_temperature_c, now_ms);
        btm_env_check_if_due(now_ms);
    } else {
        s_status.average_temperature_c = 0;
        s_status.max_temperature_c = 0;
        s_status.min_temperature_c = 0;
        s_status.state_changed = 0U;
    }

    fault_manager_update_from_thermal_status(&s_status);
    s_last_sample_tick_ms = now_ms;
    s_status.last_sample_tick_ms = now_ms;
    btm_state_run(s_status.state);

    if((s_report_enable != 0U) && (s_cfg.log != NULL)) {
        s_cfg.log("battery_thermal_task done\r\n", s_cfg.user_ctx);
    }
}
