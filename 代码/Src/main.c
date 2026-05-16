#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "main.h"
#include "systick.h"
#include "battery_thermal.h"
#include "temperature_collect.h"
#include "ds18b20.h"
#include "thermal_gpio.h"
#include "motor_pwm_gd32.h"
#include "adc_manager.h"
#include "mq9.h"
#include "bmp280.h"
#include "i2c_bus_gd32.h"
#include "power_manager.h"
#include "watchdog.h"
#include "can_comm.h"
#include <string.h>

/*
 * ============================================================================
 * 文件名称 : main.c
 * 文件功能 : 系统主入口、热管理调度与车载低功耗唤醒编排
 * 说明     :
 *   1) 上电后完成 CAN、传感器、执行器和状态机初始化；
 *   2) 运行时完成温度、气体、压力监测以及风扇/水泵/冷却控制；
 *   3) 车辆熄火后进入 Standby，支持 CAN 活动和开火信号唤醒。
 * ============================================================================
 */

/*
 * 环境压力告警阈值（单位：Pa）。
 * 说明：
 *   - 压力过低或过高都视为异常；
 *   - 这里先给出工程默认值，后续可按整机实测再标定。
 *   - 连续越界确认/解除是为了避免压力值抖动导致误报警。
 *   - 大气压力约为 101325 Pa，100kPa~120kPa 的范围适用于大多数地面环境。
 */
#ifndef APP_PRESSURE_LOW_ALARM_PA
#define APP_PRESSURE_LOW_ALARM_PA 100000L  // 100kPa = 1atm，过低视为异常
#endif
#ifndef APP_PRESSURE_HIGH_ALARM_PA
#define APP_PRESSURE_HIGH_ALARM_PA 120000L  // 120kPa = 1.2atm，过高视为异常
#endif
#ifndef APP_PRESSURE_ALARM_CONFIRM_COUNT
#define APP_PRESSURE_ALARM_CONFIRM_COUNT 3U // 连续越界确认次数
#endif
#ifndef APP_PRESSURE_ALARM_CLEAR_COUNT
#define APP_PRESSURE_ALARM_CLEAR_COUNT 5U // 连续越界解除次数
#endif

/*
 * MQ9 气体告警阈值（单位：ADC raw / mV）。
 * 说明：
 *   - MQ9 的 AO 模拟输出接到 PE5；
 *   - raw 阈值用于快速判定，mV 阈值用于后续标定扩展；
 *   - 真实项目中建议根据实测气体浓度重新标定阈值。
 */
#ifndef APP_MQ9_GAS_ALARM_RAW
#define APP_MQ9_GAS_ALARM_RAW 1800U // ADC raw 阈值，约等于 1.8V
#endif
#ifndef APP_MQ9_GAS_VREF_MV
#define APP_MQ9_GAS_VREF_MV 3300U // 参考电压，单位 mV
#endif

static battery_thermal_status_t thermal_status; // 热管理状态机内部状态
static can_comm_telemetry_t can_tel;              // CAN 遥测缓存
static uint32_t last_can_report_temp_sample_count;        // 上一次温度采样的 CAN 采样次数
static uint8_t can_ready;     // CAN 遥测缓存是否就绪
static uint32_t led_blink_tick_ms;        // LED 闪烁时基（ms），用于状态指示
static uint8_t led_blink_phase;           // LED 闪烁时基状态

uint16_t dtm_can2_receive_flag = 0U;        // CAN2 接收到的数据标志位
uint16_t dtm_can5_receive_flag = 0U;        // CAN5 接收到的数据标志位

/*
 * 打开 CPU 指令缓存与数据缓存。
 * 说明：缓存可提升运行效率，属于系统启动早期的基础配置。
 */
void cache_enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}

/*
 * 获取系统运行时间，单位 ms。
 * 这个时基用于热管理状态机做采样间隔和恢复延时判断。
 */
static uint32_t app_get_tick_ms(void *user_ctx)
{
    (void)user_ctx;
    return systick_get_ms();
}

/*
 * 读取单路温度，返回值单位为 0.1°C。
 * 这里把底层 DS18B20 读数统一转成热管理状态机使用的工程单位。
 */
static uint8_t app_read_temperature(uint8_t channel, int16_t *temperature_tenths, void *user_ctx)
{
    (void)user_ctx;
    return ds18b20_gd32_read_temperature((ds18b20_gd32_channel_t)channel, temperature_tenths);  // 直接使用 DS18B20 的读数，单位为 0.1°C    
}

/*
 * MQ9 告警判据。
 * 说明：
 *   - MQ9 模块内部已经完成采样、滤波和阈值判断；
 *   - main.c 只负责读取“是否告警”的结果，不直接参与底层 ADC 运算。
 */
static uint8_t app_read_gas_alarm(void *user_ctx)
{
    (void)user_ctx;
    return mq9_get_alarm();
}

/*
 * BMP280 压力告警判据。
 * 说明：pressure_pa 是 BMP280 经过补偿后的真实压力值，单位 Pa。
 */
static uint8_t app_read_pressure_alarm(void *user_ctx)
{
    bmp280_data_t bmp_data;
    (void)user_ctx;

    if(bmp280_read_once(&bmp_data) == 0U) {
        return 0U;
    }

    return (bmp_data.valid != 0U && bmp_data.pressure_pa >= 120000) ? 1U : 0U;
}

/*
 * 读取 BMP280 真实压力值（单位 Pa）。
 * 该值会交给 battery_thermal 做更严谨的压力越界判定。
 */
static uint8_t app_read_pressure_value(int32_t *pressure_pa, void *user_ctx)
{
    bmp280_data_t bmp_data;
    (void)user_ctx;

    if((pressure_pa == NULL) || (bmp280_read_once(&bmp_data) == 0U) || (bmp_data.valid == 0U)) {
        return 0U;
    }

    *pressure_pa = bmp_data.pressure_pa;
    return 1U;
}

/*
 * 把热管理状态机的运行状态同步到 CAN 遥测缓存。
 * 这样 CAN 发送函数就不需要直接读取各个模块内部状态，职责更清晰。
 * @brief 将电池热管理状态同步至CAN TEL数据结构
 *
 * 该函数负责从内部电池热状态结构体中提取关键数据，
 * 并将其映射到用于CAN通信的全局TEL结构体中。
 *
 * @param status 指向电池热状态结构体的指针，包含最新的热管理数据、传感器读数及故障标志
 */
static void app_sync_can_tel_from_status(const battery_thermal_status_t *status)
{
    /* 同步运行、加热及报告使能状态 */
    can_tel.run_enable = status->run_enable;
    can_tel.heat_enable = status->heat_enable;
    can_tel.report_enable = status->thermal_report_enable;

    /* 同步当前热管理状态及温度数据（单位：0.1摄氏度） */
    can_tel.state = status->state;
    can_tel.avg_temp_tenth_c = status->average_temperature_c;
    can_tel.max_temp_tenth_c = status->max_temperature_c;
    can_tel.min_temp_tenth_c = status->min_temperature_c;

    /* 同步温度有效性及传感器计数 */
    can_tel.temp_valid = status->temperature_valid;
    can_tel.temp_sensor_valid_count = status->temp_sensor_valid_count;

    /* 同步气体与压力相关的报警及就绪状态 */
    can_tel.gas_alarm = status->gas_alarm;
    can_tel.pressure_alarm = status->pressure_alarm;
    can_tel.gas_ready = status->gas_ready;
    can_tel.pressure_ready = status->pressure_ready;

    /* 同步各子系统及传感器的故障标志 */
    can_tel.temp_sensor_fault = status->temp_sensor_fault;    // 温度传感器故障
    can_tel.fan_current_fault = status->fan_current_fault;    // 风扇电流故障  
    can_tel.pump_current_fault = status->pump_current_fault;  //水泵电流故障
    can_tel.cooler_current_fault = status->cooler_current_fault;      // 冷却器电流故障
    can_tel.gate_current_fault = status->gate_current_fault;    // 气体相关执行器电流故障 （这个是电磁阀,负责将电池包里面的压力泄掉）  
    can_tel.gas_sensor_fault = status->gas_sensor_fault;          // 气体传感器故障
    can_tel.pressure_sensor_fault = status->pressure_sensor_fault;        // 压力传感器故障
}

/*
 * 根据热管理状态设置执行器遥测值。
 * 注意：这里是“状态显示”和“通信上报”，不直接驱动物理硬件。
 */
/**
 * @brief 根据电池热状态同步执行器状态
 * 
 * 该函数根据传入的电池热管理状态，控制CAN报文中的各个散热及报警执行器的使能位。
 * 不同状态对应不同的散热策略，从完全关闭到全功率散热并报警。
 * 
 * @param state 电池热状态，取值包括 NORMAL, MONITOR, EMERGENCY, ESCAPE
 * @return 无返回值
 */
static void app_actuator_state(battery_thermal_state_t state)
{
    switch(state) {
    /* 正常状态：关闭所有散热和报警设备 */
    case NORMAL:
        can_tel.cooler_enable = 0U;
        can_tel.buzzer_enable = 0U;
        can_tel.gate_enable = 0U;
        can_tel.fan_enable = 0U;
        can_tel.pump_enable = 0U;
        break;

    /* 监控状态：开启风扇和水泵进行常规散热 */
    case MONITOR:
        can_tel.cooler_enable = 0U;   // 监控态不启动制冷器
        can_tel.buzzer_enable = 0U;   // 监控态不启动蜂鸣器
        can_tel.gate_enable = 0U;     // 监控态不启动电磁阀
        can_tel.fan_enable = 1U;   // 监控态开启风扇，一方面个水管降温，一方面可以给电池表面降温
        can_tel.pump_enable = 1U;   // 监控态开启水泵，提供补充水
        break;

    /* 紧急状态：增强散热，开启制冷器、风扇和水泵 */
    case EMERGENCY:
        can_tel.cooler_enable = 1U;
        can_tel.buzzer_enable = 0U;
        can_tel.gate_enable = 0U;
        can_tel.fan_enable = 1U;
        can_tel.pump_enable = 1U;
        break;

    /* 逃逸状态或默认情况：启动所有执行器，包括报警器，以应对极端过热情况 */
    case ESCAPE:
    default:
        can_tel.cooler_enable = 1U;
        can_tel.buzzer_enable = 1U;
        can_tel.gate_enable = 1U;
        can_tel.fan_enable = 1U;
        can_tel.pump_enable = 1U;
        break;
    }
}

/*
 * 按需发送 CAN 遥测报文。
 * force=1 时强制发送一次；否则仅当温度采样计数变化时发送。
 */
/**
 * @brief 发送CAN总线状态报告
 *
 * 该函数负责检查CAN通信就绪状态，并根据强制发送标志、温度采样计数变化或热管理状态，
 * 更新遥测数据并发送各类状态报告（状态、温度、环境、执行器、故障）到指定的CAN总线。
 *
 * @param force 强制发送标志。非0值表示强制发送报告，忽略采样计数变化检查。
 * @return 无
 */
static void app_send_can_reports(uint8_t force)
{
    /* 如果CAN未就绪，直接返回 */
    if(can_ready == 0U) {
        return;
    }

    /* 判断是否需要发送报告：强制发送、温度采样计数发生变化或处于ESCAPE状态 ，到这个地方s_can_ready一定等于1*/
    if((force != 0U) || (thermal_status.temp_sample_count != last_can_report_temp_sample_count) ||
       (thermal_status.state == ESCAPE)) {
        /* 更新遥测数据结构 */
        can_comm_update_telemetry(&can_tel);

        /* 发送各类CAN报告 */
        (void)send_state_report(CAN_COMM_BUS_DTM_CAN5);//
        (void)send_temp_report(CAN_COMM_BUS_DTM_CAN5);
        (void)send_env_report(CAN_COMM_BUS_DTM_CAN5);
        (void)send_actuator_report(CAN_COMM_BUS_DTM_CAN5);
        (void)send_fault_report(CAN_COMM_BUS_DTM_CAN5);

        /* 更新上次发送报告时的温度采样计数 */
        last_can_report_temp_sample_count = thermal_status.temp_sample_count;
    }
}


/**
 * @brief 进入指定的电池热管理状态，并配置相应的硬件执行器及上报状态。
 *
 * 该函数根据传入的热管理状态（NORMAL/MONITOR/EMERGENCY/ESCAPE），
 * 控制加热器、冷却器、风扇、水泵、蜂鸣器、排气扇及继电器等硬件设备的开关与PWM占空比，
 * 同时更新LED指示灯状态，重置LED闪烁计时器，并通过CAN总线发送当前状态报告。
 *
 * @param[in] state     目标电池热管理状态。
 * @param[in] user_ctx  用户上下文指针（当前未使用）。
 *
 * @return 无返回值。
 */
static void app_state_enter(battery_thermal_state_t state, void *user_ctx)
{
    (void)user_ctx;

    switch(state) {
    case NORMAL:
        /* 正常监控态：维持加热，关闭风扇/水泵，表示电池处于基础保温阶段。 */
        gpio_set_channel(GPIO_CH_HEATER, 1U); 
        gpio_set_channel(GPIO_CH_COOLER, 0U);
        gpio_set_channel(GPIO_CH_BUZZER, 0U);
        gpio_set_channel(GPIO_CH_GATE, 0U);
        gpio_set_channel(GPIO_CH_RELAY_PWR, 1U);
        pwm_set_enable(PWM_FAN, 0U);
        pwm_set_enable(PWM_PUMP, 0U);
        gd_eval_led_on(LED4);
        gd_eval_led_off(LED1);
        gd_eval_led_off(LED2);
        gd_eval_led_off(LED3);
        break;
    case MONITOR:
        /* 密切关注态：继续加热，同时开启风扇/水泵 50% PWM 做辅助热管理。 */
        gpio_set_channel(GPIO_CH_HEATER, 1U);
        gpio_set_channel(GPIO_CH_COOLER, 0U);
        gpio_set_channel(GPIO_CH_BUZZER, 0U);
        gpio_set_channel(GPIO_CH_GATE, 0U);
        pwm_set_enable(PWM_FAN, 1U);
        pwm_set_enable(PWM_PUMP, 1U);
        pwm_set_duty_percent(PWM_FAN, 50U);
        pwm_set_duty_percent(PWM_PUMP, 50U);
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED4);
        gd_eval_led_off(LED2);
        gd_eval_led_off(LED3);
        break;
        case EMERGENCY:
        /* 危险紧急态：强制关闭加热，风扇/水泵全速运行，并开启制冷侧输出。 */
        gpio_set_channel(GPIO_CH_HEATER, 0U);
        gpio_set_channel(GPIO_CH_COOLER, 1U);
        gpio_set_channel(GPIO_CH_BUZZER, 0U);
        gpio_set_channel(GPIO_CH_GATE, 0U);
        pwm_set_enable(PWM_FAN, 1U);
        pwm_set_enable(PWM_PUMP, 1U);
        pwm_set_duty_percent(PWM_FAN, 100U);
        pwm_set_duty_percent(PWM_PUMP, 100U);
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        gd_eval_led_off(LED3);
        gd_eval_led_off(LED4);
        break;
    case ESCAPE:
    default:
        /* 终极逃生态：维持全部关键执行器全速/全开，并由运行回调负责 LED 剧烈闪烁。 */
        gpio_set_channel(GPIO_CH_HEATER, 0U);
        gpio_set_channel(GPIO_CH_COOLER, 1U);
        gpio_set_channel(GPIO_CH_BUZZER, 1U);
        gpio_set_channel(GPIO_CH_GATE, 1U);
        gpio_set_channel(GPIO_CH_RELAY_PWR, 1U);
        pwm_set_enable(PWM_FAN, 1U);
        pwm_set_enable(PWM_PUMP, 1U);
        pwm_set_duty_percent(PWM_FAN, 100U);
        pwm_set_duty_percent(PWM_PUMP, 100U);
        gd_eval_led_on(LED1);
        gd_eval_led_on(LED2);
        gd_eval_led_on(LED3);
        gd_eval_led_on(LED4);
        break;
    }

    led_blink_phase = 1U;
    led_blink_tick_ms = systick_get_ms();
    can_tel.state = state;
    app_actuator_state(state);
    app_send_can_reports(1U);
}

/*
 * 热管理状态运行回调。
 * 这里主要做状态镜像同步、CAN 遥测更新和周期性上报。
 */
/**
 * @brief 运行电池热管理应用状态机，处理LED指示、CAN数据同步及传感器读取。
 *
 * @param state     当前的电池热管理状态（当前未使用）。
 * @param status    指向电池热管理状态结构体的指针，包含温度采样计数、当前状态等信息。
 * @param user_ctx  用户上下文指针（当前未使用）。
 */
static void app_state_run(battery_thermal_state_t state, const battery_thermal_status_t *status, void *user_ctx)
{
    uint16_t mq9_raw = 0U;
    uint32_t now_ms = systick_get_ms();
    (void)state;
    (void)user_ctx;

    /* 更新全局热状态并同步CAN遥测数据及执行器状态 */
    thermal_status = *status;
    app_sync_can_tel_from_status(status);
    app_actuator_state(status->state);

    /* 根据当前状态控制LED指示灯闪烁模式 */
    if(status->state == EMERGENCY) {
        /* 紧急状态下，LED4以500ms间隔闪烁 */
        if((now_ms - led_blink_tick_ms) >= 500U) {
            led_blink_tick_ms = now_ms;
            led_blink_phase ^= 1U;
        }
        if(led_blink_phase != 0U) {
            gd_eval_led_on(LED4);
        } else {
            gd_eval_led_off(LED4);
        }
    } else if(status->state == ESCAPE) {
        /* 逃生状态下，所有LED以200ms间隔同步闪烁 */
        if((now_ms - led_blink_tick_ms) >= 200U) {
            led_blink_tick_ms = now_ms;
            led_blink_phase ^= 1U;
        }
        if(led_blink_phase != 0U) {
            gd_eval_led_on(LED1);
            gd_eval_led_on(LED2);
            gd_eval_led_on(LED3);
            gd_eval_led_on(LED4);
        } else {
            gd_eval_led_off(LED1);
            gd_eval_led_off(LED2);
            gd_eval_led_off(LED3);
            gd_eval_led_off(LED4);
        }
    }

    /* 读取MQ9气体传感器原始数据并更新CAN遥测结构 */
    if(adc_manager_read_raw(ADC_MANAGER_CH_MQ9_GAS, &mq9_raw) != 0U) {
        can_tel.mq9_raw = mq9_raw;
    }

    /* 每5次温度采样发送一次CAN报告 */
    if(status->temp_sample_count != 0U) {
        if((status->temp_sample_count % 5U) == 0U) {
            app_send_can_reports(0U);
        }
    }
}

/*
 * 状态退出回调。
 * 作用：
 *   1) 清理当前状态下的 LED 闪烁控制，避免新状态继承旧状态的显示节奏；
 *   2) 作为后续扩展点，可在这里补充“退出状态时的安全收尾动作”；
 *   3) 当前不直接关闭执行器，避免状态切换过程中产生不必要的瞬态扰动。
 */
static void app_state_exit(battery_thermal_state_t state, void *user_ctx)
{
    (void)state;
    (void)user_ctx;

    /* 复位闪烁相位与计时基准，确保下一个状态从干净的显示状态开始。 */
    led_blink_phase = 0U;
    led_blink_tick_ms = systick_get_ms();
}

/**
 * @brief 处理接收到的 CAN 命令帧。
 *
 * 这个函数是 CAN 命令的“上层入口”回调：
 * 当 can_comm 模块收到一帧合法的命令帧后，会解析出 cmd / arg0 / arg1，
 * 再把它们交给这个函数来执行具体控制动作。
 *
 * 命令帧可以理解为下面这种结构（逻辑结构，不一定和底层 CAN 8 字节原始帧完全一一对应）：
 *   - cmd  : 命令字，表示“要做什么”
 *   - arg0 : 参数0，通常表示开关量或第一个控制值
 *   - arg1 : 参数1，通常用于扩展参数或第二个控制值
 *
 * 举例：
 *   - RUN_ENABLE   cmd=0x01, arg0=1  -> 允许系统运行
 *   - RUN_ENABLE   cmd=0x01, arg0=0  -> 禁止系统运行
 *   - FORCE_ESCAPE  cmd=0x07, arg0/arg1 通常不需要
 *   - QUERY_STATUS  cmd=0x09, arg0/arg1 通常不需要
 *
 * @param cmd      命令字，来自 can_comm_cmd_t，用来决定执行哪一种控制动作
 * @param arg0     命令参数 0；常见用途是 0/1 开关、阈值、模式选择等
 * @param arg1     命令参数 1；当前协议里主要预留给扩展使用
 * @param user_ctx  用户上下文指针；注册回调时传入，便于关联业务对象，当前未使用
 */
static void app_can_command_handler(uint8_t cmd, uint8_t arg0, uint8_t arg1, void *user_ctx)
{
    (void)arg1;
    (void)user_ctx;

    switch((can_comm_cmd_t)cmd) {
    /* 处理运行、加热及报告功能的使能控制 */
    case CAN_COMM_CMD_RUN_ENABLE:
        set_run_enable(arg0 ? 1U : 0U);
        break;
    case CAN_COMM_CMD_HEAT_ENABLE:
        set_heat_enable(arg0 ? 1U : 0U);
        break;
    case CAN_COMM_CMD_REPORT_ENABLE:
        set_report_enable(arg0 ? 1U : 0U);
        break;

    /* 处理系统状态的强制切换 */
    case CAN_COMM_CMD_FORCE_NORMAL:
        force_state(NORMAL);
        break;
    case CAN_COMM_CMD_FORCE_MONITOR:
        force_state(MONITOR);
        break;
    case CAN_COMM_CMD_FORCE_EMERGENCY:
        force_state(EMERGENCY);
        break;
    case CAN_COMM_CMD_FORCE_ESCAPE:
        force_state(ESCAPE);
        break;

    /* 处理故障清除及状态查询请求，触发CAN报告发送 */
    case CAN_COMM_CMD_CLEAR_FAULT:
        app_send_can_reports(1U);
        break;
    case CAN_COMM_CMD_QUERY_STATUS:
        app_send_can_reports(1U);
        break;
    case CAN_COMM_CMD_NONE:
    default:
        break;
    }
}


/**
 * @brief  主函数，负责系统初始化、外设配置、热管理模块启动及主循环调度。
 *         该系统包含电池热管理、CAN通信、传感器数据采集及低功耗电源管理功能。
 * 
 * @param  void  无输入参数
 * @return int   程序退出状态码（在嵌入式系统中通常不返回）
 */
int main(void)
{
    battery_thermal_config_t thermal_cfg;
    temperature_collect_config_t temp_cfg;
    thermal_gpio_config_t gpio_cfg;
    uint8_t standby_requested = 0U;
    uint8_t sleep_request_issued = 0U;
    power_manager_wakeup_source_t wakeup_source = POWER_MANAGER_WAKEUP_NONE;

    /* 启用缓存并配置系统滴答定时器作为时间基准 */
    cache_enable();
    systick_config();

    /* 初始化评估板LED并默认关闭，用于状态指示 */
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
    gd_eval_led_init(LED3);
    gd_eval_led_init(LED4);
    gd_eval_led_off(LED1);
    gd_eval_led_off(LED2);
    gd_eval_led_off(LED3);
    gd_eval_led_off(LED4);

    /* 初始化CAN通信模块并注册命令回调，启动看门狗以防止系统跑飞 */
    can_comm_init();
    can_comm_set_cmd_callback(app_can_command_handler, NULL);
    watchdog_init(WATCHDOG_DEFAULT_TIMEOUT_MS);

    /* 初始化各类传感器接口：DS18B20温度传感器、I2C总线、BMP280气压传感器、ADC管理器、MQ9气体传感器及PWM输出 */
    ds18b20_gd32_adapter_init();
    i2c1_bus_init();
    (void)bmp280_init(BMP280_I2C_ADDR_0X76);
    adc_manager_init();
    mq9_init(NULL);
    (void)mq9_task();
    pwm_gd32_init(20000U);

    /* 配置热管理相关的GPIO引脚，包括冷却器、加热器、蜂鸣器、门控及电源继电器，并设置默认状态 */
    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.channel_cfg[GPIO_CH_COOLER].active_level = THERMAL_GPIO_ACTIVE_LOW;
    gpio_cfg.channel_cfg[GPIO_CH_HEATER].active_level = THERMAL_GPIO_ACTIVE_LOW;
    gpio_cfg.channel_cfg[GPIO_CH_BUZZER].active_level = THERMAL_GPIO_ACTIVE_LOW;
    gpio_cfg.channel_cfg[GPIO_CH_GATE].active_level = THERMAL_GPIO_ACTIVE_LOW;
    gpio_cfg.channel_cfg[GPIO_CH_RELAY_PWR].active_level = THERMAL_GPIO_ACTIVE_LOW;
    gpio_cfg.channel_cfg[GPIO_CH_RELAY_PWR].default_enable = 1U;
    thermal_gpio_init(&gpio_cfg);
    thermal_gpio_set_all_off();
    gpio_set_channel(GPIO_CH_RELAY_PWR, 1U);

    /* 初始化温度采集模块，绑定温度读取回调函数 */
    temp_cfg.read_temperature = app_read_temperature;
    temp_cfg.user_ctx = NULL;
    temperature_collect_init(&temp_cfg);

    /* 配置并初始化电池热管理核心模块，设置传感器数量、时间获取接口、状态机回调及报警阈值 */
    
    /* 清零配置结构体，确保所有未显式赋值的字段均为默认值（0或NULL） */
    memset(&thermal_cfg, 0, sizeof(thermal_cfg));
    
    /* 设置启用的温度传感器通道总数，需与硬件实际连接及 temperature_collect 模块配置一致 */
    thermal_cfg.sensor_count = TEMPERATURE_COLLECT_CHANNEL_COUNT;
    
    /* 注册系统时间获取回调，用于状态机内部的时间片计算和延时判断 */
    thermal_cfg.get_tick_ms = app_get_tick_ms;
    
    /* 注册状态机生命周期回调：进入新状态时的处理逻辑（如硬件初始化、LED指示更新） */
    thermal_cfg.on_state_enter = app_state_enter;
    /* 注册状态机运行回调：每个周期执行的业务逻辑（如数据同步、周期性上报、LED闪烁控制） */
    thermal_cfg.on_state_run = app_state_run;
    /* 注册状态机退出回调：离开当前状态时的清理逻辑（如复位闪烁计时器） */
    thermal_cfg.on_state_exit = app_state_exit;
    
    /* 注册底层传感器读取回调函数 */
    thermal_cfg.read_temperature = app_read_temperature;   /* 读取单路温度值 */
    thermal_cfg.read_gas_alarm = app_read_gas_alarm;       /* 读取气体传感器告警状态 */
    thermal_cfg.read_pressure_alarm = app_read_pressure_alarm; /* 读取压力传感器告警状态 */
    thermal_cfg.read_pressure_value = app_read_pressure_value; /* 读取具体压力数值（Pa） */
    
    /* 配置压力告警阈值及滤波参数 */
    thermal_cfg.pressure_low_alarm_pa = APP_PRESSURE_LOW_ALARM_PA;       /* 压力下限告警阈值（Pa） */
    thermal_cfg.pressure_high_alarm_pa = APP_PRESSURE_HIGH_ALARM_PA;     /* 压力上限告警阈值（Pa） */
    thermal_cfg.pressure_alarm_confirm_count = APP_PRESSURE_ALARM_CONFIRM_COUNT; /* 压力越界确认次数，用于防抖 */
    thermal_cfg.pressure_alarm_clear_count = APP_PRESSURE_ALARM_CLEAR_COUNT;     /* 压力恢复解除确认次数，用于防抖 */
    
    /* 用户上下文指针，此处未使用特定上下文数据，设为 NULL */
    thermal_cfg.user_ctx = NULL;
    
    /* 执行电池热管理模块初始化，应用上述配置并重置内部状态 */
    battery_thermal_init(&thermal_cfg);

    /* 启用系统运行、加热及数据上报功能 */
    set_run_enable(1U);
    set_heat_enable(1U);
    set_report_enable(1U);

    while(1) {
        /* 当热管理系统处于ESCAPE状态且未发起睡眠请求时，请求进入睡眠模式 */
        if((sleep_request_issued == 0U) && (battery_thermal_get_state() == ESCAPE)) {
            sleep_request_issued = 1U;
            power_manager_request_sleep();
        }

        /* 处理电源管理任务，检查是否满足进入Standby模式的条件 */
        power_manager_task();
        if((standby_requested == 0U) && (power_manager_get_state() == POWER_MANAGER_STATE_READY_TO_SLEEP)) {
            standby_requested = 1U;
            gd_eval_led_on(LED1);
            /*
             * 熄火后进入 Standby：
             * - 先关闭业务侧非必要活动；
             * - 再进入 PMU standby；
             * - 唤醒源为 CAN 总线活动（PH12/WKUP15）或开火信号（PL3/WKUP35）。
             */
            power_manager_enter_standby();
        }

        /* 检测系统是否从Standby模式唤醒，若是则执行唤醒后的重新初始化流程 */
        if(power_manager_get_state() == POWER_MANAGER_STATE_WAKEUP) {
            power_manager_on_wakeup();
            wakeup_source = power_manager_get_wakeup_source();
            standby_requested = 0U;

            /* 唤醒后重新初始化关键业务模块，并根据唤醒来源做日志/状态区分。 */
            can_comm_init();
            can_comm_set_cmd_callback(app_can_command_handler, NULL);
            ds18b20_gd32_adapter_init();
            i2c1_bus_init();
            (void)bmp280_init(BMP280_I2C_ADDR_0X76);
            adc_manager_init();
            mq9_init(NULL);
            (void)mq9_task();
            pwm_gd32_init(20000U);
            battery_thermal_init(&thermal_cfg);
            set_run_enable(1U);
            set_heat_enable(1U);
            set_report_enable(1U);

            if(wakeup_source == POWER_MANAGER_WAKEUP_CAN) {
                gd_eval_led_on(LED2);
            } else if(wakeup_source == POWER_MANAGER_WAKEUP_IGNITION) {
                gd_eval_led_on(LED3);
            }
            gd_eval_led_off(LED1);
        }

        /* 执行电池热管理任务、CAN通信任务并喂狗，维持系统正常运行 */
        battery_thermal_task();
        can_comm_task();
        watchdog_feed();
    }
}
