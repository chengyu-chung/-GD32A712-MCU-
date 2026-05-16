#include "motor_pwm_gd32.h"
#include "gd32a7xx.h"
#include "gd32a7xx_timer.h"
#include "gd32a7xx_gpio.h"
#include "gd32a7xx_rcu.h"
#include <string.h>

/*
 * ============================================================================
 * 模块名称 : motor_pwm_gd32
 * 文件功能 : 风扇/水泵 PWM 输出封装
 * 设计目标 :
 *   1) 统一配置定时器 PWM 输出；
 *   2) 为风扇和水泵提供占空比调节接口；
 *   3) 保存通道状态，方便上层读取当前配置。
 *
 * 硬件映射 :
 *   - PE2 -> TIMER20_CH0 -> Fan
 *   - PE4 -> TIMER20_CH1 -> Pump
 * ============================================================================
 */

#ifndef PWM_DEFAULT_FREQ_HZ
#define PWM_DEFAULT_FREQ_HZ 20000U    // 默认 PWM 频率 20 kHz，适用于大多数风扇和水泵
#endif

#ifndef PWM_TIMER_CLK_HZ
#define PWM_TIMER_CLK_HZ 80000000U    // 默认时钟频率 80 MHz，根据实际定时器时钟频率调整
#endif

/* 每个 PWM 通道的运行状态缓存。 */
static pwm_channel_state_t s_state[PWM_CH_MAX];
static uint16_t s_period;

/*
 * 计算 PWM 周期计数值。
 * 若输入频率无效，则回退到默认频率，避免除零或周期异常。
 */
/**
 * @brief 计算电机PWM定时器周期值
 *
 * 根据指定的PWM频率计算对应的定时器计数周期。如果输入频率为0，则使用默认频率。
 * 计算结果会被限制在16位无符号整数的有效范围内（1 ~ 0xFFFF）。
 *
 * @param pwm_freq_hz PWM频率，单位为Hz。若为0，则自动替换为默认频率 PWM_DEFAULT_FREQ_HZ
 * @return uint16_t 计算得到的定时器周期值，范围为 [1, 0xFFFF]
 */
static uint16_t pwm_calc_period(uint32_t pwm_freq_hz)
{
    uint32_t period;

    /* 处理非法或零频率输入，使用默认频率 */
    if(pwm_freq_hz == 0U) {
        pwm_freq_hz = PWM_DEFAULT_FREQ_HZ;
    }

    /* 根据定时器时钟频率和目标PWM频率计算周期 */
    period = PWM_TIMER_CLK_HZ / pwm_freq_hz;

    /* 确保周期值在16位定时器的有效范围内 */
    if(period == 0U) {
        period = 1U;
    }
    if(period > 0xFFFFU) {
        period = 0xFFFFU;
    }

    return (uint16_t)period;
}

/*
 * 根据占空比计算比较值。
 * 输入 100% 时直接返回周期值，保证输出接近全开。
 */
/**
 * @brief 根据占空比百分比和周期计算PWM脉冲宽度
 *
 * @param duty_percent 占空比百分比 (0-100)
 * @param period PWM周期值
 * @return uint16_t 计算得到的脉冲宽度值
 */
static uint16_t pwm_calc_pulse(uint8_t duty_percent, uint16_t period)
{
    /* 当占空比为100%时，脉冲宽度等于周期 */
    if(duty_percent >= 100U) {
        return period;
    }

    /* 使用32位中间变量防止乘法溢出，计算实际脉冲宽度 */
    return (uint16_t)(((uint32_t)period * (uint32_t)duty_percent) / 100U);
}

/*
 * 初始化 PWM 模块。
 * 执行内容：
 *   1) 清空通道状态；
 *   2) 配置 GPIO 复用功能；
 *   3) 配置 TIMER20 为 PWM 输出模式；
 *   4) 使能定时器输出。
 */
/**
 * @brief 初始化 GD32 TIMER20 用于电机 PWM 控制
 * 
 * 该函数配置 TIMER20 以产生指定频率的 PWM 信号，分别用于控制风扇（Channel 0）
 * 和水泵（Channel 1）。它包括时钟使能、GPIO 复用配置、定时器参数设置以及
 * PWM 输出通道的具体配置。
 *
 * @param pwm_freq_hz PWM 信号的频率，单位为赫兹 (Hz)
 * @return void
 */
void pwm_gd32_init(uint32_t pwm_freq_hz)
{
    timer_parameter_struct tcfg;
    timer_oc_parameter_struct ocfg;

    /* 清零全局状态结构体 */
    memset(s_state, 0, sizeof(s_state));

    /* 使能 GPIOE 和 TIMER20 的外设时钟 */
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_TIMER20);

    /* PE2 -> AF2 -> TIMER20_CH0 (Fan) */
    /* PE4 -> AF2 -> TIMER20_CH1 (Pump) */
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2 | GPIO_PIN_4);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_2 | GPIO_PIN_4);
    gpio_af_set(GPIOE, GPIO_AF_2, GPIO_PIN_2 | GPIO_PIN_4);

    /* 复位 TIMER20 寄存器到默认值 */
    timer_deinit(TIMER20);

    /* 初始化定时器基本参数结构体并配置时基 */
    timer_struct_para_init(&tcfg);
    s_period = pwm_calc_period(pwm_freq_hz);
    tcfg.prescaler = 0U;
    tcfg.alignedmode = TIMER_COUNTER_EDGE;
    tcfg.counterdirection = TIMER_COUNTER_UP;
    tcfg.period = (uint32_t)(s_period - 1U);
    tcfg.clockdivision = TIMER_CKDIV_DIV1;
    tcfg.repetitioncounter = 0U;

    timer_init(TIMER20, &tcfg);

    /* 配置输出比较参数 */
    memset(&ocfg, 0, sizeof(ocfg));
    ocfg.ocpolarity = TIMER_OC_POLARITY_HIGH;
    ocfg.outputstate = TIMER_CCX_ENABLE;
    ocfg.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    ocfg.outputnstate = TIMER_CCXN_DISABLE;
    ocfg.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    ocfg.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* 配置 Channel 0 (风扇) 的 PWM 输出模式 */
    timer_channel_output_config(TIMER20, TIMER_CH_0, &ocfg);
    timer_channel_output_pulse_value_config(TIMER20, TIMER_CH_0, 0U);
    timer_channel_output_mode_config(TIMER20, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER20, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* 配置 Channel 1 (水泵) 的 PWM 输出模式 */
    timer_channel_output_config(TIMER20, TIMER_CH_1, &ocfg);
    timer_channel_output_pulse_value_config(TIMER20, TIMER_CH_1, 0U);
    timer_channel_output_mode_config(TIMER20, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER20, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* 启用主输出、自动重装载影子寄存器并启动定时器 */
    timer_primary_output_config(TIMER20, ENABLE);
    timer_auto_reload_shadow_enable(TIMER20);
    timer_enable(TIMER20);

    /* 更新全局状态中的周期值 */
    s_state[PWM_FAN].period = s_period;
    s_state[PWM_PUMP].period = s_period;
}

/*
 * 设置指定 PWM 通道的占空比。
 * 如果通道当前未使能，则实际输出保持为 0。
 */
/**
 * @brief 设置电机PWM通道的占空比百分比
 * 
 * @param ch PWM通道标识，需小于MOTOR_PWM_CH_MAX
 * @param duty_percent 占空比百分比值(0-100)，若超过100则自动截断为100
 * 
 * @return 无返回值
 */
void pwm_set_duty_percent(pwm_channel_t ch, uint8_t duty_percent)
{
    uint16_t pulse;

    // 验证通道有效性，无效则直接返回
    if(ch >= PWM_CH_MAX) {
        return;
    }

    // 限制占空比最大值为100%
    if(duty_percent > 100U) {
        duty_percent = 100U;
    }

    s_state[ch].duty_percent = duty_percent;

    // 根据通道使能状态计算脉冲值：禁用时为0，启用时根据占空比和周期计算
    if(s_state[ch].enabled == 0U) {
        pulse = 0U;
    } else {
        pulse = pwm_calc_pulse(duty_percent, s_period);
    }

    // 根据通道类型配置对应定时器通道的脉冲值
    if(ch == PWM_FAN) {
        timer_channel_output_pulse_value_config(TIMER20, TIMER_CH_0, pulse);
    } else {
        timer_channel_output_pulse_value_config(TIMER20, TIMER_CH_1, pulse);
    }

    s_state[ch].pulse = pulse;
}


/**
 * @brief 使能或禁用指定的电机PWM通道
 *
 * @param ch   PWM通道标识，需小于 PWM_CH_MAX
 * @param enable 使能标志：非0值表示使能，0表示禁用
 */
void pwm_set_enable(pwm_channel_t ch, uint8_t enable)
{
    /* 参数有效性检查，防止数组越界 */
    if(ch >= PWM_CH_MAX) {
        return;
    }

    /* 更新通道的使能状态，并将enable值规范化为0或1 */
    s_state[ch].enabled = (enable != 0U) ? 1U : 0U;
    /* 重新应用当前占空比以触发硬件状态更新 */
    pwm_set_duty_percent(ch, s_state[ch].duty_percent);
}

/*
 * 读取 PWM 通道状态快照。
 * 适用于调试、上报或状态镜像同步。
 */
/**
 * @brief 获取指定电机PWM通道的状态
 *
 * @param ch   电机PWM通道标识
 * @param state 用于输出通道状态的指针，调用者需确保该指针有效
 */
void pwm_get_state(pwm_channel_t ch, pwm_channel_state_t *state)
{
    // 参数有效性检查：通道越界或状态指针为空时直接返回
    if((ch >= PWM_CH_MAX) || (state == NULL)) {
        return;
    }

    *state = s_state[ch];
}
