#include "power_manager.h"
#include "gd32a7xx.h"
#include "gd32a712_evb.h"
#include "gd32a7xx_pmu.h"
#include "gd32a7xx_rcu.h"
#include "gd32a7xx_gpio.h"
#include "gd32a7xx_exti.h"
#include "gd32a7xx_syscfg.h"

/*
 * ============================================================================
 * 模块名称 : power_manager
 * 文件功能 : 车辆熄火/唤醒电源管理实现
 * 设计目标 :
 *   1) 参考官方 PMU standby demo，完成低功耗进入流程；
 *   2) 支持车载熄火后进入 Standby；
 *   3) 支持 CAN 活动唤醒和开火信号唤醒后恢复运行。
 *
 * 设计参考 :
 *   - 官方 gd32a7xx_pmu.c 的 standby 流程；
 *   - 官方 demo 的低功耗配置与唤醒思路；
 *   - 本项目板级连接：PC13 熄火、PH12/CAN_RX 唤醒、PL3 开火唤醒。
 * ============================================================================
 */

#define POWER_GPIO_OFF_PORT       GPIOC
#define POWER_GPIO_OFF_PIN        GPIO_PIN_13
#define POWER_GPIO_OFF_RCU        RCU_GPIOC
#define POWER_GPIO_OFF_EXTI_LINE  EXTI_13
#define POWER_GPIO_OFF_PORT_SRC   EXTI_SOURCE_GPIOC
#define POWER_GPIO_OFF_PIN_SRC    EXTI_SOURCE_PIN13
#define POWER_GPIO_OFF_IRQn       EXTI10_15_IRQn

#define POWER_GPIO_WAKE_PORT      GPIOL
#define POWER_GPIO_WAKE_PIN       GPIO_PIN_3
#define POWER_GPIO_WAKE_RCU       RCU_GPIOL
#define POWER_GPIO_WAKE_EXTI_LINE EXTI_3
#define POWER_GPIO_WAKE_PORT_SRC  EXTI_SOURCE_GPIOL
#define POWER_GPIO_WAKE_PIN_SRC   EXTI_SOURCE_PIN3
#define POWER_GPIO_WAKE_IRQn      EXTI3_IRQn

#define POWER_CAN_RX_PORT         GPIOH
#define POWER_CAN_RX_PIN          GPIO_PIN_12
#define POWER_CAN_RX_RCU          RCU_GPIOH
#define POWER_CAN_RX_EXTI_LINE    EXTI_12
#define POWER_CAN_RX_PORT_SRC     EXTI_SOURCE_GPIOH
#define POWER_CAN_RX_PIN_SRC      EXTI_SOURCE_PIN12
#define POWER_CAN_RX_IRQn         EXTI10_15_IRQn

static volatile power_manager_state_t s_state = POWER_MANAGER_STATE_RUN;
static volatile uint8_t s_wakeup_pending = 0U;
static volatile uint8_t s_sleep_request = 0U;
static volatile power_manager_wakeup_source_t s_last_wakeup_source = POWER_MANAGER_WAKEUP_NONE;

/*
 * 进入 Standby 前，先把系统时钟切回 IRC48M，并关闭不必要的高速时钟源。
 * 这是官方 demo 的标准低功耗做法，目的是降低待机前的动态功耗。
 */
static void power_manager_lowpower_config(void)
{
    rcu_system_clock_source_config(RCU_CKSYSSRC_IRC48M);
    while(RCU_SCSS_IRC48M != (RCU_CFG0 & RCU_CFG0_SCSS));

    rcu_osci_off(RCU_PLL_CK);
    rcu_osci_off(RCU_HXTAL);
}

/*
 * 配置待机唤醒相关 GPIO。
 * 这里采用输入上拉方式，确保空闲态电平稳定，降低误触发概率。
 *
 * 线路说明：
 *   - PC13：熄火信号；
 *   - PH12：CAN0_RX，同时作为 CAN 活动唤醒输入；
 *   - PL3 ：开火唤醒输入。
 *
 * 说明：WKUP 引脚最终以“上升沿”作为唤醒条件，
 * 尽管 CAN 物理层活动中可能先观察到低电平跳变。
 */
static void power_manager_gpio_init(void)
{
    rcu_periph_clock_enable(POWER_GPIO_OFF_RCU);
    rcu_periph_clock_enable(POWER_GPIO_WAKE_RCU);
    rcu_periph_clock_enable(POWER_CAN_RX_RCU);
    rcu_periph_clock_enable(RCU_SYSCFG);

    gpio_mode_set(POWER_GPIO_OFF_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, POWER_GPIO_OFF_PIN);
    gpio_mode_set(POWER_GPIO_WAKE_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, POWER_GPIO_WAKE_PIN);
    gpio_mode_set(POWER_CAN_RX_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, POWER_CAN_RX_PIN);

    syscfg_exti_line_config(POWER_GPIO_OFF_PORT_SRC, POWER_GPIO_OFF_PIN_SRC);
    syscfg_exti_line_config(POWER_GPIO_WAKE_PORT_SRC, POWER_GPIO_WAKE_PIN_SRC);
    syscfg_exti_line_config(POWER_CAN_RX_PORT_SRC, POWER_CAN_RX_PIN_SRC);

    exti_init(POWER_GPIO_OFF_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_init(POWER_GPIO_WAKE_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_init(POWER_CAN_RX_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_RISING);

    exti_interrupt_flag_clear(POWER_GPIO_OFF_EXTI_LINE);
    exti_interrupt_flag_clear(POWER_GPIO_WAKE_EXTI_LINE);
    exti_interrupt_flag_clear(POWER_CAN_RX_EXTI_LINE);

    nvic_irq_enable(POWER_GPIO_OFF_IRQn, 2U, 0U);
    nvic_irq_enable(POWER_GPIO_WAKE_IRQn, 2U, 0U);
    nvic_irq_enable(POWER_CAN_RX_IRQn, 2U, 0U);
}

/*
 * 配置唤醒源。
 * 这里遵循官方 demo 的思路：
 *   1) 先完成板级输入脚和 EXTI 线路配置；
 *   2) 再清理 PMU 相关状态标志；
 *   3) 最后进入 Standby。
 *
 * 备注：如果后续芯片手册确认需要专用 WKUP 使能位，可在此处补充。
 */
static void power_manager_wakeup_source_config(void)
{
    power_manager_gpio_init();
    pmu_backup_write_enable();
    pmu_all_flags_clear();
    pmu_backup_write_disable();
}

/*
 * 电源管理模块初始化。
 * 只负责建立状态机基础状态，不主动进入低功耗。
 */
void power_manager_init(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    s_state = POWER_MANAGER_STATE_RUN;
    s_wakeup_pending = 0U;
    s_sleep_request = 0U;
    s_last_wakeup_source = POWER_MANAGER_WAKEUP_NONE;
}

/* 请求进入待机。 */
void power_manager_request_sleep(void)
{
    s_sleep_request = 1U;
}

/*
 * 电源管理轮询任务。
 * 当前策略是：
 *   - 上层先发出 sleep 请求；
 *   - 这里把请求转换成 READY_TO_SLEEP 状态；
 *   - 由 main.c 统一执行进入 Standby 的动作。
 */
void power_manager_task(void)
{
    if((s_state == POWER_MANAGER_STATE_RUN) && (s_sleep_request != 0U)) {
        s_state = POWER_MANAGER_STATE_READY_TO_SLEEP;
        s_sleep_request = 0U;
    }
}

/*
 * 真正进入 Standby。
 * 进入前先降时钟、再配置唤醒源，最后调用官方库的 pmu_to_standbymode()。
 */
void power_manager_enter_standby(void)
{
    power_manager_lowpower_config();
    power_manager_wakeup_source_config();
    s_state = POWER_MANAGER_STATE_SLEEPING;
    pmu_to_standbymode();
    s_state = POWER_MANAGER_STATE_WAKEUP;
    s_wakeup_pending = 1U;
}

/*
 * 唤醒后的状态恢复。
 * 这里通过唤醒时引脚电平做来源区分：
 *   - PH12 保持低电平更偏向 CAN 活动唤醒；
 *   - 否则认为是开火信号唤醒。
 */
void power_manager_on_wakeup(void)
{
    if(s_wakeup_pending == 0U) {
        return;
    }

    s_wakeup_pending = 0U;

    if(RESET == gpio_input_bit_get(POWER_GPIO_WAKE_PORT, POWER_GPIO_WAKE_PIN)) {
        s_last_wakeup_source = POWER_MANAGER_WAKEUP_CAN;
    } else {
        s_last_wakeup_source = POWER_MANAGER_WAKEUP_IGNITION;
    }

    s_state = POWER_MANAGER_STATE_RUN;
}

/* 获取当前电源状态。 */
power_manager_state_t power_manager_get_state(void)
{
    return s_state;
}

/* 获取最近一次唤醒来源。 */
power_manager_wakeup_source_t power_manager_get_wakeup_source(void)
{
    return s_last_wakeup_source;
}
