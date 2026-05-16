#include "watchdog.h"
#include "gd32a7xx.h"
#include "gd32a7xx_rcu.h"
#include "gd32a7xx_fwdgt.h"

/*
 * ============================================================================
 * 模块名称 : watchdog
 * 文件功能 : 独立看门狗封装实现
 * 说明     :
 *   当前使用 GD32 的 FWDGT（独立看门狗）。它不依赖系统主时钟，
 *   更适合“主循环卡死后自动复位”的场景。
 * ============================================================================
 */

/* GD32 标准外设库里 FWDGT 以 FWDGT0 作为外设实例。 */
#ifndef WATCHDOG_INSTANCE
#define WATCHDOG_INSTANCE FWDGT0
#endif

static volatile watchdog_state_t s_state = WATCHDOG_STATE_IDLE;

/*
 * 根据目标超时时间选择预分频和装载值。
 * FWDGT 计数时钟来自 IRC40K，经预分频后写入 reload 值。
 */
static void watchdog_apply_timeout(uint32_t timeout_ms)
{
    uint32_t reload_value;
    uint32_t timeout = (timeout_ms == 0U) ? WATCHDOG_DEFAULT_TIMEOUT_MS : timeout_ms;

    /* FWDGT 计数频率约为 40k / 256 = 156.25Hz，1 tick 约 6.4ms。 */
    reload_value = (timeout * 1000U) / 6400U;
    if(reload_value == 0U) {
        reload_value = 1U;
    }
    if(reload_value > 0x0FFFU) {
        reload_value = 0x0FFFU;
    }

    rcu_periph_clock_enable(RCU_FWDGT0);
    fwdgt_config(WATCHDOG_INSTANCE, reload_value, FWDGT_PSC_DIV256);
    fwdgt_enable(WATCHDOG_INSTANCE);
    (void)fwdgt_counter_reload(WATCHDOG_INSTANCE);
}

void watchdog_init(uint32_t timeout_ms)
{
    watchdog_apply_timeout(timeout_ms);
    s_state = WATCHDOG_STATE_RUNNING;
}

void watchdog_feed(void)
{
    if(s_state == WATCHDOG_STATE_RUNNING) {
        (void)fwdgt_counter_reload(WATCHDOG_INSTANCE);
    }
}

watchdog_state_t watchdog_get_state(void)
{
    return s_state;
}
