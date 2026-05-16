#include "gd32a7xx.h"
#include "systick.h"

/*
 * 模块：系统时基
 * 作用：
 * 1) 提供 1ms 周期中断节拍
 * 2) 提供阻塞式毫秒延时（仅初始化阶段建议使用）
 * 3) 提供系统运行毫秒计数
 */
volatile static uint32_t delay;
volatile static uint32_t g_systick_ms;

/* 配置 SysTick 为 1ms 节拍 */
void systick_config(void)
{
    if(SysTick_Config(SystemCoreClock / 1000U)) {
        while(1) {
        }
    }
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
    g_systick_ms = 0U;
}
void delay_1ms(uint32_t count)
{
    delay = count;
    while(0U != delay) {
    }
}
void delay_decrement(void)
{
    if(0U != delay) {
        delay--;
    }
    g_systick_ms++;
}
uint32_t systick_get_ms(void)
{
    return g_systick_ms;
}

/* 说明：SysTick_Handler 由工程中的 gd32a7xx_it.c 统一实现。 */