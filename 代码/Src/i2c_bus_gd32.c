#include "i2c_bus_gd32.h"
#include "gd32a7xx.h"
#include "gd32a7xx_i2c.h"
#include "gd32a7xx_gpio.h"
#include "gd32a7xx_rcu.h"

/*
 * ============================================================================
 * 模块名称 : i2c_bus_gd32
 * 文件功能 : GD32 I2C1 统一总线封装
 * 设计目标 :
 *   1) 统一完成 I2C1 及对应 GPIO 的初始化；
 *   2) 提供寄存器写入与连续读出接口；
 *   3) 封装超时等待与起停流程，减少上层业务代码重复编写。
 *
 * 适用对象 :
 *   - BMP280 等 I2C 从设备；
 *   - 其他需要挂接在 I2C1 上的外设。
 *
 * 单位与约定 :
 *   - dev_addr_7bit：7bit 从机地址；
 *   - reg：寄存器地址；
 *   - value / buf：寄存器值或连续数据；
 *   - 返回值：1=成功，0=失败。
 * ============================================================================
 */

#ifndef I2C1_TIMEOUT
#define I2C1_TIMEOUT 100000U
#endif

/*
 * I2C1 时序参数。
 * 说明：这些参数对应当前工程板级配置，若后续更换主频或速率，需重新标定。
 */
#define I2C1_PSC      7U
#define I2C1_SCLDELY  8U
#define I2C1_SDADELY  2U
#define I2C1_SCLH     19U
#define I2C1_SCLL     39U

/*
 * 等待 I2C 标志位变为“置位”。
 * 作用：
 *   - 给总线状态、发送缓冲区、接收缓冲区等硬件状态留出超时保护；
 *   - 避免程序因总线异常而无限阻塞。
 */
static uint8_t i2c1_wait_flag_set(uint32_t flag)
{
    uint32_t t = I2C1_TIMEOUT;
    while((i2c_flag_get(I2C1, flag) == RESET) && (t != 0U)) {
        t--;
    }
    return (t == 0U) ? 0U : 1U;
}

/*
 * 等待 I2C 标志位变为“清除”。
 * 常用于等待总线空闲、忙标志释放等场景。
 */
static uint8_t i2c1_wait_flag_clear(uint32_t flag)
{
    uint32_t t = I2C1_TIMEOUT;
    while((i2c_flag_get(I2C1, flag) != RESET) && (t != 0U)) {
        t--;
    }
    return (t == 0U) ? 0U : 1U;
}

/*
 * 初始化 I2C1 总线。
 * 执行内容：
 *   1) 使能 GPIOH 和 I2C1 时钟；
 *   2) 将 PH6/PH9 配置为 I2C1 SCL/SDA 复用开漏；
 *   3) 重置并配置 I2C1 时序；
 *   4) 使能模拟噪声滤波和关闭数字滤波；
 *   5) 打开 I2C1。
 */
void i2c1_bus_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_I2C1);

    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6 | GPIO_PIN_9);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_OD, GPIO_OSPEED_LEVEL_2, GPIO_PIN_6 | GPIO_PIN_9);
    gpio_af_set(GPIOH, GPIO_AF_5, GPIO_PIN_6 | GPIO_PIN_9);

    i2c_deinit(I2C1);
    i2c_disable(I2C1);

    i2c_timing_config(I2C1, I2C1_PSC, I2C1_SCLDELY, I2C1_SDADELY);
    i2c_master_clock_config(I2C1, I2C1_SCLH, I2C1_SCLL);
    i2c_analog_noise_filter_enable(I2C1);
    i2c_digital_noise_filter_config(I2C1, FILTER_DISABLE);
    i2c_address_config(I2C1, 0x00U, I2C_ADDFORMAT_7BITS);

    i2c_enable(I2C1);
}

/*
 * 向指定 I2C 从机写 1 个寄存器值。
 * 流程：
 *   1) 等待总线空闲；
 *   2) 发送从机地址 + 寄存器地址 + 数据；
 *   3) 等待传输完成并检查总线释放。
 */
uint8_t i2c1_write_reg(uint8_t dev_addr_7bit, uint8_t reg, uint8_t value)
{
    if(i2c1_wait_flag_clear(I2C_FLAG_I2CBSY) == 0U) {
        return 0U;
    }

    i2c_transfer_byte_number_config(I2C1, 2U);
    i2c_reload_disable(I2C1);
    i2c_automatic_end_enable(I2C1);
    i2c_master_addressing(I2C1, (uint32_t)(dev_addr_7bit << 1U), I2C_MASTER_TRANSMIT);
    i2c_start_on_bus(I2C1);

    if(i2c1_wait_flag_set(I2C_FLAG_TBE) == 0U) {
        i2c_stop_on_bus(I2C1);
        return 0U;
    }
    i2c_data_transmit(I2C1, reg);

    if(i2c1_wait_flag_set(I2C_FLAG_TBE) == 0U) {
        i2c_stop_on_bus(I2C1);
        return 0U;
    }
    i2c_data_transmit(I2C1, value);

    if(i2c1_wait_flag_set(I2C_FLAG_TC) == 0U) {
        i2c_stop_on_bus(I2C1);
        return 0U;
    }

    if(i2c1_wait_flag_clear(I2C_FLAG_I2CBSY) == 0U) {
        return 0U;
    }

    return 1U;
}

/*
 * 从指定 I2C 从机连续读取多个寄存器值。
 * 流程：
 *   1) 先写入寄存器地址；
 *   2) 再切换到读模式连续接收 len 字节；
 *   3) 超时或总线异常时返回失败。
 */
uint8_t i2c1_read_regs(uint8_t dev_addr_7bit, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if((buf == NULL) || (len == 0U)) {
        return 0U;
    }

    if(i2c1_wait_flag_clear(I2C_FLAG_I2CBSY) == 0U) {
        return 0U;
    }

    i2c_transfer_byte_number_config(I2C1, 1U);
    i2c_reload_disable(I2C1);
    i2c_automatic_end_disable(I2C1);
    i2c_master_addressing(I2C1, (uint32_t)(dev_addr_7bit << 1U), I2C_MASTER_TRANSMIT);
    i2c_start_on_bus(I2C1);

    if(i2c1_wait_flag_set(I2C_FLAG_TBE) == 0U) {
        i2c_stop_on_bus(I2C1);
        return 0U;
    }
    i2c_data_transmit(I2C1, reg);

    if(i2c1_wait_flag_set(I2C_FLAG_TC) == 0U) {
        i2c_stop_on_bus(I2C1);
        return 0U;
    }

    i2c_transfer_byte_number_config(I2C1, (len > 255U) ? 255U : (uint8_t)len);
    i2c_reload_disable(I2C1);
    i2c_automatic_end_enable(I2C1);
    i2c_master_addressing(I2C1, (uint32_t)(dev_addr_7bit << 1U), I2C_MASTER_RECEIVE);
    i2c_start_on_bus(I2C1);

    for(i = 0U; i < len; i++) {
        if(i2c1_wait_flag_set(I2C_FLAG_RBNE) == 0U) {
            i2c_stop_on_bus(I2C1);
            return 0U;
        }
        buf[i] = i2c_data_receive(I2C1);
    }

    if(i2c1_wait_flag_clear(I2C_FLAG_I2CBSY) == 0U) {
        return 0U;
    }

    return 1U;
}
