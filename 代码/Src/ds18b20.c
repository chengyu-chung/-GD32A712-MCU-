#include "ds18b20.h"
#include <stddef.h>
#include <string.h>

/*
 * ============================================================================
 * 模块名称 : ds18b20
 * 文件功能 : DS18B20 1-Wire 底层驱动 + GD32 平台适配实现
 * 设计目标 :
 *   1) 提供通用 1-Wire 时序读写接口；
 *   2) 通过 GD32 GPIO 适配层管理 4 路独立 DS18B20 通道；
 *   3) 向上层提供“按通道读取温度”的统一接口。
 *
 * 单位约定 :
 *   - 温度返回值：0.1°C（temperature_tenths）；
 *   - 延时参数：us（微秒）；
 *   - GPIO 端口/引脚：由 GD32 平台库定义。
 * ============================================================================
 */

/* ----------------------------- 通用 1-Wire 驱动 ----------------------------- */
static void ds18b20_write_one_wire(ds18b20_t *dev, uint8_t level)
{
    if((dev != NULL) && (dev->bus.write_level != NULL)) {
        dev->bus.write_level(level, dev->bus.user_ctx);
    }
}

static uint8_t ds18b20_read_one_wire(ds18b20_t *dev)
{
    if((dev != NULL) && (dev->bus.read_level != NULL)) {
        return dev->bus.read_level(dev->bus.user_ctx);
    }
    return 1U;
}

static void ds18b20_delay(ds18b20_t *dev, uint32_t us)
{
    if((dev != NULL) && (dev->bus.delay_us != NULL)) {
        dev->bus.delay_us(us, dev->bus.user_ctx);
    }
}

void ds18b20_init(ds18b20_t *dev, const ds18b20_bus_ops_t *ops)
{
    if((dev == NULL) || (ops == NULL)) {
        return;
    }
    dev->bus = *ops;
}

uint8_t ds18b20_reset(ds18b20_t *dev)
{
    uint8_t presence = 0U;

    if(dev == NULL) {
        return 0U;
    }

    if(dev->bus.set_output != NULL) {
        dev->bus.set_output(dev->bus.user_ctx);
    }

    ds18b20_write_one_wire(dev, 0U);
    ds18b20_delay(dev, 480U);

    if(dev->bus.set_input != NULL) {
        dev->bus.set_input(dev->bus.user_ctx);
    }
    ds18b20_delay(dev, 70U);

    presence = (uint8_t)(ds18b20_read_one_wire(dev) == 0U ? 1U : 0U);
    ds18b20_delay(dev, 410U);
    return presence;
}

void ds18b20_write_bit(ds18b20_t *dev, uint8_t bit)
{
    if(dev == NULL) {
        return;
    }

    if(dev->bus.set_output != NULL) {
        dev->bus.set_output(dev->bus.user_ctx);
    }

    if(bit != 0U) {
        ds18b20_write_one_wire(dev, 0U);
        ds18b20_delay(dev, 6U);
        ds18b20_write_one_wire(dev, 1U);
        ds18b20_delay(dev, 64U);
    } else {
        ds18b20_write_one_wire(dev, 0U);
        ds18b20_delay(dev, 60U);
        ds18b20_write_one_wire(dev, 1U);
        ds18b20_delay(dev, 10U);
    }
}

uint8_t ds18b20_read_bit(ds18b20_t *dev)
{
    uint8_t bit = 1U;

    if(dev == NULL) {
        return 1U;
    }

    if(dev->bus.set_output != NULL) {
        dev->bus.set_output(dev->bus.user_ctx);
    }

    ds18b20_write_one_wire(dev, 0U);
    ds18b20_delay(dev, 6U);

    if(dev->bus.set_input != NULL) {
        dev->bus.set_input(dev->bus.user_ctx);
    }

    ds18b20_delay(dev, 9U);
    bit = ds18b20_read_one_wire(dev);
    ds18b20_delay(dev, 55U);
    return bit;
}

void ds18b20_write_byte(ds18b20_t *dev, uint8_t data)
{
    uint8_t i;

    if(dev == NULL) {
        return;
    }

    for(i = 0U; i < 8U; i++) {
        ds18b20_write_bit(dev, (uint8_t)(data & 0x01U));
        data >>= 1;
    }
}

uint8_t ds18b20_read_byte(ds18b20_t *dev)
{
    uint8_t i;
    uint8_t data = 0U;

    if(dev == NULL) {
        return 0U;
    }

    for(i = 0U; i < 8U; i++) {
        data >>= 1;
        if(ds18b20_read_bit(dev) != 0U) {
            data |= 0x80U;
        }
    }
    return data;
}

uint8_t ds18b20_start_conversion(ds18b20_t *dev)
{
    if(ds18b20_reset(dev) == 0U) {
        return 0U;
    }
    ds18b20_write_byte(dev, 0xCCU);
    ds18b20_write_byte(dev, 0x44U);
    return 1U;
}

uint8_t ds18b20_read_temperature(ds18b20_t *dev, int16_t *temperature_tenths)
{
    uint8_t lsb;
    uint8_t msb;
    int16_t raw;

    if((dev == NULL) || (temperature_tenths == NULL)) {
        return 0U;
    }

    if(ds18b20_reset(dev) == 0U) {
        return 0U;
    }

    ds18b20_write_byte(dev, 0xCCU);
    ds18b20_write_byte(dev, 0xBEU);

    lsb = ds18b20_read_byte(dev);
    msb = ds18b20_read_byte(dev);
    raw = (int16_t)((uint16_t)msb << 8) | lsb;
    *temperature_tenths = (int16_t)((raw * 10) / 16);
    return 1U;
}

/* ----------------------------- GD32 平台适配层 ----------------------------- */
static ds18b20_gd32_channel_handle_t s_channels[DS18B20_GD32_CHANNEL_COUNT];

static void ds18b20_gd32_set_output(void *user_ctx)
{
    ds18b20_gd32_channel_handle_t *ch = (ds18b20_gd32_channel_handle_t *)user_ctx;
    if(ch == NULL) {
        return;
    }

    gpio_mode_set(ch->gpio_port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, ch->gpio_pin);
    gpio_output_options_set(ch->gpio_port, GPIO_OTYPE_OD, GPIO_OSPEED_LEVEL_2, ch->gpio_pin);
}

static void ds18b20_gd32_set_input(void *user_ctx)
{
    ds18b20_gd32_channel_handle_t *ch = (ds18b20_gd32_channel_handle_t *)user_ctx;
    if(ch == NULL) {
        return;
    }

    gpio_mode_set(ch->gpio_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, ch->gpio_pin);
}

static void ds18b20_gd32_write_level(uint8_t level, void *user_ctx)
{
    ds18b20_gd32_channel_handle_t *ch = (ds18b20_gd32_channel_handle_t *)user_ctx;
    if((ch == NULL) || (ch->gpio_port == 0U)) {
        return;
    }

    if(level != 0U) {
        gpio_bit_set(ch->gpio_port, ch->gpio_pin);
    } else {
        gpio_bit_reset(ch->gpio_port, ch->gpio_pin);
    }
}

static uint8_t ds18b20_gd32_read_level(void *user_ctx)
{
    ds18b20_gd32_channel_handle_t *ch = (ds18b20_gd32_channel_handle_t *)user_ctx;
    if((ch == NULL) || (ch->gpio_port == 0U)) {
        return 1U;
    }

    return (gpio_input_bit_get(ch->gpio_port, ch->gpio_pin) != RESET) ? 1U : 0U;
}

static void ds18b20_gd32_delay_us(uint32_t us, void *user_ctx)
{
    volatile uint32_t i;
    (void)user_ctx;

    for(i = 0U; i < (us * 20U); i++) {
        __NOP();
    }
}

void ds18b20_gd32_adapter_init(void)
{
    static const struct {
        uint32_t port;
        uint32_t pin;
    } cfg[DS18B20_GD32_CHANNEL_COUNT] = {
        { GPIOI, GPIO_PIN_9 },
        { GPIOI, GPIO_PIN_10 },
        { GPIOI, GPIO_PIN_11 },
        { GPIOL, GPIO_PIN_0 },
    };

    uint8_t i;

    memset(s_channels, 0, sizeof(s_channels));

    rcu_periph_clock_enable(RCU_GPIOI);
    rcu_periph_clock_enable(RCU_GPIOL);

    for(i = 0U; i < DS18B20_GD32_CHANNEL_COUNT; i++) {
        s_channels[i].gpio_port = cfg[i].port;
        s_channels[i].gpio_pin = cfg[i].pin;
        s_channels[i].valid = 1U;
        s_channels[i].bus_ops.set_output = ds18b20_gd32_set_output;
        s_channels[i].bus_ops.set_input = ds18b20_gd32_set_input;
        s_channels[i].bus_ops.write_level = ds18b20_gd32_write_level;
        s_channels[i].bus_ops.read_level = ds18b20_gd32_read_level;
        s_channels[i].bus_ops.delay_us = ds18b20_gd32_delay_us;
        s_channels[i].bus_ops.user_ctx = &s_channels[i];
        ds18b20_init(&s_channels[i].dev, &s_channels[i].bus_ops);
        ds18b20_gd32_set_input(&s_channels[i]);
    }
}

uint8_t ds18b20_gd32_read_temperature(ds18b20_gd32_channel_t channel, int16_t *temperature_tenths)
{
    if((channel >= DS18B20_GD32_CH_MAX) || (temperature_tenths == NULL) || (s_channels[channel].valid == 0U)) {
        return 0U;
    }

    return ds18b20_read_temperature(&s_channels[channel].dev, temperature_tenths);
}

uint8_t ds18b20_gd32_read_all(int16_t temperature_tenths[DS18B20_GD32_CHANNEL_COUNT])
{
    uint8_t i;
    uint8_t ok = 1U;
    int16_t temp;

    if(temperature_tenths == NULL) {
        return 0U;
    }

    for(i = 0U; i < DS18B20_GD32_CHANNEL_COUNT; i++) {
        if(ds18b20_gd32_read_temperature((ds18b20_gd32_channel_t)i, &temp) != 0U) {
            temperature_tenths[i] = temp;
        } else {
            temperature_tenths[i] = 0;
            ok = 0U;
        }
    }

    return ok;
}
