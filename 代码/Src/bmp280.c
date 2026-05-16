#include "bmp280.h"
#include "i2c_bus_gd32.h"

/*
 * ============================================================================
 * 模块名称 : bmp280
 * 文件功能 : BMP280 I2C 访问、校准参数读取与温压补偿计算
 * 说明     :
 *   - 本文件提供 BMP280 的完整补偿版驱动；
 *   - 输出真实工程量：温度(0.01°C) 与压力(Pa)；
 *   - 若硬件地址或芯片 ID 不符，初始化将失败。
 * ============================================================================
 */

/* 临时状态 */
static uint8_t s_bmp280_addr = BMP280_I2C_ADDR_0X76;
static uint8_t s_inited = 0U;
static bmp280_calib_data_t s_calib;
static int32_t s_t_fine = 0;

static uint8_t bmp280_read_u8(uint8_t reg, uint8_t *val)
{
    return i2c1_read_regs(s_bmp280_addr, reg, val, 1U);
}

static uint8_t bmp280_read_u16_le(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    if(i2c1_read_regs(s_bmp280_addr, reg, buf, 2U) == 0U) return 0U;
    *val = (uint16_t)((uint16_t)buf[1] << 8) | buf[0];
    return 1U;
}

static uint8_t bmp280_read_s16_le(uint8_t reg, int16_t *val)
{
    uint16_t tmp;
    if(bmp280_read_u16_le(reg, &tmp) == 0U) return 0U;
    *val = (int16_t)tmp;
    return 1U;
}

static uint8_t bmp280_write_u8(uint8_t reg, uint8_t val)
{
    return i2c1_write_reg(s_bmp280_addr, reg, val);
}

static uint8_t bmp280_read_coefficients(void)
{
    if(bmp280_read_u16_le(BMP280_REGISTER_DIG_T1, &s_calib.dig_T1) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_T2, &s_calib.dig_T2) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_T3, &s_calib.dig_T3) == 0U) return 0U;

    if(bmp280_read_u16_le(BMP280_REGISTER_DIG_P1, &s_calib.dig_P1) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_P2, &s_calib.dig_P2) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_P3, &s_calib.dig_P3) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_P4, &s_calib.dig_P4) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_P5, &s_calib.dig_P5) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_P6, &s_calib.dig_P6) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_P7, &s_calib.dig_P7) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_P8, &s_calib.dig_P8) == 0U) return 0U;
    if(bmp280_read_s16_le(BMP280_REGISTER_DIG_P9, &s_calib.dig_P9) == 0U) return 0U;

    return 1U;
}

static int32_t bmp280_compensate_temperature(int32_t adc_T)
{
    int32_t var1;
    int32_t var2;
    int32_t T;

    var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) * ((int32_t)s_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) * ((int32_t)s_calib.dig_T3)) >> 14;
    s_t_fine = var1 + var2;
    T = (s_t_fine * 5 + 128) >> 8;
    return T; /* 0.01°C */
}

static uint32_t bmp280_compensate_pressure(int32_t adc_P)
{
    int64_t var1;
    int64_t var2;
    int64_t p;

    var1 = ((int64_t)s_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) + ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_calib.dig_P1) >> 33;

    if(var1 == 0) {
        return 0U;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);
    return (uint32_t)p / 256U;
}

uint8_t bmp280_init(uint8_t dev_addr_7bit)
{
    uint8_t chip_id = 0U;

    s_bmp280_addr = dev_addr_7bit;

    if(bmp280_read_u8(BMP280_REGISTER_CHIPID, &chip_id) == 0U) {
        s_inited = 0U;
        return 0U;
    }
    if(chip_id != BMP280_CHIP_ID_EXPECTED) {
        s_inited = 0U;
        return 0U;
    }

    /* 软复位可选 */
    (void)bmp280_write_u8(BMP280_REGISTER_RESET, 0xB6U);

    if(bmp280_read_coefficients() == 0U) {
        s_inited = 0U;
        return 0U;
    }

    /* 温度/压力过采样 x1，正常模式 */
    if(bmp280_write_u8(BMP280_REGISTER_CONTROL, 0x27U) == 0U) {
        s_inited = 0U;
        return 0U;
    }
    /* 0x00: standby 0.5ms, filter off */
    (void)bmp280_write_u8(BMP280_REGISTER_CONFIG, 0x00U);

    s_inited = 1U;
    return 1U;
}

uint8_t bmp280_read_once(bmp280_data_t *out)
{
    uint8_t raw[6];
    int32_t adc_T;
    int32_t adc_P;

    if((out == 0) || (s_inited == 0U)) {
        return 0U;
    }

    if(i2c1_read_regs(s_bmp280_addr, BMP280_REGISTER_PRESSUREDATA, raw, 6U) == 0U) {
        out->valid = 0U;
        return 0U;
    }

    adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
    adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);

    out->temperature_centi_c = bmp280_compensate_temperature(adc_T);
    out->pressure_pa = (int32_t)bmp280_compensate_pressure(adc_P);
    out->valid = 1U;

    return 1U;
}

uint8_t bmp388_init(uint8_t dev_addr_7bit)
{
    return bmp280_init(dev_addr_7bit);
}

uint8_t bmp388_read_once(bmp280_data_t *out)
{
    return bmp280_read_once(out);
}
