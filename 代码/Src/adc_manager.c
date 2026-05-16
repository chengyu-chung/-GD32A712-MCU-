#include "adc_manager.h"
#include "gd32a7xx_adc.h"
#include "gd32a7xx_gpio.h"
#include "gd32a7xx_rcu.h"

/*
 * ============================================================================
 * 模块名称 : adc_manager
 * 文件功能 : ADC 统一采样管理（单次触发、轮询取数）
 * 设计目标 :
 *   1) 统一初始化 ADC0 与对应模拟输入 GPIO；
 *   2) 提供按“逻辑通道”读取 12bit 原始码值接口；
 *   3) 提供按参考电压换算 mV 接口，便于上层做阈值判断与控制。
 *
 * 术语说明 :
 *   - raw_12bit : ADC 原始码值，范围 0~4095（12bit，LSB）。
 *   - vref_mv   : ADC 参考电压，单位 mV（毫伏）。
 *   - mv        : 换算后的输入电压，单位 mV。
 * ============================================================================
 */

/*
 * 轮询等待转换完成的超时计数阈值（软件循环次数）。
 * 说明：非时间单位，实际等待时长受 CPU 主频与编译优化影响。
 */
#define ADC_MANAGER_TIMEOUT             100000U

/*
 * 本模块受管逻辑通道数量（与 s_cfg[] 条目数严格一致）。
 * 单位：个。
 */
#define ADC_MANAGER_CHANNEL_COUNT       5U

/*
 * 默认采样时间配置值（对应厂商库采样时间枚举/编码）。
 * 单位：ADC 采样周期档位编码（非直接物理时间单位）。
 */
#define ADC_MANAGER_SAMPLE_TIME_DEFAULT 15U

/*
 * 模块初始化状态位：
 *   0U = 未初始化；
 *   1U = 已初始化。
 * 用途：防止重复初始化 ADC 与 GPIO，避免运行时重配置风险。
 */
static uint8_t s_inited = 0U;

/*
 * 通道配置表（只读）：建立“逻辑通道 -> 硬件资源”的静态映射关系。
 * 字段含义：
 *   [0] channel      : 逻辑通道 ID（adc_manager_channel_t）；
 *   [1] gpio_rcu     : GPIO 外设时钟门控 ID；
 *   [2] gpio_port    : GPIO 端口基址；
 *   [3] gpio_pin     : GPIO 引脚掩码；
 *   [4] adc_channel  : ADC 硬件通道号；
 *   [5] sample_time  : ADC 采样时间配置值；
 *   [6] routine_rank : 常规序列 Rank（顺序号，从 0 开始）。
 *
 * 工程约束：
 *   - 若修改通道数量/映射，需同步校核 ADC_MANAGER_CHANNEL_COUNT；
 *   - Rank 必须连续且不重复。
 */
static const adc_manager_channel_cfg_t s_cfg[ADC_MANAGER_CHANNEL_COUNT] = {
    {ADC_MANAGER_CH_MQ9_GAS,        RCU_GPIOE, GPIOE, GPIO_PIN_5,  4U,  ADC_MANAGER_SAMPLE_TIME_DEFAULT, 0U},
    {ADC_MANAGER_CH_FAN_CURRENT,    RCU_GPIOE, GPIOE, GPIO_PIN_6,  3U,  ADC_MANAGER_SAMPLE_TIME_DEFAULT, 1U},
    {ADC_MANAGER_CH_PUMP_CURRENT,   RCU_GPIOB, GPIOB, GPIO_PIN_11, 15U, ADC_MANAGER_SAMPLE_TIME_DEFAULT, 2U},
    {ADC_MANAGER_CH_COOLER_CURRENT, RCU_GPIOH, GPIOH, GPIO_PIN_7,  13U, ADC_MANAGER_SAMPLE_TIME_DEFAULT, 3U},
    {ADC_MANAGER_CH_GAS_CURRENT,    RCU_GPIOH, GPIOH, GPIO_PIN_8,  12U, ADC_MANAGER_SAMPLE_TIME_DEFAULT, 4U}
};

/*
 * 函数名称 : adc_manager_find
 * 功能描述 : 在静态配置表中查找指定逻辑通道的硬件映射配置。
 * 输入参数 :
 *   - channel : 逻辑通道 ID（adc_manager_channel_t）。
 * 输出参数 : 无。
 * 返 回 值 :
 *   - 非空指针：查找成功，返回对应配置项地址；
 *   - 0       ：查找失败（通道无效或未配置）。
 * 时序/约束 :
 *   - 仅内部使用；
 *   - 线性查找，复杂度 O(N)。
 */
static const adc_manager_channel_cfg_t *adc_manager_find(adc_manager_channel_t channel)
{
    uint32_t i;

    for(i = 0U; i < ADC_MANAGER_CHANNEL_COUNT; i++) {
        if(s_cfg[i].channel == channel) {
            return &s_cfg[i];
        }
    }

    return 0;
}

/*
 * 函数名称 : adc_manager_gpio_init_one
 * 功能描述 : 初始化单个 ADC 输入通道对应 GPIO 为模拟输入模式。
 * 输入参数 :
 *   - cfg : 通道配置指针（不可为 NULL）。
 * 输出参数 : 无。
 * 返 回 值 : 无。
 * 说明     :
 *   - 使能 GPIO 时钟；
 *   - 配置为 GPIO_MODE_ANALOG，关闭上下拉（GPIO_PUPD_NONE）。
 */
static void adc_manager_gpio_init_one(const adc_manager_channel_cfg_t *cfg)
{
    rcu_periph_clock_enable(cfg->gpio_rcu);
    gpio_mode_set(cfg->gpio_port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, cfg->gpio_pin);
}

/*
 * 函数名称 : adc_manager_init
 * 功能描述 : 初始化 ADC 管理模块及硬件资源。
 * 输入参数 : 无。
 * 输出参数 : 无。
 * 返 回 值 : 无。
 * 执行流程 :
 *   1) 判定是否已初始化，已初始化则直接返回；
 *   2) 使能 ADC0 时钟并复位 ADC；
 *   3) 配置模式、对齐方式、分辨率；
 *   4) 配置常规序列长度与各 Rank 通道；
 *   5) 配置单次转换模式；
 *   6) 执行偏移校准；
 *   7) 使能 ADC 并置初始化状态位。
 * 注意事项 :
 *   - 该函数应在系统启动阶段调用一次；
 *   - 若后续需支持连续转换或 DMA，需调整本函数模式配置。
 */
void adc_manager_init(void)
{
    uint32_t i;
    const adc_manager_channel_cfg_t *cfg;

    if(s_inited != 0U) {
        return;
    }

    rcu_periph_clock_enable(RCU_ADC0);
    adc_deinit(ADC0);

    adc_mode_config(ADC_MODE_FREE);
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_resolution_config(ADC0, ADC_RESOLUTION_12B);

    /* 官方标准库要求：先配序列长度，再配每个 rank 的通道/采样时间。 */
    adc_channel_length_config(ADC0, ADC_ROUTINE_SEQUENCE, ADC_MANAGER_CHANNEL_COUNT);

    for(i = 0U; i < ADC_MANAGER_CHANNEL_COUNT; i++) {
        cfg = &s_cfg[i];
        adc_manager_gpio_init_one(cfg);
        adc_sequence_channel_config(ADC0,
                                    ADC_ROUTINE_SEQUENCE,
                                    cfg->routine_rank,
                                    cfg->adc_channel,
                                    cfg->sample_time);
    }

    /* 单次转换模式：每次软件触发执行一次常规序列转换。 */
    adc_routine_sequence_conversion_mode_config(ADC0, ADC_ONE_SHOT_MODE);

    /* 校准：偏移校准可提升静态精度（上电后执行一次）。 */
    adc_calibration_mode_config(ADC0, ADC_CALIBRATION_OFFSET);
    (void)adc_calibration_enable(ADC0);

    adc_enable(ADC0);
    s_inited = 1U;
}

/*
 * 函数名称 : adc_manager_read_raw
 * 功能描述 : 读取指定逻辑通道的 ADC 12bit 原始码值。
 * 输入参数 :
 *   - channel   : 逻辑通道 ID（adc_manager_channel_t）。
 * 输出参数 :
 *   - raw_12bit : 输出原始码值指针，单位 LSB，范围 0~4095。
 * 返 回 值 :
 *   - 1U : 读取成功；
 *   - 0U : 读取失败（未初始化/参数无效/通道无效/超时）。
 * 超时机制 :
 *   - 采用轮询 EORC 标志位并以 ADC_MANAGER_TIMEOUT 为退出阈值。
 * 备注     :
 *   - 末尾对读取值进行 0x0FFF 掩码，确保返回值为 12bit 有效数据。
 */
uint8_t adc_manager_read_raw(adc_manager_channel_t channel, uint16_t *raw_12bit)
{
    uint32_t timeout = ADC_MANAGER_TIMEOUT;
    const adc_manager_channel_cfg_t *cfg;

    if((s_inited == 0U) || (raw_12bit == 0)) {
        return 0U;
    }

    cfg = adc_manager_find(channel);
    if(cfg == 0) {
        return 0U;
    }

    adc_flag_clear(ADC0, ADC_FLAG_EORC);
    adc_sequence_software_trigger_enable(ADC0, ADC_ROUTINE_SEQUENCE);

    while((adc_flag_get(ADC0, ADC_FLAG_EORC) == RESET) && (timeout != 0U)) {
        timeout--;
    }

    if(timeout == 0U) {
        return 0U;
    }

    *raw_12bit = adc_sequence_data_read(ADC0, ADC_ROUTINE_SEQUENCE);
    *raw_12bit &= 0x0FFFU;
    (void)cfg;
    return 1U;
}

/*
 * 函数名称 : adc_manager_read_mv
 * 功能描述 : 读取指定逻辑通道并换算为毫伏值（mV）。
 * 输入参数 :
 *   - channel : 逻辑通道 ID（adc_manager_channel_t）；
 *   - vref_mv : ADC 参考电压，单位 mV（例如 3300）。
 * 输出参数 :
 *   - mv      : 输出电压值指针，单位 mV。
 * 返 回 值 :
 *   - 1U : 读取与换算成功；
 *   - 0U : 失败（参数无效或底层读取失败）。
 * 换算公式 :
 *   mv = raw_12bit * vref_mv / 4095
 * 说明     :
 *   - 采用无符号 32bit 中间变量避免乘法溢出；
 *   - 结果为截断值（向下取整）。
 */
uint8_t adc_manager_read_mv(adc_manager_channel_t channel, uint16_t vref_mv, uint16_t *mv)
{
    uint16_t raw;
    uint32_t value_mv;

    if((mv == 0U) || (vref_mv == 0U)) {
        return 0U;
    }

    if(adc_manager_read_raw(channel, &raw) == 0U) {
        return 0U;
    }

    value_mv = ((uint32_t)raw * (uint32_t)vref_mv) / 4095U;
    *mv = (uint16_t)value_mv;
    return 1U;
}
