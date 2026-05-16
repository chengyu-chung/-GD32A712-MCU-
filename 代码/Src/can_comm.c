#include "can_comm.h"
#include "fault_manager.h"
#include "systick.h"
#include <string.h>

/*
 * ============================================================================
 * 模块名称 : can_comm
 * 文件功能 : 热管理 CAN FD 通信收发模块
 *
 * 一、这个模块负责什么
 *   1) 把热管理业务数据打包成 CAN FD 报文并发送出去；
 *   2) 接收来自总线的命令帧，并在校验通过后交给上层业务处理；
 *   3) 对关键报文做 CRC16 校验，避免“数据内容正确性”出问题；
 *   4) 对接收超时、丢帧、CRC 错误进行统计，并联动故障管理模块。
 *
 * 二、为什么要有应用层协议
 *   CAN 总线本身有硬件 CRC，但那只能保证“链路传输没坏”；
 *   业务层还需要自己规定：
 *   - 这帧是谁发的（ID）
 *   - 这帧是哪种业务（type）
 *   - 这帧是否重复或丢失（seq）
 *   - 这帧里有多少有效数据（payload_len）
 *   - 数据内容是否完整（CRC16）
 *
 * 三、统一帧格式
 *   [0] type        报文类型
 *   [1] seq         序号，发送一帧加 1，便于检测重复/丢帧
 *   [2] payload_len 业务数据长度
 *   [3] flags       预留控制位，目前用于扩展
 *   [4..] payload   业务数据
 *   [... ] CRC16    对前面所有字节做 CRC16-CCITT
 *
 * 四、当前支持的业务 ID
 *   0x180 状态帧   ：运行状态、使能状态、告警状态
 *   0x181 温度帧   ：平均/最高/最低温度、有效性统计
 *   0x182 环境帧   ：MQ2 原始值、气体/压力告警
 *   0x183 执行器帧 ：风扇、水泵、加热、制冷、蜂鸣器、排风
 *   0x184 故障帧   ：各类故障位图与通信故障
 *   0x185 命令帧   ：运行控制、模式切换、状态查询等
 * ============================================================================
 */

/* CAN 标准帧 ID（11bit）。 */
#define CAN_COMM_ID_STATE    0x180U
#define CAN_COMM_ID_TEMP     0x181U
#define CAN_COMM_ID_ENV      0x182U
#define CAN_COMM_ID_ACT      0x183U
#define CAN_COMM_ID_FAULT    0x184U
#define CAN_COMM_ID_CMD      0x185U

/* 统一帧头字段定义。 */
#define CAN_COMM_FRAME_TYPE_STATE     0x01U  /* 状态帧 */
#define CAN_COMM_FRAME_TYPE_TEMP      0x02U  /* 温度帧 */
#define CAN_COMM_FRAME_TYPE_ENV       0x03U  /* 环境帧 */
#define CAN_COMM_FRAME_TYPE_ACT       0x04U  /* 执行器帧 */
#define CAN_COMM_FRAME_TYPE_FAULT     0x05U  /* 故障帧 */
#define CAN_COMM_FRAME_TYPE_CMD       0x10U  /* 命令帧 */
#define CAN_COMM_FRAME_HEADER_LEN     4U     /* type + seq + payload_len + flags */
#define CAN_COMM_FRAME_CRC_LEN        2U     /* CRC16 两个字节 */
#define CAN_COMM_FLAG_ACK_REQ         0x01U  /* 预留：需要应答 */
#define CAN_COMM_FLAG_SEQ_WRAP        0xFFU  /* 序号回绕值 */

/* 接收超时、丢帧统计与载荷长度限制。 */
#define CAN_COMM_RX_TIMEOUT_MS        3000U  /* 超过该时间没收到有效命令，认为通信异常 */
#define CAN_COMM_RX_DROP_WARN_COUNT    3U     /* 连续检测到序号异常的告警阈值 */
#define CAN_COMM_MAX_PAYLOAD_BYTES    58U    /* 64 字节总长里扣除头和 CRC 后的最大载荷 */

/* 标准帧掩码（11bit）。 */
#define CAN_COMM_ID_MASK              0x7FFU

/*
 * 模块内部运行状态。
 * 说明：
 *   s_inited           模块是否完成初始化
 *   s_tx_seq           发送序号，每发一帧加 1
 *   s_rx_last_seq      上一次收到的序号，用于检测重复/跳变
 *   s_rx_seq_valid     是否已经收到过有效序号
 *   s_last_cmd_tick_ms 最近一次收到有效命令的时间戳
 *   s_rx_drop_count    序号异常累计次数
 *   s_tel              上层更新进来的遥测镜像
 *   s_cmd_cb           命令回调
 *   s_cmd_user_ctx     命令回调上下文
 */
static uint8_t s_inited = 0U;
static uint8_t s_tx_seq = 0U;
static uint8_t s_rx_last_seq = 0U;
static uint8_t s_rx_seq_valid = 0U;
static uint32_t s_last_cmd_tick_ms = 0U;
static uint8_t s_rx_drop_count = 0U;
static can_comm_telemetry_t s_tel;
static can_comm_cmd_cb_t s_cmd_cb = NULL;
static void *s_cmd_user_ctx = NULL;

/* 统一读取系统毫秒时基。 */
static uint32_t can_comm_now_ms(void)
{
    return systick_get_ms();
}

/*
 * CRC16-CCITT 逐字节更新。
 * 输入：当前 CRC 值 + 一个新字节
 * 输出：更新后的 CRC 值
 */
static uint16_t can_comm_crc16_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for(uint8_t i = 0U; i < 8U; i++) {
        if((crc & 0x8000U) != 0U) {
            crc = (uint16_t)((crc << 1) ^ CAN_COMM_CRC16_POLY);
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

/*
 * 计算完整帧的 CRC16。
 * 规则：对 type/seq/len/flags/payload 全部参与校验，
 * 不包括最后 2 字节 CRC16 自身。
 */
static uint16_t can_comm_crc16_calc(const uint8_t *data, uint8_t len)
{
    uint16_t crc = CAN_COMM_CRC16_INIT;
    if(data == NULL) {
        return crc;
    }
    for(uint8_t i = 0U; i < len; i++) {
        crc = can_comm_crc16_update(crc, data[i]);
    }
    return crc;
}

/* 发送序号自增。序号回绕到 0xFF 后继续从 0 开始。 */
static uint8_t can_comm_next_seq(void)
{
    s_tx_seq = (uint8_t)(s_tx_seq + 1U);
    return s_tx_seq;
}

/*
 * 把计算出来的 CRC16 写入缓冲区末尾。
 * 参数 len_without_crc 表示前面已经写好的数据长度。
 */
static void can_comm_write_crc16(uint8_t *frame, uint8_t len_without_crc)
{
    uint16_t crc = can_comm_crc16_calc(frame, len_without_crc);
    frame[len_without_crc] = (uint8_t)(crc >> 8);
    frame[len_without_crc + 1U] = (uint8_t)(crc & 0xFFU);
}

/*
 * 检查接收序号是否合理。
 * 作用：
 *   1) 第一次收到序号时，直接接受；
 *   2) 如果和上一次相同，认为是重复帧，直接丢弃；
 *   3) 如果不是连续递增，认为可能发生过丢帧，做一次计数；
 *   4) 连续异常达到阈值后，置通信故障。
 */
static uint8_t can_comm_check_seq(uint8_t seq)
{
    if(s_rx_seq_valid == 0U) {
        s_rx_last_seq = seq;
        s_rx_seq_valid = 1U;
        return 1U;
    }

    if(seq == s_rx_last_seq) {
        return 0U;
    }

    if((uint8_t)(s_rx_last_seq + 1U) != seq) {
        if(s_rx_drop_count < 255U) {
            s_rx_drop_count++;
        }
        if(s_rx_drop_count >= CAN_COMM_RX_DROP_WARN_COUNT) {
            fault_manager_set_comm_fault(1U);
        }
    } else {
        s_rx_drop_count = 0U;
    }

    s_rx_last_seq = seq;
    return 1U;
}

/* 收到有效命令后，刷新“最近接收时间”。 */
static void can_comm_mark_rx_ok(void)
{
    s_last_cmd_tick_ms = can_comm_now_ms();
}

/* 收到错误帧时，直接通知故障模块。 */
static void can_comm_mark_rx_fault(void)
{
    fault_manager_set_comm_fault(1U);
}

/*
 * 组装统一协议帧。
 * 输入：
 *   type        报文类型
 *   seq         序号
 *   payload     业务数据
 *   payload_len 业务数据长度
 * 输出：
 *   frame       完整 CAN FD 帧（含头部和 CRC）
 */
static void can_comm_build_frame(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t payload_len,
                                 can_comm_frame_t *frame)
{
    uint8_t total_len;

    if((frame == NULL) || (payload_len > CAN_COMM_MAX_PAYLOAD_BYTES)) {
        return;
    }

    total_len = (uint8_t)(CAN_COMM_FRAME_HEADER_LEN + payload_len + CAN_COMM_FRAME_CRC_LEN);
    memset(frame, 0, sizeof(*frame));
    frame->data[0] = type;
    frame->data[1] = seq;
    frame->data[2] = payload_len;
    frame->data[3] = 0U;
    if((payload != NULL) && (payload_len != 0U)) {
        memcpy(&frame->data[CAN_COMM_FRAME_HEADER_LEN], payload, payload_len);
    }
    can_comm_write_crc16(frame->data, (uint8_t)(CAN_COMM_FRAME_HEADER_LEN + payload_len));
    frame->data_len = total_len;
    frame->brs = 1U;
    frame->fdf = 1U;
}
/*
 * 协议统一说明：
 *   1) 所有上报帧都遵循同一套头部格式；
 *   2) 上层业务只需要关注 payload 里的内容；
 *   3) 接收端先看 type，再看 payload_len，再验 CRC16；
 *   4) 这样可以避免“解析到一半就误判”的问题。
 *
 * 统一帧布局：
 *   byte0  = type
 *   byte1  = seq
 *   byte2  = payload_len
 *   byte3  = flags
 *   byte4~ = payload
 *   最后2字节 = CRC16
 */

/* GPIO 与 CAN 外设的底层初始化。 */
static uint8_t s_inited_gpio = 0U;

static void can_comm_gpio_config(void)
{
    rcu_dtm_can_clock_config(DTM_CAN0, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN0);
    rcu_dtm_can_clock_config(DTM_CAN1, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN1);
    rcu_dtm_can_clock_config(DTM_CAN2, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN2);
    rcu_dtm_can_clock_config(DTM_CAN3, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN3);
    rcu_dtm_can_clock_config(DTM_CAN4, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN4);
    rcu_dtm_can_clock_config(DTM_CAN5, RCU_DTM_CANSRC_PCLK2);
    rcu_periph_clock_enable(RCU_DTM_CAN5);

    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOM);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOE);

    can_deinit(DTM_CAN0);
    can_deinit(DTM_CAN2);
    can_deinit(DTM_CAN4);
    can_deinit(DTM_CAN5);

    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_10);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_af_set(GPIOH, GPIO_AF_4, GPIO_PIN_10);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_12);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOH, GPIO_AF_8, GPIO_PIN_12);

    gpio_output_options_set(GPIOM, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_2);
    gpio_mode_set(GPIOM, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_af_set(GPIOM, GPIO_AF_3, GPIO_PIN_2);
    gpio_output_options_set(GPIOM, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_3);
    gpio_mode_set(GPIOM, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_af_set(GPIOM, GPIO_AF_8, GPIO_PIN_3);

    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_11);
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11);
    gpio_af_set(GPIOH, GPIO_AF_1, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_12);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOB, GPIO_AF_8, GPIO_PIN_12);

    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_15);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_15);
    gpio_af_set(GPIOE, GPIO_AF_3, GPIO_PIN_15);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL_2, GPIO_PIN_14);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_af_set(GPIOE, GPIO_AF_8, GPIO_PIN_14);

    s_inited_gpio = 1U;
}

/*
 * 生成 CAN 控制器的通用配置。
 * 这里把初始化参数、滤波器、接收 FIFO、发送缓冲区集中在一起配置，
 * 这样后面改协议时只需要看一个地方。
 */
static void can_comm_cfg_common(can_parameter_struct *can_param, can_filter_struct *can_filter,
                                can_rx_fifo_struct *rx_fifo0_config, can_tx_buffer_struct *tx_buffer_config,
                                can_filter_element_struct *filter_element)
{
    can_struct_para_init(CAN_INIT_STRUCT, can_param);
    can_struct_para_init(CAN_FILTER_STRUCT, can_filter);
    can_struct_para_init(CAN_RX_FIFO_STRUCT, rx_fifo0_config);
    can_struct_para_init(CAN_TX_BUFFER_STRUCT, tx_buffer_config);
    can_struct_para_init(CAN_FILTER_ELEMENT_STRUCT, filter_element);

    can_param->auto_retransmission_enable = ENABLE;
    can_param->transmit_pause_enable = DISABLE;
    can_param->edge_filter_enable = ENABLE;
    can_param->protocol_exception_enable = ENABLE;
    can_param->wide_message_enable = ENABLE;

    can_filter->non_match_std_frame_accept = CAN_ACCEPT_INTO_RXFIFO0;
    can_filter->non_match_ext_frame_accept = CAN_ACCEPT_INTO_RXFIFO0;
    can_filter->remote_std_frame_accept = CAN_REMOTE_FILTER;
    can_filter->remote_ext_frame_accept = CAN_REMOTE_FILTER;
    can_filter->filter_start_address_std_frame = 0U;
    can_filter->filter_start_address_ext_frame = 0U;
    can_filter->filter_number_std_frame = 1U;
    can_filter->filter_number_ext_frame = 0U;
    can_filter->and_mask_ext_frame = 0U;

    rx_fifo0_config->fifo_operation_mode = CAN_RXFIFO_BLOCKING;
    rx_fifo0_config->fifo_watermark = 0U;
    rx_fifo0_config->fifo_size = 8U;
    rx_fifo0_config->fifo_start_address = 0x100U;
    rx_fifo0_config->fifo_element_size = CAN_RXFS_16BYTES;

    tx_buffer_config->tx_buffer_start_address = 0x300U;
    tx_buffer_config->dedicate_buffer_size = 4U;
    tx_buffer_config->fifo_or_queue_mode = CAN_TXFIFO_OPERATION;
    tx_buffer_config->fifo_or_queue_size = 4U;
    tx_buffer_config->tx_buffer_element_size = CAN_TXBS_16BYTES;

    can_param->filter_config = can_filter;
    can_param->rx_fifo0_config = rx_fifo0_config;
    can_param->rx_fifo1_config = NULL;
    can_param->rx_buffer_config = NULL;
    can_param->tx_buffer_config = tx_buffer_config;
    can_param->tx_event_fifo_config = NULL;

    filter_element->filter_type = CAN_FILTER_DUAL;
    filter_element->config = CAN_FILTER_TO_RXFIFO0;
    filter_element->id1 = CAN_COMM_ID_CMD;
    filter_element->id2_or_mask_or_rxbuffercfg = CAN_COMM_ID_CMD;
}

void can_comm_init(void)
{
    can_parameter_struct can_param;
    can_filter_struct can_filter;
    can_rx_fifo_struct rx_fifo0_config;
    can_tx_buffer_struct tx_buffer_config;
    can_filter_element_struct filter_element;
    can_fd_parameter_struct can_fd_param;

    memset(&s_tel, 0, sizeof(s_tel));
    s_tx_seq = 0U;
    s_rx_last_seq = 0U;
    s_rx_seq_valid = 0U;
    s_last_cmd_tick_ms = 0U;
    s_rx_drop_count = 0U;

    can_comm_gpio_config();
    can_comm_cfg_common(&can_param, &can_filter, &rx_fifo0_config, &tx_buffer_config, &filter_element);
    can_struct_para_init(CAN_FD_INIT_STRUCT, &can_fd_param);

    can_param.prescaler = 8U;
    can_param.resync_jump_width = 2U;
    can_param.time_segment_1 = 13U;
    can_param.time_segment_2 = 6U;

    while(SET != can_sram_init_state_get(DTM_CAN0)) { }
    can_init(DTM_CAN0, &can_param);
    while(SET != can_sram_init_state_get(DTM_CAN2)) { }
    can_init(DTM_CAN2, &can_param);
    while(SET != can_sram_init_state_get(DTM_CAN4)) { }
    can_init(DTM_CAN4, &can_param);
    while(SET != can_sram_init_state_get(DTM_CAN5)) { }
    can_init(DTM_CAN5, &can_param);

    can_filter_set(DTM_CAN0, CAN_FF_STANDARD, 0U, &filter_element);
    can_filter_set(DTM_CAN2, CAN_FF_STANDARD, 0U, &filter_element);
    can_filter_set(DTM_CAN4, CAN_FF_STANDARD, 0U, &filter_element);
    can_filter_set(DTM_CAN5, CAN_FF_STANDARD, 0U, &filter_element);

    can_interrupt_enable(DTM_CAN5, CAN_INT_RFIFO0_NEW);
    can_mcan_interrupt_line_config(DTM_CAN5, CAN_INTR_LINE0, CAN_INT_RFIFO0_NEW);
    can_mcan_interrupt_line_enable(DTM_CAN5, CAN_INTR_LINE0);
    nvic_irq_enable(DTM_CAN5_INT0_IRQn, 0U, 0U);

    can_param.prescaler = 4U;
    can_param.resync_jump_width = 2U;
    can_param.time_segment_1 = 13U;
    can_param.time_segment_2 = 6U;
    can_fd_param.bitrate_switch_enable = ENABLE;
    can_fd_param.iso_can_fd_enable = ENABLE;
    can_fd_param.tdc_enable = DISABLE;
    can_fd_param.prescaler = 2U;
    can_fd_param.resync_jump_width = 2U;
    can_fd_param.time_segment_1 = 13U;
    can_fd_param.time_segment_2 = 6U;

    can_operating_mode_enable(DTM_CAN0, CAN_MODE_INIT);
    can_fd_config(DTM_CAN0, &can_fd_param);
    can_operating_mode_enable(DTM_CAN2, CAN_MODE_INIT);
    can_fd_config(DTM_CAN2, &can_fd_param);
    can_operating_mode_enable(DTM_CAN4, CAN_MODE_NORMAL);
    can_operating_mode_enable(DTM_CAN5, CAN_MODE_NORMAL);

    s_inited = 1U;
}

ErrStatus can_comm_send(can_comm_bus_t bus, const can_comm_frame_t *frame)
{
    can_transmit_message_struct tx;
    uint8_t mailbox;
    uint32_t target;

    if((s_inited == 0U) || (frame == NULL) || (frame->data_len > sizeof(frame->data))) {
        return ERROR;
    }

    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx);
    tx.id = frame->std_id & CAN_COMM_ID_MASK;
    tx.rtr = CAN_FT_DATA;
    tx.xtd = CAN_FF_STANDARD;
    tx.esi = 0U;
    tx.brs = frame->brs ? ENABLE : DISABLE;
    tx.fdf = frame->fdf ? ENABLE : DISABLE;
    tx.message_marker = 0U;
    tx.ev_fifo_control = CAN_TXEVENT_FIFO_DISABLE;
    tx.data_bytes = frame->data_len;
    memcpy(tx.data, frame->data, frame->data_len);

    switch(bus) {
    case CAN_COMM_BUS_DTM_CAN4: target = DTM_CAN4; break;
    case CAN_COMM_BUS_DTM_CAN5: target = DTM_CAN5; break;
    case CAN_COMM_BUS_DTM_CAN0: target = DTM_CAN0; break;
    case CAN_COMM_BUS_DTM_CAN2: target = DTM_CAN2; break;
    default: return ERROR;
    }

    mailbox = can_message_transmit_prepare(target, &tx);
    if(mailbox == 0xFFU) {
        return ERROR;
    }
    can_message_transmit_add(target, mailbox);
    return SUCCESS;
}

/**
 * @brief 更新CAN通信遥测数据
 * 
 * 将传入的遥测数据结构复制到内部静态变量中，用于后续的CAN通信发送或处理。
 * 
 * @param telemetry 指向待更新的遥测数据结构的指针。如果为NULL，则不执行任何操作。
 */
void can_comm_update_telemetry(const can_comm_telemetry_t *telemetry)
{
    /* 检查指针有效性，防止空指针解引用 */
    if(telemetry != NULL) {
        s_tel = *telemetry;
    }
}

/**
 * @brief 设置CAN通信命令回调函数及用户上下文。
 *
 * 该函数用于注册当接收到CAN命令时需要执行的回调函数，并保存与之关联的用户自定义数据指针。
 *
 * @param cb       要设置的命令回调函数指针。若为NULL，则禁用回调。
 * @param user_ctx 用户自定义上下文指针，将在回调函数被调用时作为参数传入。可为NULL。
 */
void can_comm_set_cmd_callback(can_comm_cmd_cb_t cb, void *user_ctx)
{
    s_cmd_cb = cb;
    s_cmd_user_ctx = user_ctx;
}



/**
 * @brief 通用CAN通信发送函数
 * 
 * 该函数负责构建并发送一个CAN通信帧。它会自动生成序列号，校验 payload 长度，
 * 构建数据帧，并通过指定的总线发送。
 *
 * @param bus       CAN通信总线标识
 * @param type      消息类型
 * @param payload   指向有效载荷数据的指针
 * @param payload_len 有效载荷数据的长度
 *
 * @return ErrStatus 返回执行状态：SUCCESS表示发送成功，ERROR表示发送失败或参数无效
 */
static ErrStatus can_comm_send_common(can_comm_bus_t bus, uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    can_comm_frame_t frame;
    uint8_t seq = can_comm_next_seq();

    /* 检查有效载荷长度是否超出最大限制 */
    if(payload_len > CAN_COMM_MAX_PAYLOAD_BYTES) {
        return ERROR;
    }

    /* 构建CAN通信帧 */
    can_comm_build_frame(type, seq, payload, payload_len, &frame);

    /* 验证构建后的帧数据长度是否有效 */
    if(frame.data_len == 0U) {
        return ERROR;
    }

    /* 通过指定总线发送帧 */
    return can_comm_send(bus, &frame);
}

/**
 * @brief 发送CAN通信状态报告
 * 
 * @param bus CAN总线通道标识
 * @return ErrStatus 发送结果状态
 */
ErrStatus send_state_report(can_comm_bus_t bus)
{
    uint8_t payload[10] = {0};

    /* 组装状态报告数据载荷 */
    payload[0] = (uint8_t)s_tel.state;
    payload[1] = s_tel.run_enable;
    payload[2] = s_tel.heat_enable;
    payload[3] = s_tel.report_enable;
    payload[4] = s_tel.gas_alarm;
    payload[5] = s_tel.pressure_alarm;
    payload[6] = s_tel.gas_ready;
    payload[7] = s_tel.pressure_ready;
    return can_comm_send_common(bus, CAN_COMM_FRAME_TYPE_STATE, payload, 8U);
}

/**
 * @brief 发送温度报告数据到CAN总线
 * 
 * @param bus CAN通信总线句柄
 * @return ErrStatus 发送结果状态
 */
ErrStatus send_temp_report(can_comm_bus_t bus)
{
    uint8_t payload[10] = {0};
    int16_t avg = s_tel.avg_temp_tenth_c;
    int16_t maxv = s_tel.max_temp_tenth_c;
    int16_t minv = s_tel.min_temp_tenth_c;

    // 将平均温度、最高温度和最低温度转换为大端格式存入payload
    payload[0] = (uint8_t)(avg >> 8);
    payload[1] = (uint8_t)(avg & 0xFF);
    payload[2] = (uint8_t)(maxv >> 8);
    payload[3] = (uint8_t)(maxv & 0xFF);
    payload[4] = (uint8_t)(minv >> 8);
    payload[5] = (uint8_t)(minv & 0xFF);

    // 填入温度有效性标志和有效传感器计数
    payload[6] = s_tel.temp_valid;
    payload[7] = s_tel.temp_sensor_valid_count;

    return can_comm_send_common(bus, CAN_COMM_FRAME_TYPE_TEMP, payload, 8U);
}

/**
 * @brief 发送环境状态报告
 *
 * 该函数将MQ9传感器原始数据、气体报警状态、压力报警状态以及相应的就绪状态
 * 打包成CAN报文 payload，并通过指定的CAN总线发送环境类型帧。
 *
 * @param bus CAN通信总线标识
 * @return ErrStatus 发送结果状态
 */
ErrStatus send_env_report(can_comm_bus_t bus)
{
    uint8_t payload[10] = {0};

    /* 组装payload：MQ9原始值(2字节) + 气体报警 + 压力报警 + 气体就绪 + 压力就绪 */
    payload[0] = (uint8_t)(s_tel.mq9_raw >> 8);
    payload[1] = (uint8_t)(s_tel.mq9_raw & 0xFF);
    payload[2] = s_tel.gas_alarm;
    payload[3] = s_tel.pressure_alarm;
    payload[4] = s_tel.gas_ready;
    payload[5] = s_tel.pressure_ready;
    return can_comm_send_common(bus, CAN_COMM_FRAME_TYPE_ENV, payload, 6U);
}

/**
 * @brief 发送执行器状态报告
 *
 * @param bus CAN通信总线句柄
 * @return ErrStatus 发送结果状态
 */
ErrStatus send_actuator_report(can_comm_bus_t bus)
{
    uint8_t payload[10] = {0};

    /* 组装执行器状态数据负载 */
    payload[0] = s_tel.fan_enable;
    payload[1] = s_tel.pump_enable;
    payload[2] = s_tel.heater_enable;
    payload[3] = s_tel.cooler_enable;
    payload[4] = s_tel.buzzer_enable;
    payload[5] = s_tel.gate_enable;
    return can_comm_send_common(bus, CAN_COMM_FRAME_TYPE_ACT, payload, 6U);
}

/**
 * @brief 发送故障报告到CAN总线
 * 
 * 该函数收集当前的故障状态和故障位图，将其组装成CAN消息 payload，
 * 并通过指定的CAN总线发送故障报告帧。
 * 
 * @param bus CAN通信总线标识，指定消息发送的目标总线
 * @return ErrStatus 发送结果状态，成功返回OK，失败返回ERROR
 */
ErrStatus send_fault_report(can_comm_bus_t bus)
{
    uint8_t payload[10] = {0};
    fault_manager_status_t fault_status;
    uint16_t bitmap;

    /* 获取当前故障管理器状态和故障位图 */
    fault_manager_get_status(&fault_status);
    bitmap = fault_manager_get_bitmap();

    /* 组装CAN消息payload：前两位为故障位图（大端序），后续字节为具体故障状态 */
    payload[0] = (uint8_t)(bitmap >> 8);
    payload[1] = (uint8_t)(bitmap & 0xFF);
    payload[2] = fault_status.temp_sensor_fault;
    payload[3] = fault_status.fan_current_fault;
    payload[4] = fault_status.pump_current_fault;
    payload[5] = fault_status.cooler_current_fault;
    payload[6] = fault_status.gate_current_fault;
    payload[7] = fault_status.comm_fault;

    /* 通过CAN总线发送故障报告帧 */
    return can_comm_send_common(bus, CAN_COMM_FRAME_TYPE_FAULT, payload, 8U);
}

/**
 * @brief 发送CAN命令帧
 *
 * @param bus CAN通信总线句柄
 * @param cmd 命令字
 * @param arg0 第一个参数
 * @param arg1 第二个参数
 * @return ErrStatus 返回执行状态，成功返回OK，失败返回ERROR
 */
ErrStatus send_cmd(can_comm_bus_t bus, uint8_t cmd, uint8_t arg0, uint8_t arg1)
{
    uint8_t payload[4];
    can_comm_frame_t frame;

    /* 组装命令载荷数据 */
    payload[0] = cmd;
    payload[1] = arg0;
    payload[2] = arg1;
    payload[3] = 0U;

    /* 构建CAN通信帧并检查帧有效性 */
    can_comm_build_frame(CAN_COMM_FRAME_TYPE_CMD, can_comm_next_seq(), payload, 3U, &frame);
    if(frame.data_len == 0U) {
        return ERROR;
    }

    /* 设置标准帧ID为命令ID */
    frame.std_id = CAN_COMM_ID_CMD;

    /* 通过指定总线发送帧 */
    return can_comm_send(bus, &frame);
}

/**
 * @brief 处理接收到的CAN命令帧
 * 
 * 该函数对接收到的CAN消息进行完整性校验（长度、CRC）、类型检查及序列号验证。
 * 若校验通过，则标记接收成功并调用用户注册的命令回调函数。
 * 若任何一步校验失败，则标记接收故障并直接返回。
 *
 * @param rx 指向接收到的CAN消息结构体的指针。若为NULL，函数直接返回。
 * @return 无返回值
 */
static void handle_cmd_frame(const can_receive_message_struct *rx)
{
    uint8_t payload_len;
    uint16_t rx_crc;
    uint16_t calc_crc;
    uint8_t expected_len;

    /* 参数有效性检查 */
    if(rx == NULL) {
        return;
    }

    /* 检查数据长度是否满足最小帧结构要求（头部 + CRC） */
    if(rx->data_bytes < (CAN_COMM_FRAME_HEADER_LEN + CAN_COMM_CRC_LEN)) {
        can_comm_mark_rx_fault();
        return;
    }

    /* 提取期望的有效载荷长度，并计算实际有效载荷长度 */
    expected_len = rx->data[2];
    payload_len = (uint8_t)(rx->data_bytes - CAN_COMM_FRAME_HEADER_LEN - CAN_COMM_CRC_LEN);

    /* 校验期望长度与实际载荷长度是否一致 */
    if(expected_len != payload_len) {
        can_comm_mark_rx_fault();
        return;
    }

    /* 计算数据部分的CRC值 */
    calc_crc = can_comm_crc16_calc(rx->data, (uint8_t)(CAN_COMM_FRAME_HEADER_LEN + payload_len));

    /* 从接收数据中提取发送方提供的CRC值（大端序） */
    rx_crc = (uint16_t)((((uint16_t)rx->data[CAN_COMM_FRAME_HEADER_LEN + payload_len]) << 8) |
                        ((uint16_t)rx->data[CAN_COMM_FRAME_HEADER_LEN + payload_len + 1U]));

    /* 校验计算出的CRC与接收到的CRC是否匹配 */
    if(calc_crc != rx_crc) {
        can_comm_mark_rx_fault();
        return;
    }

    /* 确认帧类型为命令帧 */
    if(rx->data[0] != CAN_COMM_FRAME_TYPE_CMD) {
        return;
    }

    /* 检查序列号，若无效则丢弃该帧 */
    if(can_comm_check_seq(rx->data[1]) == 0U) {
        return;
    }

    /* 标记接收成功 */
    can_comm_mark_rx_ok();

    /* 若已注册命令回调函数，则执行回调，传递命令参数及用户上下文 */
    if(s_cmd_cb != NULL) {
        s_cmd_cb(rx->data[4], rx->data[5], rx->data[6], s_cmd_user_ctx);
    }
}

/**
 * @brief CAN通信任务处理函数
 * 
 * 该函数负责处理CAN总线上的接收消息，包括超时检测和命令帧处理。
 * 函数首先检查模块初始化状态，然后检测接收超时情况，最后循环处理接收到的命令帧。
 * 
 * @param void 无输入参数
 * @return void 无返回值
 */
void can_comm_task(void)
{
    can_receive_message_struct rx;
    uint32_t now_ms;

    /* 检查模块是否已初始化，未初始化则直接返回 */
    if(s_inited == 0U) {
        return;
    }

    now_ms = can_comm_now_ms();

    /* 检测接收超时：如果上次收到命令的时间不为0且当前时间与上次时间的差值超过超时阈值，则设置通信超时故障 */
    if((s_last_cmd_tick_ms != 0U) && ((now_ms - s_last_cmd_tick_ms) > CAN_COMM_RX_TIMEOUT_MS)) {
        fault_manager_set_comm_timeout(1U);
        s_last_cmd_tick_ms = now_ms;
    }

    /* 循环处理CAN接收FIFO中的新消息，直到没有新消息为止 */
    while(can_interrupt_flag_get(DTM_CAN5, CAN_INT_FLAG_RFIFO0_NEW) == SET) {
        can_message_receive(DTM_CAN5, CAN_RXFIFO0, &rx);

        /* 只处理命令ID的消息，非命令ID的消息跳过 */
        if(rx.id != CAN_COMM_ID_CMD) {
            continue;
        }

        handle_cmd_frame(&rx);
    }
}
