/*
 * File name: communication_cm.c
 * Description: Source file containing extraction & build/update message functions
 */

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <strings.h>
#include "communication.h"
#include "uomodri_cm_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/
#define LSB_32(x)               ((x) & 0x0000FFFF)
#define MSB_32(x)               (((x) >> 16) & 0x0000FFFF)
#define LSB_16(x)               ((x) & 0x00FF)
#define MSB_16(x)               (((x) >> 8) & 0x00FF)
#define MSG_RX_CFG              (0x01)
#define MSG_RX_CMD              (0x02)
#define MSG_TX_CFG              (0x01)
#define MSG_TX_CMD              (0x02)
#define MSG_CMD_LIST_MASK       (0x07FF07FF)
#define PD_KP_MASK              (0x0001)
#define PD_KI_MASK              (0x0002)
#define POS_REF_MASK            (0x0004)
#define VEL_REF_MASK            (0x0008)
#define IQ_FF_MASK              (0x0010)
#define ISAT_MASK               (0x0020)
#define CMD_MASK                (0x0040)
#define PI_KP_ID_MASK           (0x0080)
#define PI_KI_ID_MASK           (0x0100)
#define PI_KP_IQ_MASK           (0x0200)
#define PI_KI_IQ_MASK           (0x0400)
#define PD_KP_POS               (0)
#define PD_KI_POS               (1)
#define POS_REF_POS             (2)
#define VEL_REF_POS             (3)
#define IQ_FF_POS               (4)
#define ISAT_POS                (5)
#define CMD_POS                 (6)
#define PI_KP_ID_POS            (7)
#define PI_KI_ID_POS            (8)
#define PI_KP_IQ_POS            (9)
#define PI_KI_IQ_POS            (10)

#define MSG_TX_HEADER_SIZE      (2)
#define MSG_TX_COUNTER_SIZE     (4)
#define MSG_TX_MSG_TYPE_SIZE    (1)


//typedef enum
//{
//    m1_pd_kp            = 0,
//    m1_pd_ki            = 1,
//    m1_posref           = 2,
//    m1_velref           = 3,
//    m1_iqff             = 4,
//    m1_isat             = 5,
//    m1_cmd              = 6,
//    m1_pi_kp_id         = 7,
//    m1_pi_ki_id         = 8,
//    m1_pi_kp_iq         = 9,
//    m1_pi_ki_iq         = 10,
//    DISP_RSV1_0         = 11,
//    DISP_RSV1_1         = 12,
//    DISP_RSV1_2         = 13,
//    DISP_RSV1_3         = 14,
//    DISP_RSV1_4         = 15,
//    m2_pd_kp            = 16,
//    m2_pd_ki            = 17,
//    m2_posref           = 18,
//    m2_velref           = 19,
//    m2_iqff             = 20,
//    m2_isat             = 21,
//    m2_cmd              = 22,
//    m2_pi_kp_id         = 23,
//    m2_pi_ki_id         = 24,
//    m2_pi_kp_iq         = 25,
//    m2_pi_ki_iq         = 26,
//    DISP_RSV2_0         = 27,
//    DISP_RSV2_1         = 28,
//    DISP_RSV2_2         = 29,
//    DISP_RSV2_3         = 30,
//    DISP_RSV2_4         = 31,
//} com_dbg_rx_command_bit_mask_e;

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       Message created/updates for debug
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
//void COM_msgDbgTx(com_dbg_tx_msg_t* p_msg, uint8_t msg_type)
void COM_msgDbgTx(dbg_uart_addr_t* p_addr, dbg_uart_msg_t* p_msg)
{
    p_msg->counter          = (uint32_t)(*p_addr->p_itCnt);
    p_msg->idref            = (float_t)(*p_addr->p_idref);
    p_msg->id               = (float_t)(*p_addr->p_id);
    p_msg->resistor         = (float_t)(*p_addr->p_res);
    p_msg->inductor         = (float_t)(*p_addr->p_induc);
    p_msg->crc              = COM_crc8Fast(0, (uint8_t *)p_msg, (sizeof(dbg_uart_msg_t) - 1));
    return;
}

/**
 * @brief       Message extract for debug (display \& command)
 * @param p_foc Pointer on data structure array used for motor control.
 * @param p_msg Pointer on the message created /updated for debug.
 */
void COM_msgDbgRx(com_dbg_rx_msg_t* p_msg, uint8_t rx_byte)
{
    static uint32_t cmd_lst = 0;
    static uint8_t  i = 0, j = 0;

    switch(p_msg->state)
    {
    case COM_DBG_RX_WORD_A:
    default:
        i                               = 0;
        j                               = 0;
        cmd_lst                         = 0;
        p_msg->state                    = (rx_byte == 'a')  ? (COM_DBG_RX_WORD_B)               : (COM_DBG_RX_WORD_A);
        p_msg->crc                      = (rx_byte == 'a')  ? (COM_crc8Fast(0, &rx_byte, 1))    : (0);
        p_msg->valid                    = false;

        break;

    case COM_DBG_RX_WORD_B:
        p_msg->state                    = (rx_byte == 'b')  ? (COM_DBG_RX_CNT)                          : (COM_DBG_RX_WORD_A);
        p_msg->crc                      = (rx_byte == 'b')  ? (COM_crc8Fast(p_msg->crc, &rx_byte, 1))   : (0);
        break;

    case COM_DBG_RX_CNT:
        p_msg->counter.byte[i++] = rx_byte;
        p_msg->state                    = (i < sizeof(counter_u))   ? (COM_DBG_RX_CNT)  : (COM_DBG_RX_MSG_TYPE);
        i                               = (i < sizeof(counter_u))   ? (i)               : (0);
        p_msg->crc                      = COM_crc8Fast(p_msg->crc, &rx_byte, 1);
        break;

     case COM_DBG_RX_MSG_TYPE:
         i                              = 0;
         j                              = 0;
         cmd_lst                        = (rx_byte == MSG_RX_CMD)   ? (p_msg->cmd_lst.all & MSG_CMD_LIST_MASK)  : (0);
         p_msg->state                   = (rx_byte == MSG_RX_CMD)   ? (COM_DBG_RX_CMD_MSG)                      : (COM_DBG_RX_WORD_A);
         p_msg->state                   = (rx_byte == MSG_RX_CMD) && (!cmd_lst) ? (COM_DBG_RX_CRC)              : (p_msg->state);
         p_msg->state                   = (rx_byte == MSG_RX_CFG)   ? (COM_DBG_RX_CFG_LIST_4_TX)                : (p_msg->state);
         p_msg->crc                     = (rx_byte == MSG_RX_CMD)   ? (COM_crc8Fast(p_msg->crc, &rx_byte, 1))   : (p_msg->crc);
         p_msg->crc                     = (rx_byte == MSG_RX_CFG)   ? (COM_crc8Fast(p_msg->crc, &rx_byte, 1))   : (0);
         p_msg->msgType                 = (rx_byte == MSG_RX_CMD)   ? (MSG_RX_CMD)                              : (0);
         p_msg->msgType                 = (rx_byte == MSG_RX_CFG)   ? (MSG_RX_CFG)                              : (p_msg->msgType);
         break;

     case COM_DBG_RX_CFG_LIST_4_TX:
         p_msg->cfg_lst.byte[i++]       = rx_byte;
         p_msg->state                   = (i < sizeof(config_u))    ? (COM_DBG_RX_CFG_LIST_4_TX)    : (COM_DBG_RX_CMD_LIST_4_RX);
         i                              = (i < sizeof(config_u))    ? (i)                           : (0);
         p_msg->crc                     = COM_crc8Fast(p_msg->crc, &rx_byte, 1);
         break;

     case COM_DBG_RX_CMD_LIST_4_RX:
         p_msg->cmd_lst.byte[i++]       = rx_byte;
         p_msg->state                   = (i < sizeof(command_u))   ? (COM_DBG_RX_CMD_LIST_4_RX)    : (COM_DBG_RX_CRC);
         i                              = (i < sizeof(command_u))   ? (i)                           : (0);
         p_msg->crc                     = COM_crc8Fast(p_msg->crc, &rx_byte, 1);
         break;

     case COM_DBG_RX_CMD_MSG:
         i                              = (i % 4)   ? (i)       : (0);
         j                              = (i % 4)   ? (j)       : (ffs(cmd_lst) - 1);
         cmd_lst                       &= (i % 4)   ? (cmd_lst) : (~(1U << j));
         p_msg->cmd_msg[j].uint_8b[i++] = rx_byte;
         p_msg->state                   = (cmd_lst || (i % 4))  ? (COM_DBG_RX_CMD_MSG)  : (COM_DBG_RX_CRC);
         p_msg->crc                     = COM_crc8Fast(p_msg->crc, &rx_byte, 1);
         break;

     case COM_DBG_RX_CRC:
         p_msg->state                   = COM_DBG_RX_WORD_A;
//         p_msg->msgType             = (p_msg->crc == rx_byte)   ? (msg_type)    : (p_msg->msgType);
         p_msg->valid                   = (p_msg->crc == rx_byte)   ? (true)    : (false);
         break;
     }

}

uint32_t COM_crc32(uint16_t* p_buf, size_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  while (len--)
    {
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ MSB_16(*p_buf)) & 0xFF];
      crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ LSB_16(*p_buf)) & 0xFF];
      p_buf++;
    }

  return(crc);
}


uint8_t COM_crc8(uint8_t *p_buf, size_t len)
{
    uint8_t crc = 0, i = 0, j = 0;
    for(i = 0; i < len; i++)
    {
        uint8_t inbyte = p_buf[i];
        for(j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if(mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return(crc);
}


uint8_t COM_crc8Fast(uint8_t crc, uint8_t* p_buf, size_t len)
{
//    uint8_t crc = 0;
    size_t i = 0;
    for(i = 0; i < len; i++)
        crc = crc8_table[p_buf[i] ^ crc];
    return crc;
}
