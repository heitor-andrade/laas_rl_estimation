//
// Included Files
//
//#include <stdint.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <string.h>
//#include <float.h>

/***********************************************************************
 * INCLUDES
 ***********************************************************************/
#include "main_cm.h"
//#include "foc.h"
#include "communication.h"
#include "uomodri_cm_user_defines.h"
#include "uomodri_cm_hal_config_handlers.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/************************************************************************
 * FUNCTIONS DECLARATION
 ************************************************************************/
inline void IPC_CpuToCmAddrSync(dbg_uart_addr_t*);

/************************************************************************
 * LOCAL VARIABLES
 ************************************************************************/
#pragma DATA_ALIGN(ucControlTable, 1024)
UDMA_ControlTable ucControlTable[32];
dbg_uart_msg_t dbg_uart_tx_msg =
{
 .header[0]         = 'a',
 .header[1]         = 'b',
 .counter           = 0U,
 .idref             = 0.0f,
 .id                = 0.0f,
 .resistor          = 0.0f,
 .inductor          = 0.0f,
 .crc               = 0,
};

dbg_uart_addr_t dbg_uart_addr =
{
 .p_itCnt           = NULL,
 .p_idref           = NULL,
 .p_id              = NULL,
 .p_res             = NULL,
 .p_induc           = NULL,
};

/************************************************************************
 * GLOBAL VARIABLES
 ************************************************************************/
extern const hal_cm_t hal_cm;

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief   Main function running on CM processor. MCU init.
 */
void main(void)
{
    uint32_t i = 0;
    // Initialize device clock and peripherals
    CM_init();
    // Hardware Abstraction Layer initialization
    HAL_CM_init(&hal_cm);
    // IPC address translation
    IPC_CpuToCmAddrSync(&dbg_uart_addr);

    // Loop forever. Wait for IPC interrupts
    while(1)
    {
        IPC_waitForFlag(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//        DBG_PIN1_SET;
//        GPIO_writePin(EXT_DBG_SPI_SOMI, 1);
//        EXT_DBG_SPI_SOMI_SET;
        GPIO_writePin(EXT_DBG_SPI_SOMI, 1);
        // Create debug message
        COM_msgDbgTx(&dbg_uart_addr, &dbg_uart_tx_msg);
        // Acknowledge IT associated to input flag.
        IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
//        DBG_PIN0_CLEAR;
//        EXT_DBG_SPI_SIMO_CLEAR;
//        GPIO_writePin(EXT_DBG_SPI_SIMO, 0);
//        EXT_DBG_SPI_SOMI_CLEAR;
        GPIO_writePin(EXT_DBG_SPI_SOMI, 0);
        if((i % 1) == 0)
        {
            // Wait Tx buffer + UART Tx module are empty.
            while(UART_isBusy(UART0_BASE));
            // Force purge of Rx buffer.
            while(UART_readCharNonBlocking(UART0_BASE) != -1);
            // Get the index associated to uDMA UART0_TX configuration
            //    while((cm_udmaCfgList[udmaCfgIdx].dmaChannel != UDMA_CHANNEL_UART0_TX) ? (udmaCfgIdx++) : (udmaCfgIdx));
            // Enable the uDMA UART0 Tx channel
            //        while(UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel));
            //        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
            //        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
            //        ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
            //        UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
            if(!UDMA_isChannelEnabled(UDMA_BASE, cm_udmaCfgList[0].dmaChannel))
            {
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr &= ~(UDMA_DMACHCTL_XFERSIZE_M | UDMA_DMACHCTL_XFERMODE_M);
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferMode) << UDMA_DMACHCTL_XFERMODE_S);//cm_udmaCfgList[udmaCfgIdx].transferMode;
                ucControlTable[UDMA_CHANNEL_UART0_TX].chControlAttr |= ((cm_udmaCfgList[0].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);//((cm_udmaCfgList[udmaCfgIdx].transferSize - 1) << UDMA_DMACHCTL_XFERSIZE_S);
                UDMA_enableChannel(UDMA_BASE, UDMA_CHANNEL_UART0_TX);
            }
        }
        i++;
//        // Acknowledge IT associated to input flag.
//        IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    }
}

inline void IPC_CpuToCmAddrSync(dbg_uart_addr_t* p_addr)
{
    uint32_t command;// = NULL;
    uint32_t addr;// = NULL;
    uint32_t data;// = NULL;

    // Get address of ia of MOTOR_1
    while(!IPC_readCommand(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG, IPC_ADDR_CORRECTION_ENABLE, &command, &addr, &data));
    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_CPU1_TO_CM_FLAG);
    p_addr->p_itCnt     = (float_t*)addr;
    p_addr->p_idref     = (float_t*)addr + 1;
    p_addr->p_id        = (float_t*)addr + 2;
    p_addr->p_res       = (float_t*)addr + 3;
    p_addr->p_induc     = (float_t*)addr + 4;
}

//
// End of File
//
