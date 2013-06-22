/*******************************************************************************
* File Name: ETH.h
* Version 2.40
*
* Description:
*  Contains the function prototypes, constants and register definition
*  of the SPI Master Component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIM_ETH_H)
#define CY_SPIM_ETH_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component SPI_Master_v2_40 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define ETH_INTERNAL_CLOCK             (1u)

#if(0u != ETH_INTERNAL_CLOCK)
    #include "ETH_IntClock.h"
#endif /* (0u != ETH_INTERNAL_CLOCK) */

#define ETH_MODE                       (1u)
#define ETH_DATA_WIDTH                 (8u)
#define ETH_MODE_USE_ZERO              (1u)
#define ETH_BIDIRECTIONAL_MODE         (0u)

/* Internal interrupt handling */
#define ETH_TX_BUFFER_SIZE             (4u)
#define ETH_RX_BUFFER_SIZE             (4u)
#define ETH_INTERNAL_TX_INT_ENABLED    (0u)
#define ETH_INTERNAL_RX_INT_ENABLED    (0u)

#define ETH_SINGLE_REG_SIZE            (8u)
#define ETH_USE_SECOND_DATAPATH        (ETH_DATA_WIDTH > ETH_SINGLE_REG_SIZE)

#define ETH_FIFO_SIZE                  (4u)
#define ETH_TX_SOFTWARE_BUF_ENABLED    ((0u != ETH_INTERNAL_TX_INT_ENABLED) && \
                                                     (ETH_TX_BUFFER_SIZE > ETH_FIFO_SIZE))

#define ETH_RX_SOFTWARE_BUF_ENABLED    ((0u != ETH_INTERNAL_RX_INT_ENABLED) && \
                                                     (ETH_RX_BUFFER_SIZE > ETH_FIFO_SIZE))


/***************************************
*        Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct
{
    uint8 enableState;
    uint8 cntrPeriod;
    #if(CY_UDB_V0)
        uint8 saveSrTxIntMask;
        uint8 saveSrRxIntMask;
    #endif /* (CY_UDB_V0) */

} ETH_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void  ETH_Init(void)                           ;
void  ETH_Enable(void)                         ;
void  ETH_Start(void)                          ;
void  ETH_Stop(void)                           ;

void  ETH_EnableTxInt(void)                    ;
void  ETH_EnableRxInt(void)                    ;
void  ETH_DisableTxInt(void)                   ;
void  ETH_DisableRxInt(void)                   ;

void  ETH_Sleep(void)                          ;
void  ETH_Wakeup(void)                         ;
void  ETH_SaveConfig(void)                     ;
void  ETH_RestoreConfig(void)                  ;

void  ETH_SetTxInterruptMode(uint8 intSrc)     ;
void  ETH_SetRxInterruptMode(uint8 intSrc)     ;
uint8 ETH_ReadTxStatus(void)                   ;
uint8 ETH_ReadRxStatus(void)                   ;
void  ETH_WriteTxData(uint8 txData)  \
                                                            ;
uint8 ETH_ReadRxData(void) \
                                                            ;
uint8 ETH_GetRxBufferSize(void)                ;
uint8 ETH_GetTxBufferSize(void)                ;
void  ETH_ClearRxBuffer(void)                  ;
void  ETH_ClearTxBuffer(void)                  ;
void  ETH_ClearFIFO(void)                              ;
void  ETH_PutArray(const uint8 buffer[], uint8 byteCount) \
                                                            ;

#if(0u != ETH_BIDIRECTIONAL_MODE)
    void  ETH_TxEnable(void)                   ;
    void  ETH_TxDisable(void)                  ;
#endif /* (0u != ETH_BIDIRECTIONAL_MODE) */

CY_ISR_PROTO(ETH_TX_ISR);
CY_ISR_PROTO(ETH_RX_ISR);


/**********************************
*   Variable with external linkage
**********************************/

extern uint8 ETH_initVar;


/***************************************
*           API Constants
***************************************/

#define ETH_TX_ISR_NUMBER     ((uint8) (ETH_TxInternalInterrupt__INTC_NUMBER))
#define ETH_RX_ISR_NUMBER     ((uint8) (ETH_RxInternalInterrupt__INTC_NUMBER))

#define ETH_TX_ISR_PRIORITY   ((uint8) (ETH_TxInternalInterrupt__INTC_PRIOR_NUM))
#define ETH_RX_ISR_PRIORITY   ((uint8) (ETH_RxInternalInterrupt__INTC_PRIOR_NUM))


/***************************************
*    Initial Parameter Constants
***************************************/

#define ETH_INT_ON_SPI_DONE    ((uint8) (0u   << ETH_STS_SPI_DONE_SHIFT))
#define ETH_INT_ON_TX_EMPTY    ((uint8) (1u   << ETH_STS_TX_FIFO_EMPTY_SHIFT))
#define ETH_INT_ON_TX_NOT_FULL ((uint8) (0u << \
                                                                           ETH_STS_TX_FIFO_NOT_FULL_SHIFT))
#define ETH_INT_ON_BYTE_COMP   ((uint8) (0u  << ETH_STS_BYTE_COMPLETE_SHIFT))
#define ETH_INT_ON_SPI_IDLE    ((uint8) (0u   << ETH_STS_SPI_IDLE_SHIFT))

/* Disable TX_NOT_FULL if software buffer is used */
#define ETH_INT_ON_TX_NOT_FULL_DEF ((ETH_TX_SOFTWARE_BUF_ENABLED) ? \
                                                                        (0u) : (ETH_INT_ON_TX_NOT_FULL))

/* TX interrupt mask */
#define ETH_TX_INIT_INTERRUPTS_MASK    (ETH_INT_ON_SPI_DONE  | \
                                                     ETH_INT_ON_TX_EMPTY  | \
                                                     ETH_INT_ON_TX_NOT_FULL_DEF | \
                                                     ETH_INT_ON_BYTE_COMP | \
                                                     ETH_INT_ON_SPI_IDLE)

#define ETH_INT_ON_RX_FULL         ((uint8) (0u << \
                                                                          ETH_STS_RX_FIFO_FULL_SHIFT))
#define ETH_INT_ON_RX_NOT_EMPTY    ((uint8) (1u << \
                                                                          ETH_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define ETH_INT_ON_RX_OVER         ((uint8) (0u << \
                                                                          ETH_STS_RX_FIFO_OVERRUN_SHIFT))

/* RX interrupt mask */
#define ETH_RX_INIT_INTERRUPTS_MASK    (ETH_INT_ON_RX_FULL      | \
                                                     ETH_INT_ON_RX_NOT_EMPTY | \
                                                     ETH_INT_ON_RX_OVER)
/* Nubmer of bits to receive/transmit */
#define ETH_BITCTR_INIT            (((uint8) (ETH_DATA_WIDTH << 1u)) - 1u)


/***************************************
*             Registers
***************************************/

#if(CY_PSOC3 || CY_PSOC5)
    #define ETH_TXDATA_REG (* (reg8 *) \
                                                ETH_BSPIM_sR8_Dp_u0__F0_REG)
    #define ETH_TXDATA_PTR (  (reg8 *) \
                                                ETH_BSPIM_sR8_Dp_u0__F0_REG)
    #define ETH_RXDATA_REG (* (reg8 *) \
                                                ETH_BSPIM_sR8_Dp_u0__F1_REG)
    #define ETH_RXDATA_PTR (  (reg8 *) \
                                                ETH_BSPIM_sR8_Dp_u0__F1_REG)
#else   /* PSOC4 */
    #if(ETH_USE_SECOND_DATAPATH)
        #define ETH_TXDATA_REG (* (reg16 *) \
                                          ETH_BSPIM_sR8_Dp_u0__16BIT_F0_REG)
        #define ETH_TXDATA_PTR (  (reg16 *) \
                                          ETH_BSPIM_sR8_Dp_u0__16BIT_F0_REG)
        #define ETH_RXDATA_REG (* (reg16 *) \
                                          ETH_BSPIM_sR8_Dp_u0__16BIT_F1_REG)
        #define ETH_RXDATA_PTR         (  (reg16 *) \
                                          ETH_BSPIM_sR8_Dp_u0__16BIT_F1_REG)
    #else
        #define ETH_TXDATA_REG (* (reg8 *) \
                                                ETH_BSPIM_sR8_Dp_u0__F0_REG)
        #define ETH_TXDATA_PTR (  (reg8 *) \
                                                ETH_BSPIM_sR8_Dp_u0__F0_REG)
        #define ETH_RXDATA_REG (* (reg8 *) \
                                                ETH_BSPIM_sR8_Dp_u0__F1_REG)
        #define ETH_RXDATA_PTR (  (reg8 *) \
                                                ETH_BSPIM_sR8_Dp_u0__F1_REG)
    #endif /* (ETH_USE_SECOND_DATAPATH) */
#endif     /* (CY_PSOC3 || CY_PSOC5) */

#define ETH_AUX_CONTROL_DP0_REG (* (reg8 *) \
                                        ETH_BSPIM_sR8_Dp_u0__DP_AUX_CTL_REG)
#define ETH_AUX_CONTROL_DP0_PTR (  (reg8 *) \
                                        ETH_BSPIM_sR8_Dp_u0__DP_AUX_CTL_REG)

#if(ETH_USE_SECOND_DATAPATH)
    #define ETH_AUX_CONTROL_DP1_REG  (* (reg8 *) \
                                        ETH_BSPIM_sR8_Dp_u1__DP_AUX_CTL_REG)
    #define ETH_AUX_CONTROL_DP1_PTR  (  (reg8 *) \
                                        ETH_BSPIM_sR8_Dp_u1__DP_AUX_CTL_REG)
#endif /* (ETH_USE_SECOND_DATAPATH) */

#define ETH_COUNTER_PERIOD_REG     (* (reg8 *) ETH_BSPIM_BitCounter__PERIOD_REG)
#define ETH_COUNTER_PERIOD_PTR     (  (reg8 *) ETH_BSPIM_BitCounter__PERIOD_REG)
#define ETH_COUNTER_CONTROL_REG    (* (reg8 *) ETH_BSPIM_BitCounter__CONTROL_AUX_CTL_REG)
#define ETH_COUNTER_CONTROL_PTR    (  (reg8 *) ETH_BSPIM_BitCounter__CONTROL_AUX_CTL_REG)

#define ETH_TX_STATUS_REG          (* (reg8 *) ETH_BSPIM_TxStsReg__STATUS_REG)
#define ETH_TX_STATUS_PTR          (  (reg8 *) ETH_BSPIM_TxStsReg__STATUS_REG)
#define ETH_RX_STATUS_REG          (* (reg8 *) ETH_BSPIM_RxStsReg__STATUS_REG)
#define ETH_RX_STATUS_PTR          (  (reg8 *) ETH_BSPIM_RxStsReg__STATUS_REG)

#define ETH_CONTROL_REG            (* (reg8 *) \
                                      ETH_BSPIM_BidirMode_SyncCtl_CtrlReg__CONTROL_REG)
#define ETH_CONTROL_PTR            (  (reg8 *) \
                                      ETH_BSPIM_BidirMode_SyncCtl_CtrlReg__CONTROL_REG)

#define ETH_TX_STATUS_MASK_REG     (* (reg8 *) ETH_BSPIM_TxStsReg__MASK_REG)
#define ETH_TX_STATUS_MASK_PTR     (  (reg8 *) ETH_BSPIM_TxStsReg__MASK_REG)
#define ETH_RX_STATUS_MASK_REG     (* (reg8 *) ETH_BSPIM_RxStsReg__MASK_REG)
#define ETH_RX_STATUS_MASK_PTR     (  (reg8 *) ETH_BSPIM_RxStsReg__MASK_REG)

#define ETH_TX_STATUS_ACTL_REG     (* (reg8 *) ETH_BSPIM_TxStsReg__STATUS_AUX_CTL_REG)
#define ETH_TX_STATUS_ACTL_PTR     (  (reg8 *) ETH_BSPIM_TxStsReg__STATUS_AUX_CTL_REG)
#define ETH_RX_STATUS_ACTL_REG     (* (reg8 *) ETH_BSPIM_RxStsReg__STATUS_AUX_CTL_REG)
#define ETH_RX_STATUS_ACTL_PTR     (  (reg8 *) ETH_BSPIM_RxStsReg__STATUS_AUX_CTL_REG)

#if(ETH_USE_SECOND_DATAPATH)
    #define ETH_AUX_CONTROLDP1     (ETH_AUX_CONTROL_DP1_REG)
#endif /* (ETH_USE_SECOND_DATAPATH) */


/***************************************
*       Register Constants
***************************************/

/* Status Register Definitions */
#define ETH_STS_SPI_DONE_SHIFT             (0x00u)
#define ETH_STS_TX_FIFO_EMPTY_SHIFT        (0x01u)
#define ETH_STS_TX_FIFO_NOT_FULL_SHIFT     (0x02u)
#define ETH_STS_BYTE_COMPLETE_SHIFT        (0x03u)
#define ETH_STS_SPI_IDLE_SHIFT             (0x04u)
#define ETH_STS_RX_FIFO_FULL_SHIFT         (0x04u)
#define ETH_STS_RX_FIFO_NOT_EMPTY_SHIFT    (0x05u)
#define ETH_STS_RX_FIFO_OVERRUN_SHIFT      (0x06u)

#define ETH_STS_SPI_DONE           ((uint8) (0x01u << ETH_STS_SPI_DONE_SHIFT))
#define ETH_STS_TX_FIFO_EMPTY      ((uint8) (0x01u << ETH_STS_TX_FIFO_EMPTY_SHIFT))
#define ETH_STS_TX_FIFO_NOT_FULL   ((uint8) (0x01u << ETH_STS_TX_FIFO_NOT_FULL_SHIFT))
#define ETH_STS_BYTE_COMPLETE      ((uint8) (0x01u << ETH_STS_BYTE_COMPLETE_SHIFT))
#define ETH_STS_SPI_IDLE           ((uint8) (0x01u << ETH_STS_SPI_IDLE_SHIFT))
#define ETH_STS_RX_FIFO_FULL       ((uint8) (0x01u << ETH_STS_RX_FIFO_FULL_SHIFT))
#define ETH_STS_RX_FIFO_NOT_EMPTY  ((uint8) (0x01u << ETH_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define ETH_STS_RX_FIFO_OVERRUN    ((uint8) (0x01u << ETH_STS_RX_FIFO_OVERRUN_SHIFT))

/* TX and RX masks for clear on read bits */
#define ETH_TX_STS_CLR_ON_RD_BYTES_MASK    (0x09u)
#define ETH_RX_STS_CLR_ON_RD_BYTES_MASK    (0x40u)

/* StatusI Register Interrupt Enable Control Bits */
/* As defined by the Register map for the AUX Control Register */
#define ETH_INT_ENABLE     (0x10u) /* Enable interrupt from statusi */
#define ETH_TX_FIFO_CLR    (0x01u) /* F0 - TX FIFO */
#define ETH_RX_FIFO_CLR    (0x02u) /* F1 - RX FIFO */
#define ETH_FIFO_CLR       (ETH_TX_FIFO_CLR | ETH_RX_FIFO_CLR)

/* Bit Counter (7-bit) Control Register Bit Definitions */
/* As defined by the Register map for the AUX Control Register */
#define ETH_CNTR_ENABLE    (0x20u) /* Enable CNT7 */

/* Bi-Directional mode control bit */
#define ETH_CTRL_TX_SIGNAL_EN  (0x01u)

/* Datapath Auxillary Control Register definitions */
#define ETH_AUX_CTRL_FIFO0_CLR         (0x01u)
#define ETH_AUX_CTRL_FIFO1_CLR         (0x02u)
#define ETH_AUX_CTRL_FIFO0_LVL         (0x04u)
#define ETH_AUX_CTRL_FIFO1_LVL         (0x08u)
#define ETH_STATUS_ACTL_INT_EN_MASK    (0x10u)

/* Component disabled */
#define ETH_DISABLED   (0u)


/***************************************
*       Macros
***************************************/

/* Returns true if componentn enabled */
#define ETH_IS_ENABLED (0u != (ETH_TX_STATUS_ACTL_REG & ETH_INT_ENABLE))

/* Retuns TX status register */
#define ETH_GET_STATUS_TX(swTxSts) ( (uint8)(ETH_TX_STATUS_REG | \
                                                          ((swTxSts) & ETH_TX_STS_CLR_ON_RD_BYTES_MASK)) )
/* Retuns RX status register */
#define ETH_GET_STATUS_RX(swRxSts) ( (uint8)(ETH_RX_STATUS_REG | \
                                                          ((swRxSts) & ETH_RX_STS_CLR_ON_RD_BYTES_MASK)) )


/***************************************
*       Obsolete definitions
***************************************/

/* Following definitions are for version compatibility.
*  They are obsolete in SPIM v2_30.
*  Please do not use it in new projects
*/

#define ETH_WriteByte   ETH_WriteTxData
#define ETH_ReadByte    ETH_ReadRxData
void  ETH_SetInterruptMode(uint8 intSrc)       ;
uint8 ETH_ReadStatus(void)                     ;
void  ETH_EnableInt(void)                      ;
void  ETH_DisableInt(void)                     ;

/* Obsolete register names. Not to be used in new designs */
#define ETH_TXDATA                 (ETH_TXDATA_REG)
#define ETH_RXDATA                 (ETH_RXDATA_REG)
#define ETH_AUX_CONTROLDP0         (ETH_AUX_CONTROL_DP0_REG)
#define ETH_TXBUFFERREAD           (ETH_txBufferRead)
#define ETH_TXBUFFERWRITE          (ETH_txBufferWrite)
#define ETH_RXBUFFERREAD           (ETH_rxBufferRead)
#define ETH_RXBUFFERWRITE          (ETH_rxBufferWrite)

#define ETH_COUNTER_PERIOD         (ETH_COUNTER_PERIOD_REG)
#define ETH_COUNTER_CONTROL        (ETH_COUNTER_CONTROL_REG)
#define ETH_STATUS                 (ETH_TX_STATUS_REG)
#define ETH_CONTROL                (ETH_CONTROL_REG)
#define ETH_STATUS_MASK            (ETH_TX_STATUS_MASK_REG)
#define ETH_STATUS_ACTL            (ETH_TX_STATUS_ACTL_REG)

#define ETH_INIT_INTERRUPTS_MASK  (ETH_INT_ON_SPI_DONE     | \
                                                ETH_INT_ON_TX_EMPTY     | \
                                                ETH_INT_ON_TX_NOT_FULL_DEF  | \
                                                ETH_INT_ON_RX_FULL      | \
                                                ETH_INT_ON_RX_NOT_EMPTY | \
                                                ETH_INT_ON_RX_OVER      | \
                                                ETH_INT_ON_BYTE_COMP)
                                                
/* Following definitions are for version Compatibility.
*  They are obsolete in SPIM v2_40.
*  Please do not use it in new projects
*/

#define ETH_DataWidth                  (ETH_DATA_WIDTH)
#define ETH_InternalClockUsed          (ETH_INTERNAL_CLOCK)
#define ETH_InternalTxInterruptEnabled (ETH_INTERNAL_TX_INT_ENABLED)
#define ETH_InternalRxInterruptEnabled (ETH_INTERNAL_RX_INT_ENABLED)
#define ETH_ModeUseZero                (ETH_MODE_USE_ZERO)
#define ETH_BidirectionalMode          (ETH_BIDIRECTIONAL_MODE)
#define ETH_Mode                       (ETH_MODE)
#define ETH_DATAWIDHT                  (ETH_DATA_WIDTH)
#define ETH_InternalInterruptEnabled   (0u)

#define ETH_TXBUFFERSIZE   (ETH_TX_BUFFER_SIZE)
#define ETH_RXBUFFERSIZE   (ETH_RX_BUFFER_SIZE)

#define ETH_TXBUFFER       ETH_txBuffer
#define ETH_RXBUFFER       ETH_rxBuffer

#endif /* (CY_SPIM_ETH_H) */


/* [] END OF FILE */
