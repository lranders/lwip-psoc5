/*******************************************************************************
* File Name: URT.h
* Version 2.30
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_URT_H)
#define CY_UART_URT_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define URT_RX_ENABLED                     (1u)
#define URT_TX_ENABLED                     (1u)
#define URT_HD_ENABLED                     (0u)
#define URT_RX_INTERRUPT_ENABLED           (0u)
#define URT_TX_INTERRUPT_ENABLED           (0u)
#define URT_INTERNAL_CLOCK_USED            (1u)
#define URT_RXHW_ADDRESS_ENABLED           (0u)
#define URT_OVER_SAMPLE_COUNT              (8u)
#define URT_PARITY_TYPE                    (0u)
#define URT_PARITY_TYPE_SW                 (0u)
#define URT_BREAK_DETECT                   (0u)
#define URT_BREAK_BITS_TX                  (13u)
#define URT_BREAK_BITS_RX                  (13u)
#define URT_TXCLKGEN_DP                    (1u)
#define URT_USE23POLLING                   (1u)
#define URT_FLOW_CONTROL                   (0u)
#define URT_CLK_FREQ                       (0u)
#define URT_TXBUFFERSIZE                   (4u)
#define URT_RXBUFFERSIZE                   (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef URT_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define URT_CONTROL_REG_REMOVED            (0u)
#else
    #define URT_CONTROL_REG_REMOVED            (1u)
#endif /* End URT_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct URT_backupStruct_
{
    uint8 enableState;

    #if(URT_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End URT_CONTROL_REG_REMOVED */
    #if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (URT_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End URT_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (URT_RX_ENABLED) || (URT_HD_ENABLED)*/

    #if(URT_TX_ENABLED)
        #if(URT_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End URT_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End URT_TX_ENABLED */
} URT_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void URT_Start(void) ;
void URT_Stop(void) ;
uint8 URT_ReadControlRegister(void) ;
void URT_WriteControlRegister(uint8 control) ;

void URT_Init(void) ;
void URT_Enable(void) ;
void URT_SaveConfig(void) ;
void URT_RestoreConfig(void) ;
void URT_Sleep(void) ;
void URT_Wakeup(void) ;

/* Only if RX is enabled */
#if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )

    #if(URT_RX_INTERRUPT_ENABLED)
        void  URT_EnableRxInt(void) ;
        void  URT_DisableRxInt(void) ;
        CY_ISR_PROTO(URT_RXISR);
    #endif /* URT_RX_INTERRUPT_ENABLED */

    void URT_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void URT_SetRxAddress1(uint8 address) ;
    void URT_SetRxAddress2(uint8 address) ;

    void  URT_SetRxInterruptMode(uint8 intSrc) ;
    uint8 URT_ReadRxData(void) ;
    uint8 URT_ReadRxStatus(void) ;
    uint8 URT_GetChar(void) ;
    uint16 URT_GetByte(void) ;
    uint8 URT_GetRxBufferSize(void)
                                                            ;
    void URT_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define URT_GetRxInterruptSource   URT_ReadRxStatus

#endif /* End (URT_RX_ENABLED) || (URT_HD_ENABLED) */

/* Only if TX is enabled */
#if(URT_TX_ENABLED || URT_HD_ENABLED)

    #if(URT_TX_INTERRUPT_ENABLED)
        void URT_EnableTxInt(void) ;
        void URT_DisableTxInt(void) ;
        CY_ISR_PROTO(URT_TXISR);
    #endif /* URT_TX_INTERRUPT_ENABLED */

    void URT_SetTxInterruptMode(uint8 intSrc) ;
    void URT_WriteTxData(uint8 txDataByte) ;
    uint8 URT_ReadTxStatus(void) ;
    void URT_PutChar(uint8 txDataByte) ;
    void URT_PutString(const char8 string[]) ;
    void URT_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void URT_PutCRLF(uint8 txDataByte) ;
    void URT_ClearTxBuffer(void) ;
    void URT_SetTxAddressMode(uint8 addressMode) ;
    void URT_SendBreak(uint8 retMode) ;
    uint8 URT_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define URT_PutStringConst         URT_PutString
    #define URT_PutArrayConst          URT_PutArray
    #define URT_GetTxInterruptSource   URT_ReadTxStatus

#endif /* End URT_TX_ENABLED || URT_HD_ENABLED */

#if(URT_HD_ENABLED)
    void URT_LoadRxConfig(void) ;
    void URT_LoadTxConfig(void) ;
#endif /* End URT_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_URT) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    URT_CyBtldrCommStart(void) CYSMALL ;
    void    URT_CyBtldrCommStop(void) CYSMALL ;
    void    URT_CyBtldrCommReset(void) CYSMALL ;
    cystatus URT_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus URT_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_URT)
        #define CyBtldrCommStart    URT_CyBtldrCommStart
        #define CyBtldrCommStop     URT_CyBtldrCommStop
        #define CyBtldrCommReset    URT_CyBtldrCommReset
        #define CyBtldrCommWrite    URT_CyBtldrCommWrite
        #define CyBtldrCommRead     URT_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_URT) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define URT_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define URT_SET_SPACE                              (0x00u)
#define URT_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (URT_TX_ENABLED) || (URT_HD_ENABLED) )
    #if(URT_TX_INTERRUPT_ENABLED)
        #define URT_TX_VECT_NUM            (uint8)URT_TXInternalInterrupt__INTC_NUMBER
        #define URT_TX_PRIOR_NUM           (uint8)URT_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* URT_TX_INTERRUPT_ENABLED */
    #if(URT_TX_ENABLED)
        #define URT_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define URT_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define URT_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define URT_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* URT_TX_ENABLED */
    #if(URT_HD_ENABLED)
        #define URT_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define URT_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define URT_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define URT_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* URT_HD_ENABLED */
    #define URT_TX_STS_COMPLETE            (uint8)(0x01u << URT_TX_STS_COMPLETE_SHIFT)
    #define URT_TX_STS_FIFO_EMPTY          (uint8)(0x01u << URT_TX_STS_FIFO_EMPTY_SHIFT)
    #define URT_TX_STS_FIFO_FULL           (uint8)(0x01u << URT_TX_STS_FIFO_FULL_SHIFT)
    #define URT_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << URT_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (URT_TX_ENABLED) || (URT_HD_ENABLED)*/

#if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )
    #if(URT_RX_INTERRUPT_ENABLED)
        #define URT_RX_VECT_NUM            (uint8)URT_RXInternalInterrupt__INTC_NUMBER
        #define URT_RX_PRIOR_NUM           (uint8)URT_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* URT_RX_INTERRUPT_ENABLED */
    #define URT_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define URT_RX_STS_BREAK_SHIFT             (0x01u)
    #define URT_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define URT_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define URT_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define URT_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define URT_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define URT_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define URT_RX_STS_MRKSPC           (uint8)(0x01u << URT_RX_STS_MRKSPC_SHIFT)
    #define URT_RX_STS_BREAK            (uint8)(0x01u << URT_RX_STS_BREAK_SHIFT)
    #define URT_RX_STS_PAR_ERROR        (uint8)(0x01u << URT_RX_STS_PAR_ERROR_SHIFT)
    #define URT_RX_STS_STOP_ERROR       (uint8)(0x01u << URT_RX_STS_STOP_ERROR_SHIFT)
    #define URT_RX_STS_OVERRUN          (uint8)(0x01u << URT_RX_STS_OVERRUN_SHIFT)
    #define URT_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << URT_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define URT_RX_STS_ADDR_MATCH       (uint8)(0x01u << URT_RX_STS_ADDR_MATCH_SHIFT)
    #define URT_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << URT_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define URT_RX_HW_MASK                     (0x7Fu)
#endif /* End (URT_RX_ENABLED) || (URT_HD_ENABLED) */

/* Control Register definitions */
#define URT_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define URT_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define URT_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define URT_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define URT_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define URT_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define URT_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define URT_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define URT_CTRL_HD_SEND               (uint8)(0x01u << URT_CTRL_HD_SEND_SHIFT)
#define URT_CTRL_HD_SEND_BREAK         (uint8)(0x01u << URT_CTRL_HD_SEND_BREAK_SHIFT)
#define URT_CTRL_MARK                  (uint8)(0x01u << URT_CTRL_MARK_SHIFT)
#define URT_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << URT_CTRL_PARITY_TYPE0_SHIFT)
#define URT_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << URT_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define URT_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define URT_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define URT_SEND_BREAK                         (0x00u)
#define URT_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define URT_REINIT                             (0x02u)
#define URT_SEND_WAIT_REINIT                   (0x03u)

#define URT_OVER_SAMPLE_8                      (8u)
#define URT_OVER_SAMPLE_16                     (16u)

#define URT_BIT_CENTER                         (URT_OVER_SAMPLE_COUNT - 1u)

#define URT_FIFO_LENGTH                        (4u)
#define URT_NUMBER_OF_START_BIT                (1u)
#define URT_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define URT_TXBITCTR_BREAKBITS8X   ((URT_BREAK_BITS_TX * URT_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define URT_TXBITCTR_BREAKBITS ((URT_BREAK_BITS_TX * URT_OVER_SAMPLE_COUNT) - 1u)

#define URT_HALF_BIT_COUNT   \
                            (((URT_OVER_SAMPLE_COUNT / 2u) + (URT_USE23POLLING * 1u)) - 2u)
#if (URT_OVER_SAMPLE_COUNT == URT_OVER_SAMPLE_8)
    #define URT_HD_TXBITCTR_INIT   (((URT_BREAK_BITS_TX + \
                            URT_NUMBER_OF_START_BIT) * URT_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define URT_RXBITCTR_INIT  ((((URT_BREAK_BITS_RX + URT_NUMBER_OF_START_BIT) \
                            * URT_OVER_SAMPLE_COUNT) + URT_HALF_BIT_COUNT) - 1u)


#else /* URT_OVER_SAMPLE_COUNT == URT_OVER_SAMPLE_16 */
    #define URT_HD_TXBITCTR_INIT   ((8u * URT_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define URT_RXBITCTR_INIT      (((7u * URT_OVER_SAMPLE_COUNT) - 1u) + \
                                                      URT_HALF_BIT_COUNT)
#endif /* End URT_OVER_SAMPLE_COUNT */
#define URT_HD_RXBITCTR_INIT                   URT_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 URT_initVar;
#if( URT_TX_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH))
    extern volatile uint8 URT_txBuffer[URT_TXBUFFERSIZE];
    extern volatile uint8 URT_txBufferRead;
    extern uint8 URT_txBufferWrite;
#endif /* End URT_TX_ENABLED */
#if( ( URT_RX_ENABLED || URT_HD_ENABLED ) && \
     (URT_RXBUFFERSIZE > URT_FIFO_LENGTH) )
    extern volatile uint8 URT_rxBuffer[URT_RXBUFFERSIZE];
    extern volatile uint8 URT_rxBufferRead;
    extern volatile uint8 URT_rxBufferWrite;
    extern volatile uint8 URT_rxBufferLoopDetect;
    extern volatile uint8 URT_rxBufferOverflow;
    #if (URT_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 URT_rxAddressMode;
        extern volatile uint8 URT_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End URT_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define URT__B_UART__AM_SW_BYTE_BYTE 1
#define URT__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define URT__B_UART__AM_HW_BYTE_BY_BYTE 3
#define URT__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define URT__B_UART__AM_NONE 0

#define URT__B_UART__NONE_REVB 0
#define URT__B_UART__EVEN_REVB 1
#define URT__B_UART__ODD_REVB 2
#define URT__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define URT_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define URT_NUMBER_OF_STOP_BITS    (1u)

#if (URT_RXHW_ADDRESS_ENABLED)
    #define URT_RXADDRESSMODE      (0u)
    #define URT_RXHWADDRESS1       (0u)
    #define URT_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define URT_RXAddressMode      URT_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define URT_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << URT_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << URT_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << URT_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << URT_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << URT_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << URT_RX_STS_BREAK_SHIFT) \
                                        | (0 << URT_RX_STS_OVERRUN_SHIFT))

#define URT_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << URT_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << URT_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << URT_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << URT_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef URT_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define URT_CONTROL_REG \
                            (* (reg8 *) URT_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define URT_CONTROL_PTR \
                            (  (reg8 *) URT_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End URT_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(URT_TX_ENABLED)
    #define URT_TXDATA_REG          (* (reg8 *) URT_BUART_sTX_TxShifter_u0__F0_REG)
    #define URT_TXDATA_PTR          (  (reg8 *) URT_BUART_sTX_TxShifter_u0__F0_REG)
    #define URT_TXDATA_AUX_CTL_REG  (* (reg8 *) URT_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define URT_TXDATA_AUX_CTL_PTR  (  (reg8 *) URT_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define URT_TXSTATUS_REG        (* (reg8 *) URT_BUART_sTX_TxSts__STATUS_REG)
    #define URT_TXSTATUS_PTR        (  (reg8 *) URT_BUART_sTX_TxSts__STATUS_REG)
    #define URT_TXSTATUS_MASK_REG   (* (reg8 *) URT_BUART_sTX_TxSts__MASK_REG)
    #define URT_TXSTATUS_MASK_PTR   (  (reg8 *) URT_BUART_sTX_TxSts__MASK_REG)
    #define URT_TXSTATUS_ACTL_REG   (* (reg8 *) URT_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define URT_TXSTATUS_ACTL_PTR   (  (reg8 *) URT_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(URT_TXCLKGEN_DP)
        #define URT_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) URT_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define URT_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) URT_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define URT_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) URT_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define URT_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) URT_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define URT_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) URT_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define URT_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) URT_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define URT_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) URT_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define URT_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) URT_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define URT_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) URT_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define URT_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) URT_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* URT_TXCLKGEN_DP */

#endif /* End URT_TX_ENABLED */

#if(URT_HD_ENABLED)

    #define URT_TXDATA_REG             (* (reg8 *) URT_BUART_sRX_RxShifter_u0__F1_REG )
    #define URT_TXDATA_PTR             (  (reg8 *) URT_BUART_sRX_RxShifter_u0__F1_REG )
    #define URT_TXDATA_AUX_CTL_REG     (* (reg8 *) URT_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define URT_TXDATA_AUX_CTL_PTR     (  (reg8 *) URT_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define URT_TXSTATUS_REG           (* (reg8 *) URT_BUART_sRX_RxSts__STATUS_REG )
    #define URT_TXSTATUS_PTR           (  (reg8 *) URT_BUART_sRX_RxSts__STATUS_REG )
    #define URT_TXSTATUS_MASK_REG      (* (reg8 *) URT_BUART_sRX_RxSts__MASK_REG )
    #define URT_TXSTATUS_MASK_PTR      (  (reg8 *) URT_BUART_sRX_RxSts__MASK_REG )
    #define URT_TXSTATUS_ACTL_REG      (* (reg8 *) URT_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define URT_TXSTATUS_ACTL_PTR      (  (reg8 *) URT_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End URT_HD_ENABLED */

#if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )
    #define URT_RXDATA_REG             (* (reg8 *) URT_BUART_sRX_RxShifter_u0__F0_REG )
    #define URT_RXDATA_PTR             (  (reg8 *) URT_BUART_sRX_RxShifter_u0__F0_REG )
    #define URT_RXADDRESS1_REG         (* (reg8 *) URT_BUART_sRX_RxShifter_u0__D0_REG )
    #define URT_RXADDRESS1_PTR         (  (reg8 *) URT_BUART_sRX_RxShifter_u0__D0_REG )
    #define URT_RXADDRESS2_REG         (* (reg8 *) URT_BUART_sRX_RxShifter_u0__D1_REG )
    #define URT_RXADDRESS2_PTR         (  (reg8 *) URT_BUART_sRX_RxShifter_u0__D1_REG )
    #define URT_RXDATA_AUX_CTL_REG     (* (reg8 *) URT_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define URT_RXBITCTR_PERIOD_REG    (* (reg8 *) URT_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define URT_RXBITCTR_PERIOD_PTR    (  (reg8 *) URT_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define URT_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) URT_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define URT_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) URT_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define URT_RXBITCTR_COUNTER_REG   (* (reg8 *) URT_BUART_sRX_RxBitCounter__COUNT_REG )
    #define URT_RXBITCTR_COUNTER_PTR   (  (reg8 *) URT_BUART_sRX_RxBitCounter__COUNT_REG )

    #define URT_RXSTATUS_REG           (* (reg8 *) URT_BUART_sRX_RxSts__STATUS_REG )
    #define URT_RXSTATUS_PTR           (  (reg8 *) URT_BUART_sRX_RxSts__STATUS_REG )
    #define URT_RXSTATUS_MASK_REG      (* (reg8 *) URT_BUART_sRX_RxSts__MASK_REG )
    #define URT_RXSTATUS_MASK_PTR      (  (reg8 *) URT_BUART_sRX_RxSts__MASK_REG )
    #define URT_RXSTATUS_ACTL_REG      (* (reg8 *) URT_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define URT_RXSTATUS_ACTL_PTR      (  (reg8 *) URT_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (URT_RX_ENABLED) || (URT_HD_ENABLED) */

#if(URT_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define URT_INTCLOCK_CLKEN_REG     (* (reg8 *) URT_IntClock__PM_ACT_CFG)
    #define URT_INTCLOCK_CLKEN_PTR     (  (reg8 *) URT_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define URT_INTCLOCK_CLKEN_MASK    URT_IntClock__PM_ACT_MSK
#endif /* End URT_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(URT_TX_ENABLED)
    #define URT_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End URT_TX_ENABLED */

#if(URT_HD_ENABLED)
    #define URT_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End URT_HD_ENABLED */

#if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )
    #define URT_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (URT_RX_ENABLED) || (URT_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define URT_initvar                    URT_initVar

#define URT_RX_Enabled                 URT_RX_ENABLED
#define URT_TX_Enabled                 URT_TX_ENABLED
#define URT_HD_Enabled                 URT_HD_ENABLED
#define URT_RX_IntInterruptEnabled     URT_RX_INTERRUPT_ENABLED
#define URT_TX_IntInterruptEnabled     URT_TX_INTERRUPT_ENABLED
#define URT_InternalClockUsed          URT_INTERNAL_CLOCK_USED
#define URT_RXHW_Address_Enabled       URT_RXHW_ADDRESS_ENABLED
#define URT_OverSampleCount            URT_OVER_SAMPLE_COUNT
#define URT_ParityType                 URT_PARITY_TYPE

#if( URT_TX_ENABLED && (URT_TXBUFFERSIZE > URT_FIFO_LENGTH))
    #define URT_TXBUFFER               URT_txBuffer
    #define URT_TXBUFFERREAD           URT_txBufferRead
    #define URT_TXBUFFERWRITE          URT_txBufferWrite
#endif /* End URT_TX_ENABLED */
#if( ( URT_RX_ENABLED || URT_HD_ENABLED ) && \
     (URT_RXBUFFERSIZE > URT_FIFO_LENGTH) )
    #define URT_RXBUFFER               URT_rxBuffer
    #define URT_RXBUFFERREAD           URT_rxBufferRead
    #define URT_RXBUFFERWRITE          URT_rxBufferWrite
    #define URT_RXBUFFERLOOPDETECT     URT_rxBufferLoopDetect
    #define URT_RXBUFFER_OVERFLOW      URT_rxBufferOverflow
#endif /* End URT_RX_ENABLED */

#ifdef URT_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define URT_CONTROL                URT_CONTROL_REG
#endif /* End URT_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(URT_TX_ENABLED)
    #define URT_TXDATA                 URT_TXDATA_REG
    #define URT_TXSTATUS               URT_TXSTATUS_REG
    #define URT_TXSTATUS_MASK          URT_TXSTATUS_MASK_REG
    #define URT_TXSTATUS_ACTL          URT_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(URT_TXCLKGEN_DP)
        #define URT_TXBITCLKGEN_CTR        URT_TXBITCLKGEN_CTR_REG
        #define URT_TXBITCLKTX_COMPLETE    URT_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define URT_TXBITCTR_PERIOD        URT_TXBITCTR_PERIOD_REG
        #define URT_TXBITCTR_CONTROL       URT_TXBITCTR_CONTROL_REG
        #define URT_TXBITCTR_COUNTER       URT_TXBITCTR_COUNTER_REG
    #endif /* URT_TXCLKGEN_DP */
#endif /* End URT_TX_ENABLED */

#if(URT_HD_ENABLED)
    #define URT_TXDATA                 URT_TXDATA_REG
    #define URT_TXSTATUS               URT_TXSTATUS_REG
    #define URT_TXSTATUS_MASK          URT_TXSTATUS_MASK_REG
    #define URT_TXSTATUS_ACTL          URT_TXSTATUS_ACTL_REG
#endif /* End URT_HD_ENABLED */

#if( (URT_RX_ENABLED) || (URT_HD_ENABLED) )
    #define URT_RXDATA                 URT_RXDATA_REG
    #define URT_RXADDRESS1             URT_RXADDRESS1_REG
    #define URT_RXADDRESS2             URT_RXADDRESS2_REG
    #define URT_RXBITCTR_PERIOD        URT_RXBITCTR_PERIOD_REG
    #define URT_RXBITCTR_CONTROL       URT_RXBITCTR_CONTROL_REG
    #define URT_RXBITCTR_COUNTER       URT_RXBITCTR_COUNTER_REG
    #define URT_RXSTATUS               URT_RXSTATUS_REG
    #define URT_RXSTATUS_MASK          URT_RXSTATUS_MASK_REG
    #define URT_RXSTATUS_ACTL          URT_RXSTATUS_ACTL_REG
#endif /* End  (URT_RX_ENABLED) || (URT_HD_ENABLED) */

#if(URT_INTERNAL_CLOCK_USED)
    #define URT_INTCLOCK_CLKEN         URT_INTCLOCK_CLKEN_REG
#endif /* End URT_INTERNAL_CLOCK_USED */

#define URT_WAIT_FOR_COMLETE_REINIT    URT_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_URT_H */


/* [] END OF FILE */
