/*******************************************************************************
* File Name: BOOT.h
* Version 3.30
*
* Description:
*  This file provides constants and parameter values for the I2C component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_I2C_BOOT_H)
#define CY_I2C_BOOT_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h"

/* Check if required defines such as CY_PSOC5LP are available in cy_boot */
#if !defined (CY_PSOC5LP)
    #error Component I2C_v3_30 requires cy_boot v3.10 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define BOOT_IMPLEMENTATION     (1u)
#define BOOT_MODE               (1u)
#define BOOT_ENABLE_WAKEUP      (0u)
#define BOOT_ADDR_DECODE        (1u)
#define BOOT_UDB_INTRN_CLOCK    (0u)


/* I2C implementation enum */
#define BOOT_UDB    (0x00u)
#define BOOT_FF     (0x01u)

#define BOOT_FF_IMPLEMENTED     (BOOT_FF  == BOOT_IMPLEMENTATION)
#define BOOT_UDB_IMPLEMENTED    (BOOT_UDB == BOOT_IMPLEMENTATION)

#define BOOT_UDB_INTRN_CLOCK_ENABLED    (BOOT_UDB_IMPLEMENTED && \
                                                     (0u != BOOT_UDB_INTRN_CLOCK))
/* I2C modes enum */
#define BOOT_MODE_SLAVE                 (0x01u)
#define BOOT_MODE_MASTER                (0x02u)
#define BOOT_MODE_MULTI_MASTER          (0x06u)
#define BOOT_MODE_MULTI_MASTER_SLAVE    (0x07u)
#define BOOT_MODE_MULTI_MASTER_MASK     (0x04u)

#define BOOT_MODE_SLAVE_ENABLED         (0u != (BOOT_MODE_SLAVE  & BOOT_MODE))
#define BOOT_MODE_MASTER_ENABLED        (0u != (BOOT_MODE_MASTER & BOOT_MODE))
#define BOOT_MODE_MULTI_MASTER_ENABLED  (0u != (BOOT_MODE_MULTI_MASTER_MASK & \
                                                            BOOT_MODE))
#define BOOT_MODE_MULTI_MASTER_SLAVE_ENABLED    (BOOT_MODE_MULTI_MASTER_SLAVE == \
                                                             BOOT_MODE)

/* Address detection enum */
#define BOOT_SW_DECODE      (0x00u)
#define BOOT_HW_DECODE      (0x01u)

#define BOOT_SW_ADRR_DECODE             (BOOT_SW_DECODE == BOOT_ADDR_DECODE)
#define BOOT_HW_ADRR_DECODE             (BOOT_HW_DECODE == BOOT_ADDR_DECODE)

/* Wakeup enabled */
#define BOOT_WAKEUP_ENABLED             (0u != BOOT_ENABLE_WAKEUP)

/* Adds bootloader APIs to component */
#define BOOT_BOOTLOADER_INTERFACE_ENABLED   (BOOT_MODE_SLAVE_ENABLED && \
                                                            ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_BOOT) || \
                                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)))

/* Timeout functionality */
#define BOOT_TIMEOUT_ENABLE             (0u)
#define BOOT_TIMEOUT_SCL_TMOUT_ENABLE   (0u)
#define BOOT_TIMEOUT_SDA_TMOUT_ENABLE   (0u)
#define BOOT_TIMEOUT_PRESCALER_ENABLE   (0u)
#define BOOT_TIMEOUT_IMPLEMENTATION     (0u)

/* Convert to boolean */
#define BOOT_TIMEOUT_ENABLED            (0u != BOOT_TIMEOUT_ENABLE)
#define BOOT_TIMEOUT_SCL_TMOUT_ENABLED  (0u != BOOT_TIMEOUT_SCL_TMOUT_ENABLE)
#define BOOT_TIMEOUT_SDA_TMOUT_ENABLED  (0u != BOOT_TIMEOUT_SDA_TMOUT_ENABLE)
#define BOOT_TIMEOUT_PRESCALER_ENABLED  (0u != BOOT_TIMEOUT_PRESCALER_ENABLE)

/* Timeout implementation enum. */
#define BOOT_TIMEOUT_UDB    (0x00u)
#define BOOT_TIMEOUT_FF     (0x01u)

#define BOOT_TIMEOUT_FF_IMPLEMENTED     (BOOT_TIMEOUT_FF  == \
                                                        BOOT_TIMEOUT_IMPLEMENTATION)
#define BOOT_TIMEOUT_UDB_IMPLEMENTED    (BOOT_TIMEOUT_UDB == \
                                                        BOOT_TIMEOUT_IMPLEMENTATION)

#define BOOT_TIMEOUT_FF_ENABLED         (BOOT_TIMEOUT_ENABLED && \
                                                     BOOT_TIMEOUT_FF_IMPLEMENTED && \
                                                     CY_PSOC5LP)

#define BOOT_TIMEOUT_UDB_ENABLED        (BOOT_TIMEOUT_ENABLED && \
                                                     BOOT_TIMEOUT_UDB_IMPLEMENTED)

#define BOOT_EXTERN_I2C_INTR_HANDLER    (0u)
#define BOOT_EXTERN_TMOUT_INTR_HANDLER  (0u)

#define BOOT_INTERN_I2C_INTR_HANDLER    (0u == BOOT_EXTERN_I2C_INTR_HANDLER)
#define BOOT_INTERN_TMOUT_INTR_HANDLER  (0u == BOOT_EXTERN_TMOUT_INTR_HANDLER)


/***************************************
*       Type defines
***************************************/

/* Structure to save registers before go to sleep */
typedef struct
{
    uint8 enableState;

    #if(BOOT_FF_IMPLEMENTED)
        uint8 xcfg;
        uint8 cfg;

        #if(BOOT_MODE_SLAVE_ENABLED)
            uint8 addr;
        #endif /* (BOOT_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            uint8 clkDiv;
        #else
            uint8 clkDiv1;
            uint8 clkDiv2;
        #endif /* (CY_PSOC5A) */

    #else
        uint8 control;

        #if(CY_UDB_V0)
            uint8 intMask;

            #if(BOOT_MODE_SLAVE_ENABLED)
                uint8 addr;
            #endif /* (BOOT_MODE_SLAVE_ENABLED) */
        #endif     /* (CY_UDB_V0) */

    #endif /* (BOOT_FF_IMPLEMENTED) */

    #if(BOOT_TIMEOUT_ENABLED)
        uint16 tmoutCfg;
        uint8  tmoutIntr;

        #if(BOOT_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0)
            uint8 tmoutPrd;
        #endif /* (BOOT_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0) */

    #endif /* (BOOT_TIMEOUT_ENABLED) */

} BOOT_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void BOOT_Init(void)                            ;
void BOOT_Enable(void)                          ;

void BOOT_Start(void)                           ;
void BOOT_Stop(void)                            ;

#define BOOT_EnableInt()        CyIntEnable      (BOOT_ISR_NUMBER)
#define BOOT_DisableInt()       CyIntDisable     (BOOT_ISR_NUMBER)
#define BOOT_ClearPendingInt()  CyIntClearPending(BOOT_ISR_NUMBER)
#define BOOT_SetPendingInt()    CyIntSetPending  (BOOT_ISR_NUMBER)

void BOOT_SaveConfig(void)                      ;
void BOOT_Sleep(void)                           ;
void BOOT_RestoreConfig(void)                   ;
void BOOT_Wakeup(void)                          ;

/* I2C Master functions prototypes */
#if(BOOT_MODE_MASTER_ENABLED)
    /* Read and Clear status functions */
    uint8 BOOT_MasterStatus(void)                ;
    uint8 BOOT_MasterClearStatus(void)           ;

    /* Interrupt based operation functions */
    uint8 BOOT_MasterWriteBuf(uint8 slaveAddress, uint8 * wrData, uint8 cnt, uint8 mode) \
                                                            ;
    uint8 BOOT_MasterReadBuf(uint8 slaveAddress, uint8 * rdData, uint8 cnt, uint8 mode) \
                                                            ;
    uint8 BOOT_MasterGetReadBufSize(void)       ;
    uint8 BOOT_MasterGetWriteBufSize(void)      ;
    void  BOOT_MasterClearReadBuf(void)         ;
    void  BOOT_MasterClearWriteBuf(void)        ;

    /* Manual operation functions */
    uint8 BOOT_MasterSendStart(uint8 slaveAddress, uint8 R_nW) \
                                                            ;
    uint8 BOOT_MasterSendRestart(uint8 slaveAddress, uint8 R_nW) \
                                                            ;
    uint8 BOOT_MasterSendStop(void)             ;
    uint8 BOOT_MasterWriteByte(uint8 theByte)   ;
    uint8 BOOT_MasterReadByte(uint8 acknNak)    ;

    /* This fake function use as workaround */
    void  BOOT_Workaround(void)                 ;

#endif /* (BOOT_MODE_MASTER_ENABLED) */

/* I2C Slave functions prototypes */
#if(BOOT_MODE_SLAVE_ENABLED)
    /* Read and Clear status functions */
    uint8 BOOT_SlaveStatus(void)                ;
    uint8 BOOT_SlaveClearReadStatus(void)       ;
    uint8 BOOT_SlaveClearWriteStatus(void)      ;

    void  BOOT_SlaveSetAddress(uint8 address)   ;

    /* Interrupt based operation functions */
    void  BOOT_SlaveInitReadBuf(uint8 * rdBuf, uint8 bufSize) \
                                                            ;
    void  BOOT_SlaveInitWriteBuf(uint8 * wrBuf, uint8 bufSize) \
                                                            ;
    uint8 BOOT_SlaveGetReadBufSize(void)        ;
    uint8 BOOT_SlaveGetWriteBufSize(void)       ;
    void  BOOT_SlaveClearReadBuf(void)          ;
    void  BOOT_SlaveClearWriteBuf(void)         ;

    /* Communication bootloader I2C Slave APIs */
    #if defined(CYDEV_BOOTLOADER_IO_COMP) && (BOOT_BOOTLOADER_INTERFACE_ENABLED)
        /* Physical layer functions */
        void     BOOT_CyBtldrCommStart(void) CYSMALL \
                                                            ;
        void     BOOT_CyBtldrCommStop(void)  CYSMALL \
                                                            ;
        void     BOOT_CyBtldrCommReset(void) CYSMALL \
                                                            ;
        cystatus BOOT_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) \
                                                        CYSMALL ;
        cystatus BOOT_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)  CYSMALL \
                                                            ;

        #if(CYDEV_BOOTLOADER_IO_COMP == CyBtldr_BOOT)
            #define CyBtldrCommStart    BOOT_CyBtldrCommStart
            #define CyBtldrCommStop     BOOT_CyBtldrCommStop
            #define CyBtldrCommReset    BOOT_CyBtldrCommReset
            #define CyBtldrCommWrite    BOOT_CyBtldrCommWrite
            #define CyBtldrCommRead     BOOT_CyBtldrCommRead
        #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_BOOT) */

        /* Size of Read/Write buffers for I2C bootloader  */
        #define BOOT_BTLDR_SIZEOF_READ_BUFFER   (0x80u)
        #define BOOT_BTLDR_SIZEOF_WRITE_BUFFER  (0x80u)
        #define BOOT_MIN_UNT16(a, b)            ( ((uint16)(a) < (b)) ? ((uint16) (a)) : ((uint16) (b)) )
        #define BOOT_WAIT_1_MS                  (1u)

    #endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (BOOT_BOOTLOADER_INTERFACE_ENABLED) */

#endif /* (BOOT_MODE_SLAVE_ENABLED) */

/* I2C interrupt handler */
CY_ISR_PROTO(BOOT_ISR);
#if((BOOT_FF_IMPLEMENTED) || (BOOT_WAKEUP_ENABLED))
    CY_ISR_PROTO(BOOT_WAKEUP_ISR);
#endif /* ((BOOT_FF_IMPLEMENTED) || (BOOT_WAKEUP_ENABLED)) */


/**********************************
*   Variable with external linkage
**********************************/

extern uint8 BOOT_initVar;


/***************************************
*   Initial Parameter Constants
***************************************/

#define BOOT_DATA_RATE          (100u)
#define BOOT_DEFAULT_ADDR       (6u)
#define BOOT_I2C_PAIR_SELECTED  (0u)

/* I2C pair enum */
#define BOOT_I2C_PAIR_ANY   (0x01u) /* Any pins for I2C */
#define BOOT_I2C_PAIR0      (0x01u) /* I2C0: (SCL = P12[4]) && (SCL = P12[5]) */
#define BOOT_I2C_PAIR1      (0x02u) /* I2C1: (SCL = P12[0]) && (SDA = P12[1]) */

#define BOOT_I2C1_SIO_PAIR  (BOOT_I2C_PAIR1 == BOOT_I2C_PAIR_SELECTED)
#define BOOT_I2C0_SIO_PAIR  (BOOT_I2C_PAIR0 == BOOT_I2C_PAIR_SELECTED)


/***************************************
*            API Constants
***************************************/

/* Master/Slave control constants */
#define BOOT_READ_XFER_MODE     (0x01u) /* Read */
#define BOOT_WRITE_XFER_MODE    (0x00u) /* Write */
#define BOOT_ACK_DATA           (0x01u) /* Send ACK */
#define BOOT_NAK_DATA           (0x00u) /* Send NAK */
#define BOOT_OVERFLOW_RETURN    (0xFFu) /* Senf on bus in case of overflow */

#if(BOOT_MODE_MASTER_ENABLED)
    /* "Mode" constants for MasterWriteBuf() or MasterReadBuf() function */
    #define BOOT_MODE_COMPLETE_XFER     (0x00u) /* Full transfer with Start and Stop */
    #define BOOT_MODE_REPEAT_START      (0x01u) /* Begin with a ReStart instead of a Start */
    #define BOOT_MODE_NO_STOP           (0x02u) /* Complete the transfer without a Stop */

    /* Master status */
    #define BOOT_MSTAT_CLEAR            (0x00u) /* Clear (init) status value */

    #define BOOT_MSTAT_RD_CMPLT         (0x01u) /* Read complete */
    #define BOOT_MSTAT_WR_CMPLT         (0x02u) /* Write complete */
    #define BOOT_MSTAT_XFER_INP         (0x04u) /* Master transfer in progress */
    #define BOOT_MSTAT_XFER_HALT        (0x08u) /* Transfer is halted */

    #define BOOT_MSTAT_ERR_MASK         (0xF0u) /* Mask for all errors */
    #define BOOT_MSTAT_ERR_SHORT_XFER   (0x10u) /* Master NAKed before end of packet */
    #define BOOT_MSTAT_ERR_ADDR_NAK     (0x20u) /* Slave did not ACK */
    #define BOOT_MSTAT_ERR_ARB_LOST     (0x40u) /* Master lost arbitration during communication */
    #define BOOT_MSTAT_ERR_XFER         (0x80u) /* Error during transfer */

    /* Master API returns */
    #define BOOT_MSTR_NO_ERROR          (0x00u) /* Function complete without error */
    #define BOOT_MSTR_BUS_BUSY          (0x01u) /* Bus is busy, process not started */
    #define BOOT_MSTR_NOT_READY         (0x02u) /* Master not Master on the bus or */
                                                            /*  Slave operation in progress */
    #define BOOT_MSTR_ERR_LB_NAK        (0x03u) /* Last Byte Naked */
    #define BOOT_MSTR_ERR_ARB_LOST      (0x04u) /* Master lost arbitration during communication */
    #define BOOT_MSTR_ERR_ABORT_START_GEN  (0x05u) /* Master did not generate Start, the Slave */
                                                               /* was addressed before */

#endif /* (BOOT_MODE_MASTER_ENABLED) */

#if(BOOT_MODE_SLAVE_ENABLED)
    /* Slave Status Constants */
    #define BOOT_SSTAT_RD_CMPLT     (0x01u) /* Read transfer complete */
    #define BOOT_SSTAT_RD_BUSY      (0x02u) /* Read transfer in progress */
    #define BOOT_SSTAT_RD_ERR_OVFL  (0x04u) /* Read overflow Error */
    #define BOOT_SSTAT_RD_MASK      (0x0Fu) /* Read Status Mask */
    #define BOOT_SSTAT_RD_NO_ERR    (0x00u) /* Read no Error */

    #define BOOT_SSTAT_WR_CMPLT     (0x10u) /* Write transfer complete */
    #define BOOT_SSTAT_WR_BUSY      (0x20u) /* Write transfer in progress */
    #define BOOT_SSTAT_WR_ERR_OVFL  (0x40u) /* Write overflow Error */
    #define BOOT_SSTAT_WR_MASK      (0xF0u) /* Write Status Mask  */
    #define BOOT_SSTAT_WR_NO_ERR    (0x00u) /* Write no Error */

    #define BOOT_SSTAT_RD_CLEAR     (0x0Du) /* Read Status clear */
    #define BOOT_SSTAT_WR_CLEAR     (0xD0u) /* Write Status Clear */

#endif /* (BOOT_MODE_SLAVE_ENABLED) */


/***************************************
*       I2C state machine constants
***************************************/

/* Default slave address states */
#define  BOOT_SM_IDLE           (0x10u) /* Default state - IDLE */
#define  BOOT_SM_EXIT_IDLE      (0x00u) /* Pass master and slave processing and go to IDLE */

/* Slave mode states */
#define  BOOT_SM_SLAVE          (BOOT_SM_IDLE) /* Any Slave state */
#define  BOOT_SM_SL_WR_DATA     (0x11u) /* Master writes data to slzve  */
#define  BOOT_SM_SL_RD_DATA     (0x12u) /* Master reads data from slave */

/* Master mode states */
#define  BOOT_SM_MASTER         (0x40u) /* Any master state */

#define  BOOT_SM_MSTR_RD        (0x08u) /* Any master read state          */
#define  BOOT_SM_MSTR_RD_ADDR   (0x49u) /* Master sends address with read */
#define  BOOT_SM_MSTR_RD_DATA   (0x4Au) /* Master reads data              */

#define  BOOT_SM_MSTR_WR        (0x04u) /* Any master read state           */
#define  BOOT_SM_MSTR_WR_ADDR   (0x45u) /* Master sends address with write */
#define  BOOT_SM_MSTR_WR_DATA   (0x46u) /* Master writes data              */

#define  BOOT_SM_MSTR_HALT      (0x60u) /* Master waits for ReStart */

#define BOOT_CHECK_SM_MASTER    (0u != (BOOT_SM_MASTER & BOOT_state))
#define BOOT_CHECK_SM_SLAVE     (0u != (BOOT_SM_SLAVE  & BOOT_state))


/***************************************
*              Registers
***************************************/

#if(BOOT_FF_IMPLEMENTED)
    /* Fixed Function registers */
    #define BOOT_XCFG_REG           (* (reg8 *) BOOT_I2C_FF__XCFG)
    #define BOOT_XCFG_PTR           (  (reg8 *) BOOT_I2C_FF__XCFG)

    #define BOOT_ADDR_REG           (* (reg8 *) BOOT_I2C_FF__ADR)
    #define BOOT_ADDR_PTR           (  (reg8 *) BOOT_I2C_FF__ADR)

    #define BOOT_CFG_REG            (* (reg8 *) BOOT_I2C_FF__CFG)
    #define BOOT_CFG_PTR            (  (reg8 *) BOOT_I2C_FF__CFG)

    #define BOOT_CSR_REG            (* (reg8 *) BOOT_I2C_FF__CSR)
    #define BOOT_CSR_PTR            (  (reg8 *) BOOT_I2C_FF__CSR)

    #define BOOT_DATA_REG           (* (reg8 *) BOOT_I2C_FF__D)
    #define BOOT_DATA_PTR           (  (reg8 *) BOOT_I2C_FF__D)

    #define BOOT_MCSR_REG           (* (reg8 *) BOOT_I2C_FF__MCSR)
    #define BOOT_MCSR_PTR           (  (reg8 *) BOOT_I2C_FF__MCSR)

    #define BOOT_ACT_PWRMGR_REG     (* (reg8 *) BOOT_I2C_FF__PM_ACT_CFG)
    #define BOOT_ACT_PWRMGR_PTR     (  (reg8 *) BOOT_I2C_FF__PM_ACT_CFG)
    #define BOOT_ACT_PWR_EN         (  (uint8)  BOOT_I2C_FF__PM_ACT_MSK)

    #define BOOT_STBY_PWRMGR_REG    (* (reg8 *) BOOT_I2C_FF__PM_STBY_CFG)
    #define BOOT_STBY_PWRMGR_PTR    (  (reg8 *) BOOT_I2C_FF__PM_STBY_CFG)
    #define BOOT_STBY_PWR_EN        (  (uint8)  BOOT_I2C_FF__PM_STBY_MSK)

    #define BOOT_PWRSYS_CR1_REG     (* (reg8 *) CYREG_PWRSYS_CR1)
    #define BOOT_PWRSYS_CR1_PTR     (  (reg8 *) CYREG_PWRSYS_CR1)

    /* Clock divider register depends on silicon */
    #if(CY_PSOC5A)
        #define BOOT_CLKDIV_REG     (* (reg8 *) BOOT_I2C_FF__CLK_DIV)
        #define BOOT_CLKDIV_PTR     (  (reg8 *) BOOT_I2C_FF__CLK_DIV)

    #else
        #define BOOT_CLKDIV1_REG    (* (reg8 *) BOOT_I2C_FF__CLK_DIV1)
        #define BOOT_CLKDIV1_PTR    (  (reg8 *) BOOT_I2C_FF__CLK_DIV1)

        #define BOOT_CLKDIV2_REG    (* (reg8 *) BOOT_I2C_FF__CLK_DIV2)
        #define BOOT_CLKDIV2_PTR    (  (reg8 *) BOOT_I2C_FF__CLK_DIV2)

    #endif /* (CY_PSOC5A) */

#else
    /* UDB implementation registers */
    #define BOOT_CFG_REG    (* (reg8 *) \
                                           BOOT_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)
    #define BOOT_CFG_PTR    (  (reg8 *) \
                                           BOOT_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)

    #define BOOT_CSR_REG        (* (reg8 *) BOOT_bI2C_UDB_StsReg__STATUS_REG)
    #define BOOT_CSR_PTR        (  (reg8 *) BOOT_bI2C_UDB_StsReg__STATUS_REG)

    #define BOOT_INT_MASK_REG   (* (reg8 *) BOOT_bI2C_UDB_StsReg__MASK_REG)
    #define BOOT_INT_MASK_PTR   (  (reg8 *) BOOT_bI2C_UDB_StsReg__MASK_REG)

    #define BOOT_INT_ENABLE_REG (* (reg8 *) BOOT_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)
    #define BOOT_INT_ENABLE_PTR (  (reg8 *) BOOT_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)

    #define BOOT_DATA_REG       (* (reg8 *) BOOT_bI2C_UDB_Shifter_u0__A0_REG)
    #define BOOT_DATA_PTR       (  (reg8 *) BOOT_bI2C_UDB_Shifter_u0__A0_REG)

    #define BOOT_GO_REG         (* (reg8 *) BOOT_bI2C_UDB_Shifter_u0__F1_REG)
    #define BOOT_GO_PTR         (  (reg8 *) BOOT_bI2C_UDB_Shifter_u0__F1_REG)

    #define BOOT_MCLK_PRD_REG   (* (reg8 *) BOOT_bI2C_UDB_Master_ClkGen_u0__D0_REG)
    #define BOOT_MCLK_PRD_PTR   (  (reg8 *) BOOT_bI2C_UDB_Master_ClkGen_u0__D0_REG)

    #define BOOT_MCLK_CMP_REG   (* (reg8 *) BOOT_bI2C_UDB_Master_ClkGen_u0__D1_REG)
    #define BOOT_MCLK_CMP_PTR   (  (reg8 *) BOOT_bI2C_UDB_Master_ClkGen_u0__D1_REG)

    #if(BOOT_MODE_SLAVE_ENABLED)
        #define BOOT_ADDR_REG       (* (reg8 *) BOOT_bI2C_UDB_Shifter_u0__D0_REG)
        #define BOOT_ADDR_PTR       (  (reg8 *) BOOT_bI2C_UDB_Shifter_u0__D0_REG)

        #define BOOT_PERIOD_REG     (* (reg8 *) BOOT_bI2C_UDB_Slave_BitCounter__PERIOD_REG)
        #define BOOT_PERIOD_PTR     (  (reg8 *) BOOT_bI2C_UDB_Slave_BitCounter__PERIOD_REG)

        #define BOOT_COUNTER_REG    (* (reg8 *) BOOT_bI2C_UDB_Slave_BitCounter__COUNT_REG)
        #define BOOT_COUNTER_PTR    (  (reg8 *) BOOT_bI2C_UDB_Slave_BitCounter__COUNT_REG)

        #define BOOT_COUNTER_AUX_CTL_REG  (* (reg8 *) \
                                                        BOOT_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG)
        #define BOOT_COUNTER_AUX_CTL_PTR  (  (reg8 *) \
                                                        BOOT_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG)

    #endif /* (BOOT_MODE_SLAVE_ENABLED) */

#endif /* (BOOT_FF_IMPLEMENTED) */


/***************************************
*        Registers Constants
***************************************/

/* BOOT_I2C_IRQ */
#define BOOT_ISR_NUMBER     ((uint8) BOOT_I2C_IRQ__INTC_NUMBER)
#define BOOT_ISR_PRIORITY   ((uint8) BOOT_I2C_IRQ__INTC_PRIOR_NUM)

/* I2C Slave Data Register */
#define BOOT_SLAVE_ADDR_MASK    (0x7Fu)
#define BOOT_SLAVE_ADDR_SHIFT   (0x01u)
#define BOOT_DATA_MASK          (0xFFu)
#define BOOT_READ_FLAG          (0x01u)

#define BOOT_FF_RESET_DELAY     (0x02u)

#if(BOOT_FF_IMPLEMENTED)
    /* XCFG I2C Extended Configuration Register */
    #define BOOT_XCFG_CLK_EN        (0x80u) /* Enable gated clock to block */
    #define BOOT_XCFG_I2C_ON        (0x40u) /* Enable I2C as wake up source*/
    #define BOOT_XCFG_RDY_TO_SLEEP  (0x20u) /* I2C ready go to sleep */
    #define BOOT_XCFG_FORCE_NACK    (0x10u) /* Force NACK all incomming transactions */
    #define BOOT_XCFG_NO_BC_INT     (0x08u) /* No interrupt on byte complete */
    #define BOOT_XCFG_BUF_MODE      (0x02u) /* Enable buffer mode */
    #define BOOT_XCFG_HDWR_ADDR_EN  (0x01u) /* Enable Hardware address match */

    /* CFG I2C Configuration Register */
    #define BOOT_CFG_SIO_SELECT     (0x80u) /* Pin Select for SCL/SDA lines */
    #define BOOT_CFG_PSELECT        (0x40u) /* Pin Select */
    #define BOOT_CFG_BUS_ERR_IE     (0x20u) /* Bus Error Interrupt Enable */
    #define BOOT_CFG_STOP_IE        (0x10u) /* Enable Interrupt on STOP condition */
    #define BOOT_CFG_CLK_RATE_MSK   (0x0Cu) /* Clock rate select  **CHECK**  */
    #define BOOT_CFG_CLK_RATE_100   (0x00u) /* Clock rate select 100K */
    #define BOOT_CFG_CLK_RATE_400   (0x04u) /* Clock rate select 400K */
    #define BOOT_CFG_CLK_RATE_050   (0x08u) /* Clock rate select 50K  */
    #define BOOT_CFG_CLK_RATE_RSVD  (0x0Cu) /* Clock rate select Invalid */
    #define BOOT_CFG_EN_MSTR        (0x02u) /* Enable Master operation */
    #define BOOT_CFG_EN_SLAVE       (0x01u) /* Enable Slave operation */

    #define BOOT_CFG_CLK_RATE_LESS_EQUAL_50 (0x04u) /* Clock rate select <= 50kHz */
    #define BOOT_CFG_CLK_RATE_GRATER_50     (0x00u) /* Clock rate select > 50kHz */

    /* CSR I2C Control and Status Register */
    #define BOOT_CSR_BUS_ERROR      (0x80u) /* Active high when bus error has occured */
    #define BOOT_CSR_LOST_ARB       (0x40u) /* Set to 1 if lost arbitration in host mode */
    #define BOOT_CSR_STOP_STATUS    (0x20u) /* Set if Stop has been detected */
    #define BOOT_CSR_ACK            (0x10u) /* ACK response */
    #define BOOT_CSR_NAK            (0x00u) /* NAK response */
    #define BOOT_CSR_ADDRESS        (0x08u) /* Set in firmware 0 = status bit, 1 Address is slave */
    #define BOOT_CSR_TRANSMIT       (0x04u) /* Set in firmware 1 = transmit, 0 = receive */
    #define BOOT_CSR_LRB            (0x02u) /* Last received bit */
    #define BOOT_CSR_LRB_ACK        (0x00u) /* Last received bit was an ACK */
    #define BOOT_CSR_LRB_NAK        (0x02u) /* Last received bit was an NAK */
    #define BOOT_CSR_BYTE_COMPLETE  (0x01u) /* Informs that last byte has been sent */
    #define BOOT_CSR_STOP_GEN       (0x00u) /* Generate a stop condition */
    #define BOOT_CSR_RDY_TO_RD      (0x00u) /* Set to recieve mode */

    /* MCSR I2C Master Control and Status Register */
    #define BOOT_MCSR_STOP_GEN      (0x10u) /* Firmware sets this bit to initiate a Stop condition */
    #define BOOT_MCSR_BUS_BUSY      (0x08u) /* Status bit, Set at Start and cleared at Stop condition */
    #define BOOT_MCSR_MSTR_MODE     (0x04u) /* Status bit, Set at Start and cleared at Stop condition */
    #define BOOT_MCSR_RESTART_GEN   (0x02u) /* Firmware sets this bit to initiate a ReStart condition */
    #define BOOT_MCSR_START_GEN     (0x01u) /* Firmware sets this bit to initiate a Start condition */

    /* CLK_DIV I2C Clock Divide Factor Register */
    #define BOOT_CLK_DIV_MSK    (0x07u) /* Status bit, Set at Start and cleared at Stop condition */
    #define BOOT_CLK_DIV_1      (0x00u) /* Divide input clock by  1 */
    #define BOOT_CLK_DIV_2      (0x01u) /* Divide input clock by  2 */
    #define BOOT_CLK_DIV_4      (0x02u) /* Divide input clock by  4 */
    #define BOOT_CLK_DIV_8      (0x03u) /* Divide input clock by  8 */
    #define BOOT_CLK_DIV_16     (0x04u) /* Divide input clock by 16 */
    #define BOOT_CLK_DIV_32     (0x05u) /* Divide input clock by 32 */
    #define BOOT_CLK_DIV_64     (0x06u) /* Divide input clock by 64 */

    /* PWRSYS_CR1 to handle Sleep */
    #define BOOT_PWRSYS_CR1_I2C_REG_BACKUP  (0x04u) /* Enables, power to I2C regs while sleep */

#else
    /* CONTROL REG bits location */
    #define BOOT_CTRL_START_SHIFT           (7u)
    #define BOOT_CTRL_STOP_SHIFT            (6u)
    #define BOOT_CTRL_RESTART_SHIFT         (5u)
    #define BOOT_CTRL_NACK_SHIFT            (4u)
    #define BOOT_CTRL_ANY_ADDRESS_SHIFT     (3u)
    #define BOOT_CTRL_TRANSMIT_SHIFT        (2u)
    #define BOOT_CTRL_ENABLE_MASTER_SHIFT   (1u)
    #define BOOT_CTRL_ENABLE_SLAVE_SHIFT    (0u)
    #define BOOT_CTRL_START_MASK            ((uint8) (0x01u << BOOT_CTRL_START_SHIFT))
    #define BOOT_CTRL_STOP_MASK             ((uint8) (0x01u << BOOT_CTRL_STOP_SHIFT))
    #define BOOT_CTRL_RESTART_MASK          ((uint8) (0x01u << BOOT_CTRL_RESTART_SHIFT))
    #define BOOT_CTRL_NACK_MASK             ((uint8) (0x01u << BOOT_CTRL_NACK_SHIFT))
    #define BOOT_CTRL_ANY_ADDRESS_MASK      ((uint8) (0x01u << BOOT_CTRL_ANY_ADDRESS_SHIFT))
    #define BOOT_CTRL_TRANSMIT_MASK         ((uint8) (0x01u << BOOT_CTRL_TRANSMIT_SHIFT))
    #define BOOT_CTRL_ENABLE_MASTER_MASK    ((uint8) (0x01u << BOOT_CTRL_ENABLE_MASTER_SHIFT))
    #define BOOT_CTRL_ENABLE_SLAVE_MASK     ((uint8) (0x01u << BOOT_CTRL_ENABLE_SLAVE_SHIFT))

    /* STATUS REG bits location */
    #define BOOT_STS_LOST_ARB_SHIFT         (6u)
    #define BOOT_STS_STOP_SHIFT             (5u)
    #define BOOT_STS_BUSY_SHIFT             (4u)
    #define BOOT_STS_ADDR_SHIFT             (3u)
    #define BOOT_STS_MASTER_MODE_SHIFT      (2u)
    #define BOOT_STS_LRB_SHIFT              (1u)
    #define BOOT_STS_BYTE_COMPLETE_SHIFT    (0u)
    #define BOOT_STS_LOST_ARB_MASK          ((uint8) (0x01u << BOOT_STS_LOST_ARB_SHIFT))
    #define BOOT_STS_STOP_MASK              ((uint8) (0x01u << BOOT_STS_STOP_SHIFT))
    #define BOOT_STS_BUSY_MASK              ((uint8) (0x01u << BOOT_STS_BUSY_SHIFT))
    #define BOOT_STS_ADDR_MASK              ((uint8) (0x01u << BOOT_STS_ADDR_SHIFT))
    #define BOOT_STS_MASTER_MODE_MASK       ((uint8) (0x01u << BOOT_STS_MASTER_MODE_SHIFT))
    #define BOOT_STS_LRB_MASK               ((uint8) (0x01u << BOOT_STS_LRB_SHIFT))
    #define BOOT_STS_BYTE_COMPLETE_MASK     ((uint8) (0x01u << BOOT_STS_BYTE_COMPLETE_SHIFT))

    /* AUX_CTL bits definition */
    #define BOOT_COUNTER_ENABLE_MASK        (0x20u) /* Enable 7-bit counter     */
    #define BOOT_INT_ENABLE_MASK            (0x10u) /* Enable intr from statusi */
    #define BOOT_CNT7_ENABLE                (BOOT_COUNTER_ENABLE_MASK)
    #define BOOT_INTR_ENABLE                (BOOT_INT_ENABLE_MASK)

#endif /* (BOOT_FF_IMPLEMENTED) */


/***************************************
*        Marco
***************************************/

/* ACK and NACK for data and address checks */
#define BOOT_CHECK_ADDR_ACK(csr)    ((BOOT_CSR_LRB_ACK | BOOT_CSR_ADDRESS) == \
                                                 ((BOOT_CSR_LRB    | BOOT_CSR_ADDRESS) &  \
                                                  (csr)))


#define BOOT_CHECK_ADDR_NAK(csr)    ((BOOT_CSR_LRB_NAK | BOOT_CSR_ADDRESS) == \
                                                 ((BOOT_CSR_LRB    | BOOT_CSR_ADDRESS) &  \
                                                  (csr)))

#define BOOT_CHECK_DATA_ACK(csr)    (0u == ((csr) & BOOT_CSR_LRB_NAK))

/* MCSR conditions check */
#define BOOT_CHECK_BUS_FREE(mcsr)       (0u == ((mcsr) & BOOT_MCSR_BUS_BUSY))
#define BOOT_CHECK_MASTER_MODE(mcsr)    (0u != ((mcsr) & BOOT_MCSR_MSTR_MODE))

/* CSR conditions check */
#define BOOT_WAIT_BYTE_COMPLETE(csr)    (0u == ((csr) & BOOT_CSR_BYTE_COMPLETE))
#define BOOT_WAIT_STOP_COMPLETE(csr)    (0u == ((csr) & (BOOT_CSR_BYTE_COMPLETE | \
                                                                     BOOT_CSR_STOP_STATUS)))
#define BOOT_CHECK_BYTE_COMPLETE(csr)   (0u != ((csr) & BOOT_CSR_BYTE_COMPLETE))
#define BOOT_CHECK_STOP_STS(csr)        (0u != ((csr) & BOOT_CSR_STOP_STATUS))
#define BOOT_CHECK_LOST_ARB(csr)        (0u != ((csr) & BOOT_CSR_LOST_ARB))
#define BOOT_CHECK_ADDRESS_STS(csr)     (0u != ((csr) & BOOT_CSR_ADDRESS))

/* Software start and end of transaction check */
#define BOOT_CHECK_RESTART(mstrCtrl)    (0u != ((mstrCtrl) & BOOT_MODE_REPEAT_START))
#define BOOT_CHECK_NO_STOP(mstrCtrl)    (0u != ((mstrCtrl) & BOOT_MODE_NO_STOP))

/* Send read or write completion depends on state */
#define BOOT_GET_MSTAT_CMPLT ((0u != (BOOT_state & BOOT_SM_MSTR_RD)) ? \
                                                 (BOOT_MSTAT_RD_CMPLT) : (BOOT_MSTAT_WR_CMPLT))

/* Returns 7-bit slave address and used for software address match */
#define BOOT_GET_SLAVE_ADDR(dataReg)   (((dataReg) >> BOOT_SLAVE_ADDR_SHIFT) & \
                                                                  BOOT_SLAVE_ADDR_MASK)

#if(BOOT_FF_IMPLEMENTED)
    /* Check enable of module */
    #define BOOT_I2C_ENABLE_REG     (BOOT_ACT_PWRMGR_REG)
    #define BOOT_IS_I2C_ENABLE(reg) (0u != ((reg) & BOOT_ACT_PWR_EN))
    #define BOOT_IS_ENABLED         (0u != (BOOT_ACT_PWRMGR_REG & BOOT_ACT_PWR_EN))

    #define BOOT_CHECK_PWRSYS_I2C_BACKUP    (0u != (BOOT_PWRSYS_CR1_I2C_REG_BACKUP & \
                                                                BOOT_PWRSYS_CR1_REG))

    /* Check start condition generation */
    #define BOOT_CHECK_START_GEN(mcsr)  ((0u != ((mcsr) & BOOT_MCSR_START_GEN)) && \
                                                     (0u == ((mcsr) & BOOT_MCSR_MSTR_MODE)))

    #define BOOT_CLEAR_START_GEN        do{ \
                                                        BOOT_MCSR_REG &=                                   \
                                                                           ((uint8) ~BOOT_MCSR_START_GEN); \
                                                    }while(0)

    /* Stop interrupt */
    #define BOOT_ENABLE_INT_ON_STOP     do{ \
                                                        BOOT_CFG_REG |= BOOT_CFG_STOP_IE; \
                                                    }while(0)

    #define BOOT_DISABLE_INT_ON_STOP    do{ \
                                                        BOOT_CFG_REG &=                                 \
                                                                           ((uint8) ~BOOT_CFG_STOP_IE); \
                                                    }while(0)

    /* Transmit data */
    #define BOOT_TRANSMIT_DATA          do{ \
                                                        BOOT_CSR_REG = BOOT_CSR_TRANSMIT; \
                                                    }while(0)

    #define BOOT_ACK_AND_TRANSMIT       do{ \
                                                        BOOT_CSR_REG = (BOOT_CSR_ACK |      \
                                                                                    BOOT_CSR_TRANSMIT); \
                                                    }while(0)

    #define BOOT_NAK_AND_TRANSMIT       do{ \
                                                        BOOT_CSR_REG = BOOT_CSR_NAK; \
                                                    }while(0)

    /* Special case: udb needs to ack, ff needs to nak */
    #define BOOT_ACKNAK_AND_TRANSMIT    do{ \
                                                        BOOT_CSR_REG  = (BOOT_CSR_NAK |      \
                                                                                     BOOT_CSR_TRANSMIT); \
                                                    }while(0)
    /* Receive data */
    #define BOOT_ACK_AND_RECEIVE        do{ \
                                                        BOOT_CSR_REG = BOOT_CSR_ACK; \
                                                    }while(0)

    #define BOOT_NAK_AND_RECEIVE        do{ \
                                                        BOOT_CSR_REG = BOOT_CSR_NAK; \
                                                    }while(0)

    #define BOOT_READY_TO_READ          do{ \
                                                        BOOT_CSR_REG = BOOT_CSR_RDY_TO_RD; \
                                                    }while(0)

    /* Master condition generation */
    #define BOOT_GENERATE_START         do{ \
                                                        BOOT_MCSR_REG = BOOT_MCSR_START_GEN; \
                                                    }while(0)

    #if(CY_PSOC5A)
        #define BOOT_GENERATE_RESTART \
                        do{ \
                            BOOT_MCSR_REG = BOOT_MCSR_RESTART_GEN; \
                            BOOT_CSR_REG  = BOOT_CSR_NAK;          \
                        }while(0)

        #define BOOT_GENERATE_STOP      do{ \
                                                        BOOT_CSR_REG = BOOT_CSR_STOP_GEN; \
                                                    }while(0)

    #else   /* PSoC3 ES3 handlees zero lenght packets */
        #define BOOT_GENERATE_RESTART \
                        do{ \
                            BOOT_MCSR_REG = (BOOT_MCSR_RESTART_GEN | \
                                                         BOOT_MCSR_STOP_GEN);    \
                            BOOT_CSR_REG  = BOOT_CSR_TRANSMIT;       \
                        }while(0)

        #define BOOT_GENERATE_STOP \
                        do{ \
                            BOOT_MCSR_REG = BOOT_MCSR_STOP_GEN; \
                            BOOT_CSR_REG  = BOOT_CSR_TRANSMIT;  \
                        }while(0)
    #endif /* (CY_PSOC5A) */

    /* Master manual APIs compatible defines */
    #define BOOT_GENERATE_RESTART_MANUAL    BOOT_GENERATE_RESTART
    #define BOOT_GENERATE_STOP_MANUAL       BOOT_GENERATE_STOP
    #define BOOT_TRANSMIT_DATA_MANUAL       BOOT_TRANSMIT_DATA
    #define BOOT_READY_TO_READ_MANUAL       BOOT_READY_TO_READ
    #define BOOT_ACK_AND_RECEIVE_MANUAL     BOOT_ACK_AND_RECEIVE

#else

    /* Masks to enalbe interrupts from Status register */
    #define BOOT_STOP_IE_MASK           (BOOT_STS_STOP_MASK)
    #define BOOT_BYTE_COMPLETE_IE_MASK  (BOOT_STS_BYTE_COMPLETE_MASK)

    /* FF compatibility: CSR gegisters definitions */
    #define BOOT_CSR_LOST_ARB       (BOOT_STS_LOST_ARB_MASK)
    #define BOOT_CSR_STOP_STATUS    (BOOT_STS_STOP_MASK)
    #define BOOT_CSR_BUS_ERROR      (0x00u)
    #define BOOT_CSR_ADDRESS        (BOOT_STS_ADDR_MASK)
    #define BOOT_CSR_TRANSMIT       (BOOT_CTRL_TRANSMIT_MASK)
    #define BOOT_CSR_LRB            (BOOT_STS_LRB_MASK)
    #define BOOT_CSR_LRB_NAK        (BOOT_STS_LRB_MASK)
    #define BOOT_CSR_LRB_ACK        (0x00u)
    #define BOOT_CSR_BYTE_COMPLETE  (BOOT_STS_BYTE_COMPLETE_MASK)

    /* FF compatibility: MCSR gegisters definitions */
    #define BOOT_MCSR_REG           (BOOT_CSR_REG)   /* UDB incoporates master and slave regs */
    #define BOOT_MCSR_BUS_BUSY      (BOOT_STS_BUSY_MASK)       /* Is bus is busy              */
    #define BOOT_MCSR_START_GEN     (BOOT_CTRL_START_MASK)     /* Generate Sart condition     */
    #define BOOT_MCSR_RESTART_GEN   (BOOT_CTRL_RESTART_MASK)   /* Generates RESTART condition */
    #define BOOT_MCSR_MSTR_MODE     (BOOT_STS_MASTER_MODE_MASK)/* Define if active Master     */

    /* Data to write into TX FIFO to release FSM */
    #define BOOT_RELEASE_FSM         (0x00u)
    
    /* Check enable of module */
    #define BOOT_I2C_ENABLE_REG     (BOOT_CFG_REG)
    #define BOOT_IS_I2C_ENABLE(reg) ((0u != ((reg) & BOOT_ENABLE_MASTER)) || \
                                                 (0u != ((reg) & BOOT_ENABLE_SLAVE)))

    #define BOOT_IS_ENABLED         (0u != (BOOT_CFG_REG & BOOT_ENABLE_MS))

    /* Check start condition generation */
    #define BOOT_CHECK_START_GEN(mcsr)  ((0u != (BOOT_CFG_REG &        \
                                                             BOOT_MCSR_START_GEN)) \
                                                    &&                                         \
                                                    (0u == ((mcsr) & BOOT_MCSR_MSTR_MODE)))

    #define BOOT_CLEAR_START_GEN        do{ \
                                                        BOOT_CFG_REG &=                 \
                                                        ((uint8) ~BOOT_MCSR_START_GEN); \
                                                    }while(0)


    /* Stop interrupt */
    #define BOOT_ENABLE_INT_ON_STOP     do{ \
                                                       BOOT_INT_MASK_REG |= BOOT_STOP_IE_MASK; \
                                                    }while(0)

    #define BOOT_DISABLE_INT_ON_STOP    do{ \
                                                        BOOT_INT_MASK_REG &=                               \
                                                                             ((uint8) ~BOOT_STOP_IE_MASK); \
                                                    }while(0)


    /* Transmit data */
    #define BOOT_TRANSMIT_DATA      do{ \
                                                    BOOT_CFG_REG = (BOOT_CTRL_TRANSMIT_MASK | \
                                                                                BOOT_CTRL_DEFAULT);       \
                                                    BOOT_GO_REG  = BOOT_RELEASE_FSM;          \
                                                }while(0)

    #define BOOT_ACK_AND_TRANSMIT   BOOT_TRANSMIT_DATA


    #define BOOT_NAK_AND_TRANSMIT   do{ \
                                                    BOOT_CFG_REG = (BOOT_CTRL_NACK_MASK     | \
                                                                                BOOT_CTRL_TRANSMIT_MASK | \
                                                                                BOOT_CTRL_DEFAULT);       \
                                                    BOOT_GO_REG  =  BOOT_RELEASE_FSM;         \
                                                }while(0)

    /* Receive data */
    #define BOOT_READY_TO_READ      do{ \
                                                    BOOT_CFG_REG = BOOT_CTRL_DEFAULT; \
                                                    BOOT_GO_REG  =  BOOT_RELEASE_FSM; \
                                                }while(0)

    #define BOOT_ACK_AND_RECEIVE    BOOT_READY_TO_READ

    #define BOOT_NAK_AND_RECEIVE    do{ \
                                                    BOOT_CFG_REG = (BOOT_CTRL_NACK_MASK | \
                                                                                BOOT_CTRL_DEFAULT);   \
                                                    BOOT_GO_REG  =  BOOT_RELEASE_FSM;     \
                                                }while(0)

    /* Master condition generation */
    #define BOOT_GENERATE_START     do{ \
                                                    BOOT_CFG_REG = (BOOT_CTRL_START_MASK | \
                                                                                 BOOT_CTRL_DEFAULT);   \
                                                    BOOT_GO_REG  =  BOOT_RELEASE_FSM;      \
                                                }while(0)

    #define BOOT_GENERATE_RESTART   do{ \
                                                    BOOT_CFG_REG = (BOOT_CTRL_RESTART_MASK | \
                                                                                BOOT_CTRL_NACK_MASK    | \
                                                                                BOOT_CTRL_DEFAULT);      \
                                                    BOOT_GO_REG  =  BOOT_RELEASE_FSM;        \
                                                }while(0)


    #define BOOT_GENERATE_STOP      do{ \
                                                    BOOT_CFG_REG = (BOOT_CTRL_NACK_MASK | \
                                                                                BOOT_CTRL_STOP_MASK | \
                                                                                BOOT_CTRL_DEFAULT);   \
                                                    BOOT_GO_REG  =  BOOT_RELEASE_FSM;     \
                                                }while(0)

    /* Master manual APIs compatible defines */
    /* These defines wait while byte complete is cleared after command issued */
    #define BOOT_GENERATE_RESTART_MANUAL    \
                                        do{             \
                                            BOOT_GENERATE_RESTART;                                    \
                                            while(BOOT_CHECK_BYTE_COMPLETE(BOOT_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define BOOT_GENERATE_STOP_MANUAL   \
                                        do{         \
                                            BOOT_GENERATE_STOP;                                       \
                                            while(BOOT_CHECK_BYTE_COMPLETE(BOOT_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define BOOT_TRANSMIT_DATA_MANUAL   \
                                        do{         \
                                            BOOT_TRANSMIT_DATA;                                       \
                                            while(BOOT_CHECK_BYTE_COMPLETE(BOOT_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define BOOT_READY_TO_READ_MANUAL   \
                                        do{         \
                                            BOOT_READY_TO_READ;      \
                                            while(BOOT_CHECK_BYTE_COMPLETE(BOOT_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define BOOT_ACK_AND_RECEIVE_MANUAL \
                                        do{         \
                                            BOOT_ACK_AND_RECEIVE;                                     \
                                            while(BOOT_CHECK_BYTE_COMPLETE(BOOT_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)
#endif /* (BOOT_FF_IMPLEMENTED) */

/* Comon for FF and UDB: used to release bus after lost arb */
#define BOOT_BUS_RELEASE    BOOT_READY_TO_READ


/***************************************
*     Default register init constants
***************************************/

#define BOOT_DISABLE    (0u)
#define BOOT_ENABLE     (1u)

#if(BOOT_FF_IMPLEMENTED)
    /* BOOT_XCFG_REG: bits definition */
    #define BOOT_DEFAULT_XCFG_HW_ADDR_EN ((BOOT_HW_ADRR_DECODE) ? \
                                                        (BOOT_XCFG_HDWR_ADDR_EN) : (0u))

    #define BOOT_DEFAULT_XCFG_I2C_ON    ((BOOT_WAKEUP_ENABLED) ? \
                                                        (BOOT_XCFG_I2C_ON) : (0u))


    #define BOOT_DEFAULT_CFG_SIO_SELECT ((BOOT_I2C1_SIO_PAIR) ? \
                                                        (BOOT_CFG_SIO_SELECT) : (0u))


    /* BOOT_CFG_REG: bits definition */
    #define BOOT_DEFAULT_CFG_PSELECT    ((BOOT_WAKEUP_ENABLED) ? \
                                                        (BOOT_CFG_PSELECT) : (0u))

    #define BOOT_DEFAULT_CLK_RATE0  ((BOOT_DATA_RATE <= 50u) ?        \
                                                    (BOOT_CFG_CLK_RATE_050) :     \
                                                    ((BOOT_DATA_RATE <= 100u) ?   \
                                                        (BOOT_CFG_CLK_RATE_100) : \
                                                        (BOOT_CFG_CLK_RATE_400)))

    #define BOOT_DEFAULT_CLK_RATE1  ((BOOT_DATA_RATE <= 50u) ?           \
                                                 (BOOT_CFG_CLK_RATE_LESS_EQUAL_50) : \
                                                 (BOOT_CFG_CLK_RATE_GRATER_50))

    #define BOOT_DEFAULT_CLK_RATE   ((CY_PSOC5A) ? (BOOT_DEFAULT_CLK_RATE0) : \
                                                               (BOOT_DEFAULT_CLK_RATE1))


    #define BOOT_ENABLE_MASTER      ((BOOT_MODE_MASTER_ENABLED) ? \
                                                 (BOOT_CFG_EN_MSTR) : (0u))

    #define BOOT_ENABLE_SLAVE       ((BOOT_MODE_SLAVE_ENABLED) ? \
                                                 (BOOT_CFG_EN_SLAVE) : (0u))

    #define BOOT_ENABLE_MS      (BOOT_ENABLE_MASTER | BOOT_ENABLE_SLAVE)


    /* BOOT_DEFAULT_XCFG_REG */
    #define BOOT_DEFAULT_XCFG   (BOOT_XCFG_CLK_EN         | \
                                             BOOT_DEFAULT_XCFG_I2C_ON | \
                                             BOOT_DEFAULT_XCFG_HW_ADDR_EN)

    /* BOOT_DEFAULT_CFG_REG */
    #define BOOT_DEFAULT_CFG    (BOOT_DEFAULT_CFG_SIO_SELECT | \
                                             BOOT_DEFAULT_CFG_PSELECT    | \
                                             BOOT_DEFAULT_CLK_RATE       | \
                                             BOOT_ENABLE_MASTER          | \
                                             BOOT_ENABLE_SLAVE)

    /*BOOT_DEFAULT_DIVIDE_FACTOR_REG */
    #define BOOT_DEFAULT_DIVIDE_FACTOR  ((CY_PSOC5A) ? ((uint8) 3u) : ((uint16) 15u))

#else
    /* BOOT_CFG_REG: bits definition  */
    #define BOOT_ENABLE_MASTER  ((BOOT_MODE_MASTER_ENABLED) ? \
                                             (BOOT_CTRL_ENABLE_MASTER_MASK) : (0u))

    #define BOOT_ENABLE_SLAVE   ((BOOT_MODE_SLAVE_ENABLED) ? \
                                             (BOOT_CTRL_ENABLE_SLAVE_MASK) : (0u))

    #define BOOT_ENABLE_MS      (BOOT_ENABLE_MASTER | BOOT_ENABLE_SLAVE)


    #define BOOT_DEFAULT_CTRL_ANY_ADDR   ((BOOT_HW_ADRR_DECODE) ? \
                                                      (0u) : (BOOT_CTRL_ANY_ADDRESS_MASK))

    /* BOOT_DEFAULT_CFG_REG */
    #define BOOT_DEFAULT_CFG    (BOOT_DEFAULT_CTRL_ANY_ADDR)

    /* All CTRL default bits to be used in macro */
    #define BOOT_CTRL_DEFAULT   (BOOT_DEFAULT_CTRL_ANY_ADDR | BOOT_ENABLE_MS)

    /* Master clock generator: d0 and d1 */
    #define BOOT_MCLK_PERIOD_VALUE  (0x0Fu)
    #define BOOT_MCLK_COMPARE_VALUE (0x08u)

    /* Slave bit-counter: contorol period */
    #define BOOT_PERIOD_VALUE       (0x07u)

    /* BOOT_DEFAULT_INT_MASK */
    #define BOOT_DEFAULT_INT_MASK   (BOOT_BYTE_COMPLETE_IE_MASK)

    /* BOOT_DEFAULT_MCLK_PRD_REG */
    #define BOOT_DEFAULT_MCLK_PRD   (BOOT_MCLK_PERIOD_VALUE)

    /* BOOT_DEFAULT_MCLK_CMP_REG */
    #define BOOT_DEFAULT_MCLK_CMP   (BOOT_MCLK_COMPARE_VALUE)

    /* BOOT_DEFAULT_PERIOD_REG */
    #define BOOT_DEFAULT_PERIOD     (BOOT_PERIOD_VALUE)

#endif /* (BOOT_FF_IMPLEMENTED) */


/***************************************
*       Obsolete
***************************************/

/* Following code are OBSOLETE and must not be used 
 * starting from I2C 3.20
 */
 
#define BOOT_SSTAT_RD_ERR       (0x08u)
#define BOOT_SSTAT_WR_ERR       (0x80u)
#define BOOT_MSTR_SLAVE_BUSY    (BOOT_MSTR_NOT_READY)
#define BOOT_MSTAT_ERR_BUF_OVFL (0x80u)
#define BOOT_SSTAT_RD_CMPT      (BOOT_SSTAT_RD_CMPLT)
#define BOOT_SSTAT_WR_CMPT      (BOOT_SSTAT_WR_CMPLT)
#define BOOT_MODE_MULTI_MASTER_ENABLE    (BOOT_MODE_MULTI_MASTER_MASK)
#define BOOT_DATA_RATE_50       (50u)
#define BOOT_DATA_RATE_100      (100u)
#define BOOT_DEV_MASK           (0xF0u)
#define BOOT_SM_SL_STOP         (0x14u)
#define BOOT_SM_MASTER_IDLE     (0x40u)
#define BOOT_HDWR_DECODE        (0x01u)

#endif /* CY_I2C_BOOT_H */


/* [] END OF FILE */
