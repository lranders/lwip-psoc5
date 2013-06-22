/*******************************************************************************
* File Name: ETC.h
* Version 2.0
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_ETC_H)
#define CY_CLOCK_ETC_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
* Conditional Compilation Parameters
***************************************/

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component cy_clock_v2_0 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*        Function Prototypes
***************************************/

void ETC_Start(void) ;
void ETC_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void ETC_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void ETC_StandbyPower(uint8 state) ;
void ETC_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 ETC_GetDividerRegister(void) ;
void ETC_SetModeRegister(uint8 modeBitMask) ;
void ETC_ClearModeRegister(uint8 modeBitMask) ;
uint8 ETC_GetModeRegister(void) ;
void ETC_SetSourceRegister(uint8 clkSource) ;
uint8 ETC_GetSourceRegister(void) ;
#if defined(ETC__CFG3)
void ETC_SetPhaseRegister(uint8 clkPhase) ;
uint8 ETC_GetPhaseRegister(void) ;
#endif /* defined(ETC__CFG3) */

#define ETC_Enable()                       ETC_Start()
#define ETC_Disable()                      ETC_Stop()
#define ETC_SetDivider(clkDivider)         ETC_SetDividerRegister(clkDivider, 1)
#define ETC_SetDividerValue(clkDivider)    ETC_SetDividerRegister((clkDivider) - 1, 1)
#define ETC_SetMode(clkMode)               ETC_SetModeRegister(clkMode)
#define ETC_SetSource(clkSource)           ETC_SetSourceRegister(clkSource)
#if defined(ETC__CFG3)
#define ETC_SetPhase(clkPhase)             ETC_SetPhaseRegister(clkPhase)
#define ETC_SetPhaseValue(clkPhase)        ETC_SetPhaseRegister((clkPhase) + 1)
#endif /* defined(ETC__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define ETC_CLKEN              (* (reg8 *) ETC__PM_ACT_CFG)
#define ETC_CLKEN_PTR          ((reg8 *) ETC__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define ETC_CLKSTBY            (* (reg8 *) ETC__PM_STBY_CFG)
#define ETC_CLKSTBY_PTR        ((reg8 *) ETC__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define ETC_DIV_LSB            (* (reg8 *) ETC__CFG0)
#define ETC_DIV_LSB_PTR        ((reg8 *) ETC__CFG0)
#define ETC_DIV_PTR            ((reg16 *) ETC__CFG0)

/* Clock MSB divider configuration register. */
#define ETC_DIV_MSB            (* (reg8 *) ETC__CFG1)
#define ETC_DIV_MSB_PTR        ((reg8 *) ETC__CFG1)

/* Mode and source configuration register */
#define ETC_MOD_SRC            (* (reg8 *) ETC__CFG2)
#define ETC_MOD_SRC_PTR        ((reg8 *) ETC__CFG2)

#if defined(ETC__CFG3)
/* Analog clock phase configuration register */
#define ETC_PHASE              (* (reg8 *) ETC__CFG3)
#define ETC_PHASE_PTR          ((reg8 *) ETC__CFG3)
#endif /* defined(ETC__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define ETC_CLKEN_MASK         ETC__PM_ACT_MSK
#define ETC_CLKSTBY_MASK       ETC__PM_STBY_MSK

/* CFG2 field masks */
#define ETC_SRC_SEL_MSK        ETC__CFG2_SRC_SEL_MASK
#define ETC_MODE_MASK          (~(ETC_SRC_SEL_MSK))

#if defined(ETC__CFG3)
/* CFG3 phase mask */
#define ETC_PHASE_MASK         ETC__CFG3_PHASE_DLY_MASK
#endif /* defined(ETC__CFG3) */

#endif /* CY_CLOCK_ETC_H */


/* [] END OF FILE */
