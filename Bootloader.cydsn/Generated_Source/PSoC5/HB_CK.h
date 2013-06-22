/*******************************************************************************
* File Name: HB_CK.h
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

#if !defined(CY_CLOCK_HB_CK_H)
#define CY_CLOCK_HB_CK_H

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

void HB_CK_Start(void) ;
void HB_CK_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void HB_CK_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void HB_CK_StandbyPower(uint8 state) ;
void HB_CK_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 HB_CK_GetDividerRegister(void) ;
void HB_CK_SetModeRegister(uint8 modeBitMask) ;
void HB_CK_ClearModeRegister(uint8 modeBitMask) ;
uint8 HB_CK_GetModeRegister(void) ;
void HB_CK_SetSourceRegister(uint8 clkSource) ;
uint8 HB_CK_GetSourceRegister(void) ;
#if defined(HB_CK__CFG3)
void HB_CK_SetPhaseRegister(uint8 clkPhase) ;
uint8 HB_CK_GetPhaseRegister(void) ;
#endif /* defined(HB_CK__CFG3) */

#define HB_CK_Enable()                       HB_CK_Start()
#define HB_CK_Disable()                      HB_CK_Stop()
#define HB_CK_SetDivider(clkDivider)         HB_CK_SetDividerRegister(clkDivider, 1)
#define HB_CK_SetDividerValue(clkDivider)    HB_CK_SetDividerRegister((clkDivider) - 1, 1)
#define HB_CK_SetMode(clkMode)               HB_CK_SetModeRegister(clkMode)
#define HB_CK_SetSource(clkSource)           HB_CK_SetSourceRegister(clkSource)
#if defined(HB_CK__CFG3)
#define HB_CK_SetPhase(clkPhase)             HB_CK_SetPhaseRegister(clkPhase)
#define HB_CK_SetPhaseValue(clkPhase)        HB_CK_SetPhaseRegister((clkPhase) + 1)
#endif /* defined(HB_CK__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define HB_CK_CLKEN              (* (reg8 *) HB_CK__PM_ACT_CFG)
#define HB_CK_CLKEN_PTR          ((reg8 *) HB_CK__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define HB_CK_CLKSTBY            (* (reg8 *) HB_CK__PM_STBY_CFG)
#define HB_CK_CLKSTBY_PTR        ((reg8 *) HB_CK__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define HB_CK_DIV_LSB            (* (reg8 *) HB_CK__CFG0)
#define HB_CK_DIV_LSB_PTR        ((reg8 *) HB_CK__CFG0)
#define HB_CK_DIV_PTR            ((reg16 *) HB_CK__CFG0)

/* Clock MSB divider configuration register. */
#define HB_CK_DIV_MSB            (* (reg8 *) HB_CK__CFG1)
#define HB_CK_DIV_MSB_PTR        ((reg8 *) HB_CK__CFG1)

/* Mode and source configuration register */
#define HB_CK_MOD_SRC            (* (reg8 *) HB_CK__CFG2)
#define HB_CK_MOD_SRC_PTR        ((reg8 *) HB_CK__CFG2)

#if defined(HB_CK__CFG3)
/* Analog clock phase configuration register */
#define HB_CK_PHASE              (* (reg8 *) HB_CK__CFG3)
#define HB_CK_PHASE_PTR          ((reg8 *) HB_CK__CFG3)
#endif /* defined(HB_CK__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define HB_CK_CLKEN_MASK         HB_CK__PM_ACT_MSK
#define HB_CK_CLKSTBY_MASK       HB_CK__PM_STBY_MSK

/* CFG2 field masks */
#define HB_CK_SRC_SEL_MSK        HB_CK__CFG2_SRC_SEL_MASK
#define HB_CK_MODE_MASK          (~(HB_CK_SRC_SEL_MSK))

#if defined(HB_CK__CFG3)
/* CFG3 phase mask */
#define HB_CK_PHASE_MASK         HB_CK__CFG3_PHASE_DLY_MASK
#endif /* defined(HB_CK__CFG3) */

#endif /* CY_CLOCK_HB_CK_H */


/* [] END OF FILE */
