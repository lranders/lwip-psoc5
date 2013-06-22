/*******************************************************************************
* File Name: LDON_CK.h
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

#if !defined(CY_CLOCK_LDON_CK_H)
#define CY_CLOCK_LDON_CK_H

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

void LDON_CK_Start(void) ;
void LDON_CK_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void LDON_CK_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void LDON_CK_StandbyPower(uint8 state) ;
void LDON_CK_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 LDON_CK_GetDividerRegister(void) ;
void LDON_CK_SetModeRegister(uint8 modeBitMask) ;
void LDON_CK_ClearModeRegister(uint8 modeBitMask) ;
uint8 LDON_CK_GetModeRegister(void) ;
void LDON_CK_SetSourceRegister(uint8 clkSource) ;
uint8 LDON_CK_GetSourceRegister(void) ;
#if defined(LDON_CK__CFG3)
void LDON_CK_SetPhaseRegister(uint8 clkPhase) ;
uint8 LDON_CK_GetPhaseRegister(void) ;
#endif /* defined(LDON_CK__CFG3) */

#define LDON_CK_Enable()                       LDON_CK_Start()
#define LDON_CK_Disable()                      LDON_CK_Stop()
#define LDON_CK_SetDivider(clkDivider)         LDON_CK_SetDividerRegister(clkDivider, 1)
#define LDON_CK_SetDividerValue(clkDivider)    LDON_CK_SetDividerRegister((clkDivider) - 1, 1)
#define LDON_CK_SetMode(clkMode)               LDON_CK_SetModeRegister(clkMode)
#define LDON_CK_SetSource(clkSource)           LDON_CK_SetSourceRegister(clkSource)
#if defined(LDON_CK__CFG3)
#define LDON_CK_SetPhase(clkPhase)             LDON_CK_SetPhaseRegister(clkPhase)
#define LDON_CK_SetPhaseValue(clkPhase)        LDON_CK_SetPhaseRegister((clkPhase) + 1)
#endif /* defined(LDON_CK__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define LDON_CK_CLKEN              (* (reg8 *) LDON_CK__PM_ACT_CFG)
#define LDON_CK_CLKEN_PTR          ((reg8 *) LDON_CK__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define LDON_CK_CLKSTBY            (* (reg8 *) LDON_CK__PM_STBY_CFG)
#define LDON_CK_CLKSTBY_PTR        ((reg8 *) LDON_CK__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define LDON_CK_DIV_LSB            (* (reg8 *) LDON_CK__CFG0)
#define LDON_CK_DIV_LSB_PTR        ((reg8 *) LDON_CK__CFG0)
#define LDON_CK_DIV_PTR            ((reg16 *) LDON_CK__CFG0)

/* Clock MSB divider configuration register. */
#define LDON_CK_DIV_MSB            (* (reg8 *) LDON_CK__CFG1)
#define LDON_CK_DIV_MSB_PTR        ((reg8 *) LDON_CK__CFG1)

/* Mode and source configuration register */
#define LDON_CK_MOD_SRC            (* (reg8 *) LDON_CK__CFG2)
#define LDON_CK_MOD_SRC_PTR        ((reg8 *) LDON_CK__CFG2)

#if defined(LDON_CK__CFG3)
/* Analog clock phase configuration register */
#define LDON_CK_PHASE              (* (reg8 *) LDON_CK__CFG3)
#define LDON_CK_PHASE_PTR          ((reg8 *) LDON_CK__CFG3)
#endif /* defined(LDON_CK__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define LDON_CK_CLKEN_MASK         LDON_CK__PM_ACT_MSK
#define LDON_CK_CLKSTBY_MASK       LDON_CK__PM_STBY_MSK

/* CFG2 field masks */
#define LDON_CK_SRC_SEL_MSK        LDON_CK__CFG2_SRC_SEL_MASK
#define LDON_CK_MODE_MASK          (~(LDON_CK_SRC_SEL_MSK))

#if defined(LDON_CK__CFG3)
/* CFG3 phase mask */
#define LDON_CK_PHASE_MASK         LDON_CK__CFG3_PHASE_DLY_MASK
#endif /* defined(LDON_CK__CFG3) */

#endif /* CY_CLOCK_LDON_CK_H */


/* [] END OF FILE */
