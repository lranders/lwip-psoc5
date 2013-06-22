/*******************************************************************************
* File Name: ETH_IntClock.h
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

#if !defined(CY_CLOCK_ETH_IntClock_H)
#define CY_CLOCK_ETH_IntClock_H

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

void ETH_IntClock_Start(void) ;
void ETH_IntClock_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void ETH_IntClock_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void ETH_IntClock_StandbyPower(uint8 state) ;
void ETH_IntClock_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 ETH_IntClock_GetDividerRegister(void) ;
void ETH_IntClock_SetModeRegister(uint8 modeBitMask) ;
void ETH_IntClock_ClearModeRegister(uint8 modeBitMask) ;
uint8 ETH_IntClock_GetModeRegister(void) ;
void ETH_IntClock_SetSourceRegister(uint8 clkSource) ;
uint8 ETH_IntClock_GetSourceRegister(void) ;
#if defined(ETH_IntClock__CFG3)
void ETH_IntClock_SetPhaseRegister(uint8 clkPhase) ;
uint8 ETH_IntClock_GetPhaseRegister(void) ;
#endif /* defined(ETH_IntClock__CFG3) */

#define ETH_IntClock_Enable()                       ETH_IntClock_Start()
#define ETH_IntClock_Disable()                      ETH_IntClock_Stop()
#define ETH_IntClock_SetDivider(clkDivider)         ETH_IntClock_SetDividerRegister(clkDivider, 1)
#define ETH_IntClock_SetDividerValue(clkDivider)    ETH_IntClock_SetDividerRegister((clkDivider) - 1, 1)
#define ETH_IntClock_SetMode(clkMode)               ETH_IntClock_SetModeRegister(clkMode)
#define ETH_IntClock_SetSource(clkSource)           ETH_IntClock_SetSourceRegister(clkSource)
#if defined(ETH_IntClock__CFG3)
#define ETH_IntClock_SetPhase(clkPhase)             ETH_IntClock_SetPhaseRegister(clkPhase)
#define ETH_IntClock_SetPhaseValue(clkPhase)        ETH_IntClock_SetPhaseRegister((clkPhase) + 1)
#endif /* defined(ETH_IntClock__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define ETH_IntClock_CLKEN              (* (reg8 *) ETH_IntClock__PM_ACT_CFG)
#define ETH_IntClock_CLKEN_PTR          ((reg8 *) ETH_IntClock__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define ETH_IntClock_CLKSTBY            (* (reg8 *) ETH_IntClock__PM_STBY_CFG)
#define ETH_IntClock_CLKSTBY_PTR        ((reg8 *) ETH_IntClock__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define ETH_IntClock_DIV_LSB            (* (reg8 *) ETH_IntClock__CFG0)
#define ETH_IntClock_DIV_LSB_PTR        ((reg8 *) ETH_IntClock__CFG0)
#define ETH_IntClock_DIV_PTR            ((reg16 *) ETH_IntClock__CFG0)

/* Clock MSB divider configuration register. */
#define ETH_IntClock_DIV_MSB            (* (reg8 *) ETH_IntClock__CFG1)
#define ETH_IntClock_DIV_MSB_PTR        ((reg8 *) ETH_IntClock__CFG1)

/* Mode and source configuration register */
#define ETH_IntClock_MOD_SRC            (* (reg8 *) ETH_IntClock__CFG2)
#define ETH_IntClock_MOD_SRC_PTR        ((reg8 *) ETH_IntClock__CFG2)

#if defined(ETH_IntClock__CFG3)
/* Analog clock phase configuration register */
#define ETH_IntClock_PHASE              (* (reg8 *) ETH_IntClock__CFG3)
#define ETH_IntClock_PHASE_PTR          ((reg8 *) ETH_IntClock__CFG3)
#endif /* defined(ETH_IntClock__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define ETH_IntClock_CLKEN_MASK         ETH_IntClock__PM_ACT_MSK
#define ETH_IntClock_CLKSTBY_MASK       ETH_IntClock__PM_STBY_MSK

/* CFG2 field masks */
#define ETH_IntClock_SRC_SEL_MSK        ETH_IntClock__CFG2_SRC_SEL_MASK
#define ETH_IntClock_MODE_MASK          (~(ETH_IntClock_SRC_SEL_MSK))

#if defined(ETH_IntClock__CFG3)
/* CFG3 phase mask */
#define ETH_IntClock_PHASE_MASK         ETH_IntClock__CFG3_PHASE_DLY_MASK
#endif /* defined(ETH_IntClock__CFG3) */

#endif /* CY_CLOCK_ETH_IntClock_H */


/* [] END OF FILE */
