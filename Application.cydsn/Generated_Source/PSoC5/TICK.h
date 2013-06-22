/*******************************************************************************
* File Name: TICK.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_TICK_H)
#define CY_ISR_TICK_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void TICK_Start(void);
void TICK_StartEx(cyisraddress address);
void TICK_Stop(void);

CY_ISR_PROTO(TICK_Interrupt);

void TICK_SetVector(cyisraddress address);
cyisraddress TICK_GetVector(void);

void TICK_SetPriority(uint8 priority);
uint8 TICK_GetPriority(void);

void TICK_Enable(void);
uint8 TICK_GetState(void);
void TICK_Disable(void);

void TICK_SetPending(void);
void TICK_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the TICK ISR. */
#define TICK_INTC_VECTOR            ((reg32 *) TICK__INTC_VECT)

/* Address of the TICK ISR priority. */
#define TICK_INTC_PRIOR             ((reg8 *) TICK__INTC_PRIOR_REG)

/* Priority of the TICK interrupt. */
#define TICK_INTC_PRIOR_NUMBER      TICK__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable TICK interrupt. */
#define TICK_INTC_SET_EN            ((reg32 *) TICK__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the TICK interrupt. */
#define TICK_INTC_CLR_EN            ((reg32 *) TICK__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the TICK interrupt state to pending. */
#define TICK_INTC_SET_PD            ((reg32 *) TICK__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the TICK interrupt. */
#define TICK_INTC_CLR_PD            ((reg32 *) TICK__INTC_CLR_PD_REG)


#endif /* CY_ISR_TICK_H */


/* [] END OF FILE */
