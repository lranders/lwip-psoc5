/*******************************************************************************
* File Name: ETE.h
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
#if !defined(CY_ISR_ETE_H)
#define CY_ISR_ETE_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void ETE_Start(void);
void ETE_StartEx(cyisraddress address);
void ETE_Stop(void);

CY_ISR_PROTO(ETE_Interrupt);

void ETE_SetVector(cyisraddress address);
cyisraddress ETE_GetVector(void);

void ETE_SetPriority(uint8 priority);
uint8 ETE_GetPriority(void);

void ETE_Enable(void);
uint8 ETE_GetState(void);
void ETE_Disable(void);

void ETE_SetPending(void);
void ETE_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the ETE ISR. */
#define ETE_INTC_VECTOR            ((reg32 *) ETE__INTC_VECT)

/* Address of the ETE ISR priority. */
#define ETE_INTC_PRIOR             ((reg8 *) ETE__INTC_PRIOR_REG)

/* Priority of the ETE interrupt. */
#define ETE_INTC_PRIOR_NUMBER      ETE__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable ETE interrupt. */
#define ETE_INTC_SET_EN            ((reg32 *) ETE__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the ETE interrupt. */
#define ETE_INTC_CLR_EN            ((reg32 *) ETE__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the ETE interrupt state to pending. */
#define ETE_INTC_SET_PD            ((reg32 *) ETE__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the ETE interrupt. */
#define ETE_INTC_CLR_PD            ((reg32 *) ETE__INTC_CLR_PD_REG)


#endif /* CY_ISR_ETE_H */


/* [] END OF FILE */
