/*******************************************************************************
* File Name: EINT.h  
* Version 1.90
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_EINT_H) /* Pins EINT_H */
#define CY_PINS_EINT_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "EINT_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v1_90 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 EINT__PORT == 15 && ((EINT__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    EINT_Write(uint8 value) ;
void    EINT_SetDriveMode(uint8 mode) ;
uint8   EINT_ReadDataReg(void) ;
uint8   EINT_Read(void) ;
uint8   EINT_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define EINT_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define EINT_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define EINT_DM_RES_UP          PIN_DM_RES_UP
#define EINT_DM_RES_DWN         PIN_DM_RES_DWN
#define EINT_DM_OD_LO           PIN_DM_OD_LO
#define EINT_DM_OD_HI           PIN_DM_OD_HI
#define EINT_DM_STRONG          PIN_DM_STRONG
#define EINT_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define EINT_MASK               EINT__MASK
#define EINT_SHIFT              EINT__SHIFT
#define EINT_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define EINT_PS                     (* (reg8 *) EINT__PS)
/* Data Register */
#define EINT_DR                     (* (reg8 *) EINT__DR)
/* Port Number */
#define EINT_PRT_NUM                (* (reg8 *) EINT__PRT) 
/* Connect to Analog Globals */                                                  
#define EINT_AG                     (* (reg8 *) EINT__AG)                       
/* Analog MUX bux enable */
#define EINT_AMUX                   (* (reg8 *) EINT__AMUX) 
/* Bidirectional Enable */                                                        
#define EINT_BIE                    (* (reg8 *) EINT__BIE)
/* Bit-mask for Aliased Register Access */
#define EINT_BIT_MASK               (* (reg8 *) EINT__BIT_MASK)
/* Bypass Enable */
#define EINT_BYP                    (* (reg8 *) EINT__BYP)
/* Port wide control signals */                                                   
#define EINT_CTL                    (* (reg8 *) EINT__CTL)
/* Drive Modes */
#define EINT_DM0                    (* (reg8 *) EINT__DM0) 
#define EINT_DM1                    (* (reg8 *) EINT__DM1)
#define EINT_DM2                    (* (reg8 *) EINT__DM2) 
/* Input Buffer Disable Override */
#define EINT_INP_DIS                (* (reg8 *) EINT__INP_DIS)
/* LCD Common or Segment Drive */
#define EINT_LCD_COM_SEG            (* (reg8 *) EINT__LCD_COM_SEG)
/* Enable Segment LCD */
#define EINT_LCD_EN                 (* (reg8 *) EINT__LCD_EN)
/* Slew Rate Control */
#define EINT_SLW                    (* (reg8 *) EINT__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define EINT_PRTDSI__CAPS_SEL       (* (reg8 *) EINT__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define EINT_PRTDSI__DBL_SYNC_IN    (* (reg8 *) EINT__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define EINT_PRTDSI__OE_SEL0        (* (reg8 *) EINT__PRTDSI__OE_SEL0) 
#define EINT_PRTDSI__OE_SEL1        (* (reg8 *) EINT__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define EINT_PRTDSI__OUT_SEL0       (* (reg8 *) EINT__PRTDSI__OUT_SEL0) 
#define EINT_PRTDSI__OUT_SEL1       (* (reg8 *) EINT__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define EINT_PRTDSI__SYNC_OUT       (* (reg8 *) EINT__PRTDSI__SYNC_OUT) 


#if defined(EINT__INTSTAT)  /* Interrupt Registers */

    #define EINT_INTSTAT                (* (reg8 *) EINT__INTSTAT)
    #define EINT_SNAP                   (* (reg8 *) EINT__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_EINT_H */


/* [] END OF FILE */
