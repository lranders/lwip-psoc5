/*******************************************************************************
* File Name: LDON.h  
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

#if !defined(CY_PINS_LDON_H) /* Pins LDON_H */
#define CY_PINS_LDON_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "LDON_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v1_90 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 LDON__PORT == 15 && ((LDON__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    LDON_Write(uint8 value) ;
void    LDON_SetDriveMode(uint8 mode) ;
uint8   LDON_ReadDataReg(void) ;
uint8   LDON_Read(void) ;
uint8   LDON_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define LDON_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define LDON_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define LDON_DM_RES_UP          PIN_DM_RES_UP
#define LDON_DM_RES_DWN         PIN_DM_RES_DWN
#define LDON_DM_OD_LO           PIN_DM_OD_LO
#define LDON_DM_OD_HI           PIN_DM_OD_HI
#define LDON_DM_STRONG          PIN_DM_STRONG
#define LDON_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define LDON_MASK               LDON__MASK
#define LDON_SHIFT              LDON__SHIFT
#define LDON_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define LDON_PS                     (* (reg8 *) LDON__PS)
/* Data Register */
#define LDON_DR                     (* (reg8 *) LDON__DR)
/* Port Number */
#define LDON_PRT_NUM                (* (reg8 *) LDON__PRT) 
/* Connect to Analog Globals */                                                  
#define LDON_AG                     (* (reg8 *) LDON__AG)                       
/* Analog MUX bux enable */
#define LDON_AMUX                   (* (reg8 *) LDON__AMUX) 
/* Bidirectional Enable */                                                        
#define LDON_BIE                    (* (reg8 *) LDON__BIE)
/* Bit-mask for Aliased Register Access */
#define LDON_BIT_MASK               (* (reg8 *) LDON__BIT_MASK)
/* Bypass Enable */
#define LDON_BYP                    (* (reg8 *) LDON__BYP)
/* Port wide control signals */                                                   
#define LDON_CTL                    (* (reg8 *) LDON__CTL)
/* Drive Modes */
#define LDON_DM0                    (* (reg8 *) LDON__DM0) 
#define LDON_DM1                    (* (reg8 *) LDON__DM1)
#define LDON_DM2                    (* (reg8 *) LDON__DM2) 
/* Input Buffer Disable Override */
#define LDON_INP_DIS                (* (reg8 *) LDON__INP_DIS)
/* LCD Common or Segment Drive */
#define LDON_LCD_COM_SEG            (* (reg8 *) LDON__LCD_COM_SEG)
/* Enable Segment LCD */
#define LDON_LCD_EN                 (* (reg8 *) LDON__LCD_EN)
/* Slew Rate Control */
#define LDON_SLW                    (* (reg8 *) LDON__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define LDON_PRTDSI__CAPS_SEL       (* (reg8 *) LDON__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define LDON_PRTDSI__DBL_SYNC_IN    (* (reg8 *) LDON__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define LDON_PRTDSI__OE_SEL0        (* (reg8 *) LDON__PRTDSI__OE_SEL0) 
#define LDON_PRTDSI__OE_SEL1        (* (reg8 *) LDON__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define LDON_PRTDSI__OUT_SEL0       (* (reg8 *) LDON__PRTDSI__OUT_SEL0) 
#define LDON_PRTDSI__OUT_SEL1       (* (reg8 *) LDON__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define LDON_PRTDSI__SYNC_OUT       (* (reg8 *) LDON__PRTDSI__SYNC_OUT) 


#if defined(LDON__INTSTAT)  /* Interrupt Registers */

    #define LDON_INTSTAT                (* (reg8 *) LDON__INTSTAT)
    #define LDON_SNAP                   (* (reg8 *) LDON__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_LDON_H */


/* [] END OF FILE */
