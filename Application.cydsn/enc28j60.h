/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
/*
 * enc28j60.h
 *
 */

#ifndef ENC28J60_H_
#define ENC28J60_H_

#include <stdint.h>
#include <stdbool.h>

#include <lwip/netif.h>

#define RBM_OP           0x3a //Read Buffer Memory.
#define WCR_OP           0x40 //Write Control Register.
#define WBM_OP           0x7a //Write Buffer Memory.
#define BFS_OP           0x80 //Bit Field Set.
#define BFC_OP           0xa0 //Bit Field Clear.
#define RESET_OP         0xff //Soft Reset.

#define MAXFRAMELEN      1518

err_t enc28j60_init(struct netif *netif);

/**** API ****/
void enc_init(const uint8_t *mac);

/**
 * Function which does all the heavy work
 * It should be called when the ENC28J60 has signaled an interrupt
 */
void enc_action(struct netif *netif);

/**
 * Send an ethernet packet. Function will block until
 * transmission has completed.
 * TODO: Return if the transmission was successful or not
 */
void enc_send_packet(const uint8_t *buf, uint16_t count);

#endif /* ENC28J60_H_ */
//[] END OF FILE
