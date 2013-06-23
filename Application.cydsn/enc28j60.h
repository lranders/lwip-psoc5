/*
 * Copyright (c) 2013. All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * Author: Lars Randers <lranders@mail.dk>
 *
 * Credits go to: Paul Fleischer <https://github.com/xpgdk/stellaris-enc28j60-booster.git>
 *                and
 *                Kartik M <kmmankad@gmail.com>
 *
 * who both provided me with an excellent starting point for this driver.
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
