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
 */
#ifndef DMA_H
#define DMA_H

/* DMA Configuration for DMA_TX */
#define DMA_TX_BYTES_PER_BURST       (1u)
#define DMA_TX_REQUEST_PER_BURST     (1u)
#define DMA_TX_SRC_BASE              (CYDEV_SRAM_BASE)
#define DMA_TX_DST_BASE              (CYDEV_PERIPH_BASE)

/* DMA Configuration for DMA_RX */
#define DMA_RX_BYTES_PER_BURST       (1u)
#define DMA_RX_REQUEST_PER_BURST     (1u)
#define DMA_RX_SRC_BASE              (CYDEV_PERIPH_BASE)
#define DMA_RX_DST_BASE              (CYDEV_SRAM_BASE)

/* Variable declarations for DMA_Tx */
extern uint8 TXChannel;
extern uint8 TX_TD[2];

/* Variable declarations for DMA_Rx */
extern uint8 RXChannel;
extern uint8 RX_TD[2];

#endif

//[] END OF FILE
