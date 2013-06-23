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
#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

#define NO_SYS						1
#define LWIP_NETCONN				0
#define LWIP_SOCKET					0

#define SYS_LIGHTWEIGHT_PROT		0

#define LWIP_NETIF_STATUS_CALLBACK	1

#define LWIP_DHCP					1

#define MEM_ALIGNMENT				4

/* Heap memory size */
#define MEM_SIZE					6 * 1024

#define ETH_PAD_SIZE				0

#define IP_REASSEMBLY				0
#define IP_FRAG						0

#define LWIP_HTTPD_SUPPORT_V09      0

#if 1
#define LWIP_DEBUG
#define LWIP_DBG_MIN_LEVEL			0x00
//#define NETIF_DEBUG				LWIP_DBG_ON
//#define ETHARP_DEBUG				LWIP_DBG_ON
#define DHCP_DEBUG					LWIP_DBG_ON
#define HTTPD_DEBUG					LWIP_DBG_ON
#endif

#endif
//[] END OF FILE
