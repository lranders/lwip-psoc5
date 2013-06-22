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

#if 0
#define LWIP_DEBUG
#define LWIP_DBG_MIN_LEVEL			0x00
#define NETIF_DEBUG					LWIP_DBG_ON | LWIP_DBG_TRACE
#define ETHARP_DEBUG				LWIP_DBG_ON | LWIP_DBG_TRACE
#define DHCP_DEBUG					LWIP_DBG_ON
#endif

#endif
//[] END OF FILE
