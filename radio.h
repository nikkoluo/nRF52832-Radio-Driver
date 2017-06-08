/*******************************************************************************************************
 * @file		radio.h
 * @brief		radio driver 
 * @version		1.0
 * @date		2017-6-8
 * @author	Nikko luo
 *******************************************************************************************************
 *******************************************************************************************************
 * History:
 *------------------------------------------------------------------------------------------------------
 *|  Date       | Version   |   Description
 *| 2017-6-8    | V1.0.0    |  First release
 *******************************************************************************************************/
 
#ifndef __RADIO_H__
#define __RADIO_H__

/* Includes ------------------------------------------------------------------------------------------ */
#include <stdint.h>
#include "nrf.h"

#if defined(__cplusplus)
extern "C" {
#endif

void	Radio_Init(void);	


int32_t Radio_Send(uint8_t *buf, int32_t len);	
int32_t Radio_ListenLoopback(uint8_t *buf, int32_t bufSize, int32_t *Rssi, void (*fpRcv_cbFunc)(uint8_t*, int32_t));
	
#ifdef __cplusplus
}
#endif

#endif // end of __RADIO_H__
/*********************************************************************************************************
											End Of File
*********************************************************************************************************/
