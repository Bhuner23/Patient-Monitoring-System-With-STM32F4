/*
 * dht.h
 *
 *  Created on: Jun 28, 2020
 *      Author: Controllerstech.com
 */

#ifndef DS_H_
#define DS_H_

#include "stm32f4xx_hal.h"

typedef struct
{
	uint8_t T_Integer;
	uint8_t Sign;
	float T_Fractional;
	float T_Reel;
}DS_DataTypedef;


void DS_GetData (DS_DataTypedef *DS_Data);

#endif /* INC_DS_H_ */
