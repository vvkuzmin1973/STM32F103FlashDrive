/*
 * cdebug.c
 *
 *  Created on: 18 февр. 2019 г.
 *      Author: home
 */

#include "cdebug.h"
//#include "cdebug.h"


#ifndef BLINK_PORT
#define BLINK_PORT GPIOC
#endif

#ifndef BLINK_PIN
#define BLINK_PIN GPIO_PIN_13
#endif

void dBlink (uint8_t cnt, uint16_t  delay, uint16_t enddelay) {

	for (int i=0; i<cnt; i++) {
        HAL_GPIO_WritePin(BLINK_PORT, BLINK_PIN, GPIO_PIN_RESET);
	    HAL_Delay(delay);
        HAL_GPIO_WritePin(BLINK_PORT, BLINK_PIN, GPIO_PIN_SET);
	    HAL_Delay(delay);
	}
    HAL_Delay(enddelay);
}

void dDebugSend (UART_HandleTypeDef *huart, uint8_t *dChars,...) {
	HAL_UART_Transmit(huart, dChars , strlen(dChars),100);
  	//sprintf(dBuff, )
}
