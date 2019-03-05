/*
 * cdebug.h
 *
 *  Created on: 18 февр. 2019 г.
 *      Author: home
 */

#ifndef CDEBUG_H_
#define CDEBUG_H_




#include <stdint.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#ifdef __cplusplus
 extern "C" {
#endif

void dBlink (uint8_t cnt, uint16_t  delay, uint16_t enddelay);
void dDebugSend (UART_HandleTypeDef *huart, uint8_t *dChars,...);

#ifdef __cplusplus
 }
#endif


#endif /* CDEBUG_H_ */
