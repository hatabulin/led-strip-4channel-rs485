/*
 * utils.c
 *
 *  Created on: 5 нояб. 2022 г.
 *      Author: pupki
 */
#include "main.h"

extern UART_HandleTypeDef huart1;

void logger(char *buf) {
	HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
}
