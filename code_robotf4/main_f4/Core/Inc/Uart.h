/*
 * Uart.h
 *
 *  Created on: Mar 19, 2025
 *      Author: APC
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include "main.h"
#include "stdbool.h"

extern uint8_t  rx_data[2];
extern uint16_t but_data ;
extern uint8_t Buffer[2];

void get_uart();
int16_t compass();

#define button_up         0x01
#define button_down       0x02
#define button_right      0x04
#define button_left       0x08
#define button_L1         0x10
#define button_R1         0x20
#define button_L2         0x40
#define button_R2         0x80
#define button_cross      0x100
#define button_cricle     0x200
#define button_square     0x400
#define button_triangle   0x800
#define button_share      0x2000
#define button_options    0x4000
#define button_L3         0x1000
#define button_R3         0x8000 //16 bit 1000 4 16 bit
#endif /* INC_UART_H_ */
