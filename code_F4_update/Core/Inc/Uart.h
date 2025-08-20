/*
 * Uart.h
 *
 *  Created on: Apr 24, 2025
 *      Author: APC
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "Map_robot.h"

#define HIGH	1
#define LOW		0

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

extern uint8_t  rx_data[2];
extern float f1, f2;
extern uint16_t but_data ;
extern uint8_t Buffer[2];
extern int16_t value_compass;
extern int16_t angle;

#pragma pack(push, 1) // chống padding
typedef struct {
    int32_t v_goc;
    int32_t v_dai;
} Trans_data;
#pragma pack(pop)

extern Trans_data trans_data;

#pragma pack(push, 1)
typedef struct {
    float var1;
    float var2;
    uint8_t checksum;
} DataPacket;
#pragma pack(pop)
extern DataPacket receivedPacket;
enum UART1_stages{   //  for compass
	STAGE_NO_READ_COMPASS,
	STAGE_READ_START,
	STAGE_BYTE_H,
	STAGE_BYTE_L,
	STAGE_RESET_COMPASS
};

void get_uart();
void uart2(void);
int16_t compass();
void mPrintf(const char *format,...);
void reset_compass();

#define button_up         0x01
#define button_down       0x02
#define button_right      0x04
#define button_left       0x08
#define button_L1         0x10
#define button_R1         0x20
#define button_L2         0x40
#define button_R2         0x80
#define button_cross      0x100
#define button_circle     0x200
#define button_square     0x400
#define button_triangle   0x800
#define button_share      0x2000
#define button_options    0x4000
#define button_L3         0x1000
#define button_R3         0x8000 //16 bit 1000 4 16 bit
#endif /* INC_UART_H_ */
