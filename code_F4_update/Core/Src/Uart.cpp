/*
 * Uart.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: APC
 */
#include "Uart.h"


// UART_1 for compass // ERROR
// UART_2 for compass
// UART_3 for LIDAR or debug
// UART_4 for ESP or debug

uint16_t but_data ;
uint8_t  rx_data[2];
uint8_t byte_rx;         // Nhận từng byte
uint8_t rx_index = 0;    // Vị trí hiện tại trong buffer
float f1, f2;
bool status_read = true;
//uint32_t rx_index = 0;
Trans_data trans_data;
uint32_t debuggg =0;
// angle
UART1_stages uart1_stages = STAGE_NO_READ_COMPASS;
int16_t value_compass = 0;
int16_t delta_value_compass = 0;
uint8_t Buffer[2];
uint32_t deduggg2 =0;

int16_t compass(){
	static uint32_t time = 0;

	if(uart1_stages == STAGE_NO_READ_COMPASS){
		char data = 'z';
		if(HAL_UART_Transmit_IT(&huart2,(uint8_t *)&data, 1) == HAL_OK){
			time = HAL_GetTick();
			uart1_stages = STAGE_READ_START;
		}
	}else{
		if(HAL_GetTick() - time >= 1000){
			uart1_stages = STAGE_NO_READ_COMPASS;
		}
	}
	return value_compass - delta_value_compass;
}

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == huart4.Instance){
		//but_data = ((uint16_t)rx_data[0] << 8 ) | rx_data[1];
		if(data == '\n')
				{
					mData[idx] = '\0';
					idx = 0;
					HAL_UART_Transmit(&huart4, (uint8_t *)mData, strlen(mData), 100);
					HAL_UART_Transmit(&huart4, (uint8_t *)"\r\n", 2, 100);
				}
				else
				{
					mData[idx++] = data;
					if(idx >= sizeof(mData))
						idx = 0;
				}
				HAL_UART_Receive_IT(&huart4, &data, 1);

		HAL_UART_Receive_IT(&huart4,rx_data,1);
		debuggg++;

	}

	if(huart->Instance == huart2.Instance){

		//mPrintf("receive_okay \n ",0);
		value_compass = (int16_t)((Buffer[0] << 8) | Buffer[1]);
		uart1_stages = STAGE_NO_READ_COMPASS;
		//HAL_UART_Receive_IT(&huart1, (uint8_t*)&Buffer, 2);
	}
}
#endif


DataPacket receivedPacket;
uint8_t rxBuffer[sizeof(DataPacket)];
uint8_t rxIndex = 0;
bool packetReady = false;
uint8_t receivedByte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == huart4.Instance){

		 rxBuffer[rxIndex++] = receivedByte; // Lưu byte vào buffer



		 // Nếu nhận đủ struct
		 if (rxIndex >= sizeof(DataPacket)) {
		     memcpy((void*)&receivedPacket, (void*)rxBuffer, sizeof(DataPacket));

		     // Kiểm tra checksum
		     uint8_t calcChecksum = 0;
		     uint8_t* ptr = (uint8_t*)&receivedPacket;
		     for (uint32_t i = 0; i < sizeof(DataPacket) - 1; i++) {
		            calcChecksum ^= ptr[i];
		         }

		     if (calcChecksum == receivedPacket.checksum) {
		             packetReady = true; // Dữ liệu hợp lệ
		     } else {
		           	 packetReady = false;
		     }

		     rxIndex = 0; // Reset buffer
		  }

		        // Tiếp tục nhận byte tiếp theo
		 	 	debuggg++;
		        HAL_UART_Receive_IT(&huart4, (uint8_t*)&receivedByte, 1);
		    }
	if(huart->Instance == huart2.Instance){

			//mPrintf("receive_okay \n ",0);
			value_compass = (int16_t)((Buffer[0] << 8) | Buffer[1]);
			uart1_stages = STAGE_NO_READ_COMPASS;
			//HAL_UART_Receive_IT(&huart1, (uint8_t*)&Buffer, 2);
		}



}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart2.Instance){

		if(uart1_stages == STAGE_RESET_COMPASS){
			//st_reset_angle = true;
			uart1_stages = STAGE_NO_READ_COMPASS;

		}else{

			//mPrintf("trans_okay \n ",0);
			HAL_UART_Receive_IT(&huart2, (uint8_t*)&Buffer, 2);
		}

	}
	if(huart->Instance == huart4.Instance){
		deduggg2++;
		HAL_UART_Receive_IT(&huart4,(uint8_t*)&receivedByte,1);
	}
}


void get_uart(){
	char data = 'o';
	static uint32_t time =0;
	if (status_read) {
		HAL_UART_Transmit_IT(&huart4,(uint8_t *)&data, 1);
		time = HAL_GetTick();
		status_read = false;
	}
	else {
		if (HAL_GetTick() - time >= 100){
			status_read = true;
		}
	}

}
void uart2() {
    static uint32_t time = 0;
    trans_data.v_goc = 15;
    trans_data.v_dai = (really_speed*7.0)/6000;
    if (status_read) {
        // Gửi toàn bộ struct
        HAL_UART_Transmit_IT(&huart4, (uint8_t*)&trans_data, sizeof(trans_data));

        time = HAL_GetTick();
        status_read = false;
    }
    else {
        if (HAL_GetTick() - time >= 100) {
            status_read = true;
        }
    }
}

void reset_compass(){
	char flag_reset = 'a';
//	if(st_reset_angle){
//
//		uart5_stages = STAGE_RESET_COMPASS;
		HAL_UART_Transmit(&huart2, (uint8_t*)&flag_reset, 1, 300);
//		st_reset_angle = false;
//	}
	delta_value_compass = value_compass;


//	HAL_UART_Transmit_IT(&huart5, (uint8_t*)&flag_reset, 1);
//	uart5_stages = STAGE_RESET_COMPASS;
}

// function to debug
char buff[40];
void mPrintf(const char *format,...){
	va_list args;
	va_start(args, format);
	vsnprintf(buff, sizeof(buff), format, args);
	va_end(args);
	// UART
	static unsigned long time_trans = 0;
	if (HAL_GetTick() - time_trans >= 300 ){
	HAL_UART_Transmit(&huart3,(uint8_t*)buff,40,1000);
	time_trans = HAL_GetTick();
	}
}

