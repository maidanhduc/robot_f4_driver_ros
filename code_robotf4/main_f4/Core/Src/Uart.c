#include "uart.h"


extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

uint16_t but_data ;
uint8_t  rx_data[2];
bool status_read = true;
uint32_t rx_index = 0;

uint8_t Buffer[2];
extern bool status_readAngle;
int16_t value;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == huart4.Instance){
		but_data = ((uint16_t)rx_data[0] << 8 ) | rx_data[1];
		HAL_UART_Receive_IT(&huart4,rx_data,2);
		rx_index++;
	}
	if (huart->Instance == huart5.Instance){
		HAL_UART_Receive_IT(&huart5, (uint8_t *) &Buffer[0], 1);
	}
}
void get_uart(){
	char data = 'o';
	static uint32_t time =0;
	if (status_read) {
		HAL_UART_Transmit(&huart4,(uint8_t*)&data,1,300);
		time = HAL_GetTick();
		status_read = false;
	}
	else {
		if (HAL_GetTick() - time >= 100){
			status_read = true;
		}
	}

}

int16_t Compass(){

	static uint32_t time = 0;
	if(status_readAngle){
		char data = 'z';
		HAL_UART_Receive_IT(&huart5, (uint8_t *) &Buffer[0], 1);
		HAL_UART_Transmit(&huart5, (uint8_t *)&data ,1,10);
		time = HAL_GetTick();

		status_readAngle = false;
	}else{
		if(HAL_GetTick() - time >= 1000){
			status_readAngle = true;
		}
	}


	return value;
}
