#include "main.h"
#include "i2c_lcd.h"
#include <stdio.h>

static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);

char msg[64];
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;
int datacheck=0;

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	if(RxHeader.DLC==2)
	{
		datacheck=1;
	}
}
int main(void)
{
	MX_CAN1_Init();
	MX_I2C1_Init();
	HAL_CAN_Start(&hcan1);
	lcd_init();

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	TxHeader.DLC=2;
	TxHeader.IDE=CAN_ID_STD;
	TxHeader.RTR=CAN_RTR_DATA;
	TxHeader.StdId=0x103;

	while (1)
	{
		if(datacheck)
		{
			char temp[10];
			char message[32];
			sprintf(temp, "%u", RxData[2]);

			if(RxData[0]==1)
			{HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
			lcd_put_cur(0,0);
			lcd_send_string("ALERT DE GAZ");
			HAL_Delay(3000);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);

			}else if(RxData[0]==0){
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("%GAZ=0");
				HAL_Delay(100);

			}
			if(RxData[1]==1){
				strcpy(message, "ALERT TEMP: ");
				strcat(message, temp);
				lcd_put_cur(1,0);
				lcd_send_string(message);
			}else if(RxData[1]==0){

				strcpy(message, "TEMP NORMAL: ");
				strcat(message, temp);
				lcd_put_cur(1,0);
				lcd_send_string(message);
			}
			datacheck=0;
		}
	}
}
static void MX_CAN1_Init(void)
{
	CAN_FilterTypeDef canfilterconfig;
	canfilterconfig.FilterActivation=CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank=10;
	canfilterconfig.FilterFIFOAssignment=CAN_FILTER_FIFO1;
	canfilterconfig.FilterIdHigh=0x446<<5;
	canfilterconfig.FilterIdLow=0;
	canfilterconfig.FilterMaskIdHigh=0x446<<5;
	canfilterconfig.FilterMaskIdLow=0x0000;
	canfilterconfig.FilterMode=CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale=CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank=0;

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

}
