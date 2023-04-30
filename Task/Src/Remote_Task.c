#include "Remote_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "bsp_can.h"
#include "BoardCommuni.h"


void Remote_Task_Init(void);


void Remote_Task_Init(void){
    ECF_CAN_Rx_Callback_Register(&can2_manage, CAN_A2B_RC_Rx);
}

void Remote_Task(void *pvParameters)
{
    Remote_Task_Init();
    vTaskDelay(500);

    while(1)
    {

        Remote_Data_Deal();
        vTaskDelay(2);
    }
}
