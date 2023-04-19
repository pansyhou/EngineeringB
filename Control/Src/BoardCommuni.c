/************************** Dongguan-University of Technology -ACE**************************
 * @file BoardCommuni.c
 * @brief
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-07-12
 *
 *
 * @history
 * <table>
 * Date       Version Author Description
 * 2022-07-12   1.0   侯文辉
 * @verbatim
 * ==============================================================================
 *  本来写了很多都在底盘板接收遥控的，但是21还是20年出现过底盘电池或者其他干扰较大，上场动不了，
 *  于是遥控接收就调整到云台板收，将部分数据用串口+DMA发送，减少CPU占用提高发送效率
 *
 * 	同时写一套用CAN通讯的版本，冗余备用
 * ==============================================================================
 * @endverbatim
 ************************** Dongguan-University of Technology -ACE***************************/
#include "BoardCommuni.h"
#include "main.h"
#include "RemoteDeal.h"
#include "SYSInit.h"
#include "Detect_Task.h"
#include "bsp_can.h"
#include "stm32f4xx_hal_dma.h"

//底盘测试模式
#define Chassis_TEST_MODE 0

#define Data_Size 12
uint8_t RC_buffer[Data_Size];
extern REMOTE_t REMOTE;
extern error_t error_list;
#define RC_CHANNAL_ERROR_VALUE 700

#if Chassis_TEST_MODE
uint8_t Jscope_RC_s1;
uint8_t Jscope_RC_s2;
int16_t Jscope_RC_ch0;
int16_t Jscope_RC_ch1;
int16_t Jscope_RC_ch2;

#endif


/****************CAN版本*****************/

int id=0;
void CAN_A2B_RC_Rx(CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    //判断遥控器是不是在线
    if (!Is_Online(DBUS_TOE)) {
        if(header->StdId==0x100){
            REMOTE.RC_ctrl->rc.ch[0] = (int16_t)((int16_t) data[0]<< 8 | (int16_t) data[1] );
            REMOTE.RC_ctrl->rc.ch[1] = (int16_t)((int16_t) data[2]<< 8 | (int16_t) data[3] );
            REMOTE.RC_ctrl->rc.ch[2] = (int16_t)((int16_t) data[4]<< 8 | (int16_t) data[5] );
            REMOTE.RC_ctrl->rc.ch[3] = (int16_t)((int16_t) data[6]<< 8 | (int16_t) data[7] );
        } else if (header->StdId == 0x102) {                      
            REMOTE.RC_ctrl->rc.ch[4] = (int16_t)((int16_t) data[0]<< 8 | (int16_t) data[1] );
            REMOTE.RC_ctrl->rc.s1 = (uint8_t)data[2];
            REMOTE.RC_ctrl->rc.s2 = (uint8_t)data[3];
        }
    }

}

void CAN_A2B_KM_Rx(CAN_HandleTypeDef *header,uint8_t *data)
{

}

