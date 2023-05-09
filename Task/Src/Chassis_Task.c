/*****************************东莞理工学院ACE实验室 *****************************
 * @file Chassis_Task.c
 * @brief 
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 2.0
 * @date 2022-07-06
 * 
 * 
 * @history
 * <table>
 * Date       Version Author Description
 * 2022-07-06   1.0   侯文辉     
 * @verbatim 
 * ==============================================================================
 * ==============================================================================
 * @endverbatim
*******************************东莞理工学院ACE实验室 *****************************/
#include "ChassisMotor.h"
#include "bsp_dr16.h"
#include "RemoteDeal.h"
#include "Chassis_Task.h"
#include "SYSInit.h"
Chassis_t Chassis;
static void Chassis_Init(void)
{
    //函数映射
    Chassis.Fsm_Init=Chassis_FSM_Init;
    Chassis.Indepen=Chassis_Indepen_Drive;
    Chassis.Wheel_Init=Wheel_Motor_Init;
    Chassis.Poweroff = Chassis_PowerOff_Drive;

    //数据初始化
    Chassis.RC=Return_RemoteDeal_Point();     //获取遥控器指针
    Chassis.Wheel_Init(&Chassis.C);     //轮子初始化
    Chassis.Fsm=Return_Chassis_FSM();   //获取fsm状态机指针
    Chassis.Fsm_Init();                 //状态机初始化
    vTaskDelay(20);
}


void Chassis_Task(void *pvParameters)
{
//    //vTaskDelay(500);
    Chassis_Init();

    while(1)
    {
        FSM_Deal(Chassis.Fsm,Chassis.RC->RC_ctrl->rc.s1,Chassis.RC->RC_ctrl->rc.s2);
        vTaskDelay(2);
//				Test_Log("hi\r\n");
    }
}

