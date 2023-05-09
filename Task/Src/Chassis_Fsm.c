/*****************************东莞理工学院ACE实验室 *****************************
 * @file Chassis_Fsm.c
 * @brief 底盘fsm
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-07-06
 * 
 * 
 * @history
 * <table>
 * Date       Version Author Description
 * 2022-07-06   1.0   侯文辉     
 * @verbatim 
 * ==============================================================================
 * 用于初始化、绑定fsm各个指针函数（不完全初始化
 * 
 * ==============================================================================
 * @endverbatim
*******************************东莞理工学院ACE实验室 *****************************/
#include "Chassis_Fsm.h"
#include "Chassis_Task.h"
#include "bsp_can.h"
#include "SYSInit.h"
#include "RemoteDeal.h"
/*底盘控制总结构体*/
extern Chassis_t Chassis;

FSM_t Chassis_FSM;  //底盘状态机
State_t OFFLINE;    //离线模式
State_t INDEPEN;    //独立模式
State_t ROTATION;   //大陀螺模式
State_t KEYBOARD;   //键盘模式
State_t HAGEUP;   //独立悬挂模式
State_t Chassis_State_Table[State_Line][State_Column]; //状态参数表






/************************** Dongguan-University of Technology -ACE**************************
 * @brief 底盘状态机控制指针获取
 * 
 * @return FSM_t* 
************************** Dongguan-University of Technology -ACE***************************/
FSM_t *Return_Chassis_FSM(void)
{
    return &Chassis_FSM;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 底盘状态机初始化
 * 
************************** Dongguan-University of Technology -ACE***************************/
void Chassis_FSM_Init(void)
{
    Chassis_FSM.State_Table = Chassis_State_Table;
    Chassis_FSM.Last_State = NULL;
    Chassis_FSM.Current_State = NULL;
    Chassis_FSM.State_Change = StateChange; //状态机状态变更函数

    /*OFFLINE状态初始化*/
    OFFLINE.Behavior_Process = NULL;
    OFFLINE.State_Process = Offline_State;
    OFFLINE.State_Prepare = Offline_Prepare;
	
    /*INDEPEN状态初始化*/
    INDEPEN.Behavior_Process = NULL;
    INDEPEN.State_Process = Indepen_State;
    INDEPEN.State_Prepare = Indepen_Prepare;
	
    /*HAGEUP状态初始化*/
    HAGEUP.Behavior_Process = NULL;
    HAGEUP.State_Process = HageUP_State;
    HAGEUP.State_Prepare = HageUP_Prepare;

    // /*WIGGLE状态初始化*/
    // WIGGLE.Behavior_Process = NULL;
    // WIGGLE.State_Process = Wiggle_State;
    // WIGGLE.State_Prepare = Wiggle_Prepare;
	
    // /*ROTATION状态初始化*/
    // ROTATION.Behavior_Process = NULL;
    // ROTATION.State_Process = Rotation_State;
    // ROTATION.State_Prepare = Rotation_Prepare;
		
     /*KEYBOARD状态初始化*/
     KEYBOARD.State_Prepare = KeyBoard_Prepare;
     KEYBOARD.State_Process = KeyBoard_State;
     KEYBOARD.Behavior_Process = KeyBoard_bhv;
		
    // /*CTRL_GRASP状态初始化*/
    // CTRL_GRASP.Behavior_Process = NULL;
    // CTRL_GRASP.State_Process = CTRL_GRASP_State;
    // CTRL_GRASP.State_Prepare = CTRL_GRASP_Prepare;
		
    // /*RESCUE状态初始化*/
    // RESCUE.Behavior_Process = NULL;
    // RESCUE.State_Process = Rescue_State;
    // RESCUE.State_Prepare = Rescue_Prepare;

    /*底盘状态机初始化*/
    Chassis_State_Table[0][0] = KEYBOARD;    //s1=1 ,s2=1 底盘独立INDEPEN
    Chassis_State_Table[0][2] = OFFLINE;     //s1=1  s2=3 
    Chassis_State_Table[0][1] = OFFLINE;    //s1=1  s2=2 
    Chassis_State_Table[1][0] = OFFLINE;     //s1=2  s2=1 
    Chassis_State_Table[1][1] = INDEPEN; //s1=2  s2=2
    Chassis_State_Table[1][2] = OFFLINE; //s1=2  s2=3 夹取
    Chassis_State_Table[2][0] = OFFLINE;   //s1=3 s2=1  键盘
    Chassis_State_Table[2][1] = OFFLINE; //s1=3 s2=2  
    Chassis_State_Table[2][2] = OFFLINE; //s1=3 s2=3  
}


/***********************OFFLINE*************************/
/*OFFLINE状态准备函数*/
static void Offline_Prepare(void)
{
    vTaskDelay(20);
    for (int i = 0; i < 4; i++) {
        Chassis.C.WheelMotor_EPB_Pos[i] = Chassis.C.WheelMotor[i].Encoder->Encode_Record_Val;
    }
//		HAL_NVIC_SystemReset();
    Chassis_FSM.Current_State->Behavior_Process = PowerOff_bhv;
}

/*离线状态处理*/
static void Offline_State(void)
{
    
}

/*断电行为函数*/
static void PowerOff_bhv(void)
{
    Chassis.Poweroff(&Chassis.C);
}


/***********************INDEPEN*************************/

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 独立准备函数
 * 
************************** Dongguan-University of Technology -ACE***************************/
static void Indepen_Prepare(void)
{
    Chassis_FSM.Current_State->Behavior_Process=Chassis_Normal_bhv;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 独立状态处理函数
 * 
************************** Dongguan-University of Technology -ACE***************************/
static void Indepen_State(void)
{
    
}    

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 底盘正常运动行为函数
 * 
************************** Dongguan-University of Technology -ACE***************************/
static void Chassis_Normal_bhv(void)
{
    Chassis.Indepen(&Chassis.C, -Chassis.RC->RC_ctrl->rc.ch[3], -Chassis.RC->RC_ctrl->rc.ch[2], -Chassis.RC->RC_ctrl->rc.ch[0],Chassis_Speed_Exp);
    Lift_Up_Drive(&Chassis.C, Chassis.RC->RC_ctrl->rc.ch[4] * 20);
}

static void KeyBoard_Prepare(void)     /*KEYBOARD状态准备函数*/
{

}
static void KeyBoard_State(void)       /*键盘状态处理*/
{
//    if (Chassis.RC->state.Global_Status == Follow_Independent) {
//        Chassis.Indepen(&Chassis.C, Chassis.RC->RC_ctrl->key.kv0, Chassis.RC->RC_ctrl->key.kv1,
//                        Chassis.RC->RC_ctrl->mouse.x, Chassis_Speed_Exp);
//    }
}

//底盘独立模式：
static void KeyBoard_bhv(void)       /*键盘状态处理*/
{
    if (Chassis.RC->state.Global_Status == Follow_Independent) {
        Chassis.Indepen(&Chassis.C, -Chassis.RC->RC_ctrl->key.kv0, -Chassis.RC->RC_ctrl->key.kv1,
                        -Chassis.RC->RC_ctrl->mouse.x*Chassis_Mouse_Speed_Exp, Chassis_Speed_Exp);
        switch (Chassis.RC->state.Camera_Status) {
            case Camara_To_Horizontal:{
                Chassis_CameraTransmission_Auto_Drive(&Chassis.C, Camara_Horizontal_PITCH_POS, Camara_FORWARD_YAW_POS);
            }break;
            case Camara_To_Ore:{
                Chassis_CameraTransmission_Auto_Drive(&Chassis.C, Camara_Horizontal_PITCH_POS, Camara_Ore_YAW_POS);
            }break;
            case Camara_To_RescueCatch: {
                Chassis_CameraTransmission_Auto_Drive(&Chassis.C, Camara_DOWM_PITCH_POS, Camara_FORWARD_YAW_POS);
            }break;
        }

    }

}



/************************** Dongguan-University of Technology -ACE**************************
 * @brief 独立悬挂准备函数
 * 
************************** Dongguan-University of Technology -ACE***************************/
static void HageUP_Prepare(void)
{
    Chassis_FSM.Current_State->Behavior_Process=HageUP_Normal_bhv;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 独立状态处理函数
 * 
************************** Dongguan-University of Technology -ACE***************************/
static void HageUP_State(void)
{
    
}   

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 底盘正常运动行为函数
 * 
************************** Dongguan-University of Technology -ACE***************************/
static void HageUP_Normal_bhv(void)
{
//    Chassis.Indepen(&Chassis.C, -Chassis.RC->RC_ctrl->rc.ch[3], -Chassis.RC->RC_ctrl->rc.ch[2], -Chassis.RC->RC_ctrl->rc.ch[0],0);
    Lift_Up_Drive(&Chassis.C, 0);
}

