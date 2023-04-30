#include "ChassisMotor.h"
#include "rmmotor.h"
#include "maths.h"
#include "bsp_can.h"
#include "Chassis_Task.h"
#include "bsp_Motor_Encoder.h"
#include "Detect_Task.h"

//CAN1接受中断进detecthook频率（也可以理解为计数器，到点再进）
#define CAN_IRQ_DETECT_FREQ 3000
#define MAX_ABS_HIGHT 155648
extern Chassis_t Chassis;
extern REMOTE_t REMOTE;


//CAN中断计数�?
static int Can_IRQ_Count;

/**
 * CAN1电机结算
 * 目前�?测是否在线的方法非常的骚，利用一个计数器，到3k之后就进钩子函数将灯电亮
 * 如果4个电机有�?个掉线�?	�频率必然会降低或�?�直接不亮，就可以�?�一排除
 * 但是，其实可以直接动底盘直接看哪个电机不转的...
 * @param header
 * @param data
 */
void Motor_Data_Deal(CAN_RxHeaderTypeDef *header, uint8_t *data) {
    Can_IRQ_Count++;
    switch (header->StdId) {
        case CAN_M3508_MOTOR1_ID: {
            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])), //电机位置
                                  (int16_t) ((data[2] << 8) + (data[3])), //电机速度
                                  1);
            Chassis.C.WheelMotor[0].anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
            Chassis.C.WheelMotor[0].temp = (int16_t) (data[6]);//当前电机温度
        }
            break;
        case CAN_M3508_MOTOR2_ID: {
            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])), //电机位置
                                  (int16_t) ((data[2] << 8) + (data[3])), //电机速度
                                  2);
            Chassis.C.WheelMotor[1].anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
            Chassis.C.WheelMotor[1].temp = (int16_t) (data[6]);//当前电机温度
        }
            break;
        case CAN_M3508_MOTOR3_ID: {
            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])), //电机位置
                                  (int16_t) ((data[2] << 8) + (data[3])), //电机速度
                                  3);
            Chassis.C.WheelMotor[2].anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
            Chassis.C.WheelMotor[2].temp = (int16_t) (data[6]);//当前电机温度
        }
            break;
        case CAN_M3508_MOTOR4_ID: {
            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])), //电机位置
                                  (int16_t) ((data[2] << 8) + (data[3])), //电机速度
                                  4);
            Chassis.C.WheelMotor[3].anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
            Chassis.C.WheelMotor[3].temp = (int16_t) (data[6]);//当前电机温度
        }
            break;
        case CAN_M3508_MOTOR7_ID: {
            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])), //电机位置
                                  (int16_t) ((data[2] << 8) + (data[3])), //电机速度
                                  7);
            Chassis.C.liftUp.Lift_Motor[0].anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
            Chassis.C.liftUp.Lift_Motor[0].temp = (int16_t) (data[6]);//当前电机温度
        }break;
				case CAN_M3508_MOTOR8_ID: {
            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])), //电机位置
                                  (int16_t) ((data[2] << 8) + (data[3])), //电机速度
                                  8);
            Chassis.C.liftUp.Lift_Motor[1].anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
            Chassis.C.liftUp.Lift_Motor[1].temp = (int16_t) (data[6]);//当前电机温度
        }break;

    }
    if (Can_IRQ_Count == CAN_IRQ_DETECT_FREQ) {
        detect_hook(CHASSIS_MOTOR1_TOE);
        Can_IRQ_Count = 0;
    }

}






/************************** Dongguan-University of Technology -ACE**************************
 * @brief 底盘轮子电机初始�?(pid+发�?�函数映�?)
 *
 * @param C
 ************************** Dongguan-University of Technology -ACE***************************/
void Wheel_Motor_Init(C_t *C) {

    ECF_CAN_Rx_Callback_Register(&can1_manage, Motor_Data_Deal);

    float Spid[9][3] =
            {
                    {WHEEL_MOTOR1_P, WHEEL_MOTOR1_I, WHEEL_MOTOR1_D},
                    {WHEEL_MOTOR2_P, WHEEL_MOTOR2_I, WHEEL_MOTOR2_D},
                    {WHEEL_MOTOR3_P, WHEEL_MOTOR3_I, WHEEL_MOTOR3_D},
                    {WHEEL_MOTOR4_P, WHEEL_MOTOR4_I, WHEEL_MOTOR4_D},
                    {RESCUE_S_P,     RESCUE_S_I,     RESCUE_S_D},
                    {BARRIER_S_P,    BARRIER_S_I,    BARRIER_S_D},
                    {Lift_Motor1_Spid_P, Lift_Motor1_Spid_I, Lift_Motor1_Spid_D},
                    {Lift_Motor2_Spid_P, Lift_Motor2_Spid_I, Lift_Motor2_Spid_D}
            };
    float Ppid[9][3] =
            {
                    {WHEEL_MOTOR1_POS_P, WHEEL_MOTOR1_POS_I, WHEEL_MOTOR1_POS_D},
                    {WHEEL_MOTOR2_POS_P, WHEEL_MOTOR2_POS_I, WHEEL_MOTOR2_POS_D},
                    {WHEEL_MOTOR3_POS_P, WHEEL_MOTOR3_POS_I, WHEEL_MOTOR3_POS_D},
                    {WHEEL_MOTOR4_POS_P, WHEEL_MOTOR4_POS_I, WHEEL_MOTOR4_POS_D},
                    {RESCUE_S_P,     RESCUE_S_I,     RESCUE_S_D},
                    {BARRIER_S_P,    BARRIER_S_I,    BARRIER_S_D},
                    {Lift_Motor1_Ppid_P, Lift_Motor1_Ppid_I, Lift_Motor1_Ppid_D},
                    {Lift_Motor2_Ppid_P, Lift_Motor2_Ppid_I, Lift_Motor2_Ppid_D}
            };
    // 函数映射
    uint8_t i = 0;
    C->Can1_Send_205_TO_208 = CAN1_C620_OR_C610_205_TO_208_SendMsg;
    C->Can_Send_Gm6020 = CAN1_GM6020_1_TO_4_SendMsg;
    C->Can_Send_Wheel = CAN1_C620_OR_C610_201_TO_204_SendMsg;

    //前四个电机初始化
    for (i = 0; i < 4; i++) {
        //电机结构体清空
        MotorValZero(&C->WheelMotor[i]);
        C->WheelMotor[i].Encoder = Encoder_Init(M3508, i + 1);
        //速度环初始化
        PidInit(&C->WheelMotor[i].SPID, Spid[i][0], Spid[i][1], Spid[i][2], Integral_Limit | Output_Limit);
        PidInitMode(&C->WheelMotor[i].SPID, Integral_Limit, 200, 200);
        PidInitMode(&C->WheelMotor[i].SPID, Output_Limit, 5000, 0);
        //位置环初始化
        PidInit(&C->WheelMotor[i].PPID, Ppid[i][0], Ppid[i][1], Ppid[i][2], Integral_Limit | Output_Limit);
        PidInitMode(&C->WheelMotor[i].PPID, Integral_Limit, 200, 200);
        PidInitMode(&C->WheelMotor[i].PPID, Output_Limit, 5000, 0);
    }

    MotorValZero(&C->liftUp.Lift_Motor[0]);
    MotorValZero(&C->liftUp.Lift_Motor[1]);
    C->liftUp.Lift_Motor[0].Encoder=Encoder_Init(M3508, 7);
    C->liftUp.Lift_Motor[1].Encoder=Encoder_Init(M3508, 8);

    // PID init powered by ECF
    PidInit(&C->liftUp.Lift_Motor[0].SPID, Spid[6][0], Spid[6][1], Spid[6][2], Integral_Limit | Output_Limit);
    PidInit(&C->liftUp.Lift_Motor[0].PPID, Ppid[6][0], Ppid[6][1], Ppid[6][2], Integral_Limit | Output_Limit);

    PidInit(&C->liftUp.Lift_Motor[1].SPID, Spid[7][0], Spid[7][1], Spid[7][2], Integral_Limit | Output_Limit);
    PidInit(&C->liftUp.Lift_Motor[1].PPID, Ppid[7][0], Ppid[7][1], Ppid[7][2], Integral_Limit | Output_Limit);

    //���ֺ�����޷�
    PidInitMode(&C->liftUp.Lift_Motor[0].SPID, Integral_Limit, 200, 200);
    PidInitMode(&C->liftUp.Lift_Motor[0].SPID, Output_Limit, 9000, 0);
    PidInitMode(&C->liftUp.Lift_Motor[0].PPID, Integral_Limit, 200, 200);
    PidInitMode(&C->liftUp.Lift_Motor[0].PPID, Output_Limit, 9000, 0);

    PidInitMode(&C->liftUp.Lift_Motor[1].SPID, Integral_Limit, 200, 200);
    PidInitMode(&C->liftUp.Lift_Motor[1].SPID, Output_Limit, 9000, 0);
    PidInitMode(&C->liftUp.Lift_Motor[1].PPID, Integral_Limit, 200, 200);
    PidInitMode(&C->liftUp.Lift_Motor[1].PPID, Output_Limit, 9000, 0);

//    //救援�?5初始�?
//    MotorValZero(&C->WheelMotor[4]);
//    C->WheelMotor[4].ID = 5;
//    C->WheelMotor[4].Encoder = C->Get_Encoder(5);
//    C->WheelMotor[4].Radio = 36; //救援�? 5
//    PID_Init(&C->WheelMotor[4].SPID, Spid[4][0], Spid[4][1], Spid[4][2], 5000, 100); //速度环初始化
//
//    //救援�?6初始�?
//    MotorValZero(&C->WheelMotor[5]);
//    C->WheelMotor[5].ID = 6;
//    C->WheelMotor[5].Encoder = C->Get_Encoder(6);
//    C->WheelMotor[5].Radio = 36; //救援�? 6
//    PID_Init(&C->WheelMotor[5].SPID, Spid[5][0], Spid[5][1], Spid[5][2], 100, 100); //速度环初始化

}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 底盘独立模式，右手系，食指X ，中指Y
 *  https://pansyhou.top/Decomposition-of-mecanum-wheel-motion
 * @param C 底盘结构�?
 * @param X_IN
 * @param Y_IN
 * @param Z_IN
 * @param ExpRescue 这什么玩
 ************************** Dongguan-University of Technology -ACE***************************/
static int32_t Lift_MotorLockPos[2];
// 小轮法向量向中心
// 1,2
// 3,4
void Chassis_Indepen_Drive(C_t *C, float X_IN, float Y_IN, float Z_IN, int16_t ExpRescue) {

    uint8_t i = 0;
    int16_t Val[4] = {0, 0, 0, 0}; //记录电机实时速度�?
    int16_t MAX = 0;               //四个电机里的�?大�?�度
    float SPID_OUT[4];             // PIDout

    if (Z_IN != 0)
        Z_IN = -Z_IN;

    /*底盘麦轮运动分解*/
    C->WheelMotor[0].ExpSpeed = -(X_IN + Y_IN - Z_IN);
    C->WheelMotor[1].ExpSpeed = (X_IN - Y_IN + Z_IN);
    C->WheelMotor[2].ExpSpeed = -(X_IN - Y_IN - Z_IN);
    C->WheelMotor[3].ExpSpeed = (X_IN + Y_IN + Z_IN);

    /*速度增益*/
    C->WheelMotor[0].ExpSpeed *= ExpRescue;
    C->WheelMotor[1].ExpSpeed *= ExpRescue;
    C->WheelMotor[2].ExpSpeed *= ExpRescue;
    C->WheelMotor[3].ExpSpeed *= ExpRescue;

    /*PID处理*/
    for (i = 0; i < 4; i++) {
        PidCalculate(&C->WheelMotor[i].SPID, C->WheelMotor[i].ExpSpeed, C->WheelMotor[i].Encoder->Speed[1]); //得到输出�?
        //        Val[i] = C->WheelMotor[i].Encoder->Speed[1];    //记录电机实时速度�?
    }

    // /*底盘电机运动失真处理，处理的前提是将PID输出值与速度值认为有线�?�关�?*/
    // MAX = RETURN_MAX(Val, 4); //获取�?大�?�度

    // if (MAX == 0 || MAX <= 8750) //�?大�?�度�?0或小于电机最大�?�度，正常赋�?
    // {
    //     C->WheelMotor[0].SPID.Output = SPID_OUT[0];
    //     C->WheelMotor[1].SPID.Output = SPID_OUT[1];
    //     C->WheelMotor[2].SPID.Output = SPID_OUT[2];
    //     C->WheelMotor[3].SPID.Output = SPID_OUT[3];
    // }
    // else //否则进行运动失真处理
    // {
    //     /*确保速度不为0时，才进行处�?*/
    //     if (Val[0] != 0)
    //         C->WheelMotor[0].SPID.Output = SPID_OUT[0] * int16_t_abs(Val[0]) / MAX;
    //     else
    //         C->WheelMotor[0].SPID.Output = SPID_OUT[0];

    //     if (Val[1] != 0)
    //         C->WheelMotor[1].SPID.Output = SPID_OUT[1] * int16_t_abs(Val[1]) / MAX;
    //     else
    //         C->WheelMotor[1].SPID.Output = SPID_OUT[1];

    //     if (Val[2] != 0)
    //         C->WheelMotor[2].SPID.Output = SPID_OUT[2] * int16_t_abs(Val[2]) / MAX;
    //     else
    //         C->WheelMotor[2].SPID.Output = SPID_OUT[2];

    //     if (Val[3] != 0)
    //         C->WheelMotor[3].SPID.Output = SPID_OUT[3] * int16_t_abs(Val[3]) / MAX;
    //     else
    //         C->WheelMotor[3].SPID.Output = SPID_OUT[3];
    // }

    CAN1_C620_OR_C610_201_TO_204_SendMsg(C->WheelMotor[0].SPID.out, C->WheelMotor[1].SPID.out,
                                         C->WheelMotor[2].SPID.out, C->WheelMotor[3].SPID.out);


}

void Lift_Up_Drive(C_t *C,int32_t UP_IN){
    if (UP_IN != 0) {
        Lift_MotorLockPos[0] = C->liftUp.Lift_Motor[0].Encoder->Encode_Record_Val + UP_IN;
        Lift_MotorLockPos[1] = -Lift_MotorLockPos[0];
    }//avoid some special situation like suddenly lift
    else if(abs(abs(Lift_MotorLockPos[0])- abs(C->liftUp.Lift_Motor[0].Encoder->Encode_Record_Val))>MAX_ABS_HIGHT){
        Lift_MotorLockPos[0]=C->liftUp.Lift_Motor[0].Encoder->Encode_Record_Val;
        Lift_MotorLockPos[1] = -Lift_MotorLockPos[0];
    }
    motor_position_speed_control(&C->liftUp.Lift_Motor[0].SPID, &C->liftUp.Lift_Motor[0].PPID, Lift_MotorLockPos[0],
                                 C->liftUp.Lift_Motor[0].Encoder->Encode_Record_Val,
                                 C->liftUp.Lift_Motor[0].Encoder->Speed[1]);
    motor_position_speed_control(&C->liftUp.Lift_Motor[1].SPID, &C->liftUp.Lift_Motor[1].PPID, Lift_MotorLockPos[1],
                                 C->liftUp.Lift_Motor[1].Encoder->Encode_Record_Val,
                                 C->liftUp.Lift_Motor[1].Encoder->Speed[1]);
    CAN1_C620_OR_C610_205_TO_208_SendMsg(0, 0, C->liftUp.Lift_Motor[0].SPID.out, C->liftUp.Lift_Motor[1].SPID.out);
}

void Chassis_PowerOff_Drive(C_t *C) {
//    HAL_NVIC_SystemReset();
    //P , lock chassis position
    for (int i = 0; i < 4; i++) {
        motor_position_speed_control(&C->WheelMotor[i].SPID, &C->WheelMotor[i].PPID, C->WheelMotor_EPB_Pos[i],
                                     C->WheelMotor[i].Encoder->Encode_Record_Val, C->WheelMotor[i].Encoder->Speed[1]);
    }
    CAN1_C620_OR_C610_201_TO_204_SendMsg(C->WheelMotor[0].SPID.out, C->WheelMotor[1].SPID.out,
                                         C->WheelMotor[2].SPID.out, C->WheelMotor[3].SPID.out);
	//P , lock lift position
    Lift_Up_Drive(C, 0);

}



