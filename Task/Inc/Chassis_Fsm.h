#ifndef __CHASSIA_FSM_H_
#define __CHASSIA_FSM_H_

#include "fsm.h"

/*底盘状态机初始化*/
void Chassis_FSM_Init(void);

/*返回底盘状态机控制指针*/
FSM_t *Return_Chassis_FSM(void);
/***********************OFFLINE*************************/
static void Offline_Prepare(void); /*OFFLINE状态准备函数*/
static void Offline_State(void);   /*离线状态处理*/
static void PowerOff_bhv(void);    /*断电行为函数*/

/***********************INDEPEN*************************/
static void Indepen_Prepare(void);    /*INDEPEN状态准备函数*/
static void Indepen_State(void);      /*独立状态处理*/
static void Chassis_Normal_bhv(void); /*底盘正常运动行为函数*/

// /***********************ROTATION*************************/
// static void Rotation_Prepare(void);    /*Rotation状态准备函数*/
// static void Rotation_State(void);      /*大陀螺状态处理*/
// static void Rotation_Normal_bhv(void); /*大陀螺运动行为函数*/

/***********************KEYBOARD*************************/
static void KeyBoard_Prepare(void);     /*KEYBOARD状态准备函数*/
static void KeyBoard_State(void);       /*键盘状态处理*/
static void KeyBoard_bhv(void);     /*上装状态*/

static void HageUP_Prepare(void);
static void HageUP_Normal_bhv(void);
static void HageUP_State(void);
#endif
