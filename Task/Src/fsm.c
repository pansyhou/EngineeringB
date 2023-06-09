/************************** Dongguan-University of Technology -ACE**************************
 * @file fsm.c
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-06
 * 
 * @copyright Copyright (c) 2022
 * 
 ************************** Dongguan-University of Technology -ACE***************************/
#include "fsm.h"

/*****************************东莞理工学院ACE实验室 *****************************
 * @brief 状态机处理
 * 
 * @param fsm 
 * @param s1 左拨杆数值
 * @param s2 右
 * ps：这些函数指针都会在各自的初始化函数里指向自己各个状态的函数
*******************************东莞理工学院ACE实验室 *****************************/
void FSM_Deal(FSM_t *fsm, uint8_t s1, uint8_t s2)
{
    /*误操作判断,边界保护*/
    if (s1 <= 0 || s1 > State_Line || s2 <= 0 || s2 > State_Column)
    {
        return;
    }

    /*状态指向*/
    fsm->Current_State = &fsm->State_Table[s1 - 1][s2 - 1];

    /*状态变化*/
    if (fsm->State_Change(fsm->Last_State, fsm->Current_State) == 1)
    {
        fsm->Current_State->State_Prepare();
    }

    /*保留上次状态*/
    fsm->Last_State = fsm->Current_State;

    /*执行状态*/
    fsm->Current_State->State_Process();

    /*执行实际行为*/
    fsm->Current_State->Behavior_Process();
}


/*****************************东莞理工学院ACE实验室 *****************************
 * @brief 状态变更函数（遥控器左右拨杆切状态
 * 
 * @param L_Sta Last_State上次状态
 * @param C_Sta Current_State目前状态
 * @return unsigned char 
*******************************东莞理工学院ACE实验室 *****************************/
unsigned char StateChange(State_t *L_Sta, State_t *C_Sta)
{
    return (L_Sta != C_Sta ? (1) : (0));
}
