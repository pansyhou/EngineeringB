#include "SYSInit.h"
//#include "Distance_Sensor.h"
#include "bsp_can.h"

//#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"




/************************** Dongguan-University of Technology -ACE**************************
 * @brief 系统硬件配置初始化，必须写在FreeRTOS开始之前
 *
 ************************** Dongguan-University of Technology -ACE***************************/
void Sys_Init(void) {
#if DEBUG_Mode
//	SEGGER_SYSVIEW_Conf(); // SystemView调试初始化
//	// SEGGER_SYSVIEW_Start();//
//
//	//初始化DWT计数器,SystemView会从DWT的Cycle counter获取系统当前周期数用于生成时间戳
//	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//	DWT->CYCCNT = 0;
//	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    ECF_RC_Init();
    ECF_CAN_Init();
    //遥控初始化
    Remote_Data_Init();
#if LOG_SOURCE==1
    SEGGER_RTT_Init();
#endif
//    HAL_GPIO_WritePin(GPIOG, LED1_Pin, 1);
//    HAL_GPIO_WritePin(GPIOG, LED2_Pin, 1);
//    HAL_GPIO_WritePin(GPIOG, LED3_Pin, 1);

}

#if LOG_SOURCE==1

/**
 * 测试用的log输出，自带换行，目前版本是采用RTT viewer来转发，三个点是printf的可变参数
 * @param format
 * @param ...
 */
void Test_Log(const char *format, ...)
{
    SEGGER_RTT_printf(0, format);
    SEGGER_RTT_printf(0,"\r\n");
}
//void printf_usb(const char *format, ...){
//    va_list  args;
//    uint32_t length;
//    uint8_t buff[APP_TX_DATA_SIZE];
//
//    va_start(args, format);
//    length = vsnprintf((char *)buff, APP_TX_DATA_SIZE, (char *)format, args);
//    va_end(args);
//    CDC_Transmit_FS(buff, length);
//}

#endif