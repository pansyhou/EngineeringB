#include "SYSInit.h"
//#include "Distance_Sensor.h"
#include "bsp_can.h"

//#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"




/************************** Dongguan-University of Technology -ACE**************************
 * @brief ϵͳӲ�����ó�ʼ��������д��FreeRTOS��ʼ֮ǰ
 *
 ************************** Dongguan-University of Technology -ACE***************************/
void Sys_Init(void) {
#if DEBUG_Mode
//	SEGGER_SYSVIEW_Conf(); // SystemView���Գ�ʼ��
//	// SEGGER_SYSVIEW_Start();//
//
//	//��ʼ��DWT������,SystemView���DWT��Cycle counter��ȡϵͳ��ǰ��������������ʱ���
//	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//	DWT->CYCCNT = 0;
//	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    ECF_RC_Init();
    ECF_CAN_Init();

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
 * �����õ�log������Դ����У�Ŀǰ�汾�ǲ���RTT viewer��ת������������printf�Ŀɱ����
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