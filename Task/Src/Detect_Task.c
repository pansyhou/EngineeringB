#include "Detect_Task.h"
#include "cmsis_os.h"
#include "SYSInit.h"
#include "bsp_can.h"
extern osThreadId Task_ChassisHandle;
/**
 * 检测任务初始化
 * @param time 系统时间（FreeRTOS时间戳）
 */
static void detect_init(uint32_t time);

void Detect_Task(void const *pvParameters);

extern bool_t RC_Check_Data_IS_ERROR(void);

error_t error_list[ERROR_LIST_LENGHT + 1];

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif

bool_t Is_Online(uint8_t toe){
    if(error_list[toe].is_lost == 0)return 0;
    else return 1;
}


static void detect_init(uint32_t time) {
    //设置各设备离线时间，上线稳定工作时间，优先级 offlineTime onlinetime priority
    uint16_t set_item[ERROR_LIST_LENGHT][3] =
            {
                    {800, 100, 15},   //DBUS
                    {30, 40, 15},   //CAN2 Bus
                    {10, 10, 11},    //Chassis 1 motor
                    {10, 10, 11},   //Chassis 2 motor
                    {10, 10, 11},   //Chassis 3 motor
                    {10, 10, 11},   //Chassis 4 motor
            };

    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++) {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].data_is_error_fun = NULL;
        error_list[i].solve_lost_fun = NULL;
        error_list[i].solve_data_error_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].error_exist = 1;
        error_list[i].is_lost = 1;
        error_list[i].data_is_error = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }

    error_list[DBUS_TOE].solve_lost_fun = &RC_Check_Data_IS_ERROR;
}

/**
 *  获取设备对应的错误状态
 * @param toe:设备目录
 * @return true(错误) 或者false(没错误)
 */
bool_t toe_is_error(uint8_t toe) {
    return (error_list[toe].error_exist == 1);
}

/**
 * 钩子函数，记录时间
 * @param toe:设备目录
 */
void detect_hook(uint8_t toe) {

    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();

    if (error_list[toe].is_lost) {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }

    if (error_list[toe].data_is_error_fun != NULL) {
        if (error_list[toe].data_is_error_fun()) {
            error_list[toe].error_exist = 1;
            error_list[toe].data_is_error = 1;

            if (error_list[toe].solve_data_error_fun != NULL) {
                error_list[toe].solve_data_error_fun();
            }
        } else {
            error_list[toe].data_is_error = 0;
        }
    } else {
        error_list[toe].data_is_error = 0;
    }
    PGout((toe + 1)) = 0;
}

/**
  * @brief          得到错误列表
  * @param[in]      none
  * @retval         error_list的指针
  */
const error_t *get_error_list_point(void) {
    return error_list;
}


void Detect_Task(void const *pvParameters) {
    static uint32_t system_time;
    system_time = xTaskGetTickCount();
    //init,初始化
    detect_init(system_time);
    //wait a time.空闲一段时间
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1) {
//        心跳任务

//			LEDG1 = 1;
//			LEDG2 = 1;
//			LEDG3 = 1;
//			LEDG4 = 1;
//			LEDG5 = 1;
//			LEDG6 = 1;
//			LEDG7 = 1;
//			LEDG8 = 1;

        static uint8_t error_num_display = 0;
        system_time = xTaskGetTickCount();

        error_num_display = ERROR_LIST_LENGHT;
        error_list[ERROR_LIST_LENGHT].is_lost = 0;
        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < ERROR_LIST_LENGHT; i++) {
            //disable, continue
            //未使能，跳过
            if (error_list[i].enable == 0) {
                continue;
            }

            //judge offline.判断掉线
            if (system_time - error_list[i].new_time > error_list[i].set_offline_time) {
                if (error_list[i].error_exist == 0) {
                    //record error and time
                    //记录错误以及掉线时间
                    error_list[i].is_lost = 1;
                    error_list[i].error_exist = 1;
                    error_list[i].lost_time = system_time;
                }
                //judge the priority,save the highest priority ,
                //判断错误优先级， 保存优先级最高的错误码
                if (error_list[i].priority > error_list[error_num_display].priority) {
                    error_num_display = i;
                }


                error_list[ERROR_LIST_LENGHT].is_lost = 1;
                error_list[ERROR_LIST_LENGHT].error_exist = 1;
                //if solve_lost_fun != NULL, run it
                //如果提供解决函数，运行解决函数
                if (error_list[i].solve_lost_fun != NULL) {
                    error_list[i].solve_lost_fun();
                }
            } else if (system_time - error_list[i].work_time < error_list[i].set_online_time) {
                //just online, maybe unstable, only record
                //刚刚上线，可能存在数据不稳定，只记录不丢失，
                error_list[i].is_lost = 0;
                error_list[i].error_exist = 1;
            } else {
                error_list[i].is_lost = 0;
                //判断是否存在数据错误
                //judge if exist data error
                if (error_list[i].data_is_error != NULL) {
                    error_list[i].error_exist = 1;
                } else {
                    error_list[i].error_exist = 0;
                }
                //calc frequency
                //计算频率
                if (error_list[i].new_time > error_list[i].last_time) {
                    error_list[i].frequency = configTICK_RATE_HZ / (fp32) (error_list[i].new_time - error_list[i].last_time);
                }
            }
        }

        if (error_list[DBUS_TOE].is_lost == 1) {
            vTaskSuspend(Task_ChassisHandle);
            CAN1_C620_OR_C610_205_TO_208_SendMsg(0, 0, 0, 0);
        }
         else if (eTaskGetState(Task_ChassisHandle)==eSuspended&&error_list[DBUS_TOE].is_lost == 0) {
                    vTaskResume(Task_ChassisHandle);
                }

        vTaskDelay(DETECT_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}