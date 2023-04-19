# 工程底盘版

## 大致架构

![](https://pic.imgdb.cn/item/63c8d854be43e0d30e4bfcd5.jpg)

> 之前上课写的，下次更新一定用drawio画



## 任务层Task

### FSM

有限状态机通过遥控器摇杆量控制机身模式，想要学习FSM可以翻翻之前前辈的笔记或者知乎一下

> 所属源代码有：fsm.c、Chassis_Task.c、Chassis_FSM.c

1. fsm.c：存放有限状态机的主要代码
2. Chassis_Task.c：底盘控制任务，主循环里只有FSM的主要函数（检查并更改状态）
3. Chassis_FSM.c：存放状态机的各个状态以及函数，用于初始化、绑定fsm各个指针函数（不完全初始化

### DetectTask

目前用了一个很粗糙的方法将detecttask和心跳任务结合了。

这个自己看函数理解吧，就是检查有无掉线，掉线之后会处理对应的解决问题函数，另一个就是心跳任务（LED灯检查是否在线）

### RemoteTask

将Dr16的数据做平滑和按键处理



## 控制层Control

### SYSInit

系统硬件部分初始化，同时.h文件有较多的垃圾可以看看

### ChassisMotor

这里将所有的驱动都塞进去了