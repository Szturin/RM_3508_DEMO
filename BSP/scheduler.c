#include "scheduler.h"

//调度器函数库

uint8_t tasks_num;//任务数量
//调度器类型的结构体类型声明
//任务结构

typedef struct {
    void (*task_func)(void);//函数指针
    uint32_t rate_ms;//任务运行周期
    uint32_t last_run;//上次运行时间
} scheduler_task_t;


//定义调度器类型的变量 任务
//任务数组
//给scheduler_task_t类型变量tasks赋初值
static scheduler_task_t tasks[] ={
	{key_proc,10,0},
	{uart_proc,10,0} //定义uart1任务
};

void scheduler_init(void){
    tasks_num = sizeof(tasks)/sizeof(scheduler_task_t);
}

void scheduler_run(void){
    uint32_t time_now = HAL_GetTick();//假设为获取当前时间的函数
    /*任务轮询*/
    for(uint8_t i=0; i< tasks_num; i++){
        if(time_now >= tasks[i].rate_ms + tasks[i].last_run){
            tasks[i].last_run = time_now;//保存当前时间
            tasks[i].task_func();//执行对应指向的任务
        }
    }
}

//现象，设置led_proc 1000 ms时不亮
//设置100ms时亮时正常点亮
//原因：uint8_t time_now = HAL_GetTick();//假设为获取当前时间的函数
////应该定义为uint32_t