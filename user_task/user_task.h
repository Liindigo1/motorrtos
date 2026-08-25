#ifndef __usertask_H
#define __usertask_H

#include "main.h"
#include "FreeRTOS.h" // FreeRTOS的核心头文件，必须引用
#include "task.h"  // 任务相关的头文件，必须引用
#include "semphr.h"  // 头文件，必须引用

extern volatile uint8_t overflow_count;//用于tim.c
extern SemaphoreHandle_t xHallSemaphore;	
extern TaskHandle_t xAdcTaskHandle;
extern volatile uint8_t start_i;
#define PHASE_SHIFT_ANGLE (float)(42.0f)

void vHallTask(void *pvParameters);
void vAdcTask(void *pvParameters);//执行电流环
void vUrdf2Task(void *pvParameters);
void task_init(void);
void U_slope(float Uq_Target);
int8_t Hall_GetDirection(uint8_t last,uint8_t now);
void Run_check0(void);
float Start_HallGetTheta(void);
#endif 
