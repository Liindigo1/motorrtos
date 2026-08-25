#include "user_task.h"
#include "FOC.h"
#include "mymath.h"
#include "string.h"
#include "usart.h"
#include "tim.h"

volatile uint8_t start_i=0;//0-3：启动阶段/正反转切换的启动阶段；4：运行阶段
volatile uint8_t overflow_count=0;
float HallCorrectTheta;
uint8_t HallUpdateFlag;
SemaphoreHandle_t xHallSemaphore= NULL;	
TaskHandle_t xAdcTaskHandle= NULL;
#define UQ_STEP 0.2f

void task_init(void)
{
	BaseType_t xReturn;
  xHallSemaphore=xSemaphoreCreateBinary(); 
	configASSERT(xHallSemaphore != NULL);
	
  xReturn=xTaskCreate(vHallTask,"HALL",256,NULL,3,NULL); //
	configASSERT(xReturn == pdPASS);
	xReturn=xTaskCreate(vAdcTask,"adc",256,NULL,2,&xAdcTaskHandle);  //
	configASSERT(xReturn == pdPASS);
	xReturn=xTaskCreate(vUrdf2Task,"urdf2",256,NULL,1,NULL); //
	configASSERT(xReturn == pdPASS);
}


void vHallTask(void *pvParameters)//任务函数
{

	 float HallTheta = 0; 
   for(;;)
	{
		xSemaphoreTake( xHallSemaphore, portMAX_DELAY );
		
    switch (HALLstate1.current_hall)
   {
    case 0x05: HallTheta =   0.0f; break;
    case 0x04: HallTheta =  60.0f; break;  //其实霍尔自学习读出来有几度偏差
    case 0x06: HallTheta = 120.0f; break;
    case 0x02: HallTheta = 180.0f; break;
    case 0x03: HallTheta = 240.0f; break;
    case 0x01: HallTheta = 300.0f; break;

    default:
        continue; //错误不再往下计算
    }
	
	  HALLstate1.Current_HallDirection=Hall_GetDirection(HALLstate1.last_hall, HALLstate1.current_hall);//霍尔方向获取
		HALLstate1.last_hall=HALLstate1.current_hall;//更新一下
   
	  if(start_i<4){HallTheta += PHASE_SHIFT_ANGLE+30;}//启动拉中间
    else{	
	     if(HALLstate1.Current_HallDirection==1)//正转角度
	    {
         HallTheta += PHASE_SHIFT_ANGLE;}
	     else if(HALLstate1.Current_HallDirection==-1)//反转角度
         {HallTheta += PHASE_SHIFT_ANGLE+60.0f;}
     }
	
	 if((HALLstate1.Current_HallDirection+HALLstate1.Last_HallDirection)==0){start_i=0;Motor1.Wm=0;Motor1.HallThetaAdd=0;}//判断有无正>反/反>正
	
   HALLstate1.Last_HallDirection=HALLstate1.Current_HallDirection;//更新一下
	
	 Motor1.Wm=HALLstate1.Current_HallDirection*Motor1.Wm;//速度更新
   HallCorrectTheta = HallTheta;//角度
	 HallUpdateFlag=1;//更新标志

	}

}

void vAdcTask(void *pvParameters)//执行电流环
{
	
  for(;;)
  {
		static uint8_t speed_div = 0; static uint8_t current_div = 0;
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    
		Run_check0();

		
		if(HallUpdateFlag)//更新校准，只在一个任务里修改Motor1.Theta，防止冲突（或者使用调度锁/临界区）
   {
    Motor1.Theta = HallCorrectTheta;
    HallUpdateFlag = 0;
   }else{Motor1.Theta += HALLstate1.Current_HallDirection*Motor1.HallThetaAdd;}
		
//		Motor1.Theta+=0.0036;霍尔自学习时用
		
   while (Motor1.Theta >= 360.0f)//限制区间
       Motor1.Theta -= 360.0f;
   while (Motor1.Theta < 0.0f)
       Motor1.Theta += 360.0f;
	 
 
   if(++speed_div >= 50) //速度环周期
   {
		 speed_div = 0;
		 float Uq_Target;//配合位置PI使用	
		 
		 if(start_i>=4){
			 		 Motor1.Uq += Speed_Loop_VoltageZ(&SpeedpidUZ,Motor1.Wm,Motor1.Wm_Set);//速度PI稳一点
//         Uq_Target = Speed_Loop_Voltage(&SpeedpidU,Motor1.Wm,Motor1.Wm_Set);	//位置PI	
//			 	 U_slope(Uq_Target);
		 }
     else{ Motor1.Uq =0.0025*Motor1.Wm_Set;}//启动一个简单的小前馈

//     Motor1.Iq_Set=Speed_Loop(&Speedpid,Motor1.Wm,Motor1.Wm_Set);
    }
	 
	 if(++current_div >= 2)
	 {
	  current_div = 0;
		 
//		Clark_Park_i();//最新ADC电流计算
//   //电流环 -<给定电流 ->电压
//    Motor1.Uq=Current_Loop(&Currentpid,Motor1.Iq,0.2);	
		 
//	 Motor1.Ud=1.0;Motor1.Uq=0.0;//霍尔自学习/校准时用，通过Ud角度校准
//	 Motor1.Uq=5;	 //给定定值
		 
		 Park_Inverse_Transform();//减少更新频率才能有时间打印；F1没有硬件FPU
	   FOC_SVPWM(); 
	 }
		
//	 Park_Inverse_Transform();
//	 FOC_SVPWM(); 
  }

}

void vUrdf2Task(void *pvParameters)
{
	static uint8_t tempData[12] = {0,0,0,0,0,0,0,0,0,0,0x80,0x7F};
	for(;;)
  {
		float load_date[2];
	  load_date[0]=Motor1.Uq;
		load_date[1]=Motor1.Wm;
	 	memcpy(tempData, (uint8_t *)&load_date, sizeof(load_date));
    HAL_UART_Transmit_DMA(&huart2,(uint8_t *)tempData,3*4);
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void U_slope(float Uq_Target)//U变化斜坡，防止跳变
{
    float Uq_Command = Motor1.Uq ;
	
   if(Uq_Command < Uq_Target)
   {
       Uq_Command += UQ_STEP;

       if(Uq_Command > Uq_Target)
			 {Uq_Command = Uq_Target;}
   }
   else if(Uq_Command > Uq_Target)
  {
       Uq_Command -= UQ_STEP;

       if(Uq_Command < Uq_Target)
			 {Uq_Command = Uq_Target;}
   }

  Motor1.Uq = Uq_Command;
}

int8_t Hall_GetDirection(uint8_t last,uint8_t now)
{
    //正转 
    if((last == 0x05 && now == 0x04) ||
       (last == 0x04 && now == 0x06) ||
       (last == 0x06 && now == 0x02) ||
       (last == 0x02 && now == 0x03) ||
       (last == 0x03 && now == 0x01) ||
       (last == 0x01 && now == 0x05))
    {
        return 1;
    }

    //反转 
    if((last == 0x04 && now == 0x05) ||
       (last == 0x06 && now == 0x04) ||
       (last == 0x02 && now == 0x06) ||
       (last == 0x03 && now == 0x02) ||
       (last == 0x01 && now == 0x03) ||
       (last == 0x05 && now == 0x01))
    {
        return -1;
    }

    return 0;       // 非法跳变
}

void Run_check0(void)
{
   if((start_i==4)&&(overflow_count>5))//如果运行阶段霍尔长时间没有触发，状态清0
		{   

			Motor1.Wm=0;Motor1.HallThetaAdd=0;//电机相关状态
			
			Motor1.Uq=0;
			
			SpeedpidU.err=0;SpeedpidU.integral  = 0.0f;//没有微分不用清lasterr
			SpeedpidUZ.err=0;SpeedpidUZ.err_last=0;
			
		  start_i=0;//阶段退回
			
			// Hall方向状态 
      HALLstate1.Current_HallDirection = 0;
      HALLstate1.Last_HallDirection = 0;
			// 重新从当前Hall状态起步 
      HALLstate1.last_hall =
      HALLstate1.current_hall;
			Motor1.Theta=Start_HallGetTheta();//重要
			// Hall超时状态 
      overflow_count = 0;
			
		}
}

float Start_HallGetTheta(void)//开始霍尔状态获取
{
    float theta;
	  uint8_t hall;
	
    hall  = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);
    hall |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) << 1;
    hall |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) << 2;
	
    switch(hall)
    {
        case 0x05: theta =   0.0f; break;
        case 0x04: theta =  60.0f; break;
        case 0x06: theta = 120.0f; break;
        case 0x02: theta = 180.0f; break;
        case 0x03: theta = 240.0f; break;
        case 0x01: theta = 300.0f; break;

        default:
            return -1.0f;
    }

    theta += PHASE_SHIFT_ANGLE;
			
    while(theta >= 360.0f)
		  {theta -= 360.0f;}
		
		return theta;
    
}

