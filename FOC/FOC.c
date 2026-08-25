#include "main.h"
#include "mymath.h"
#include "FOC.h"
#include "math.h"
#include "tim.h"
float SQRT_3=1.73205080756888f;
static const float SVPWM_TS = 1.0f;
static const float UDC      = 24.0f;
uint8_t MOTOR1_Pn=2;//记录一下

MOTOR Motor1={.Ud=0, .Wm_Set=0,.HallThetaAdd=0,};	
HALLstate HALLstate1;
PID_TypeDef Currentpid;	
PID_TypeDef Speedpid;
PID_TypeDef SpeedpidU;
PID_TypeDef SpeedpidUZ;

void Park_Inverse_Transform(void)//反Park
{
	Motor1.Ualpha = Motor1.Ud* qfcosd(Motor1.Theta)- Motor1.Uq*qfsind(Motor1.Theta);
	Motor1.Ubeta  = Motor1.Ud* qfsind(Motor1.Theta)+ Motor1.Uq*qfcosd(Motor1.Theta);   
}
 
//SVPWM
void FOC_SVPWM(void)
{
	uint8_t N,A,B,C;
	float	Vref1,Vref2,Vref3,X,Y,Z,temp1,Tfirst,Tsecond,T0,Ta,Tb,Tc1,Tcm1,Tcm2,Tcm3; 
	
	//计算转子所在的山区
	Vref1=Motor1.Ubeta;
	Vref2=(SQRT_3 * Motor1.Ualpha- Motor1.Ubeta)/2;
	Vref3=(-SQRT_3 * Motor1.Ualpha- Motor1.Ubeta)/2;
	
	A=(Vref1>0) ? 1 : 0 ;
	B=(Vref2>0) ? 1 : 0 ;
	C=(Vref3>0) ? 1 : 0 ;
	
	N=4*C+2*B+A;
 
	temp1=SQRT_3* SVPWM_TS/ UDC;
	X=temp1*Vref1;
	Y=-temp1*Vref3;
	Z=-temp1*Vref2;
	

 
	//矢量作用时间计算
	switch(N)
	{
		case 1:
			Tfirst=  Z;
			Tsecond= Y;
			Motor1.Sector= 2;
			break;
		case 2:
			Tfirst= Y;
			Tsecond= -X;
			Motor1.Sector= 6;
			break;
		case 3:
			Tfirst= -Z;
			Tsecond= X;
			Motor1.Sector= 1;
			break;
		case 4:
			Tfirst= -X;
			Tsecond= Z;
			Motor1.Sector= 4;
			break;
		case 5:
			Tfirst= X;
			Tsecond= -Y;
			Motor1.Sector= 3;
			break;
		case 6:
			Tfirst= -Y;
			Tsecond= -Z;
			Motor1.Sector= 5;
			break;
		default:
			Tfirst= 0;
			Tsecond= 0;
			Motor1.Sector= 0;
			break;
	}
	
	//超限判断
   if((Tfirst + Tsecond) > SVPWM_TS)
  {
    float scale =SVPWM_TS / (Tfirst + Tsecond);
    Tfirst  *= scale;
    Tsecond *= scale;
   }

	T0= (SVPWM_TS- Tfirst- Tsecond)/2;
	Ta=T0;
	Tb=T0+Tfirst+Tsecond;
	Tc1=T0+Tsecond;

 
	//每相桥臂切换时间计算
	switch(N)
	{
		case 1:
			Motor1.Sector= 2;
			Tcm1=Tc1;
			Tcm2=Tb;
			Tcm3=Ta;
			break;
		case 2:
			Motor1.Sector= 6;
			Tcm1=Tb;
			Tcm2=Ta;
			Tcm3=Tc1;
			break;
		case 3:
			Motor1.Sector= 1;
			Tcm1=Tb;
			Tcm2=Tc1;
			Tcm3=Ta;
			break;
		case 4:
			Motor1.Sector= 4;
			Tcm1=Ta;
			Tcm2=Tc1;
			Tcm3=Tb;
			break;
		case 5:
			Motor1.Sector= 3;
			Tcm1=Ta;
			Tcm2=Tb;
			Tcm3=Tc1;
			break;
		case 6:
			Motor1.Sector= 5;
			Tcm1=Tc1;
			Tcm2=Ta;
			Tcm3=Tb;
			break;
		default:
			Tcm1 = 0.5f;//有问题让矢量抵消为0
      Tcm2 = 0.5f;
      Tcm3 = 0.5f;
      Motor1.Sector = 0;
			break;
	}
		
	//设置定时器1的PWM占空比
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(int)(3599*(Tcm1/ SVPWM_TS)+0.5));//+0.5进行四舍五入
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(int)(3599*(Tcm2/ SVPWM_TS)+0.5));
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(int)(3599*(Tcm3/ SVPWM_TS)+0.5));

}

void Clark_Park_i()
{
	//对电流进行Clark变换，再Park变换
	Motor1.Ialpha = Motor1.Ia;
	Motor1.Ibeta  = (Motor1.Ia + 2*Motor1.Ib)/SQRT_3;
	
	Motor1.Id =  Motor1.Ialpha* qfcosd(Motor1.Theta)+ Motor1.Ibeta* qfsind(Motor1.Theta);
	Motor1.Iq = -Motor1.Ialpha* qfsind(Motor1.Theta)+ Motor1.Ibeta* qfcosd(Motor1.Theta);

}


//电流环位置式：
float Current_Loop(PID_TypeDef *pid,float actual_i,float target_i)
{	
	  //PI环节
    float output;float i_add;
    pid->actual_val = actual_i;
    pid->target_val = target_i;
    // 电流误差 A 
    pid->err =pid->target_val-pid->actual_val;

//    //死区 
//    if(fabsf(pid->err) < 0.02f)
//    {  pid->err = 0.0f;
//    }

    // 积分增加项，单位最终为U 
    i_add=pid->Ki *pid->err *Current_TS;
   
    //假设本周期加入积分后的输出
    output =pid->Kp * pid->err + pid->integral+ i_add;
		
		//抗积分饱和
		if(!(((output>UQ_MAX) && (pid->err>0))||((output<-UQ_MAX) && (pid->err<0)))){
		pid->integral += i_add;	
	  //积分限幅 
    pid->integral =Value_SetMaxMin(pid->integral,-UQ_MAX,UQ_MAX);	
		}
	
		// PI输出 = Uq_ref 
		output =pid->Kp * pid->err+ pid->integral;
	  //合成电压最大值为SQRT_3 * UDC / 3 = 13.85 
	  output = Value_SetMaxMin(output,-UQ_MAX,UQ_MAX);
		pid->err_last = pid->err;
		
	  return output;
}


 
//速度环位置式：配合电流环在适合场景调试
float Speed_Loop(PID_TypeDef *pid,float actual_rpm,float target_rpm)
{
    float output;float i_add;
    pid->actual_val = actual_rpm;
    pid->target_val = target_rpm;
    /* 速度误差 rpm */
    pid->err =pid->target_val-pid->actual_val;

    //死区 
    if(fabsf(pid->err) < 1.0f)
    {  pid->err = 0.0f;
    }
		
    /* 积分项，单位最终为A */
		i_add=pid->Ki *pid->err *SPEED_TS;

    //假设本周期加入积分后的输出
    output =pid->Kp * pid->err + pid->integral+ i_add;
		
		//抗积分饱和
		if(!((output>IQ_MAX && pid->err>0)||(output<-IQ_MAX && pid->err<0))){
		pid->integral += i_add;	
	  //积分限幅 
    pid->integral =Value_SetMaxMin(pid->integral,-IQ_MAX,IQ_MAX);	
		}
		
		// PI输出 = Iq_ref 
		output =pid->Kp * pid->err+ pid->integral;
		
    /* Iq_ref限幅 */
    output =Value_SetMaxMin(output,-IQ_MAX,IQ_MAX);
    pid->err_last = pid->err;
		
    return output;
}

//速度环输出电压位置式：√
float Speed_Loop_Voltage(PID_TypeDef *pid,float actual_speed,float target_speed)
{
    float output;
    float i_add;

    pid->err = target_speed - actual_speed;

    i_add = pid->Ki * pid->err * SPEED_TS;

    output = pid->Kp * pid->err+ pid->integral+ i_add;

    if(!(((output > UQ_MAX) && (pid->err > 0.0f)) ||((output < -UQ_MAX) && (pid->err < 0.0f))))
    {
        pid->integral += i_add;
        pid->integral =Value_SetMaxMin(pid->integral,-UQ_MAX,UQ_MAX);
    }

    output =pid->Kp * pid->err+ pid->integral;

    return Value_SetMaxMin(output,-UQ_MAX,UQ_MAX);
}

//速度环输出电压增量式：√
float Speed_Loop_VoltageZ(PID_TypeDef *pid,float actual_speed,float target_speed)
{
    float du;

    pid->actual_val = actual_speed;
    pid->target_val = target_speed;

    //当前速度误差 
    pid->err = pid->target_val - pid->actual_val;

    du = pid->Kp * (pid->err - pid->err_last)+ pid->Ki * SPEED_TS * pid->err;

    // 保存状态 
    pid->err_last = pid->err;

    return du;
}


void PID_param_init(void)
{
	/* 初始化参数
	速度环
  J*（dw/dt）+Bω=Kt*Iq-TL    J:转动惯量，Kt力矩常数，记录一下
  Iq=Kp*ew+Ki*∫ewdt
  w->n:ω=(2*pi/60)*n
	电流环
  L*(di/dt)+Ri=u
	*/
  Speedpid.target_val=0.0;				
  Speedpid.actual_val=0.0;
	Speedpid.err = 0.0;
	Speedpid.err_last = 0.0;
	Speedpid.err_next = 0.0;
  Speedpid.Kp = 0.00185f;
  Speedpid.Ki = 0.082f;
  Speedpid.Kd = 0.0;
	
	/* 初始化参数
	我空载测试不同稳定电压的情况下电流很小
	空电机调试不适合稳定Iq长时间调电流环	，空载时几乎没有负载转矩来平衡
	J*（dw/dt）=Te-TL-Bw
	如果要求 Iq=2A 长期保持，控制器就会不断提高 Uq 来抵消越来越大的反电动势
	给电机加机械负载 / 锁转，但一定要小电流/低电流 + 短时间测试
	*/
	Currentpid.target_val=0.0;	 
  Currentpid.actual_val=0.0;
	Currentpid.err = 0.0;
	Currentpid.err_last = 0.0;
	Currentpid.err_next = 0.0;
  Currentpid.Kp = 1.06;
  Currentpid.Ki = 396.0;
  Currentpid.Kd = 0.0;
	
	/* 初始化参数
	空载电机、调试长期稳定转速运行：我使用速度闭环 + q轴电压控制
	*/
  SpeedpidU.target_val=0.0;				
  SpeedpidU.actual_val=0.0;
	SpeedpidU.err = 0.0;
	SpeedpidU.err_last = 0.0;
	SpeedpidU.err_next = 0.0;
  SpeedpidU.Kp = 0.0038f;
  SpeedpidU.Ki = 0.04f;
  SpeedpidU.Kd = 0.0;
	
	SpeedpidUZ.target_val=0.0;				
  SpeedpidUZ.actual_val=0.0;
	SpeedpidUZ.err = 0.0;
	SpeedpidUZ.err_last = 0.0;
	SpeedpidUZ.err_next = 0.0;
  SpeedpidUZ.Kp = 0.00202f;
  SpeedpidUZ.Ki = 0.0145f;
  SpeedpidUZ.Kd = 0.0;

}
