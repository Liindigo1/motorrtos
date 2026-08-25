#ifndef __FOC_H
#define __FOC_H

#include <stdint.h>

#define PI	3.14159265358979f
#define Current_TS 0.0002f     // 5000 Hz
#define SPEED_TS   0.005f     // 200 Hz
#define IQ_MAX     2.0f       // 
#define UQ_MAX     10.0f       // 

typedef struct
{
    //电压 
    float Ualpha;
    float Ubeta;
    float Ud;
    float Uq;

    //电流 
    float Ialpha;
    float Ibeta;
    float Ia;
    float Ib;
    float Ic;
    float Id;
    float Iq;

    //给定set 
    float Wm_Set;
    float Id_Set;
    float Iq_Set;

    //位置速度 
    float Theta;
    float HallThetaAdd;
    float We;
    float Wm;

    uint8_t Sector;//转子扇区

} MOTOR;

typedef struct {
    float target_val;    // 目标值
    float actual_val;    // 实际值
    float err;           // 当前误差
    float err_last;      // 上次误差  
    float err_next;      // 上上次误差
    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;            // 微分系数
	  float integral;
} PID_TypeDef;

	typedef struct
{
	uint8_t last_hall;              //上一次霍尔读取
  uint8_t current_hall;
	int8_t Last_HallDirection;      //上一次方向
	int8_t Current_HallDirection ;
}HALLstate;

extern HALLstate HALLstate1;
extern MOTOR Motor1;
extern PID_TypeDef Currentpid;
extern PID_TypeDef Speedpid;
extern PID_TypeDef SpeedpidU;
extern PID_TypeDef SpeedpidUZ;

void Park_Inverse_Transform(void);
void FOC_SVPWM(void);
void Clark_Park_i(void);
float Current_Loop(PID_TypeDef *pid,float actual_i,float target_i);
float Speed_Loop(PID_TypeDef *pid, float temp_val, float goal_val);
float Speed_Loop_Voltage(PID_TypeDef *pid,float actual_speed,float target_speed);
float Speed_Loop_VoltageZ(PID_TypeDef *pid,float actual_speed,float target_speed);
void PID_param_init(void);

#endif 
