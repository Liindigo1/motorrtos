#include "mymath.h"
 
 
const float hollyst = 0.017453292519943295769236907684886f;
 
const float sin_table[] = {
    0.0f,                                    //sin(0)
    0.17364817766693034885171662676931f ,    //sin(10)
    0.34202014332566873304409961468226f ,    //sin(20)
    0.5f ,                                   //sin(30)
    0.64278760968653932632264340990726f ,    //sin(40)
    0.76604444311897803520239265055542f ,    //sin(50)
    0.86602540378443864676372317075294f ,    //sin(60)
    0.93969262078590838405410927732473f ,    //sin(70)
    0.98480775301220805936674302458952f ,    //sin(80)
    1.0f                                     //sin(90)
};
 
const float cos_table[] = {
    1.0f ,                                   //cos(0)
    0.99984769515639123915701155881391f ,    //cos(1)
    0.99939082701909573000624344004393f,    //cos(2)
    0.99862953475457387378449205843944f ,    //cos(3)
    0.99756405025982424761316268064426f ,    //cos(4)
    0.99619469809174553229501040247389f ,    //cos(5)
    0.99452189536827333692269194498057f ,    //cos(6)
    0.99254615164132203498006158933058f ,    //cos(7)
    0.99026806874157031508377486734485f ,    //cos(8)
    0.98768834059513772619004024769344f      //cos(9)
};
 
float qfsind(float x)
{
    int sig = 0;
 
    if(x > 0.0f){
        while(x >= 360.0f) {
            x = x - 360.0f;
        }
    }else{
        while(x < 0.0f) {
            x = x + 360.0f;
        }
    }
 
    if(x >= 180.0f){
        sig = 1;
        x = x - 180.0f;
    }
 
    x = (x > 90.0f) ? (180.0f - x) : x;
 
    int a = x * 0.1f;
    float b = x - 10 * a;
    
    float y = sin_table[a] * cos_table[(int)b] + b * hollyst * sin_table[9 - a];
 
    return (sig > 0) ? -y : y;
}
 
float qfcosd(float x)
{
	return qfsind(x+90.0f);
}
 
 
//将值限制在Min~Max之间
float Value_Limit(float Value,float Min,float Max)
{
	if(Value > Min)
	{
		while(Value >= Max) 
		{
			Value = Value - Max;
		}
    }else
	{
        while(Value < Min) 
		{
            Value = Value + Max;
        }
    }
	return Value;
}
float Value_SetMaxMin(float Value,float Min,float Max)
{
	if(Value < Min)
	{Value = Min;}
	else if (Value > Max)
	{Value = Max;
	}
	else if (Min<=Value && Value<=Max)
	{Value=Value;}
	return Value;
}	
 
