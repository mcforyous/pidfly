/*
 * pid.c
 *
 *  Created on: Sep 18, 2024
 *      Author: JOHN
 */

#include "pidFly.h"
//平衡的时候 飞机后退减小，飞机前进加大
float Pitch_Zero = 0.0f;//Pitch角度的机械零点
//平衡的时候  飞机往右减小，飞机往左加大
float Roll_Zero = 0.0f;//Roll角度的机械零点  //加大50
float Yaw_Zero = 0.0f;//Yaw角度的机械零点  顺时针旋转减小


//飞机实际的欧拉角
extern float Pitch,Roll,Yaw;

//陀螺仪原始数据
extern short gx,gy,gz;

//遥控器获取的数值
extern float roll_move,pitch_move,yaw_move;
extern int oil;
//外环PID
float Pitch_Angle_Kp =65.0f;//Pitch角度环(外环)的比例系数P
float Pitch_Angle_Ki =0.002f;//Pitch角度环(外环)的比例系数I
float Pitch_Angle_Kd =-0.0f;//Pitch角度环(外环)的比例系数D

float Roll_Angle_Kp =45.0f;//Roll角度环(外环)的比例系数P
float Roll_Angle_Ki =0.002f;//Roll角度环(外环)的比例系数I
float Roll_Angle_Kd =-0.0f;//Roll角度环(外环)的比例系数D

float Yaw_Angle_Kp =30.0f;//Yaw角度环(外环)的比例系数P
float Yaw_Angle_Ki =0.0f;//Yaw角度环(外环)的比例系数I
float Yaw_Angle_Kd =-0.0f;//Yaw角度环(外环)的比例系数D

float Pitch_Angle_Inte;//Pitch角度环的积分
float Roll_Angle_Inte;//Roll角度环的积分
float Yaw_Angle_Inte;//Yaw角度环的积分

int pitch_angle_out=0;     //Pitch平衡外环的输出
int roll_angle_out=0;      //Roll平衡外环的输出
int yaw_angle_out=0;       //Yaw平衡外环的输出

//内环PID
//震荡消失0.6 取0.4
float Pitch_Gyro_Kp=0.4f ;//Pitch角速度环(内环)的比例系数P
float Pitch_Gyro_Ki=0.005f ;//Pitch角速度环(内环)的比例系数I
float Pitch_Gyro_Kd=0.15f ;//Pitch角速度环(内环)的比例系数D

//震荡消失0.75   取0.525
float Roll_Gyro_Kp=0.45f;//Roll角速度环(内环)的比例系数P
float Roll_Gyro_Ki=0.002f ;//Roll角速度环(内环)的比例系数I
float Roll_Gyro_Kd=0.1f ;//Roll角速度环(内环)的比例系数D

float Yaw_Gyro_Kp=0.5f ;//Yaw角速度环(内环)的比例系数P
float Yaw_Gyro_Ki =0.0f;//Yaw角速度环(内环)的比例系数I
float Yaw_Gyro_Kd =0.0f;//Yaw角速度环(内环)的比例系数D

float Pitch_err=0;               //Pitch内环误差值
float Pitch_last_err=0;          //Pitch内环上次误差值
float Pitch_Gyro_Inte;         //Pitch内环积分

float Roll_err=0;                //Roll内环误差值
float Roll_last_err=0;           //Roll内环上次误差值
float Roll_Gyro_Inte;          //Roll内环积分

float Yaw_err=0;                 //Yaw内环误差值
float Yaw_last_err=0;            //Yaw内环上次误差值
float Yaw_Gyro_Inte;           //Yaw内环积分

int Pitch_balance_out=0;           //Pitch平衡环的最终输出
int Roll_balance_out=0;            //Roll平衡环的最终输出
int Yaw_balance_out=0;             //Yaw平衡环的最终输出
int Yaw_Out=0;                             //Yaw环的最终输出



int Angle_InteMax = 20;//角度积分限幅
int Gyro_InteMax = 20;//角速度积分限幅




//占空比限幅，需要给飞机姿态调整预留PWM，所以油门为100的时候不能达到最高的PWM输出
int PWM_OIL=85;

//补偿电池压降  因为随着电池容量减少 电池电压会下降
float pwm_adc = 1.0f;

#define BAT_MAX 4.0f

int PWM_Out1=0;                                                     //最终作用到电机的PWM
int PWM_Out2=0;                                                      //最终作用到电机的PWM
int PWM_Out3=0;                                                      //最终作用到电机的PWM
int PWM_Out4=0;                                                   //最终作用到电机的PWM

void pid_init() {

}
//无人机PID主任务
void Fly_Pid() {

    if(oil < 30) {

          //直接作用到电机pwm输出
            Moto_Ctl(1,oil*PWM_OIL); //作用到1号电机
            Moto_Ctl(2,oil*PWM_OIL); //作用到2号电机
            Moto_Ctl(3,oil*PWM_OIL); //作用到3号电机
            Moto_Ctl(4,oil*PWM_OIL); //作用到4号电机
            //printf("电机1: %d 电机2: %d 电机3: %d 电机4: %d\n",oil*PWM_OIL,oil*PWM_OIL,oil*PWM_OIL,oil*PWM_OIL);
    }else if(oil >=30) {
        //油门数值大于30再进行PID计算
        Pitch_Zero = -14.0f;
         Roll_Zero = 3.0f;

        Pitch_Angle_Kp =60.0f;Pitch_Gyro_Kp=0.4f ;
        Roll_Angle_Kp = 45.0f;Roll_Gyro_Kp=0.45f;Roll_Gyro_Kd=0.1f;
        Yaw_Angle_Kp =30.0f;
        //油门大于60
        if(oil >=60){
            Pitch_Zero = -16.5f;
             Roll_Zero = 5.0f;

            Pitch_Angle_Kp =100.0f;Pitch_Gyro_Kp=0.55 ;
            Roll_Angle_Kp = 90.0f;Roll_Gyro_Kp = 0.55f;Roll_Gyro_Kd=0.2f;
            Yaw_Angle_Kp =50.0f;
        }
        else if(oil >=80){
            Pitch_Zero = -16.5f;
             Roll_Zero = 7.0f;

            Pitch_Angle_Kp =110.0f;Pitch_Gyro_Kp=0.6f ;
            Roll_Angle_Kp = 150.0f;Roll_Gyro_Kp = 0.65f;Roll_Gyro_Kd=0.2f;
            Yaw_Angle_Kp =75.0f;
        }
        //油门大于80
      //分别先计算出2个方向的角度环外环
      //参数分别是采集到的陀螺仪角度和加速度
        pitch_angle_out = Pitch_Balance_Angle(Pitch, (float)gy); //俯仰平衡环外环的输出
        roll_angle_out = Roll_Balance_Angle(Roll, (float)gx);    //翻滚平衡环外环的输出
        //printf("外环%d\n",roll_angle_out);

      //在计算出2个方向的角速度环内环
        //参数分别是内环的输出 和 采集到的陀螺仪角速度
        Pitch_balance_out = Pitch_Balance_Gyro(pitch_angle_out, (float)gy); //俯仰平衡环内环的输出
        Roll_balance_out = Roll_Balance_Gyro(roll_angle_out, (float)gx);    //翻滚平衡环内环的输出
        //printf("内环%d\n",Roll_balance_out);

      //Yaw方向单独计算,设计成当无人机需要改变Yaw方向的时候 设定成固定的速度

        if(yaw_move == 50.0f) {
            Yaw_Out = 0;
            yaw_angle_out = Yaw_Balance_Angle(Yaw,(float) gz);
            Yaw_balance_out = Yaw_Balance_Gyro(yaw_angle_out, (float)gz);

        }
        if(yaw_move > 55) {
            Yaw_balance_out = 0;
            Yaw_Out = yaw_control(150);
            Yaw_Zero = Yaw;
        }
        if(yaw_move <45 ) {
            Yaw_balance_out = 0;
            Yaw_Out = yaw_control(-150);
            Yaw_Zero = Yaw;
        }

      //组合    三个方向的串级pid输出到四个电机的pwm数值
        PWM_Out1 = oil * PWM_OIL + Pitch_balance_out + Roll_balance_out + Yaw_balance_out + Yaw_Out;
        PWM_Out2 = oil * PWM_OIL + Pitch_balance_out - Roll_balance_out - Yaw_balance_out - Yaw_Out;
        PWM_Out3 = oil * PWM_OIL - Pitch_balance_out + Roll_balance_out - Yaw_balance_out - Yaw_Out;
        PWM_Out4 = oil * PWM_OIL - Pitch_balance_out - Roll_balance_out + Yaw_balance_out + Yaw_Out;


        //printf("压降:%f\n",pwm_adc);
      //补偿电池压降
        PWM_Out1*=pwm_adc;
        PWM_Out2*=pwm_adc;
        PWM_Out3*=pwm_adc;
        PWM_Out4*=pwm_adc;


        //printf("电机1:%d 电机2:%d 电机3:%d 电机4:%d\n",PWM_Out1,PWM_Out2,PWM_Out3,PWM_Out4);
        //printf("%f\n",PWM_Out1);
      //作用到电机pwm输出
        Moto_Ctl(1,PWM_Out1); //作用到1号电机
        Moto_Ctl(2,PWM_Out2); //作用到2号电机
        Moto_Ctl(3,PWM_Out3); //作用到3号电机
        Moto_Ctl(4,PWM_Out4); //作用到4号电机

    }

}


/*
void getIntFly(float rollNum,float pitchNum,float yawNum,float oilNum) {
    //获取遥控器的数值
    Oil = oilNum;
    Roll_n = rollNum;
    Pitch_n = pitchNum;
    Yaw_n =  yawNum;

}
*/
/*
void getIntMpu(float *Gx,float* Gy,float* Gz,float PitchFly,float RollFly,float YawFly) {

    GyroX = *Gx;
    GyroY = *Gy;
    GyroZ = *Gz;

    pitch = PitchFly;
    roll =  RollFly;
    yaw =  YawFly;
    //printf("%f,%d,%d\n",pitch,PWM_Out1+PWM_Out2,PWM_Out3+PWM_Out4);
    //printf("$%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f;\r\n",0.0,0.0,0.0,GyroX,GyroY,GyroZ,yaw,roll,pitch,0);
}
*/
/*
//获取电池电压 来计算 电机压降
void getPwmAdc(float BatNum){
    if(BatNum == 0) {
        pwm_adc = 1.0f;
    }else {
        pwm_adc = BAT_MAX / BatNum;
    }

}
*/

//航向设置 机头旋转后将最新的机头角度赋值给航向的机械零点
//返回值为yaw的最终输出pwm值
int yaw_control(float Set_turn)
{
  int pwm_yaw=0;
  float yaw_Kp = 1.0f;//设置航向控制
  float YAW_SPEED = 1.0f;//设置飞机转向最大速率比 1表示100%速度
    pwm_yaw=yaw_Kp*YAW_SPEED*Set_turn; //有转向需求，Kp为期望飞机转向
    return pwm_yaw;
}

//Pitch角度环(外环)
//参数分别是 陀螺仪采集到的角度值 和 角速度值
int Pitch_Balance_Angle(float Angle, float Gyro) {
    float err;//误差(期望值和实测值差距)
    int PID_Angle_Out;//该环输出角度值
    float angle_num = 0.0f;//这里是获取到的遥控器的角度值(多少度)
    angle_num = pitch_move;

    err = (angle_num + Pitch_Zero)- Angle; //(Pitch角度机械零点+期望值) - （采集到的实际角度值） 这里如果期望飞机平衡则用机械零点
    if(oil < 30) {
        Pitch_Angle_Inte = 0;//积分清零
    }else if(oil >=30) {
        Pitch_Angle_Inte +=err;//积分累加分离
    }

    //积分限幅
    if(Pitch_Angle_Inte >= Angle_InteMax) {
        Pitch_Angle_Inte = Angle_InteMax;
    }else if(Pitch_Angle_Inte <= - Angle_InteMax) {
        Pitch_Angle_Inte = -Angle_InteMax;
    }
    //PID公式
    PID_Angle_Out = Pitch_Angle_Kp * err + Pitch_Angle_Ki * Pitch_Angle_Inte + Pitch_Angle_Kd * Gyro;
    return PID_Angle_Out;

}
//Pitch角速度环(内环)
//参数分别是外环的输出值 和 采集的角速度值
int Pitch_Balance_Gyro(float PID_Angle_Out,float Gyro) {
    int pwm_Out;//最终的电机输出
    Pitch_err = PID_Angle_Out - Gyro;

    if(oil < 30) {
        Pitch_Gyro_Inte = 0;//积分清零
    }else if(oil >=30) {
        Pitch_Gyro_Inte +=Pitch_err;//积分分离
    }

    //积分限幅
    if(Pitch_Angle_Inte >= Gyro_InteMax) {
        Pitch_Gyro_Inte = Gyro_InteMax;
    }else if(Pitch_Gyro_Inte <= - Gyro_InteMax) {
        Pitch_Gyro_Inte = -Gyro_InteMax;
    }
    pwm_Out =  Pitch_Gyro_Kp * Pitch_err + Pitch_Gyro_Ki * Pitch_Gyro_Inte + Pitch_Gyro_Kd * (Pitch_err - Pitch_last_err);
    Pitch_last_err = Pitch_err; // 保存本次的误差值 ， 作为下次计算时 的上次误差值使用
    return pwm_Out;

}
//R0ll角度环（外环）
int Roll_Balance_Angle(float Angle, float Gyro) {
    float err;//误差(期望值和实测值差距)
    int PID_Angle_Out;//该环输出角度值
    float angle_num = 0.0f;//这里是获取到的遥控器的数据
    angle_num = roll_move;

    err = (angle_num + Roll_Zero)- Angle; //(Pitch角度机械零点+期望值) - （采集到的实际角度值） 这里如果期望飞机平衡则用机械零点
    if(oil < 30) {
        Roll_Angle_Inte = 0;//积分清零
    }else if(oil >=30) {
        Roll_Angle_Inte +=err;//积分分离
    }

    //积分限幅
    if(Roll_Angle_Inte >= Angle_InteMax) {
        Roll_Angle_Inte = Angle_InteMax;
    }else if(Roll_Angle_Inte <= - Angle_InteMax) {
        Roll_Angle_Inte = -Angle_InteMax;
    }
    //PID公式
    PID_Angle_Out = Roll_Angle_Kp * err + Roll_Angle_Ki * Roll_Angle_Inte + Roll_Angle_Kd * Gyro;
    return PID_Angle_Out;
}
//Roll角速度环（内环）
int Roll_Balance_Gyro(float PID_Angle_Out,float Gyro) {

    int pwm_Out;//最终的电机输出
    Roll_err = PID_Angle_Out - Gyro;

    if(oil < 30) {
        Roll_Gyro_Inte = 0;//积分清零
    }else if(oil >=30) {
        Roll_Gyro_Inte +=Roll_err;//积分分离
    }

    //积分限幅
    if(Roll_Angle_Inte >= Gyro_InteMax) {
        Roll_Gyro_Inte = Gyro_InteMax;
    }else if(Roll_Gyro_Inte <= - Gyro_InteMax) {
        Roll_Gyro_Inte = -Gyro_InteMax;
    }
    pwm_Out =  Roll_Gyro_Kp * Roll_err + Roll_Gyro_Ki * Roll_Gyro_Inte + Roll_Gyro_Kd * (Roll_err - Roll_last_err);
    Roll_last_err = Roll_err; // 保存本次的误差值 ， 作为下次计算时 的上次误差值使用
    return pwm_Out;
}
//Yaw角度环（外环）
int Yaw_Balance_Angle(float Angle, float Gyro) {
    float err;//误差(期望值和实测值差距)
    int PID_Angle_Out;//该环输出角度值


    err = Yaw_Zero- Angle; //(Pitch角度机械零点+期望值) - （采集到的实际角度值） 这里如果期望飞机平衡则用机械零点
    if(oil < 30) {
        Yaw_Angle_Inte = 0;//积分清零
    }else if(oil >=30) {
        Yaw_Angle_Inte +=err;//积分分离
    }

    //积分限幅
    if(Yaw_Angle_Inte >= Angle_InteMax) {
        Yaw_Angle_Inte = Angle_InteMax;
    }else if(Yaw_Angle_Inte <= - Angle_InteMax) {
        Yaw_Angle_Inte = -Angle_InteMax;
    }
    //PID公式
    PID_Angle_Out = Yaw_Angle_Kp * err + Yaw_Angle_Ki * Yaw_Angle_Inte + Yaw_Angle_Kd * Gyro;
    return PID_Angle_Out;
}
//Yaw角速度环（内环）
int Yaw_Balance_Gyro(float PID_Angle_Out,float Gyro) {
    int pwm_Out;//最终的电机输出
    Yaw_err = PID_Angle_Out - Gyro;

    if(oil < 30) {
        Yaw_Gyro_Inte = 0;//积分清零
    }else if(oil >=30) {
        Yaw_Gyro_Inte +=Yaw_err;//积分分离
    }

    //积分限幅
    if(Yaw_Angle_Inte >= Gyro_InteMax) {
        Yaw_Gyro_Inte = Gyro_InteMax;
    }else if(Yaw_Gyro_Inte <= - Gyro_InteMax) {
        Yaw_Gyro_Inte = -Gyro_InteMax;
    }
    pwm_Out = Yaw_Gyro_Kp * Yaw_err + Yaw_Gyro_Ki * Yaw_Gyro_Inte + Yaw_Gyro_Kd * (Yaw_err - Yaw_last_err);
    Yaw_last_err = Yaw_err; // 保存本次的误差值 ， 作为下次计算时 的上次误差值使用
    return pwm_Out;
}
