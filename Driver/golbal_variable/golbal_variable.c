#include "golbal_variable.h"



u8 main_sta=0; //用作处理主函数各种if，去掉多余的flag（1打印里程计）（2调用计算里程计数据函数）（3串口接收成功）（4串口接收失败）



/***********************************************  状态  *****************************************************************/
u8 L_ForwardReverse = STOP;// 0静止	1向前	2向后(左电机)
u8 R_ForwardReverse = STOP;// 0静止	1向前	2向后(右电机)


/***********************************************  输出  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;
char odometry_data[21]={0};   //发送给串口的里程计数据数组

/***********************************************  输入  *****************************************************************/
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART_RX_STA=0;   //接收状态标记	

float RecRightWheelSpeed,RecLeftWheelSpeed;//串口得到的左右轮速度

/***********************************************  底盘参数  *****************************************************************/

//轴距校正值=原轴距/0.987
float wheel_interval= 234.0426f; //校正后轴距值，单位mm       
float deceleration_ratio=52.0f;  //减速比 1:52
float wheel_diameter=70.0f;     //轮子直径，单位mm
float pi_1_2=1.570796f;			 //π/2
float pi=3.141593f;              //π
float pi_3_2=4.712389f;			 //π*3/2
float pi_2_1=6.283186f;			 //π*2
float dt=0.005f;                 //采样时间间隔5ms
float line_number=16.0f;       //码盘线数
float oriention_interval=0;  //dt时间内方向变化值

float sin_=0;        //角度计算值
float cos_=0;

float delta_distance=0,delta_oriention=0;   //采样时间间隔内运动的距离

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;

float oriention_1=0;

int span;//采集回来的左右轮速度差值
s32 hSpeed_BufferL[SPEED_BUFFER_SIZE]={0}, hSpeed_BufferR[SPEED_BUFFER_SIZE]={0};//左右轮速度缓存数组
unsigned int hRot_Speed_L;//左电机平均转速缓存
unsigned int hRot_Speed_R;//右电机平均转速缓存
unsigned int SpeedL=0; //左电机平均转速 r/min，PID调节
unsigned int SpeedR=0; //右电机平均转速 r/min，PID调节
unsigned short int hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//电机编码数采集时间间隔
float  Milemeter_L_Motor=0,Milemeter_R_Motor=0;//dt时间内的左右轮速度,用于里程计计算
u8 bSpeed_Buffer_Index = 0;//缓存左右轮编码数到数组变量
float pulse_L = 0;//左电机 PID调节后的PWM值缓存
float pulse_R = 0;//右电机 PID调节后的PWM值缓存

s32 LeftWhellCount = 0;//电机编码计数
s32 RightWhellCount = 0;//电机编码计数

u8 serial_rec=0x31;   //接收串口数据变量

struct PID Control_left  ={0.01,0.1,0.75,0,0,0,0,0,0};//左轮PID参数，适于新电机4096
struct PID Control_right ={0.01,0.1,0.75,0,0,0,0,0,0};//右轮PID参数，适于新电机4096


