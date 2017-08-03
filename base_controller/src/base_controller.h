#ifndef __BASE_CONTROLLER_H
#define __BASE_CONTROLLER_H

#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>
//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include<cstring>

#define LEFT 0
#define RIGHT 1


/*****************************		数据帧位置定义	********************/
#define HEAD					0x00
#define OBJ						0X01
#define CMD						0X02
#define PARAM1					0X03
#define PARAM2					0X04
#define PARAM3					0X05
#define PARAM4					0X06
#define PARAM5					0X07
#define PARAM6					0X08
#define PARAM7					0X09
#define PARAM8					0X0A
#define PARAM9					0X0B
#define PARAM10					0X0C
#define PARAM11					0X0D
#define PARAM12					0X0E
#define PARAM13					0X0F
#define PARAM14					0X10
#define PARAM15					0X11
#define PARAM16					0X12
#define PARAM17					0X13
#define PARAM18					0X14
#define PARAM19					0X15
#define PARAM20					0X16
#define PARAM21					0X17
#define PARAM22					0X18
#define PARAM23					0X19
#define PARAM24					0X1A
/*****************************		数据帧内容定义	********************/
//帧头
#define ROS_FRAME_HEAD			0XA5
//对象
#define BEEP_OBJ				0X01	//蜂鸣器对象
#define MOTION_OBJ				0x02	//运动对象
#define IMU_OBJ                 0x03    //IMU对象
#define RESOLVE_OBJ				0X04	//姿态解算对象
#define DISTANCE_OBJ 			0X05	//测距对象
#define BAT_OBJ		 			0X06	//电池对象
#define IR_OBJ					0X07	//红外对象

#define DEBUG_OBJ	            0X09
#define SELF_CHECK_OBJ          0X0A
#define POS_OBJ                 0X10    //位置数据
//指令
#define WRITE_CMD				0X80		//写命令不需要返回
#define READ_CMD				0X81		//读取和返回是一对
#define RETURN_CMD				0X82
//返回参数		
#define OK_PARAM				0X01
#define ERROR_PARAM				0X02
//帧尾
#define ROS_FRAME_END			0XAA

/**********************		数据类型宏定义	**********************/
#define s16			signed 	short
#define u16			unsigned short
#define s32			signed int
#define u32			unsigned int
#define u8			unsigned char
typedef  struct
{
	s16 x;
	s16 y;
	s16 z;
} Vector3_Data_TypDef;
typedef  struct
{
	s16 w;    //q0
	s16 x;    //q1
	s16 y;    //q2
	s16 z;    //q3
}Quat_Data_TypDef;
typedef  struct
{
	float x;
	float y;
	float z;
} Vector3_Float_Data_TypDef;
typedef  struct
{
	ros::Time cur_time, last_time;///////////////////////
	Vector3_Float_Data_TypDef Location;
	float Yaw;
} Odom_TypDef;
/**********************		AHRS_Data		**********************/
typedef struct 
{
  Vector3_Data_TypDef Acc;
  Vector3_Data_TypDef Gyro;
  Vector3_Data_TypDef Mag;
  Vector3_Data_TypDef Ang;
  Quat_Data_TypDef Quat;
  s16 Ang_z_Last;
  s16 Car_Ang_z;
}IMU_TypDef;
typedef struct 
{
  u32 Lon;
  u32 Lat;
  u16 GpsHight;
  u16 GpsYaw;
  s32 GpsVel;
}GPS_TypDef;
typedef struct 
{
  IMU_TypDef IMU;
  GPS_TypDef GPS;
  ros::Time cur_time, last_time;///////////////////////
}AHRS_Data_TypDef;
/**********************		Motion_Data		**********************/
typedef struct
{
	s32 Motor_Ang_Vel[2];		//控制的,但是最终发送的形式是s16
	s32 Car_Lin_x;
	s32 Car_Ang_z;
}Control_Data_TypDef;			//运动控制数据
typedef struct
{
	s32 Pos_Last[2];
	s32 Pos[2];
	s16 Motor_Ang_Vel[2];			//反馈的
	s32 Car_Lin_x;
	s32 Car_Ang_z;
	Odom_TypDef Odom;
}Status_Data_TypDef;			//运动状态数据
typedef struct
{
	Control_Data_TypDef		Control;
	Status_Data_TypDef		Status;
}Motion_Data_Typdef;
/**********************		机械参数		**********************/
typedef struct
{
	float Whell_Perimeter;					//轮子周长单位mm
	float Reduction_Ratio;					//减速比(电机减速比*传动减速比)
	float Wheel_Lin_to_Motor_Ang;			//轮子线速度-->电机角速度 转换系数
	float Motor_Ang_to_Wheel_Lin;			//电机角速度-->轮子线速度 转换系数
	float Encoder_Lines;
	float Wheel_Track;						//轮距单位mm
}Physical_Param_TypDef;

float Wheel_Lin_to_Motor_Ang_Calculate(float wheel_lin);
float Motor_Ang_to_Wheel_Lin_Calculate(float motor_ang);
void Physical_Param_Init();

#endif
