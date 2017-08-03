/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odm

*******************************************************************/


#include "base_controller.h"
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

/*****************************************************************************/
Motion_Data_Typdef Motion_Data;
Physical_Param_TypDef Physical_Param;	//小车的机械参数

string port("/dev/ttyACM0");    //小车串口号
//string port("/dev/rfcomm0"); 
s32 baud = 38400;    //小车串口波特率
u8 beep_test[6]={ROS_FRAME_HEAD,BEEP_OBJ,WRITE_CMD,0X7F,6,ROS_FRAME_END};	//蜂鸣器测试

/************************************************************/
void Physical_Param_Init()
{
	Physical_Param.Whell_Perimeter = 3.14*0.165;	//轮子周长（原0.15 0.165）
	Physical_Param.Reduction_Ratio = 1;			//减速比
	Physical_Param.Wheel_Lin_to_Motor_Ang = 60*Physical_Param.Reduction_Ratio/Physical_Param.Whell_Perimeter;
	Physical_Param.Motor_Ang_to_Wheel_Lin = Physical_Param.Whell_Perimeter/(60*Physical_Param.Reduction_Ratio);
	Physical_Param.Wheel_Track = 0.36;	//轮距	单位m	（原0.36 0.435）
	Physical_Param.Encoder_Lines = 1600;
	
	ROS_INFO("Whell_Perimeter: [%f]", (float)Physical_Param.Whell_Perimeter);
	ROS_INFO("Reduction_Ratio: [%f]", (float)Physical_Param.Reduction_Ratio);
	ROS_INFO("Wheel_Lin_to_Motor_Ang: [%f]", (float)Physical_Param.Wheel_Lin_to_Motor_Ang);
	ROS_INFO("Motor_Ang_to_Wheel_Lin: [%f]", (float)Physical_Param.Motor_Ang_to_Wheel_Lin);
	ROS_INFO("Wheel_Track: [%f]", (float)Physical_Param.Wheel_Track);
}
/*******************************************************************************
	单位转换	轮速m/s转换成电机角速度rad/s
********************************************************************************/
float Wheel_Lin_to_Motor_Ang_Calculate(float wheel_lin)
{
	return (float)(wheel_lin*Physical_Param.Wheel_Lin_to_Motor_Ang);
}

/*******************************************************************************
	单位转换	电机角速度rad/s转换成轮速m/s
********************************************************************************/
float Motor_Ang_to_Wheel_Lin_Calculate(float motor_ang)
{
	return (float)(motor_ang*Physical_Param.Motor_Ang_to_Wheel_Lin);
}

/*******************************************************************************
	回调函数	
********************************************************************************/
void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
	//s32 Car_Linear_x=0,Car_Angular_z=0;//暂存的线速度和角速度
	u8 buf_lenth = 9;
	u8 send_buf[buf_lenth];
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口

    Motion_Data.Control.Car_Lin_x = cmd_input.linear.x *1000;	//获取/cmd_vel的线速度	单位m/s		扩大100倍
    Motion_Data.Control.Car_Ang_z = cmd_input.angular.z	*1000;	//获取/cmd_vel的角速度	单位rad/s	扩大100倍
	Motion_Data.Control.Motor_Ang_Vel[LEFT]  = (s32)((Wheel_Lin_to_Motor_Ang_Calculate(Motion_Data.Control.Car_Lin_x - 0.5*Motion_Data.Control.Car_Ang_z*Physical_Param.Wheel_Track))/1000);//左电机转动速度 RPM
    Motion_Data.Control.Motor_Ang_Vel[RIGHT] = (s32)((Wheel_Lin_to_Motor_Ang_Calculate(Motion_Data.Control.Car_Lin_x + 0.5*Motion_Data.Control.Car_Ang_z*Physical_Param.Wheel_Track))/1000);//右电机转动速度 RPM

	//ROS_INFO("Motion_Data.Control.Motor_Ang_Vel[LEFT] : [%d]", (s32)Motion_Data.Control.Motor_Ang_Vel[LEFT]);
	//ROS_INFO("Motion_Data.Control.Motor_Ang_Vel[RIGHT]: [%d]", (s32)Motion_Data.Control.Motor_Ang_Vel[RIGHT]);
	
	send_buf[HEAD]   = ROS_FRAME_HEAD;
	send_buf[OBJ]    = MOTION_OBJ;
	send_buf[CMD]    = RETURN_CMD;
	send_buf[PARAM1] = Motion_Data.Control.Motor_Ang_Vel[LEFT] >>8;
	send_buf[PARAM2] = Motion_Data.Control.Motor_Ang_Vel[LEFT] ;
	send_buf[PARAM3] = Motion_Data.Control.Motor_Ang_Vel[RIGHT] >>8;
	send_buf[PARAM4] = Motion_Data.Control.Motor_Ang_Vel[RIGHT] ;
	send_buf[buf_lenth-2]  = buf_lenth;
	send_buf[buf_lenth-1]  = ROS_FRAME_END;
	my_serial.write(send_buf,buf_lenth);
}
bool Check_Frame(u8 *data)
{
	u8 i=0,len=0,check_sum=0;
	if(data[0]==ROS_FRAME_HEAD)
	{
		for(i=0;i<32;i++)
		{
			if((data[i] == ROS_FRAME_END) && (data[i-1] == i+1))
			{
				return true;		//			len=i;
			}
			//break;
		}
		//if(len==0)
			//return false;
		/*for(i=0;i<len-2;i++)
		{
			check_sum += data[i];
		}
		if(data[len-1] == check_sum)
			return true;*/
	}
	return false;
	
}
int main(int argc, char **argv)
{
	AHRS_Data_TypDef AHRS_Data;
	string rec_buffer;  //串口数据接收变量
    serial::Serial Driver_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口

    ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
	
	/***********************	/cmd_vel	********************************/
    ros::Subscriber sub = n.subscribe("/cmd_vel", 200, callback); //订阅/cmd_vel主题
	/***********************	nav_msgs::Odometry	********************************/
    ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("base_controller/odom_raw", 200); //定义要发布/odom主题

    static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    geometry_msgs::TransformStamped odom_raw_trans;//创建一个tf发布需要使用的TransformStamped类型消息
    nav_msgs::Odometry odom_raw;//定义里程计对象
    geometry_msgs::Quaternion odom_quat; //四元数变量
    //定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
    float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                            0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                            0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                            0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                            0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                            0,  0,    0,     0,     0,     0.01};  // large covariance on rot z 
    //载入covariance矩阵
    for(int i = 0; i < 36; i++)
    {
        odom_raw.pose.covariance[i] = covariance[i];;
    } 
	
	/***********************	sensor_msgs::BatteryState	********************************/	
	//indigo：smart_battery_msgs/SmartBatteryStatus		//kinetic：sensor_msgs::BatteryState
	ros::Publisher BatteryState_pub = n.advertise<sensor_msgs::BatteryState>("base_controller/BatteryState", 200); //定义要发布/BatteryState主题
	sensor_msgs::BatteryState BatteryState;										//定义电池计对象
	/***********************	sensor_msgs::imu	********************************/
	ros::Publisher imu_data_raw_pub = n.advertise<sensor_msgs::Imu>("base_controller/imu/data_raw", 200); //定义要发布/imu主题
	geometry_msgs::Quaternion imu_quat;							//imu使用四元数
	sensor_msgs::Imu imu;										//定义imu对象
	float linear_acceleration_covariance[9] = {-1,0,0,0,0,0,0,0,0};
	float angular_velocity_covariance[9] = {999999, 0, 0, 0, 999999, 0, 0, 0, 0.000001}; 
	float orientation_covariance[9] = {999999, 0, 0, 0, 999999, 0, 0, 0, 0.000001};
	for(int i = 0; i < 9; i++)
    {
		imu.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
		imu.angular_velocity_covariance[i] = angular_velocity_covariance[i];
        imu.orientation_covariance[i] = orientation_covariance[i];
    } 
	/***********************	sensor_msgs::Mag	********************************/
	ros::Publisher imu_mag_pub = n.advertise<sensor_msgs::MagneticField>("base_controller/imu/mag", 200); //定义要发布/imu主题
	sensor_msgs::MagneticField mag;										//定义mag对象
	float magnetic_field_covariance[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	for(int i = 0; i < 9; i++)
    {
		mag.magnetic_field_covariance[i] = magnetic_field_covariance[i];
    } 
	/***********************************************************/
	Physical_Param_Init();
	Driver_serial.write(beep_test,6);	//蜂鸣器
	AHRS_Data.cur_time = ros::Time::now();			//设置AHRS_Data的时间戳
	Motion_Data.Status.Odom.cur_time = ros::Time::now();	//设置Motion_Data的时间戳
	Motion_Data.Status.Odom.Location.x = 0;
	Motion_Data.Status.Odom.Location.y = 0;
	Motion_Data.Status.Odom.Yaw = 0;
	u8 once = 1;
	
    ros::Rate loop_rate(300);//设置数据发送频率
    while(ros::ok())
    {
		u8 receive_data[32];
		string eol;
		eol = ROS_FRAME_END;//ROS_FRAME_END_1,
        rec_buffer =Driver_serial.readline(32,eol);    //获取串口发送来的数据
		const char *receive_data_char=rec_buffer.data(); //保存串口发送来的数据/
		memcpy(receive_data,receive_data_char,32);
		if(Check_Frame(receive_data))
		//if(receive_data[HEAD] == ROS_FRAME_HEAD && )		//帧头校验
		{
			switch (receive_data[OBJ])
			{
				case MOTION_OBJ:
				{
					//更新位置和时间
					Motion_Data.Status.Odom.last_time =  Motion_Data.Status.Odom.cur_time;
					Motion_Data.Status.Odom.cur_time = ros::Time::now();
					float dt = (Motion_Data.Status.Odom.cur_time-Motion_Data.Status.Odom.last_time).toSec();
					
					Motion_Data.Status.Pos_Last[LEFT]  = Motion_Data.Status.Pos[LEFT];			
					Motion_Data.Status.Pos_Last[RIGHT] = Motion_Data.Status.Pos[RIGHT];		
					//处理下位机发来的数据
					Motion_Data.Status.Motor_Ang_Vel[LEFT]  = (receive_data[PARAM1]<<8) | receive_data[PARAM2];
					Motion_Data.Status.Motor_Ang_Vel[RIGHT] = (receive_data[PARAM3]<<8) | receive_data[PARAM4];
					Motion_Data.Status.Pos[LEFT]  = (receive_data[PARAM5]<<24) | (receive_data[PARAM6] <<16) | (receive_data[PARAM7]<<8 ) | receive_data[PARAM8];                      //ROS_INFO("Pos[LEFT] : [%d]",Motion_Data.Status.Pos[LEFT] );
					Motion_Data.Status.Pos[RIGHT] = (receive_data[PARAM9]<<24) | (receive_data[PARAM10]<<16) | (receive_data[PARAM11]<<8) | receive_data[PARAM12];                     //ROS_INFO("Pos[RIGHT]: [%d]",Motion_Data.Status.Pos[RIGHT]);
					/*
					if(abs(Motion_Data.Status.Pos[LEFT]-Motion_Data.Status.Pos_Last[LEFT])>1000)	Motion_Data.Status.Pos[LEFT]=Motion_Data.Status.Pos_Last[LEFT];
					if(abs(Motion_Data.Status.Pos[RIGHT]-Motion_Data.Status.Pos_Last[RIGHT])>1000)Motion_Data.Status.Pos[RIGHT]=Motion_Data.Status.Pos_Last[RIGHT];
					*/
					/*********	计算ODOM	*************		轮速法	*************************/
					float delta_x   = (float)((Motion_Data.Status.Car_Lin_x * cos(Motion_Data.Status.Odom.Yaw)) * dt/1000.0);				//ROS_INFO("delta_x: [%f]",delta_x);
					float delta_y   = (float)((Motion_Data.Status.Car_Lin_x * sin(Motion_Data.Status.Odom.Yaw)) * dt/1000.0);				//ROS_INFO("delta_y: [%f]",delta_y);
					float delta_Yaw = (float)((Motion_Data.Status.Car_Ang_z * dt)/1000.0);                                    				//ROS_INFO("delta_Yaw: [%f]",delta_Yaw);											
					
					Motion_Data.Status.Odom.Yaw += delta_Yaw;                       																																																	
					if(Motion_Data.Status.Odom.Yaw > 3.1415926)		Motion_Data.Status.Odom.Yaw -= 6.283186;
					if(Motion_Data.Status.Odom.Yaw < -3.1415926)	Motion_Data.Status.Odom.Yaw += 6.283186;
					Motion_Data.Status.Odom.Location.x += delta_x;																														
					Motion_Data.Status.Odom.Location.y += delta_y;  
					//线速度和角速度的计算	比真实扩大1000倍
					Motion_Data.Status.Car_Lin_x = (s32)((Motor_Ang_to_Wheel_Lin_Calculate(Motion_Data.Status.Motor_Ang_Vel[RIGHT]*1000.0)+Motor_Ang_to_Wheel_Lin_Calculate(Motion_Data.Status.Motor_Ang_Vel[LEFT]*1000.0))/2);
					Motion_Data.Status.Car_Ang_z = (s32)((Motor_Ang_to_Wheel_Lin_Calculate(Motion_Data.Status.Motor_Ang_Vel[RIGHT]*1000.0)-Motor_Ang_to_Wheel_Lin_Calculate(Motion_Data.Status.Motor_Ang_Vel[LEFT]*1000.0))/(Physical_Param.Wheel_Track));

					/*********	计算ODOM	*************		编码器法	*************************///偶尔有非常大的跳跃
					/*float delta_d_l	= (float)((Motion_Data.Status.Pos[LEFT]-Motion_Data.Status.Pos_Last[LEFT])*(Physical_Param.Whell_Perimeter/Physical_Param.Encoder_Lines));			//ROS_INFO("delta_d_l: [%f]",delta_d_l);
					float delta_d_r	= (float)((Motion_Data.Status.Pos[RIGHT]-Motion_Data.Status.Pos_Last[RIGHT])*(Physical_Param.Whell_Perimeter/Physical_Param.Encoder_Lines));	    //ROS_INFO("delta_d_r: [%f]",delta_d_r);
					if(once){	delta_d_l = delta_d_r = 0;	once = 0;}                                                                                                                  //
					float delta_d	= (float)((delta_d_l+delta_d_r)/2);																													//ROS_INFO("delta_d: [%f]",delta_d);
					float delta_Yaw = (float)((delta_d_r-delta_d_l)/Physical_Param.Wheel_Track);   																						//ROS_INFO("delta_Yaw: [%f]",delta_Yaw);
					float delta_x   = (float)(delta_d * cos(delta_Yaw));//Motion_Data.Status.Odom.Yaw));																								//ROS_INFO("delta_x: [%f]",delta_x);
					float delta_y   = (float)(delta_d * sin(delta_Yaw));//Motion_Data.Status.Odom.Yaw));																								//ROS_INFO("delta_y: [%f]",delta_y);
					
					Motion_Data.Status.Odom.Yaw += delta_Yaw;                       																																																	
					if(Motion_Data.Status.Odom.Yaw > 3.1415926)		Motion_Data.Status.Odom.Yaw -= 6.283186;
					if(Motion_Data.Status.Odom.Yaw < -3.1415926)	Motion_Data.Status.Odom.Yaw += 6.283186;
					Motion_Data.Status.Odom.Location.x += delta_x;																														
					Motion_Data.Status.Odom.Location.y += delta_y;  
					//线速度和角速度的计算	比真实扩大1000倍
					Motion_Data.Status.Car_Lin_x = (s32)(delta_d*1000/dt);
					Motion_Data.Status.Car_Ang_z = (s32)(delta_Yaw*1000/dt);
					*/
					/*************************		里程计 话题 部分		************************/	
					odom_quat = tf::createQuaternionMsgFromYaw(Motion_Data.Status.Odom.Yaw);				//将偏航角转换成四元数//里程计的偏航角需要转换成四元数才能发布	单位弧度
					odom_raw.header.stamp = ros::Time::now(); 											//载入里程计时间戳		
					odom_raw.header.frame_id = "odom_raw";			//(原始的ODOM计算)												//里程计的父子坐标系
					odom_raw.child_frame_id = "base_link";  //base_footprint 

					odom_raw.pose.pose.position.x = Motion_Data.Status.Odom.Location.x;    									//里程计位置数据：x,y,z,方向
					odom_raw.pose.pose.position.y = Motion_Data.Status.Odom.Location.y;
					odom_raw.pose.pose.position.z = 0.0;

					odom_raw.pose.pose.orientation = odom_quat;   
					odom_raw.twist.twist.linear.x = (float)(Motion_Data.Status.Car_Lin_x/1000.0);			//载入线速度
					//odom_raw.twist.twist.linear.y = odom_vy;
					odom_raw.twist.twist.angular.z = (float)(Motion_Data.Status.Car_Ang_z/1000.0);   		//载入角速度
					odom_pub.publish(odom_raw);															//发布里程计
							
					break;
				}
				case IMU_OBJ:
				{//IMU对象
					//更新时间和角度
					AHRS_Data.last_time =  AHRS_Data.cur_time;
					AHRS_Data.cur_time = ros::Time::now();
					AHRS_Data.IMU.Ang_z_Last = AHRS_Data.IMU.Ang.z;
					//处理下位机发来的数据
					AHRS_Data.IMU.Acc.x	= ((receive_data[PARAM1]<<8 ) | receive_data[PARAM2] );
					AHRS_Data.IMU.Acc.y = ((receive_data[PARAM3]<<8 ) | receive_data[PARAM4] );
					AHRS_Data.IMU.Acc.z = ((receive_data[PARAM5]<<8 ) | receive_data[PARAM6] );
					AHRS_Data.IMU.Gyro.x= ((receive_data[PARAM7]<<8 ) | receive_data[PARAM8] );
					AHRS_Data.IMU.Gyro.y= ((receive_data[PARAM9]<<8 ) | receive_data[PARAM10]);
					AHRS_Data.IMU.Gyro.z= ((receive_data[PARAM11]<<8) | receive_data[PARAM12]);
					
					/*AHRS_Data.IMU.Mag.x = ((receive_data[PARAM13]<<8) | receive_data[PARAM14]);
					AHRS_Data.IMU.Mag.y = ((receive_data[PARAM15]<<8) | receive_data[PARAM16]);
					AHRS_Data.IMU.Mag.z = ((receive_data[PARAM17]<<8) | receive_data[PARAM18]);
					
					mag.header.stamp = ros::Time::now(); 											//载入时间戳		
					mag.header.frame_id = "imu_link";													//坐标系
					mag.magnetic_field.x = (float)(AHRS_Data.IMU.Mag.x/32768.0*180.0);
					mag.magnetic_field.y = (float)(AHRS_Data.IMU.Mag.y/32768.0*180.0);
					mag.magnetic_field.z = (float)(AHRS_Data.IMU.Mag.z/32768.0*180.0);
					imu_mag_pub.publish(mag);*/
					break;
				}
				case RESOLVE_OBJ:
				{//RESOLVE对象
					AHRS_Data.last_time =  AHRS_Data.cur_time;
					AHRS_Data.cur_time = ros::Time::now();
					//处理下位机发来的数据
					AHRS_Data.IMU.Ang.x	 = ((receive_data[PARAM1]<<8 ) | receive_data[PARAM2] );
					AHRS_Data.IMU.Ang.y  = ((receive_data[PARAM3]<<8 ) | receive_data[PARAM4] );
					AHRS_Data.IMU.Ang.z  = ((receive_data[PARAM5]<<8 ) | receive_data[PARAM6] );
					AHRS_Data.IMU.Quat.w = ((receive_data[PARAM7]<<8 ) | receive_data[PARAM8] );
					AHRS_Data.IMU.Quat.x = ((receive_data[PARAM9]<<8 ) | receive_data[PARAM10]);
					AHRS_Data.IMU.Quat.y = ((receive_data[PARAM11]<<8) | receive_data[PARAM12]);
					AHRS_Data.IMU.Quat.z = ((receive_data[PARAM13]<<8) | receive_data[PARAM14]);

					/************************		IMU 话题 部分		************************/	
					//imu_quat = tf::createQuaternionMsgFromRollPitchYaw(AHRS_Data.IMU.Ang.x/32768.0*3.1415926,AHRS_Data.IMU.Ang.y/32768.0*3.1415926,AHRS_Data.IMU.Ang.z/32768.0*3.1415926);////////////////////////
					imu_quat = tf::createQuaternionMsgFromYaw(AHRS_Data.IMU.Ang.z/32768.0*3.1415926);
					imu.header.stamp = AHRS_Data.cur_time;//ros::Time::now(); 											//载入时间戳		
					imu.header.frame_id = "imu_link";													//父坐标系
					imu.linear_acceleration.x = (float)(AHRS_Data.IMU.Acc.x	/32768.0*16.0);
					imu.linear_acceleration.y = (float)(AHRS_Data.IMU.Acc.y	/32768.0*16.0);
					imu.linear_acceleration.z = (float)(AHRS_Data.IMU.Acc.z	/32768.0*16.0);
					imu.angular_velocity.x    = (float)(AHRS_Data.IMU.Gyro.x/32768.0*2000.0);
					imu.angular_velocity.y    = (float)(AHRS_Data.IMU.Gyro.y/32768.0*2000.0);
					imu.angular_velocity.z    = (float)(AHRS_Data.IMU.Gyro.z/32768.0*2000.0);
					imu.orientation = imu_quat;											//欧拉角解算四元数
					/*imu.orientation.w = (float)(AHRS_Data.IMU.Quat.w/32768.0);				//下位机透传四元数部分
					imu.orientation.x = (float)(AHRS_Data.IMU.Quat.x/32768.0);
					imu.orientation.y = (float)(AHRS_Data.IMU.Quat.y/32768.0);
					imu.orientation.z = (float)(AHRS_Data.IMU.Quat.z/32768.0);*/
					imu_data_raw_pub.publish(imu);
					break;		
				}
				case DISTANCE_OBJ:
				{//距离对象
					break;
				}
				case DEBUG_OBJ:
				{//调试对象
					ROS_INFO("DEBUG: [%d]",receive_data[PARAM1]);
					break;
				}
				case BAT_OBJ:
				{
					/************************		电池部分		************************/
					BatteryState.header.stamp = ros::Time::now(); 											//载入时间戳		
					BatteryState.header.frame_id = "base_link";													//父坐标系
					BatteryState.voltage = (float)((receive_data[PARAM1]<<8 | receive_data[PARAM2])/1000);
					BatteryState.percentage = (float)BatteryState.voltage/29.5;
					//ROS_INFO("BatteryState.voltage: [%f]",BatteryState.voltage);
					//ROS_INFO("BatteryState.percentage: [%f]",BatteryState.percentage);
					BatteryState_pub.publish(BatteryState);												//发布电池状态
					break;
				}
				default:break;
			}
			//ros::spinOnce();//周期执行
		}
		
			
		/************************		TF部分		************************/
/*
		odom_raw_trans.header.stamp = ros::Time::now();										//载入坐标（tf）变换时间戳
		odom_raw_trans.header.frame_id = "odom_raw";									   	 	//发布坐标变换的父子坐标系
		odom_raw_trans.child_frame_id = "base_link";    //base_footprint    

		odom_raw_trans.transform.translation.x = Motion_Data.Status.Odom.Location.x;								//tf位置数据：x,y,z,方向
		odom_raw_trans.transform.translation.y = Motion_Data.Status.Odom.Location.y;
		odom_raw_trans.transform.translation.z = 0.0;
		odom_raw_trans.transform.rotation = odom_quat;        
		odom_broadcaster.sendTransform(odom_raw_trans);										//发布tf坐标变化
		*/

		

		ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到//程序周期性调用			//
		//ros::spin();//周期执行
		loop_rate.sleep();//周期休眠
    }
    return 0;
}