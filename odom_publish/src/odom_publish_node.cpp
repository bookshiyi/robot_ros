/******************************************************************
订阅/odom_combined和/odom_raw，发布/odom

*******************************************************************/


#include "odom_publish_node.h"
/****************************************************************************/


nav_msgs::Odometry odom;//定义里程计对象

void odom_combined_callback(const geometry_msgs::PoseWithCovarianceStamped & odom_combined)//订阅/odom_combined
{
	odom.pose = odom_combined.pose;
}
void odom_raw_callback(const nav_msgs::Odometry & odom_raw)//订阅/odom_raw
{
	odom.twist = odom_raw.twist;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_publish_node");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
	
	/***********************	/odom_combined	geometry_msgs::PoseWithCovarianceStamped********************************/
    ros::Subscriber odom_combined_sub = n.subscribe("/robot_pose_ekf/odom_combined", 200, odom_combined_callback); //订阅/odom_combined
	/***********************	/odom_raw	nav_msgs::Odometry********************************/
	ros::Subscriber odom_raw_sub = n.subscribe("/base_controller/odom_raw", 200, odom_raw_callback); //订阅/odom_combined
	/***********************	/odom	nav_msgs::Odometry********************************/
    ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 200); //发布/odom主题

    ros::Rate loop_rate(50);//设置数据发送频率
    while(ros::ok())
    {
		odom.header.stamp = ros::Time::now(); 											//载入里程计时间戳		
		odom.header.frame_id = "odom";			//(原始的ODOM计算)												//里程计的父子坐标系
		odom.child_frame_id = "base_link";  //base_footprint 
		odom_pub.publish(odom);															//发布里程计
				
		ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到//程序周期性调用			//
		//ros::spin();//周期执行
		loop_rate.sleep();//周期休眠
    }
    return 0;
}