#include <iostream>
#include "ros/ros.h"
#include <std_msgs/Float64.h>

float flag;
int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "Flag");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);
	
	std_msgs::Float64 bandera;
	
	ros::Publisher flag_pub = nh.advertise<std_msgs::Float64>("Flag_topic",100);
	
	while(ros::ok())
	{
		flag = 0;
		bandera.data = flag;
		flag_pub.publish(bandera);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

