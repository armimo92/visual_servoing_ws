#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>


float pos_x;
float pos_y;
float xp;
float yp;
float pos_z = 0;
float yaw;
float t;
float step = 0.01;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tgt_pos");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	geometry_msgs::Vector3 tgt_position;
	std_msgs::Float64 tgt_yaw;
	
	ros::Publisher tgt_pos_pub = nh.advertise<geometry_msgs::Vector3>("tgt_position",100);
	ros::Publisher tgt_yaw_pub = nh.advertise<std_msgs::Float64>("tgt_yaw",100);
	
	int i = 0;
	while(ros::ok())
	{
		if(i <= 10000)
		{
			t = i*step;
			pos_x = -3*cos(0.02*3.141592*t) +3;
			pos_y = 3*sin(0.02*3.141592*t) + 0;
			
			xp = 3*0.02*3.141592*sin(0.02*3.141592*t);
			yp = 3*0.02*3.141592*cos(0.02*3.141592*t);
			
			yaw = atan2(yp,xp);
			
			tgt_position.x = pos_x;
			tgt_position.y = pos_y;
			tgt_position.z = pos_z;
			tgt_yaw.data = yaw;
		}
			i = i+1;
			
			
			
	
		tgt_pos_pub.publish(tgt_position);
		tgt_yaw_pub.publish(tgt_yaw);
		
		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	
	
}
