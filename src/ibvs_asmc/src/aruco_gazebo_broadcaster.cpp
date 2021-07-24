#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include "gazebo_msgs/ModelState.h"
#include <tf2/LinearMath/Quaternion.h>

float pos_x;
float pos_y;
float pos_z;
float yaw;

void pos_callback(const geometry_msgs::Vector3::ConstPtr& p)
{
	pos_x = p->x;
	pos_y = p->y;
	pos_z = p->z;
}
void att_callback(const std_msgs::Float64::ConstPtr& msg)
{
	yaw = msg->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "aruco_gazebo_broadcaster");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber pos = nh.subscribe("tgt_position",100, &pos_callback);
	ros::Subscriber att = nh.subscribe("tgt_yaw",100, &att_callback);
	
	gazebo_msgs::ModelState states;
	ros::Publisher gazebo_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",100);

	states.model_name = "Aruco_marker";
	tf2::Quaternion myQuaternion;
	tf2::Quaternion q;
	tf2::Quaternion q_rot;
	
	while(ros::ok())
	{
		
		myQuaternion.setRPY(0,0,-yaw);
		q_rot.setRPY(3.141592,0,0);
		//q = q_rot * myQuaternion;
		q = myQuaternion;
		q.normalize();
		
		states.pose.position.x = pos_x;
		states.pose.position.y = -pos_y;
		states.pose.position.z = -pos_z;
		
		states.pose.orientation.x = q.x();
		states.pose.orientation.y = q.y();
		states.pose.orientation.z = q.z();
		states.pose.orientation.w = q.w();
		
		gazebo_pub.publish(states);
		
		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	
	
}
