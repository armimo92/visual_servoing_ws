#include <iostream>
#include <math.h>
#include <vector>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>


Eigen::Vector3f uav_position;
Eigen::Vector3f tgt_position;
float uav_psi_vel;
Eigen::Vector3f	p1_vr;
Eigen::Vector3f	p2_vr;
Eigen::Vector3f	p3_vr;
Eigen::Vector3f	p4_vr;
Eigen::Vector3f	p1;
Eigen::Vector3f	p2;
Eigen::Vector3f	p3;
Eigen::Vector3f	p4;
float u1,u2,u3,u4,n1,n2,n3,n4;
float uav_psi;
float p1x,p1y,p2x,p2y,p3x,p3y,p4x,p4y;
float focal_length = 0.00304;
float ug,ng;
float mu20,mu02,mu11,resta;
float qx,qy,qz; 
float qpsi = 0;
float qpsi_dot = 0;
float a;
float aD = 9.05e-8;
float step = 0.02;
int j = 0;
float t;
float x_tgt;
float y_tgt;
float xi = 0;
float yi = 0;
float new_p1x;
float new_p2x;
float new_p3x;
float new_p4x;

float new_p1y;
float new_p2y;
float new_p3y;
float new_p4y;

float angle;


void uavAttCallback(const geometry_msgs::Vector3::ConstPtr& att)
{
	uav_psi = att->z;
}

void uavPosCallback(const geometry_msgs::Vector3::ConstPtr& pos)
{
	uav_position(0) = pos->x;
	uav_position(1) = pos->y;
	uav_position(2) = pos->z;
}
void uavAttVelCallback(const geometry_msgs::Vector3::ConstPtr& av)
{
	uav_psi_vel = av->z;
}
 

Eigen::Matrix3f Ryaw(float uav_yaw)
{
	Eigen::Matrix3f Rz;
	Rz << cos(uav_yaw),-sin(uav_yaw),0,
				sin(uav_yaw),cos(uav_yaw),0,
				0,0,1;
	return Rz;			
	
}

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "camera_sim");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);
	
	ros::Subscriber yaw_sub = nh.subscribe("uav_attitude",100,&uavAttCallback);
	ros::Subscriber uav_attvel_sub = nh.subscribe("uav_attitude_vel",100,&uavAttVelCallback);
	ros::Subscriber uav_pos_sub = nh.subscribe("uav_position",100,&uavPosCallback);
	//ros::Subscriber tgt_pos_sub = nh.subscribe("uav_attitude",100,&uavAttCallback);
	
	geometry_msgs::Vector3 vector_q;
	std_msgs::Float64 Feat_qpsi;
	
	ros::Publisher q_pub = nh.advertise<geometry_msgs::Vector3>("q_linear",100);
	ros::Publisher qpsi_pub = nh.advertise<std_msgs::Float64>("q_psi",100);
	
	tgt_position << 0,0,0;
	
	while(ros::ok())
	{
		/*
		t = j*step;
		
		x_tgt = -3*cos(0.02*3.141592*t);
		y_tgt = 3*sin(0.02*3.141592*t);
		
		p1x = xi - 0.07;
		p1y = yi + 0.07;
	
		p2x = xi + 0.07; 
		p2y = yi + 0.07;
	
		p3x = xi + 0.07;
		p3y = yi - 0.07;
	
		p4x = xi - 0.07;
		p4y = yi - 0.07; 
		
		angle = atan2(y_tgt,x_tgt);
		
		new_p1x = p1x*cos(angle)-p1y*sin(angle);
		new_p2x = p2x*cos(angle)-p2y*sin(angle);
		new_p3x = p3x*cos(angle)-p3y*sin(angle);
		new_p4x = p4x*cos(angle)-p4y*sin(angle);

		new_p1y = p1x*sin(angle)+p1y*cos(angle);
		new_p2y = p2x*sin(angle)+p2y*cos(angle);
		new_p3y = p3x*sin(angle)+p3y*cos(angle);
		new_p4y = p4x*sin(angle)+p4y*cos(angle);
		*/
		p1x = -0.07;
		p1y = 0.07;
	
		p2x = 0.07; 
		p2y = 0.07;
	
		p3x = 0.07;
		p3y = -0.07;
	
		p4x = -0.07;
		p4y =- 0.07; 
		
			
		p1 << p1x,p1y,0;
		p2 << p2x,p2y,0;
		p3 << p3x,p3y,0;
		p4 << p4x,p4y,0;
		
		p1_vr = Ryaw(uav_psi).transpose()*(p1-uav_position);
		p2_vr = Ryaw(uav_psi).transpose()*(p2-uav_position);
		p3_vr = Ryaw(uav_psi).transpose()*(p3-uav_position);
		p4_vr = Ryaw(uav_psi).transpose()*(p4-uav_position);
			
		u1 = focal_length*(p1_vr(0)/p1_vr(2));
		n1 = focal_length*(p1_vr(1)/p1_vr(2));
	
		u2 = focal_length*(p2_vr(0)/p2_vr(2));
		n2 = focal_length*(p2_vr(1)/p2_vr(2));
	
		u3 = focal_length*(p3_vr(0)/p3_vr(2));
		n3 = focal_length*(p3_vr(1)/p3_vr(2));
	
		u4 = focal_length*(p4_vr(0)/p4_vr(2));
		n4 = focal_length*(p4_vr(1)/p4_vr(2));
	
		ug = (u1+u2+u3+u4)/4;
		ng = (n1+n2+n3+n4)/4;
	
		mu20 = pow((u1-ug),2) + pow((u2-ug),2) + pow((u3-ug),2) + pow((u4-ug),2);
		mu02 = pow((n1-ng),2) + pow((n2-ng),2) + pow((n3-ng),2) + pow((n4-ng),2);
		mu11 = ((u1-ug)*(n1-ng)) + ((u2-ug)*(n2-ng)) + ((u3-ug)*(n3-ng)) + ((u4-ug)*(n4-ng));
	
		resta = mu20-mu02;
		if(resta == 0 || resta < 1e-14)
		{
			resta = 1;
		}
		
		if(mu11 < 1e-14)
		{
			mu11 = 0;
		}
	
		a = mu20+mu02;
		qz = sqrt((aD/a)); 
		qy = qz*(ng/focal_length);
		qx = qz*(ug/focal_length); 
		
		qpsi = 0.5*atan(2*mu11/resta);
		//qpsi_dot = 0 - uav_psi_vel;
		//qpsi = qpsi + step*qpsi_dot;
		
		std::cout << "p1: " << u1 << ", " << n1 << std::endl;
		std::cout << "p2: " << u2 << ", " << n2 << std::endl;
		std::cout << "p3: " << u3 << ", " << n3 << std::endl;
		std::cout << "p4: " << u4 << ", " << n4 << std::endl;
		//std::cout << "a: " << a << std::endl;
		std::cout << "mu20: " << mu20 << std::endl;
		std::cout << "mu02: " << mu02 << std::endl;
		std::cout << "mu11 " << mu11 << std::endl;
		std::cout << "resta " << resta << std::endl;		
		//std::cout << "ug: " << ug << std::endl;
		//std::cout << "ng: " << ng << std::endl;
		//std::cout << "qx: " << qx << std::endl;
		//std::cout << "qy " << qy << std::endl;		
		//std::cout << "qz: " << qz << std::endl;
		//std::cout << "qpsi: " << qpsi << std::endl;
		
		//j = j+1;
		vector_q.x = -qx;
		vector_q.y = -qy;
		vector_q.z = qz;
			
		Feat_qpsi.data = qpsi;		
						
		q_pub.publish(vector_q);
		qpsi_pub.publish(Feat_qpsi);
		
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}
