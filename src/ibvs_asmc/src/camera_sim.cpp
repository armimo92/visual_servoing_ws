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
float qx,qy,qz,qpsi;
float a;
float aD = 9.05e-8;

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
	ros::Rate loop_rate(30);
	
	ros::Subscriber yaw_sub = nh.subscribe("uav_attitude",100,&uavAttCallback);
	ros::Subscriber uav_pos_sub = nh.subscribe("uav_position",100,&uavPosCallback);
	//ros::Subscriber tgt_pos_sub = nh.subscribe("uav_attitude",100,&uavAttCallback);
	
	geometry_msgs::Vector3 vector_q;
	std_msgs::Float64 Feat_qpsi;
	
	ros::Publisher q_pub = nh.advertise<geometry_msgs::Vector3>("q_linear",100);
	ros::Publisher qpsi_pub = nh.advertise<std_msgs::Float64>("q_psi",100);
	
	tgt_position << 0,0,0;
	
	while(ros::ok())
	{
		p1x = -0.07;
		p1y = 0.07;
	
		p2x = 0.07;
		p2y = 0.07;
	
		p3x = 0.07;
		p3y = -0.07;
	
		p4x = -0.07;
		p4y = -0.07; 
			
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
		mu11 = (u1-ug)*(n1-ng) + (u2-ug)*(n2-ng) + (u3-ug)*(n3-ng) + (u4-ug)*(n4-ng);
		
		resta = mu20-mu02;
		if(resta == 0)
		{
			resta = 1e-8;
		}
	
		a = mu20+mu02;
		qz = sqrt((aD/a)); 
		qy = qz*(ng/focal_length);
		qx = qz*(ug/focal_length); 
		
		qpsi = 0.5*atan(2*mu11/resta);
		
		
		std::cout << "p1: " << u1 << ", " << n1 << std::endl;
		std::cout << "p2: " << u2 << ", " << n2 << std::endl;
		std::cout << "p3: " << u3 << ", " << n3 << std::endl;
		std::cout << "p4: " << u4 << ", " << n4 << std::endl;
		std::cout << "a: " << a << std::endl;
		std::cout << "mu20: " << mu20 << std::endl;
		std::cout << "mu02: " << mu02 << std::endl;
		std::cout << "mu11 " << mu11 << std::endl;		
		std::cout << "ug: " << ug << std::endl;
		std::cout << "ng: " << ng << std::endl;
		std::cout << "qx: " << qx << std::endl;
		std::cout << "qy " << qy << std::endl;		
		std::cout << "qz: " << qz << std::endl;
		std::cout << "qpsi: " << qpsi << std::endl;
		
		
		vector_q.x = qx;
		vector_q.y = qy;
		vector_q.z = qz;
			
		Feat_qpsi.data = qpsi;		
						
		q_pub.publish(vector_q);
		qpsi_pub.publish(Feat_qpsi);
		
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}
