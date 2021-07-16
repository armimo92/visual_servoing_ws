#include <iostream>
#include <math.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "ros/ros.h"

#include <eigen3/Eigen/Dense>

#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>

Eigen::Vector3f attitude_des;
Eigen::Vector3f attitude_vel_des;
Eigen::Vector3f attitude;
Eigen::Vector3f attitude_vel;

Eigen::Vector3f error;
Eigen::Vector3f error_dot;
Eigen::Vector3f sigma;

//Adaptive sliding mode control parameters
Eigen::Vector3f lambda; //lambda_phi lambda_theta lambda_psi
Eigen::Vector3f K1; //K1_phi, K1_theta, K1_psi
Eigen::Vector3f K1_dot;//K1_phi_dot K1_theta_dot K1_psi_dot
Eigen::Vector3f K2; //K2_phi, K2_theta, K2_psi
Eigen::Vector3f k;	//kphi ktheta kpsi
Eigen::Vector3f kmin; //kmin_phi kmin_theta, kmin_psi
Eigen::Vector3f mu; //mu_phi, mu_theta, mu_psi

float step = 0.01;
float bandera;
//Outputs
Eigen::Vector3f tau; //tau_phi, //tau_theta //tau_psi


void attDesCallback(const geometry_msgs::Vector3::ConstPtr& attD)
{
	attitude_des(0) = attD->x;
	attitude_des(1) = attD->y;
	attitude_des(2) = attD->z;
}

void psiVelDesCallback(const std_msgs::Float64::ConstPtr& msg)
{

	attitude_vel_des(2) = msg->data;
}

void attCallback(const geometry_msgs::Vector3::ConstPtr& att)
{
	attitude(0) = att->x;
	attitude(1) = att->y;
	attitude(2) = att->z;
}

void attVelCallback(const geometry_msgs::Vector3::ConstPtr& attVel)
{
	attitude_vel(0) = attVel->x;
	attitude_vel(1) = attVel->y;
	attitude_vel(2) = attVel->z;
}

void FlagCallback(const std_msgs::Float64::ConstPtr& fl)
{
	bandera = fl->data;
}

float sign(float value)
{
	int result;
	
	if(value > 0)
	{
		result = 1;
	}
	
	else if(value < 0)
	{
		result = -1;
	}
	
	else if(value == 0)
	{
		result = 0;
	}
	
	return result;
}



int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "attitude_controller");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber attDes_sub = nh.subscribe("Desired_attitude_ibvs",100, &attDesCallback);
	ros::Subscriber psi_vel_des_sub = nh.subscribe("psi_vel_des_ibvs",100, &psiVelDesCallback);
	ros::Subscriber attitude_sub = nh.subscribe("uav_attitude",100, &attCallback);
	ros::Subscriber attitude_velocity_sub = nh.subscribe("uav_attitude_vel",100, &attVelCallback);
	ros::Subscriber flag_sub = nh.subscribe("Flag_topic",100, &FlagCallback);
	
	geometry_msgs::Vector3 torques_ibvs;
	ros::Publisher pub_torques = nh.advertise<geometry_msgs::Vector3>("Torques",100);
	
	lambda << 1,1,3; //lambda_phi lambda_theta lambda_psi
	K1 << 0,0,0; //K1_phi, K1_theta, K1_psi
	K1_dot << 0,0,0; //K1_phi_dot K1_theta_dot K1_psi_dot
	K2 << 1,1,1; //K2_phi, K2_theta, K2_psi
	k << 0.5,0.5,0.1;	//kphi ktheta kpsi
	kmin << 0.01,0.01,0.01; //kmin_phi kmin_theta, kmin_psi
	mu << 0.1,0.1,0.01; //mu_phi, mu_theta, mu_psi

	attitude_vel_des(0) = 0;
	attitude_vel_des(1) = 0;
	
	ros::Duration(6.1).sleep();
	while(ros::ok())
	{
	
			for(int i = 0; i <= 2; i++)
			{
				error(i) = attitude(i) - attitude_des(i);
				error_dot(i) = attitude_vel(i) - attitude_vel_des(i);
				sigma(i) = lambda(i) * error(i) + error_dot(i);
			
				if(K1(i) > kmin(i))	
				{
					K1_dot(i) = k(i) * sign(abs(sigma(i))-mu(i));
				}
				else
				{
					K1_dot(i) = kmin(i);
				}
			
				K1(i) = K1(i) + step * K1_dot(i); //New value of K1
			}
		
			tau(0) = -K1(0) * sqrt(abs(sigma(0))) * sign(sigma(0)) - K2(0) * sigma(0);
			tau(1) = -K1(1) * sqrt(abs(sigma(1))) * sign(sigma(1)) - K2(1) * sigma(1);
			tau(2) = -K1(2) * sqrt(abs(sigma(2))) * sign(sigma(2)) - K2(2) * sigma(2);
		
			torques_ibvs.x = tau(0);
			torques_ibvs.y = tau(1);
			torques_ibvs.z = tau(2);
		
			
		
		
			pub_torques.publish(torques_ibvs);
		
			ros::spinOnce();
			loop_rate.sleep();
		
	}
	
	
	return 0;
}
