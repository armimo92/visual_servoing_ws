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


Eigen::Vector4f q;
Eigen::Vector3f qlinear;
Eigen::Vector3f qlinear_dot;
Eigen::Vector3f psi_e3;
Eigen::Vector4f qD;
Eigen::Vector4f error;
Eigen::Vector4f error_dot;
Eigen::Vector3f e3;

Eigen::Vector3f uav_linear_vel_BF;
Eigen::Vector3f uav_linear_vel_VF;
Eigen::Vector4f uav_vel_vs;
float uav_psi_vel;
float uav_roll;
float uav_pitch;
float zD = 2;

Eigen::Vector4f uav_vel_vf;
Eigen::Vector3f tgt_linear_vel;
float tgt_psi_vel = 0;

Eigen::Vector4f kappa;
Eigen::Matrix4f M;
Eigen::Matrix4f M_dot;

Eigen::Vector4f sigma;
Eigen::Vector4f lambda;
Eigen::Matrix4f lambda_matrix;
Eigen::Vector4f K1;
Eigen::Vector4f K1_dot;
Eigen::Vector4f K2;
Eigen::Vector4f k;
Eigen::Vector4f kmin;
Eigen::Vector4f mu;

float step = 0.0333;
Eigen::Vector4f asmc;

Eigen::Vector4f image_filter;
Eigen::Vector4f image_filter_dot;
Eigen::Vector4f rho;
float r2 = 0.2;
float r3 = 0.1;

Eigen::Vector4f delta;
Eigen::Vector4f xi;
Eigen::Vector4f xi_dot;

float mass = 1.8;
float g = 9.81;

Eigen::Vector3f force_vf;
Eigen::Vector3f xi_lin;
Eigen::Vector3f xi_dot_lin;
float U1;
float phi_des;
float theta_des;
float phi_des_arg;
float theta_des_arg;
float psi_des = 0; 

float bandera;

float abs_arg;

void qfeatCallback(const geometry_msgs::Vector3::ConstPtr& qfeat)
{
	q(0) = qfeat->x;
	q(1) = qfeat->y;
	q(2) = qfeat->z;
	
}

void qpsiCallback(const std_msgs::Float64::ConstPtr& qpsiFeat)
{
	q(3) = qpsiFeat->data;
}

void uavVelBfCallback(const geometry_msgs::Vector3::ConstPtr& velBF)
{
	uav_linear_vel_BF(0) = velBF->x;
	uav_linear_vel_BF(1) = velBF->y;
	uav_linear_vel_BF(2) = velBF->z;
	
}

void attVelCallback(const geometry_msgs::Vector3::ConstPtr& attVel)
{
	uav_psi_vel = attVel->z;	
}

void attitude_callback(const geometry_msgs::Vector3::ConstPtr& att)
{
	uav_roll = att->x;
	uav_pitch = att->y;	
}

void FlagCallback(const std_msgs::Float64::ConstPtr& fl)
{
	bandera = fl->data;
}

Eigen::Matrix3f skewMatrix(Eigen::Vector3f vector)
{
	Eigen::Matrix3f Skew;
	
	Skew << 0, -vector(2), vector(1),
					vector(2), 0, -vector(0),
					-vector(1), vector(0), 0;
					
	return Skew;

}

Eigen::Matrix3f Rtp(float roll, float pitch)
{
    Eigen::Matrix3f pitch_mat;
    pitch_mat << cos(pitch), 0.0, sin(pitch),
        0.0, 1.0, 0.0,
        -sin(pitch), 0.0, cos(pitch);


    Eigen::Matrix3f roll_mat;
    roll_mat << 1.0, 0.0, 0.0,
        0.0, cos(roll), -sin(roll),
        0.0, sin(roll), cos(roll);

    Eigen::Matrix3f R_tp;
    R_tp = pitch_mat * roll_mat;

    return R_tp;

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
	ros::init(argc, argv, "asmc_feat");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);
	
	ros::Subscriber q_sub = nh.subscribe("q_linear",100, &qfeatCallback);
	ros::Subscriber qpsi_sub = nh.subscribe("q_psi",100, &qpsiCallback);	
	ros::Subscriber uav_vel_BF_sub = nh.subscribe("uav_vel_BF",100, &uavVelBfCallback);
	ros::Subscriber uav_att_vel_sub = nh.subscribe("uav_attitude_vel",100, &attVelCallback);
	ros::Subscriber attitude_sub = nh.subscribe("uav_attitude",100, &attitude_callback);
	ros::Subscriber flag_sub = nh.subscribe("Flag_topic",100, &FlagCallback);
	
	
	geometry_msgs::Vector3 uav_att_des;
	std_msgs::Float64 psi_vel_des;
	geometry_msgs::Vector3 thrust;
	geometry_msgs::Vector3 K1xyz;
	geometry_msgs::Vector3 xi_xyz;
	geometry_msgs::Vector3 fuerzas_virtuales;
	geometry_msgs::Vector3 sigmas;
	
	ros::Publisher att_des_pub = nh.advertise<geometry_msgs::Vector3>("Desired_attitude_ibvs",100);
	ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Vector3>("Thrust_and_des_att",100);
	ros::Publisher psi_vel_des_pub = nh.advertise<std_msgs::Float64>("psi_vel_des_ibvs",100);
	ros::Publisher K1xyz_pub = nh.advertise<geometry_msgs::Vector3>("Adaptive_gains",100);
	ros::Publisher xi_xyz_pub = nh.advertise<geometry_msgs::Vector3>("Ctrl_law_linear",100);
	ros::Publisher fvir_pub = nh.advertise<geometry_msgs::Vector3>("Fuerzas_virtuales",100);
	ros::Publisher sigmas_pub = nh.advertise<geometry_msgs::Vector3>("sigmas",100);
	
	lambda << 1,1,4,1;
	K1 << 0,0,0,0;
	K1_dot << 0,0,0,0;
	K2 << 1,1,1,0.1;
	k << 0.3,0.3,1,1;
	kmin << 0.01,0.01,0.01,0.01;
	mu << 0.01,0.01,0.1,0.2;
	asmc << 0,0,0,0;
	sigma << 0,0,0,0;
	e3 << 0,0,1;
	
	lambda_matrix << lambda(0),0,0,0,
									 0,lambda(1),0,0,
									 0,0,lambda(2),0,
									 0,0,0,lambda(3);		
	qD << 0,0,1,0;
	uav_vel_vf << 0,0,0,0;
	tgt_linear_vel << 0,0,0;
	
	delta << 0,0,0,0;
	xi << 0,0,0,0;
	xi_dot << 0,0,0,0;
	
	image_filter << 0,0,0,0;
	image_filter_dot << 0,0,0,0;
	rho << 0,0,0,0;
	 
	ros::Duration(6.1).sleep();
	
	while(ros::ok())
	{
	
		//if(bandera == 1)
		//{	
			//ERROR SIGNALS
			error = q-qD;
		
			qlinear << q(0), q(1), q(2);
			uav_linear_vel_VF = Rtp(uav_roll, uav_pitch) * uav_linear_vel_BF;
			uav_vel_vs << uav_linear_vel_VF(0), uav_linear_vel_VF(1), uav_linear_vel_VF(2), uav_psi_vel;
		
			psi_e3 << 0, 0, uav_psi_vel;
		
			qlinear_dot = -1*skewMatrix(psi_e3) * qlinear - (1/zD) * uav_linear_vel_VF + (1/zD) * 	tgt_linear_vel;
		
		
			M << (-1/zD), 0, 0, q(1),
				 0, (-1/zD), 0, -q(0),
				 0, 0, (-1/zD), 0,
				 0, 0, 0, -1;
				  
			M_dot << 0, 0, 0, qlinear_dot(1),
						 0, 0, 0, -qlinear_dot(0),
						 0, 0, 0, 0,
						 0, 0, 0, 0;
				 
		
		
			kappa << (1/zD) * tgt_linear_vel(0), (1/zD) * tgt_linear_vel(1), (1/zD) * tgt_linear_vel(2), tgt_psi_vel;
		
			error_dot = M * xi + kappa;
		
			//ASMC
			sigma(0) = lambda(0) * error(0) + error_dot(0);
			sigma(1) = lambda(1) * error(1) + error_dot(1);
			sigma(2) = lambda(2) * error(2) + error_dot(2);
			sigma(3) = lambda(3) * error(3) + error_dot(3);
		
			if(K1(0) > kmin(0))
			{
				abs_arg = std::abs(sigma(0))-mu(0);
				K1_dot(0) = k(0) * sign(abs_arg);
			}	
			else
			{
				K1_dot(0) = kmin(0);
			}
			
			if(K1(1) > kmin(1))
			{
				abs_arg = std::abs(sigma(1))-mu(1);
				K1_dot(1) = k(1) * sign(abs_arg);
			}	
			else
			{
				K1_dot(1) = kmin(1);
			}
			
			if(K1(2) > kmin(2))
			{
				abs_arg = std::abs(sigma(2))-mu(2);
				K1_dot(2) = k(2) * sign(abs_arg);
			}	
			else
			{
				K1_dot(2) = kmin(2);
			}
			
			if(K1(3) > kmin(3))
			{
				abs_arg = std::abs(sigma(3))-mu(3);
				K1_dot(3) = k(3) * sign(abs_arg);
			}	
			else
			{
				K1_dot(3) = kmin(3);
			}
			
			
			K1(0) = K1(0) + step * K1_dot(0); //New value of K1
			K1(1) = K1(1) + step * K1_dot(1); //New value of K1
			K1(2) = K1(2) + step * K1_dot(2); //New value of K1
			K1(3) = K1(3) + step * K1_dot(3); //New value of K1
			
		
			asmc(0) = -K1(0) * sqrt(std::abs(sigma(0))) * sign(sigma(0)) - K2(0) * sigma(0);
			asmc(1) = -K1(1) * sqrt(std::abs(sigma(1))) * sign(sigma(1)) - K2(1) * sigma(1);
			asmc(2) = -K1(2) * sqrt(std::abs(sigma(2))) * sign(sigma(2)) - K2(2) * sigma(2);
			asmc(3) = -K1(3) * sqrt(std::abs(sigma(3))) * sign(sigma(3)) - K2(3) * sigma(3);
		
			//rho << atan(image_filter(0)), atan(image_filter(1)), atan(image_filter(2)), atan(image_filter(3));
			//image_filter_dot = -r2 * delta - r3 * rho;
		
			delta(0) = asmc(0);
			delta(1) = asmc(1);
			delta(2) = asmc(2);
			delta(3) = asmc(3);
		
			xi_dot = M.inverse() * (delta - lambda_matrix*M*xi - lambda_matrix*kappa - M_dot * xi); //- image_filter);
		
			xi(0) = xi(0) + step * xi_dot(0);
			xi(1) = xi(1) + step * xi_dot(1);
			xi(2) = xi(2) + step * xi_dot(2);
			xi(3) = xi(3) + step * xi_dot(3);
		
			xi_lin << xi(0), xi(1), xi(2);
			xi_dot_lin << xi_dot(0), xi_dot(1), xi_dot(2);
			
			force_vf = mass * xi_dot_lin + mass*skewMatrix(psi_e3)*xi_lin;
		
			U1 = e3.transpose()*Rtp(uav_roll, uav_pitch).transpose() * (mass*g*e3 - force_vf);
		
			if(U1 < 0)
			{
				U1 = 0;
			}
			else if(U1 > 30)
			{
				U1 = 30;	
			}
				
			phi_des_arg = force_vf(1)/U1;
		
			if(phi_des_arg > 1)
			{
				phi_des_arg = 1;
			}
			else if(phi_des_arg < -1)
			{
				phi_des_arg = -1;
			}
		
			phi_des = asin(phi_des_arg);
			if(phi_des > 0.5)
			{
				phi_des = 0.5;
			}
			
			if(phi_des < -0.5)
			{
				phi_des = -0.5;
			}
		
			theta_des_arg = -force_vf(0)/(U1*cos(phi_des));
		
			if(theta_des_arg > 1)
			{
				theta_des_arg = 1;
			}
			else if(theta_des_arg < -1)
			{
				theta_des_arg = -1;
			}
		
			theta_des = asin(theta_des_arg);
			if(theta_des > 0.5)
			{
				theta_des = 0.5;
			}
			
			if(theta_des < -0.5)
			{
				theta_des = -0.5;
			}
			
			psi_des = psi_des + step * xi(3);
		
		
			//DATA TO PUBLISH
			uav_att_des.x = phi_des;
			uav_att_des.y = theta_des;
			uav_att_des.z = psi_des;
		
			psi_vel_des.data = xi(3);
			thrust.x = U1;
			
			K1xyz.x = K1(3);
			K1xyz.y = K1(1);
			K1xyz.z = K1(2);
			
			xi_xyz.x = xi(0);
			xi_xyz.y = xi(1);
			xi_xyz.z = xi(2);
			
			fuerzas_virtuales.x = force_vf(0);
			fuerzas_virtuales.y = force_vf(1);
			fuerzas_virtuales.z = force_vf(2);	
			
			sigmas.x = sigma(0);
			sigmas.y = sigma(1);
			sigmas.z = sigma(2);
			
			att_des_pub.publish(uav_att_des);
			thrust_pub.publish(thrust);
			psi_vel_des_pub.publish(psi_vel_des);
			K1xyz_pub.publish(K1xyz); 
			xi_xyz_pub.publish(xi_xyz);
			fvir_pub.publish(fuerzas_virtuales);
			sigmas_pub.publish(sigmas);
		
		
			std::cout << std::abs(sigma(2)) << std::endl;
			//std::cout << "x: " << K1(0) << std::endl;
			//std::cout << "y: " << K1(1) << std::endl;
			//std::cout << "z: " << K1(2) << std::endl;
			//std::cout << "psi: " << K1(3) << std::endl;
			//std::cout << "phi_arg: " << phi_des_arg << std::endl;
			//std::cout << "theta_arg: " << theta_des_arg << std::endl;
			//std::cout << "phi: " << phi_des << std::endl;
			//std::cout << "theta: " << theta_des << std::endl;			
			//std::cout << "phi_des: " << phi_des << std::endl;
			//std::cout << "theta_des: " << theta_des << std::endl;		
			//std::cout << "psi_des: " << psi_des << std::endl;		
		
			
			//std::cout << "  Sigma: " << std::endl;
			//std::cout << "x: " << sigma(0) << std::endl;
			//std::cout << "y: " << sigma(1) << std::endl;
			//std::cout << "z: " << sigma(2) << std::endl;
			
			ros::spinOnce();
			loop_rate.sleep();
		//}
	}
	
	return 0;
}
