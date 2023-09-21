//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

float step = 0.02;

Eigen::Vector4f desired_attitude(0,0,0,0);  //[roll_des, pitch_des, yaw_des, yawRate]
Eigen::Vector3f desired_attitude_est(0,0,0);
Eigen::Vector3f desired_attitude_vel_est(0,0,0);

Eigen::Vector2f x1_hat_dot(0,0);
Eigen::Vector2f x2_hat_dot(0,0);
Eigen::Vector2f estimation_error(0,0);
Eigen::Vector2f G1(0,0);
Eigen::Vector2f G2(0,0);
Eigen::Vector2f k1(0,0);
Eigen::Vector2f k2(0,0);

float alpha_1, alpha_2, beta_1, beta_2;

float sign(float var)
{
    float result;
    if(var>0)
    {
        result = 1;
    }
    else if(var<0)
    {
        result = -1;
    }
    else if (var == 0)
    {
        result = 0;
    }
    return result;
}

float sig(float a, float b) //sig = |a|^b * sign(a)
{
    float s;
    s = powf(std::abs(a), b) * sign(a);
    return s;
}

void desAttCallback(const geometry_msgs::Quaternion::ConstPtr& des_att)
{
	desired_attitude(0) = des_att->x;   //roll_des
	desired_attitude(1) = des_att->y;   //pitch_des
	desired_attitude(2) = des_att->z;   //yaw_des
    desired_attitude(3) = des_att->w;   //yaw rate
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "td_attitude_desired");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	

    //ROS publishers and subscribers
    ros::Subscriber des_att_sub = nh.subscribe("desired_attitude", 100, &desAttCallback);  


    ros::Publisher attitude_des_est_pub = nh.advertise<geometry_msgs::Twist>("attitude_desired_estimates",100);
    ros::Publisher attitude_des_est_err_pub = nh.advertise<geometry_msgs::Vector3>("attitude_des_td_error",100);


    geometry_msgs::Twist attitude_des_est_var;
    geometry_msgs::Vector3 attitude_des_est_err_var;
 
    desired_attitude_est << 0,0,0;
    desired_attitude_vel_est << 0,0,0;
    estimation_error << 0,0;
    x1_hat_dot << 0,0;
    x2_hat_dot << 0,0;
 
    G1 << 4, 4;
    G2 << 5, 5;
  
      
    alpha_1 = 0.7;
    alpha_2 = 2*alpha_1 - 1;
    

    beta_1 = 1.2;
    beta_2 = 2*beta_1 - 1;
    
 
  
    attitude_des_est_var.linear.x = 0;
    attitude_des_est_var.linear.y = 0;
    attitude_des_est_var.linear.z = 0;

    attitude_des_est_var.angular.x = 0;
    attitude_des_est_var.angular.y = 0;
    attitude_des_est_var.angular.z = 0;

    attitude_des_est_err_var.x = 0;
    attitude_des_est_err_var.y = 0;
    attitude_des_est_err_var.z = 0;
   
    attitude_des_est_pub.publish(attitude_des_est_var);
    attitude_des_est_err_pub.publish(attitude_des_est_err_var);
    
    attitude_des_est_pub.publish(attitude_des_est_var);
    attitude_des_est_err_pub.publish(attitude_des_est_err_var);

    ros::Duration(4.5).sleep();

    while(ros::ok())
    {
        /////// Tracking Differentiator ///////

        //x2_hat_dot = Q1*sig(error,alpha2) + D1*sig(error,beta2)
        //x1_hat_dot = x2_hat + Q2*sig(error,alpha1) + D2*sig(error,beta1)
        //Q1,Q2,D1,D2 > 0 such that the polynomial s^2 + 2s + 2 is Hurwitz
        

        for(int i = 0; i<=1; i++)
        {
            estimation_error(i) = desired_attitude(i) - desired_attitude_est(i);
    
            x2_hat_dot(i) = G2(i)*sig(estimation_error(i), alpha_2) + G2(i)*sig(estimation_error(i), beta_2);
            desired_attitude_vel_est(i) = desired_attitude_vel_est(i) + x2_hat_dot(i)*step;

            x1_hat_dot(i) = desired_attitude_vel_est(i) + G1(i)*sig(estimation_error(i), alpha_1) + G1(i)*sig(estimation_error(i), beta_1);
            desired_attitude_est(i) = desired_attitude_est(i) + x1_hat_dot(i)*step;   
             
        }

        //Estimations for desired attitude
        attitude_des_est_var.linear.x = desired_attitude_est(0);
        attitude_des_est_var.linear.y = desired_attitude_est(1);
        attitude_des_est_var.linear.z = desired_attitude(2);

        //Estimations for desired attitude velocities
        attitude_des_est_var.angular.x = desired_attitude_vel_est(0);
        attitude_des_est_var.angular.y = desired_attitude_vel_est(1);
        attitude_des_est_var.angular.z = desired_attitude(3);

        attitude_des_est_err_var.x = estimation_error(0);
        attitude_des_est_err_var.y = estimation_error(1);
        attitude_des_est_err_var.z = 0;

        attitude_des_est_pub.publish(attitude_des_est_var);
        attitude_des_est_err_pub.publish(attitude_des_est_err_var);
    
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
