//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

using namespace std;

//Declaring global variables
/////////////////Estimation variables///////////////
Eigen::Vector4f imgFeat_est(0,0,0,0);
Eigen::Vector4f imgFeat_dot_est(0,0,0,0);
Eigen::Vector4f ibvs_dist(0,0,0,0);
Eigen::Vector3f ibvs_dist_linear(0,0,0);
Eigen::Vector3f ibvs_dist_linear2(0,0,0);
/////////////////Error signal///////////////
Eigen::Vector4f error(0,0,0,0);
Eigen::Vector4f error_dot(0,0,0,0);

/////////////////Quadrotor measurements////////////
Eigen::Vector3f quad_att(0,0,0);
Eigen::Vector3f quad_vel_BF(0,0,0);
Eigen::Vector3f quad_vel_VF(0,0,0);
Eigen::Vector3f quad_attVel(0,0,0);
Eigen::Vector3f yawVel_e3(0,0,0);
float quad_mass = 2;
float gravity = 9.81;
float thrust = quad_mass * gravity;
Eigen::Vector3f attitudeEstimates(0,0,0);
Eigen::Vector3f attitudeVelEstimates(0,0,0);

/////////////////For comparison///////////////
Eigen::Vector3f imgFeatLinear(0,0,0);
Eigen::Vector3f imgFeatLinear_dot(0,0,0);
Eigen::Vector3f quad_vel_VF_real(0,0,0);
Eigen::Vector4f err_dot(0,0,0,0);
Eigen::Matrix4f Omega;
Eigen::Vector4f imgFeat(0,0,0,0);
Eigen::Vector3f tgt_vel(0,0,0);
Eigen::Vector3f tgt_accel(0,0,0);
Eigen::Vector3f tgt_vel_VF(0,0,0);
Eigen::Vector4f v_imgFeat(0,0,0,0);
Eigen::Vector4f kappa(0,0,0,0);
Eigen::Vector4f kappa_dot(0,0,0,0);

////////////////////Sliding surface and ASMC///////////////////
Eigen::Vector4f ss(0,0,0,0);
Eigen::Vector4f xi_1(0,0,0,0);
Eigen::Vector4f lambda(0,0,0,0);
Eigen::Vector4f xi_2(0,0,0,0);
Eigen::Vector4f varpi(0,0,0,0);
Eigen::Vector4f vartheta(0,0,0,0);
Eigen::Vector4f asmc(0,0,0,0);
Eigen::Vector4f k_dot(0,0,0,0);
Eigen::Vector4f k(0,0,0,0);
Eigen::Vector4f alpha(0,0,0,0);
Eigen::Vector4f beta(0,0,0,0);
///////////////////////////Control input///////////////////////////
Eigen::Vector4f ibvs_ctrl_input(0,0,0,0);
////////////////////////////Quad's VF Dynamics/////////////////////
Eigen::Vector3f quad_accel_VF(0,0,0);
Eigen::Vector3f quad_linear_forces_VF(0,0,0);
Eigen::Vector3f e3(0,0,1);

//////////////////////////Desired attitude for the quadrotor//////////
Eigen::Vector3f attitude_desired(0,0,0);
float yawRate_desired = 0;
float roll_des_arg = 0;
float pitch_des_arg = 0;

/////////////////////////Other variables//////////////////////
Eigen::Vector4f imgFeat_des(0,0,0,0);
float a = 0;
float zD = 2.5;
float tgt_YR = 0;
float tgt_YAccel = 0;
float step_size = 0.02;

/////////////////////////Functions///////////////////////////////
Eigen::Matrix3f skewMatrix(Eigen::Vector3f vector)
{
	Eigen::Matrix3f Skew;
	Skew << 0, -vector(2), vector(1),
			vector(2), 0, -vector(0),
			-vector(1), vector(0), 0;			
	return Skew;
}

Eigen::Matrix3f Ryaw(float yaw)
{   
    Eigen::Matrix3f yaw_mat;
    yaw_mat << cos(yaw), -sin(yaw),0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1;
    return yaw_mat;
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

void ibvsDistCallback(const geometry_msgs::Quaternion::ConstPtr& dist)
{
	ibvs_dist(0) = dist->x;
	ibvs_dist(1) = dist->y;
	ibvs_dist(2) = dist->z;
    ibvs_dist(3) = dist->w;
}

void imFeatCallback(const geometry_msgs::Quaternion::ConstPtr& img_features)
{
	imgFeat(0) = img_features->x;
	imgFeat(1) = img_features->y;
	imgFeat(2) = img_features->z;
    imgFeat(3) = img_features->w;
}

void ImFeatEstCallback(const geometry_msgs::Quaternion::ConstPtr& ifEst)
{
	imgFeat_est(0) = ifEst->x;
	imgFeat_est(1) = ifEst->y;
	imgFeat_est(2) = ifEst->z;
    imgFeat_est(3) = ifEst->w;
}

void ImFeatEstDotCallback(const geometry_msgs::Quaternion::ConstPtr& ifEstDot)
{
	imgFeat_dot_est(0) = ifEstDot->x;
	imgFeat_dot_est(1) = ifEstDot->y;
	imgFeat_dot_est(2) = ifEstDot->z;
    imgFeat_dot_est(3) = ifEstDot->w;
}

void aValueCallback(const std_msgs::Float64::ConstPtr& aVal)
{
	a = aVal->data;
}

void tgtYRCallback(const std_msgs::Float64::ConstPtr& tgtYR)
{
	tgt_YR = tgtYR->data;
}

void tgtVelCallback(const geometry_msgs::Vector3::ConstPtr& tgtVel)
{
	tgt_vel(0) = tgtVel->x;
    tgt_vel(1) = tgtVel->y;
    tgt_vel(2) = tgtVel->z;
}

void quadVelBFCallback(const geometry_msgs::Vector3::ConstPtr& quadVel)
{
	quad_vel_BF(0) = quadVel->x;
    quad_vel_BF(1) = quadVel->y;
    quad_vel_BF(2) = quadVel->z;
}

void quadAttVelCallback(const geometry_msgs::Vector3::ConstPtr& quadAttVel)
{
	quad_attVel(0) = quadAttVel->x;
    quad_attVel(1) = quadAttVel->y;
    quad_attVel(2) = quadAttVel->z;
}

void quadAttCallback(const geometry_msgs::Vector3::ConstPtr& quadAtt)
{
	quad_att(0) = quadAtt->x;
    quad_att(1) = quadAtt->y;
    quad_att(2) = quadAtt->z;
}

void tgtAccelCallback(const geometry_msgs::Vector3::ConstPtr& tgtAccel)
{
	tgt_accel(0) = tgtAccel->x;
    tgt_accel(1) = tgtAccel->y;
    tgt_accel(2) = tgtAccel->z;
}

void quadAttEstCallback(const geometry_msgs::Twist::ConstPtr& aE)
{
    attitudeEstimates(0) = aE->linear.x;
    attitudeEstimates(1) = aE->linear.y;
    attitudeEstimates(2) = aE->linear.z;
    
    attitudeVelEstimates(0) = aE->angular.x;
    attitudeVelEstimates(1) = aE->angular.y;
    attitudeVelEstimates(2) = aE->angular.z;
}

/////////////////////////////////Main Program//////////////////////////
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "aibvs_pos_ctrl");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	
    
////////////////////////ROS publishers///////////////////////////////////////////////////////
	ros::Publisher error_pub = nh.advertise<geometry_msgs::Quaternion>("error_visual_servoing",100);
    ros::Publisher error_dot_pub = nh.advertise<geometry_msgs::Quaternion>("error_dot_visual_servoing",100);

    ros::Publisher adaptive_gain_pub = nh.advertise<geometry_msgs::Quaternion>("adaptive_gain",100);
    ros::Publisher asmc_pub = nh.advertise<geometry_msgs::Quaternion>("asmc_output",100);
    ros::Publisher ss_pub = nh.advertise<geometry_msgs::Quaternion>("ibvs_ss",100);

    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float64>("quad_thrust",100);
    ros::Publisher z_des_pub = nh.advertise<std_msgs::Float64>("z_des",100);

    ros::Publisher desired_att_pub = nh.advertise<geometry_msgs::Quaternion>("desired_attitude",100);
    ros::Publisher psiddot_des_pub = nh.advertise<std_msgs::Float64>("yaw_ddot_desired",100);
    
    ros::Publisher real_err_dot_pub = nh.advertise<geometry_msgs::Quaternion>("real_err_dot",100);
    ros::Publisher quad_vel_VF_pub = nh.advertise<geometry_msgs::Vector3>("quad_vel_virtual",100);
    ros::Publisher quad_vel_VF_real_pub = nh.advertise<geometry_msgs::Vector3>("quad_vel_virtual_real",100);

    ros::Publisher ctrl_pub = nh.advertise<geometry_msgs::Quaternion>("ibvs_control_input",100);

////////////////////////ROS subscribers////////////////////////////////////////////////
	ros::Subscriber im_feat_sub = nh.subscribe("ImFeat_vector", 100, &imFeatCallback);
    ros::Subscriber imFeat_est_sub =  nh.subscribe("ImFeat_estimates_fxt", 100, &ImFeatEstCallback);
    ros::Subscriber imFeatDot_est_sub =  nh.subscribe("ImFeat_dot_estimates_fxt", 100, &ImFeatEstDotCallback); 
    ros::Subscriber dist_est_pub =  nh.subscribe("ibvs_dist", 100, &ibvsDistCallback);    

    ros::Subscriber a_value_sub = nh.subscribe("a_value", 100, &aValueCallback);

    ros::Subscriber tgt_vel_sub = nh.subscribe("tgt_velocity", 100, &tgtVelCallback);
    ros::Subscriber tgt_YR_sub = nh.subscribe("tgt_yaw_rate", 100, &tgtYRCallback);
    ros::Subscriber tgt_accel_sub = nh.subscribe("tgt_acceleration", 100, &tgtAccelCallback);
  

    ros::Subscriber quad_vel_BF_sub = nh.subscribe("quad_velocity_BF", 100, &quadVelBFCallback);
    ros::Subscriber attitude_est_sub = nh.subscribe("attitude_estimates",100,&quadAttEstCallback);
    ros::Subscriber quad_attVel_sub = nh.subscribe("quad_attitude_velocity", 100, &quadAttVelCallback);
    ros::Subscriber quad_att_sub = nh.subscribe("quad_attitude", 100, &quadAttCallback);

 ////////////////////////ROS variables///////////////////////////////////////////////////////
    geometry_msgs::Quaternion error_var;
    geometry_msgs::Quaternion error_dot_var;
    geometry_msgs::Quaternion adaptive_gain_var;
    geometry_msgs::Quaternion asmc_var;
    geometry_msgs::Quaternion ss_var;
    geometry_msgs::Quaternion ctrl_input_var;
    std_msgs::Float64 thrust_var;
    std_msgs::Float64 yaw_ddot_des_var;
    std_msgs::Float64 z_des_var;
    geometry_msgs::Quaternion desired_attitude_var;
    geometry_msgs::Quaternion real_err_dot_var;
    geometry_msgs::Vector3 quad_vel_VF_var;
    geometry_msgs::Vector3 quad_vel_VF_real_var;

////////////////////////Controller Gains///////////////////////////////////////////////////////

imgFeat_des << 0,0,1,0;
    
    
    xi_1 << 7, 7, 1, 4;
    xi_2 << 8, 8, 6, 6;
    lambda << 2, 2, 2, 2;
    varpi << 4, 4, 4, 4;
    vartheta << 3, 3, 3, 3;
    k << 0,0,0,0;
    k_dot << 0,0,0,0;
    alpha << 0.000003, 0.000003, 0.001, 0.0001;
    beta << 85, 85, 5, 5;

    /*xi_1 << 6, 6, 1, 3;
    xi_2 << 14, 14, 6, 6;
    lambda << 1.5, 1.5, 2, 2;
    varpi << 4, 4, 4, 4;
    vartheta << 3, 3, 3, 3;
    k << 0,0,0,0;
    k_dot << 0,0,0,0;
    alpha << 0.00000002, 0.00000002, 0.001, 0.0001;
    beta << 8000, 8000, 5, 5;*/

    quad_vel_VF << 0,0,0;
    quad_accel_VF << 0,0,0;
    kappa_dot << 0,0,0,0;
    e3 << 0,0,1;
    attitude_desired << 0,0,0;
   

    error_var.x = 0;
    error_var.y = 0;
    error_var.z = 0;
    error_var.w = 0;

    ctrl_input_var.x = 0;
    ctrl_input_var.y = 0;
    ctrl_input_var.z = 0;
    ctrl_input_var.w = 0;

    error_dot_var.x = 0;
    error_dot_var.y = 0;
    error_dot_var.z = 0;
    error_dot_var.w = 0;

    adaptive_gain_var.x = 0;
    adaptive_gain_var.y = 0;
    adaptive_gain_var.z = 0;
    adaptive_gain_var.w = 0;

    asmc_var.x = 0;
    asmc_var.y = 0;
    asmc_var.z = 0;
    asmc_var.w = 0;

    ss_var.x = 0;
    ss_var.y = 0;
    ss_var.z = 0;
    ss_var.w = 0;

    quad_vel_VF_var.x = 0;
    quad_vel_VF_var.y = 0;
    quad_vel_VF_var.z = 0;

    thrust_var.data = thrust;

    desired_attitude_var.x = 0;
    desired_attitude_var.y = 0;
    desired_attitude_var.z = 0;
    desired_attitude_var.w = 0;
	
    quad_vel_VF_pub.publish(quad_vel_VF_var);
    ss_pub.publish(ss_var);
    asmc_pub.publish(asmc_var);
    adaptive_gain_pub.publish(adaptive_gain_var);
    error_dot_pub.publish(error_dot_var);
    error_pub.publish(error_var);
    thrust_pub.publish(thrust_var);
    desired_att_pub.publish(desired_attitude_var);
    ctrl_pub.publish(ctrl_input_var);

    ros::Duration(1).sleep();
    
    while(ros::ok())
    {   
////////////////////// FOR COMPARISON ONLY//////////////////////////////////////////////////
        imgFeatLinear << imgFeat(0),imgFeat(1),imgFeat(2); 
        tgt_vel_VF = (Ryaw(attitudeEstimates(2)).transpose()) * tgt_vel;
        quad_vel_VF_real = Rtp(attitudeEstimates(0),attitudeEstimates(1)) * quad_vel_BF;
        yawVel_e3 << 0,0,attitudeVelEstimates(2);
        imgFeatLinear_dot = -skewMatrix(yawVel_e3) * imgFeatLinear - (1/zD) * quad_vel_VF_real + (1/zD) * tgt_vel_VF;
        
        Omega << (-1/zD),0,0,imgFeat(1), 
        0,(-1/zD),0,-imgFeat(0),
        0,0,(-1/zD),0,
        0,0,0,-1;

        v_imgFeat << quad_vel_VF_real(0),quad_vel_VF_real(1),quad_vel_VF_real(2),quad_attVel(2);
        kappa << (tgt_vel_VF(0)/zD), (tgt_vel_VF(1)/zD), (tgt_vel_VF(2)/zD), tgt_YR; 

        err_dot = Omega*v_imgFeat + kappa;

        //error = imgFeat - imgFeat_des;
        //error_dot = Omega*v_imgFeat + kappa;
/////////////////////////////////////////////////////////////////////////////////////////////

		error = imgFeat_est - imgFeat_des;
        error_dot = imgFeat_dot_est;
        ibvs_dist_linear <<  ibvs_dist(0), ibvs_dist(1), ibvs_dist(2);
        
        //Sliding surfaces and adaptive sliding mode controller
        for (int i = 0; i<=3; i++)
        {
            ss(i) = error(i) + xi_1(i) * powf(std::abs(error(i)),lambda(i)) * sign(error(i)) + xi_2(i) * powf(std::abs(error_dot(i)),(varpi(i)/vartheta(i))) * sign(error_dot(i));
            
             k_dot(i) = powf(alpha(i),0.5) * powf(std::abs(ss(i)),0.5) - powf(beta(i),0.5) * powf(k(i),2);

            k(i) = k(i) + k_dot(i) * step_size;

            asmc(i) = -2*k(i) * powf(std::abs(ss(i)),0.5) * sign(ss(i)) - (powf(k(i),2)/2) * ss(i);
        }
        

        //Control inputs
        /////////////yaw_rotation///////////////////////
        ibvs_ctrl_input(3) = (-asmc(3) + ibvs_dist(3) + (vartheta(3)/(varpi(3)*xi_2(3))) * sign(error_dot(3)) * powf(std::abs(error_dot(3)),(2-(varpi(3)/vartheta(3)))) * (1 + xi_1(3) * lambda(3) * powf(std::abs(error(3)),lambda(3)-1))); //yaw_ddot
        yawRate_desired = yawRate_desired + step_size * ibvs_ctrl_input(3);
        /////////////x-axis///////////////////////
        ibvs_ctrl_input(0) = zD * (-asmc(0) + ibvs_dist(0) + (vartheta(0)/(varpi(0)*xi_2(0))) * sign(error_dot(0)) * powf(std::abs(error_dot(0)),(2-(varpi(0)/vartheta(0)))) * (1 + xi_1(0) * lambda(0) * powf(std::abs(error(0)),lambda(0)-1)));
        /////////////y-axis///////////////////////
        ibvs_ctrl_input(1) = zD * (-asmc(1) + ibvs_dist(1) + (vartheta(1)/(varpi(1)*xi_2(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi(1)/vartheta(1)))) * (1 + xi_1(1) * lambda(1) * powf(std::abs(error(1)),lambda(1)-1)));
        /////////////z-axis///////////////////////
        ibvs_ctrl_input(2) = zD * (-asmc(2) + ibvs_dist(2) + (vartheta(2)/(varpi(2)*xi_2(2))) * sign(error_dot(2)) * powf(std::abs(error_dot(2)),(2-(varpi(2)/vartheta(2)))) * (1 + xi_1(2) * lambda(2) * powf(std::abs(error(2)),lambda(2)-1)));    
        
        //     + ibvs_ctrl_input(3) * imgFeat_est(1) + yawRate_desired * imgFeat_dot_est(1) 
        //     - ibvs_ctrl_input(3) * imgFeat_est(0) - yawRate_desired * imgFeat_dot_est(0)  
        
        //
        
        //Quad's virtual frame dynamics 
        quad_accel_VF << ibvs_ctrl_input(0), ibvs_ctrl_input(1), ibvs_ctrl_input(2); 

        quad_vel_VF(0)= quad_vel_VF(0) + quad_accel_VF(0)*step_size;
        quad_vel_VF(1)= quad_vel_VF(1) + quad_accel_VF(1)*step_size;
        quad_vel_VF(2)= quad_vel_VF(2) + quad_accel_VF(2)*step_size;
        
        quad_linear_forces_VF = (quad_mass * quad_accel_VF) + (quad_mass * skewMatrix(yawVel_e3)) * quad_vel_VF_real; 
        
        //////////////////Thrust///////////////////////////////
        thrust = e3.transpose() * (Rtp(attitudeEstimates(0),attitudeEstimates(1)).transpose() * ((quad_mass * gravity * e3) - quad_linear_forces_VF));
        if (thrust > 30)
        {
            thrust = 30;
        }
        else if (thrust < 0)
        {
            thrust = 0;
        }
        /////////////////Desired attitude//////////////////
        
        //////////////////Saturating the desired roll and pitch rotations up to pi/2 to avoid singularities
        roll_des_arg = quad_linear_forces_VF(1)/thrust;
        pitch_des_arg = -quad_linear_forces_VF(0)/(thrust*cos(attitude_desired(0)));

        if (roll_des_arg > 1)
        {
            roll_des_arg = 1;
        }
        else if (roll_des_arg < -1)
        {
            roll_des_arg = -1;
        }

        if (pitch_des_arg > 1)
        {
            pitch_des_arg = 1;
        }
        else if (pitch_des_arg < -1)
        {
            pitch_des_arg = -1;
        }

        attitude_desired(0) = asin(roll_des_arg); //Roll desired
        attitude_desired(1) = asin(pitch_des_arg); //Pitch desired
        
        //yawRate_desired = yawRate_desired + step_size * ibvs_ctrl_input(3);
        attitude_desired(2) = attitude_desired(2) + step_size * yawRate_desired; //Yaw desired
        
        //Publishing data
        //Control input
        ctrl_input_var.x = ibvs_ctrl_input(0);
        ctrl_input_var.y = ibvs_ctrl_input(1);
        ctrl_input_var.z = ibvs_ctrl_input(2);
        ctrl_input_var.w = ibvs_ctrl_input(3);
        //error
        error_var.x = error(0);
        error_var.y = error(1);
        error_var.z = error(2);
        error_var.w = error(3);
        //K1
        adaptive_gain_var.x = k(0);
        adaptive_gain_var.y = k(1);
        adaptive_gain_var.z = k(2);
        adaptive_gain_var.w = k(3);
        //asmc
        asmc_var.x = asmc(0);
        asmc_var.y = asmc(1);
        asmc_var.z = asmc(2);
        asmc_var.w = asmc(3);
        //Thrust
        thrust_var.data = thrust;
        //Desired attitude and yaw rate
        desired_attitude_var.x = attitude_desired(0);
        desired_attitude_var.y = attitude_desired(1);
        desired_attitude_var.z = attitude_desired(2);
        desired_attitude_var.w = yawRate_desired;
        
        yaw_ddot_des_var.data = 0;//ibvs_ctrl_input(3);

        ss_var.x = std::abs(ss(0));
        ss_var.y = std::abs(ss(1));
        ss_var.z = std::abs(ss(2));
        ss_var.w = std::abs(ss(3));

        error_dot_var.x = error_dot(0);
        error_dot_var.y = error_dot(1);
        error_dot_var.z = error_dot(2);
        error_dot_var.w = error_dot(3);

        z_des_var.data = -zD;

        quad_vel_VF_var.x = quad_vel_VF(0);
        quad_vel_VF_var.y = quad_vel_VF(1);
        quad_vel_VF_var.z = quad_vel_VF(2);

        quad_vel_VF_real_var.x = quad_vel_VF_real(0);
        quad_vel_VF_real_var.y = quad_vel_VF_real(1);
        quad_vel_VF_real_var.z = quad_vel_VF_real(2);

        error_pub.publish(error_var);
        adaptive_gain_pub.publish(adaptive_gain_var);
        asmc_pub.publish(asmc_var);
        thrust_pub.publish(thrust_var);
        desired_att_pub.publish(desired_attitude_var); 
        psiddot_des_pub.publish(yaw_ddot_des_var);
        error_dot_pub.publish(error_dot_var);
        ss_pub.publish(ss_var);
        z_des_pub.publish(z_des_var);

        real_err_dot_var.x = err_dot(0);
        real_err_dot_var.y = err_dot(1);
        real_err_dot_var.z = err_dot(2);
        real_err_dot_var.w = err_dot(3);

        real_err_dot_pub.publish(real_err_dot_var);
        quad_vel_VF_pub.publish(quad_vel_VF_var);
        quad_vel_VF_real_pub.publish(quad_vel_VF_real_var);

        ctrl_pub.publish(ctrl_input_var);

        std::cout << "error: " << error << std::endl;
        //std::cout << "pitch_des " << attitude_desired(1) << std::endl;

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
        
        
