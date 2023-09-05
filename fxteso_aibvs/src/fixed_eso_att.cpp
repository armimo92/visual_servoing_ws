//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector3f attitude(0,0,0);
Eigen::Vector3f attVelocity(0,0,0);
Eigen::Vector3f torques(0,0,0);

Eigen::Vector3f G1;
Eigen::Vector3f G2;
Eigen::Vector3f G3;
Eigen::Vector3f G4;

Eigen::Vector3f lambda;  // 0 < lambda < 1
Eigen::Vector3f varphi;  // varphi > 1 


Eigen::Vector3f att_est(0,0,0);
Eigen::Vector3f attvel_est(0,0,0);
Eigen::Vector3f att_dist_est(0,0,0);

Eigen::Vector3f x1_dot;
Eigen::Vector3f x2_dot;
Eigen::Vector3f x3_dot;

float fx = 0;
float gx_u = 0;
float step = 0.01;
float quad_mass = 2;
float jx = 0.0411;
float jy = 0.0478;
float jz = 0.0599;

Eigen::Vector3f estimation_error_angular(0,0,0);


//MATH FUNCTIONS
float sign(float var)
{   
    float x;
    if (var > 0)
    {
        x = 1;
    }
    else if (var<0)
    {
        x = -1;
    }
    else if (var == 0)
    {
        x = 0;
    }
    return x;
}

//CALLBACKS
void attCallback(const geometry_msgs::Vector3::ConstPtr& a)
{
    attitude(0) = a->x;
    attitude(1) = a->y;
    attitude(2) = a->z;
}

void avelCallback(const geometry_msgs::Vector3::ConstPtr& av)
{
    attVelocity(0) = av->x;
    attVelocity(1) = av->y;
    attVelocity(2) = av->z;
}

void torquesCallback(const geometry_msgs::Vector3::ConstPtr& to)
{
    torques(0) = to->x;
    torques(1) = to->y;
    torques(2) = to->z;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "fixed_eso_att");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    //Subscribers and publishers
    ros::Subscriber quav_att_sub = nh.subscribe("quad_attitude", 100, &attCallback);
    ros::Subscriber quav_attVel_sub = nh.subscribe("quad_attitude_velocity", 100, &avelCallback);
    ros::Subscriber torques_sub = nh.subscribe("torques", 100, &torquesCallback);

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::Publisher attitudeEstimates_pub = nh.advertise<geometry_msgs::Vector3>("attitude_estimates",100); 
    ros::Publisher attVelEstimates_pub = nh.advertise<geometry_msgs::Vector3>("attVel_estimates",100); 
    ros::Publisher disturbanceAngEstimates_pub = nh.advertise<geometry_msgs::Vector3>("disturbance_angular_estimates",100);
    ros::Publisher estimationError_angular_pub = nh.advertise<geometry_msgs::Vector3>("estimation_error_angular",100);
  
    geometry_msgs::Vector3 attitudeEstimates_var;
    geometry_msgs::Vector3 attVelEstimates_var;
    geometry_msgs::Vector3 disturbanceAngEstimates_var;
    geometry_msgs::Vector3 estimationError_angular_var;

    
    //FXTESO GAINS
    G1 << 16, 16, 16;
    G2 << 150, 150, 150;
    G3 << 450, 450, 450;
    G4 << 0.01, 0.01, 0.01;

    lambda << 0.6, 0.6, 0.6;
    varphi << 1.2, 1.2, 1.2;


    //Initial conditions
    att_est << 0,0,0;
    attvel_est << 0,0,0;
    att_dist_est << 0,0,0;

    x1_dot << 0,0,0;
    x2_dot << 0,0,0;
    x3_dot << 0,0,0;

    estimation_error_angular(0) = attitude(0) - att_est(0);
    estimation_error_angular(1) = attitude(1) - att_est(1);
    estimation_error_angular(2) = attitude(2) - att_est(2);

    attitudeEstimates_var.x = att_est(0);
    attitudeEstimates_var.y = att_est(1);
    attitudeEstimates_var.z = att_est(2);

    attVelEstimates_var.x = attvel_est(0);
    attVelEstimates_var.y = attvel_est(1);
    attVelEstimates_var.z = attvel_est(2);
    
    estimationError_angular_var.x = estimation_error_angular(0);
    estimationError_angular_var.y = estimation_error_angular(1);
    estimationError_angular_var.z = estimation_error_angular(2);

    disturbanceAngEstimates_var.x = 0;
    disturbanceAngEstimates_var.y = 0;
    disturbanceAngEstimates_var.z = 0;

    attitudeEstimates_pub.publish(attitudeEstimates_var);
    attVelEstimates_pub.publish(attVelEstimates_var);
    estimationError_angular_pub.publish(estimationError_angular_var);
    disturbanceAngEstimates_pub.publish(disturbanceAngEstimates_var);

    ros::Duration(4.7).sleep();

    while(ros::ok())
    { 
        ///////////////////////////////ATTITUDE SUBSYSTEM////////////////////////////////////////////////////
        for(int i = 0; i<=2; i++)
        {
            //Estimation error
            estimation_error_angular(i) = attitude(i) - att_est(i);
            
            x3_dot(3+i) = G3(i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),lambda(i))  +  powf(std::abs(estimation_error_angular(i)),varphi(i)) ) + G4(i) * sign(estimation_error_angular(i));
            att_dist_est(i) = att_dist_est(i) + x3_dot(i) * step;

            //Defining f(x) and g(x)u
            if(i == 0) //Roll
            {
                //fx = ((jy-jz)/jx) * attVelocity(1) * attVelocity(2);
                fx = 0;
                gx_u = torques(0)/jx;
            }
            else if (i == 1) //Pitch
            {
                //fx = ((jz-jx)/jy) * attVelocity(0) * attVelocity(2);
                fx = 0;
                gx_u = torques(1)/jy;
            }
            else //Yaw
            {
                //fx = ((jx-jy)/jz) * attVelocity(0) * attVelocity(1);
                fx = 0;
                gx_u = torques(2)/jz;
            }

            // Proceding with the estimator
            x2_dot(i) = att_dist_est(i) + fx + gx_u + G2(i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),((lambda(i) + 1)/2))  +  powf(std::abs(estimation_error_angular(i)),(varphi(i) + 1)/2) );
            attvel_est(i) = attvel_est(i) + x2_dot(i) * step;

            x1_dot(i) = attvel_est(i) + G1(i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),(lambda(i) + 2)/3)  +  powf(std::abs(estimation_error_angular(i)),(varphi(i) + 2)/3) );
            att_est(i) = att_est(i) + x1_dot(i) * step;

        }

        attitudeEstimates_var.x = att_est(0);
        attitudeEstimates_var.y = att_est(1);
        attitudeEstimates_var.z = att_est(2);

        attVelEstimates_var.x = attvel_est(0);
        attVelEstimates_var.y = attvel_est(1);
        attVelEstimates_var.z = attvel_est(2);

        disturbanceAngEstimates_var.x = att_dist_est(0);
        disturbanceAngEstimates_var.y = att_dist_est(1);
        disturbanceAngEstimates_var.z = att_dist_est(2);

        estimationError_angular_var.x = estimation_error_angular(0);
        estimationError_angular_var.y = estimation_error_angular(1);
        estimationError_angular_var.z = estimation_error_angular(2);

        attitudeEstimates_pub.publish(attitudeEstimates_var);
        attVelEstimates_pub.publish(attVelEstimates_var);
        estimationError_angular_pub.publish(estimationError_angular_var);

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}