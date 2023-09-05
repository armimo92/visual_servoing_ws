#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>


float pos_x;
float pos_y;
float xp;
float yp;
float zp;
float xpp;
float ypp;
float pos_z;
float yaw;
float yawRate;
float yawAccel;
float t;
float step = 0.01;
float arg;
float x_traj;
float y_traj;

Eigen::Vector3f quad_pos;
Eigen::Vector3f error;


void quadPosCallback(const geometry_msgs::Vector3::ConstPtr& quadPos)
{
	quad_pos(0) = quadPos->x;
    quad_pos(1) = quadPos->y;
    quad_pos(2) = quadPos->z;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tgt_pos");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Publisher tgt_pos_pub = nh.advertise<geometry_msgs::Vector3>("tgt_position",100);
	ros::Publisher tgt_yaw_pub = nh.advertise<std_msgs::Float64>("tgt_yaw",100);
	ros::Publisher tgt_vel_pub = nh.advertise<geometry_msgs::Vector3>("tgt_velocity",100);
	ros::Publisher tgt_yaw_rate_pub = nh.advertise<std_msgs::Float64>("tgt_yaw_rate",100);
	ros::Publisher tgt_accel_pub = nh.advertise<geometry_msgs::Vector3>("tgt_acceleration",100);
	ros::Publisher tgt_yaw_acceleration_pub = nh.advertise<std_msgs::Float64>("tgt_yaw_acceleration",100);

	ros::Publisher error_lin_pub = nh.advertise<geometry_msgs::Vector3>("position_error",100);
	

	ros::Subscriber quad_pos_sub = nh.subscribe("quad_position", 100, &quadPosCallback);

	

	geometry_msgs::Vector3 tgt_position;
	geometry_msgs::Vector3 positionError;
	std_msgs::Float64 tgt_yaw;
	
	geometry_msgs::Vector3 tgt_vel;
	std_msgs::Float64 tgt_yaw_vel;
	std_msgs::Float64 tgt_psi_rate;

	geometry_msgs::Vector3 tgt_accel;
	std_msgs::Float64 tgt_yaw_accel;	
	
	
	int i = 0;
	int sim_time = 100/step; //Seconds / step

	//Tgt_pert
	pos_x = -15;
	pos_y = 0;
	pos_z = 0;
	yaw = 0;

	xp = 0;
	yp = 0;
	zp = 0;
	yawRate = 0;

	tgt_position.x = pos_x;
	tgt_position.y = pos_y;
	tgt_position.z = pos_z;
	
	tgt_yaw.data = yaw;
	
	tgt_vel.x = xp;
	tgt_vel.y = yp;
	tgt_vel.z = 0;

	tgt_psi_rate.data = yawRate;
	
	ros::Duration(0.05).sleep();
	tgt_pos_pub.publish(tgt_position);
	tgt_yaw_pub.publish(tgt_yaw);
	tgt_vel_pub.publish(tgt_vel);
	tgt_yaw_rate_pub.publish(tgt_psi_rate);
	ros::Duration(2.3).sleep();
	tgt_pos_pub.publish(tgt_position);
	tgt_yaw_pub.publish(tgt_yaw);
	tgt_vel_pub.publish(tgt_vel);
	tgt_yaw_rate_pub.publish(tgt_psi_rate);

	ros::Duration(1).sleep();
	while(ros::ok())
	{
		t = i*step;
		
		
		// pos_x = -15;
		// pos_y = 0;
		// pos_z = 0;
		// yaw = 0;

		// xp = 0;
		// yp = 0;
		// yawRate = 0;
		
		xp = 0;
		yp = 0;
		yawRate = 0;
			
		if(t>=0 && t<2)
		{
			xp = 0;
			xp = 0;
			yawRate = 0;
		}

		else if (t >= 2 && t < 4)
		{
			xp = 0;
			yp = 0;
			yawRate = -0.2;
		}	
		else if (t >= 4 && t < 8)
		{
			xp = 0.3;
			yp = -0.3;
			yawRate = 0;
		}
		else if (t >= 8 && t < 14)
		{
			xp = 0.5;
			yp = -0.5;
			yawRate = 0;
		}
		else if (t >= 14 && t < 24)
		{
			xp = 0.7;
			yp = -0.7;
			yawRate = 0;
		}	
		else if (t >= 24 && t < 26)
		{
			xp = 1;
			yp = -1;
			yawRate = 0.2;
			}	
		else if (t >= 26 && t < 30)
		{
			xp = 1;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=30 && t<124.2478)
		{	
			xp = 1 * cos(0.1*(t-30));
			yp = 1 * sin(0.1*(t-30));	
			yawRate = 0.1;

			if (t>=70 && t<100)
			{
				pos_z = sin(0.05*3.141592*(t-30));
			}
			else
			{
				pos_z = 0;
			}

			if (pos_z < 0)
			{
				pos_z = 0;
			}

			if (pos_z>0.5)
			{
				pos_z = 0.5;
			}

		}
		else if (t>=124.2478 && t<140)
		{
			xp = -1;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=140 && t<145)
		{
			xp = -0.7;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=145 && t<150)
		{
			xp = -0.4;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=150 && t<155)
		{
			xp = -0.3;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=155 && t<160)
		{
			xp = -0.2;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=160 && t<165)
		{
			xp = -0.1;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=165)
		{
			xp = 0;
			yp = 0;
			yawRate = 0;
		}
		
		pos_x = pos_x + xp * step;
		pos_y = pos_y + yp * step;
		yaw = yaw + yawRate * step;	


		error(0) = quad_pos(0) - pos_x;
		error(1) = quad_pos(1) - pos_y;
		error(2) = quad_pos(2) + 2.5 + pos_z;

		tgt_position.x = pos_x;
		tgt_position.y = pos_y;
		tgt_position.z = -pos_z;
	
		tgt_yaw.data = yaw;

		tgt_vel.x = xp;
		tgt_vel.y = yp;
		tgt_vel.z = 0;
		tgt_psi_rate.data = yawRate;

		tgt_accel.x = 0;
		tgt_accel.y = 0;
		tgt_accel.z = 0;
		tgt_yaw_accel.data = 0;


		positionError.x = error(0);
		positionError.y = error(1);
		positionError.z = error(2);

		tgt_pos_pub.publish(tgt_position);
		tgt_yaw_pub.publish(tgt_yaw);
		
		tgt_vel_pub.publish(tgt_vel);
		tgt_yaw_rate_pub.publish(tgt_psi_rate);

		tgt_accel_pub.publish(tgt_accel);

		error_lin_pub.publish(positionError);
		

		i = i+1;

		std::cout << yawRate << std::endl;

		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	return 0;	
}
