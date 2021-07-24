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

cv::Mat frame;		//

int p1x = 0;
int p1y = 0;

int p2x = 0;
int p2y = 0;

int p3x = 0;
int p3y = 0;

int p4x = 0;
int p4y = 0;

float focal_length = 3.04e-3;//in meters
float pixel_size = 1.12e-6; //in meters
float new_focal = focal_length/pixel_size; 

float p1x_vs;
float p1y_vs;

float p2x_vs;
float p2y_vs;

float p3x_vs;
float p3y_vs;

float p4x_vs;
float p4y_vs;

float ug;
float ng;

float mu20;
float mu02;
float mu11;

float resta;

Eigen::Vector3f p1_vs_cf;
Eigen::Vector3f p2_vs_cf;
Eigen::Vector3f p3_vs_cf;
Eigen::Vector3f p4_vs_cf;

Eigen::Vector3f p1_vs_vf;
Eigen::Vector3f p2_vs_vf;
Eigen::Vector3f p3_vs_vf;
Eigen::Vector3f p4_vs_vf;

float beta_p1;
float beta_p2;
float beta_p3;
float beta_p4;

float qx;
float qy;
float qz;
float qpsi;

float a;
float aD = 4.09e-9;

float siono;

Eigen::RowVector3f Row_e3;

Eigen::Vector3f uav_att;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		frame = cv_bridge::toCvShare(msg, "bgr8")->image; //Saving the image
	}
	
	catch(cv_bridge::Exception& e)
	{ 
		ROS_ERROR("Couldn't convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void attitude_callback(const geometry_msgs::Vector3::ConstPtr& att)
{
	uav_att(0) = att->x;
	uav_att(1) = att->y;
	uav_att(2) = att->z;
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

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "aruco_detect");

	ros::NodeHandle nh;
	
	ros::Rate loop_rate(50);	
	
	image_transport::ImageTransport it(nh);
	
	image_transport::Subscriber sub = it.subscribe("/quad/camera/image_raw", 100, imageCallback);
	
	ros::Subscriber attitude_sub = nh.subscribe("uav_attitude",100, &attitude_callback);
	
	geometry_msgs::Vector3 vector_q;
	std_msgs::Float64 Feat_qpsi;
	std_msgs::Float64 hay_imagen_o_no;
	geometry_msgs::Pose2D punto1_vis;
	geometry_msgs::Pose2D punto2_vis;
	geometry_msgs::Pose2D punto3_vis;
	geometry_msgs::Pose2D punto4_vis;
	
	ros::Publisher q_pub = nh.advertise<geometry_msgs::Vector3>("q_linear",100);
	ros::Publisher qpsi_pub = nh.advertise<std_msgs::Float64>("q_psi",100);
	ros::Publisher hay_imagen_pub = nh.advertise<std_msgs::Float64>("hay_imagen",100);
	ros::Publisher p1_pub = nh.advertise<geometry_msgs::Pose2D>("punto1",100);
	ros::Publisher p2_pub = nh.advertise<geometry_msgs::Pose2D>("punto2",100);
	ros::Publisher p3_pub = nh.advertise<geometry_msgs::Pose2D>("punto3",100);
	ros::Publisher p4_pub = nh.advertise<geometry_msgs::Pose2D>("punto4",100);
	
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
	
	Row_e3 << 0,0,1;
	
	while (ros::ok())
	{	
		//INITIALIZE THE DETECTOR PARAMETERS USING DEFAULT VALUES
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		//VECTORS THAT WOULD CONTAIN THE DETECTED MARKER CORNERS AND THE REJECTED CANDIDATES
		std::vector<std::vector<cv::Point2f> > markerCorners, rejectCandidates;
		//SAVE THE MARKER ID
		std::vector<int> markerIds;

		if(!frame.empty())
		{
			cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectCandidates);
		}
		else
		{
			std::cout << "empty " << std::endl;
		}
		

		if (markerCorners.empty())
		{
			std::cout << "No hay marca" << std::endl;
					
	
		}
		else
		{
			//Extracción de esquinas
			cv::Point p1 = markerCorners[0].at(0);
			cv::Point p2 = markerCorners[0].at(1);
			cv::Point p3 = markerCorners[0].at(2);
			cv::Point p4 = markerCorners[0].at(3);

			//Coordenadas en XY de las esquinas con cambio de sentido en Y y origen en el centro de la imagen
			/*
										+y	^		
												|
												|
												|
			      -x <--------|---------> +x
			      						|
			      						|
			      						|
			      						|
			      				-y		
					
			*/
			p1x = p1.x-frame.size().width/2;
			p1y = -(p1.y-frame.size().height/2);
			
			p2x = p2.x-frame.size().width/2;
			p2y = -(p2.y-frame.size().height/2);
			
			p3x = p3.x-frame.size().width/2;
			p3y = -(p3.y-frame.size().height/2);
			
			p4x = p4.x-frame.size().width/2;
			p4y = -(p4.y-frame.size().height/2);
			
				
			p1_vs_cf << p1x, p1y, new_focal;
			p2_vs_cf << p2x, p2y, new_focal;
			p3_vs_cf << p3x, p3y, new_focal;
			p4_vs_cf << p4x, p4y, new_focal;
			
			//Escalar beta para conversión de camera frame a virual frame
			beta_p1 = new_focal/(Row_e3*Rtp(uav_att(0),uav_att(1))*p1_vs_cf);
			beta_p2 = new_focal/(Row_e3*Rtp(uav_att(0),uav_att(1))*p2_vs_cf);
			beta_p3 = new_focal/(Row_e3*Rtp(uav_att(0),uav_att(1))*p3_vs_cf);
			beta_p4 = new_focal/(Row_e3*Rtp(uav_att(0),uav_att(1))*p4_vs_cf);
			
			//Conversión de coordenadas camera frame a virual frame
			p1_vs_vf = beta_p1*Rtp(uav_att(0),uav_att(1))*p1_vs_cf;
			p2_vs_vf = beta_p2*Rtp(uav_att(0),uav_att(1))*p2_vs_cf;
			p3_vs_vf = beta_p3*Rtp(uav_att(0),uav_att(1))*p3_vs_cf;
			p4_vs_vf = beta_p4*Rtp(uav_att(0),uav_att(1))*p4_vs_cf;
			
			p1_vs_vf = p1_vs_vf*pixel_size;
			p2_vs_vf = p2_vs_vf*pixel_size;
			p3_vs_vf = p3_vs_vf*pixel_size;
			p4_vs_vf = p4_vs_vf*pixel_size;
			
			//Momentos ordinarios. Centroide
			ug = (p1_vs_vf(0) + p2_vs_vf(0) + p3_vs_vf(0) + p4_vs_vf(0)) / 4;
			ng = (p1_vs_vf(1) + p2_vs_vf(1) + p3_vs_vf(1) + p4_vs_vf(1)) / 4;
			
			//Momentos centrados
			mu20 = powf((p1_vs_vf(0) - ug), 2) +  powf((p2_vs_vf(0) - ug), 2) +  powf((p3_vs_vf(0) - ug), 2) +  powf((p4_vs_vf(0) - ug), 2);
			mu02 = powf((p1_vs_vf(1) - ng), 2) +  powf((p2_vs_vf(1) - ng), 2) +  powf((p3_vs_vf(1) - ng), 2) +  powf((p4_vs_vf(1) - ng), 2);
			mu11 = (p1_vs_vf(0) - ug) * (p1_vs_vf(1) - ng) + (p2_vs_vf(0) - ug) * (p2_vs_vf(1) - ng) + (p3_vs_vf(0) - ug) * (p3_vs_vf(1) - ng) + (p4_vs_vf(0) - ug) * (p4_vs_vf(1) - ng);
			resta = mu20-mu02;
			if(resta == 0 || std::abs(resta) < 8e-11)
			{
				resta = 1;
			}	
				
			if(std::abs(mu11) < 1e-10)
			{
				mu11 = 0;
			}
			
			//Vector de características q
			a = mu20 + mu02;
			qz = sqrt(aD/a);
			qx = qz * ng/focal_length;
			qy = qz * ug/focal_length;
			qpsi = 0.5 * atanf((2*mu11)/resta);
			
			
			vector_q.x = qx;
			vector_q.y = qy;
			vector_q.z = qz;
			
			Feat_qpsi.data = qpsi;		
			
			punto1_vis.x = p1_vs_vf(0);
			punto1_vis.y = p1_vs_vf(1);
			
			punto2_vis.x = p2_vs_vf(0);
			punto2_vis.y = p2_vs_vf(1);
			
			punto3_vis.x = p3_vs_vf(0);
			punto3_vis.y = p3_vs_vf(1);
			
			punto4_vis.x = p4_vs_vf(0);
			punto4_vis.y = p4_vs_vf(1);
			
			p1_pub.publish(punto1_vis);
			p2_pub.publish(punto2_vis);
			p3_pub.publish(punto3_vis);
			p4_pub.publish(punto4_vis);
			
			
						
			q_pub.publish(vector_q);
			qpsi_pub.publish(Feat_qpsi);
	
			
			std::cout << "p1: " << p1_vs_vf(0) << ", " << p1_vs_vf(1) << std::endl;
			std::cout << "p2: " << p2_vs_vf(0) << ", " << p2_vs_vf(1) << std::endl;
			std::cout << "p3: " << p3_vs_vf(0) << ", " << p3_vs_vf(1) << std::endl;
			std::cout << "p4: " << p4_vs_vf(0) << ", " << p4_vs_vf(1) << std::endl;			
			std::cout << "ug: " << ug << std::endl;
			std::cout << "ng: " << ng << std::endl;
			std::cout << "mu20: " << mu20 << std::endl;
			std::cout << "mu02: " << mu02 << std::endl;
			std::cout << "mu11: " << mu11 << std::endl;
			std::cout << "resta: " << resta << std::endl;
			std::cout << "qx: " << qx << std::endl;
			std::cout << "qy: " << qy << std::endl;
			std::cout << "qz: " << qz << std::endl;
			std::cout << "qpsi: " << qpsi << std::endl;

		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}		
	
	return 0;
}
