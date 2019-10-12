#ifndef NAVIGATION_CONTROL_H
#define NAVIGATION_CONTROL_H

#include <ros/ros.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/thread.hpp>

#define PI 3.14159265

class NavigationControl
{
private:
    	bool is_trajectory_received;
    	double erro_lateral, erro_orientacao;
	double k1,k2;
	double speed;
	
   	double prev_lateral_error;
   	double error_sum;
   	double prev_time;
   	double kp,ki,kd;

	ros::NodeHandle *rosNode;
	image_transport::Subscriber image_sub;
	ros::Subscriber poseArray_sub;
    	ros::Subscriber pose2D_sub;
	
	ros::Publisher command_pub;

	std::vector<cv::Point2d> points;
	geometry_msgs::PoseArray path;
	geometry_msgs::Pose2D pose;	
	cv::Mat g_last_image;	

	std_msgs::Float32MultiArray command;

	boost::mutex g_image_mutex;
	std::string g_window_name;
	
	void image_receive(const sensor_msgs::ImageConstPtr& msg);
	void vehiclePose_receive(const geometry_msgs::Pose2D::ConstPtr& msg);
	void trajectory_receive(const geometry_msgs::PoseArray::ConstPtr& msg);
	void image_processing(const cv::Mat &image);
	void imageShow(const cv::Mat &image);
	//void onMouse(int event, int x, int y, int flag);	
	//static void onMouse(int event, int x, int y, int flag, void* userdata);
	void send_command(float steering=0.0, float speed=0.0);
	double distanceCalculate(double x0, double y0, double x1, double y1);
	double anguloCalculate(double x0, double y0, double x1, double y1);
	void find_errors(double &erro_lateral, double &erro_orientacao);

	double control(double &erro_lateral, double &erro_orientacao, double kp_, double kd_, double ki_);
	bool is_navigation_finished(double radius);

public:
    NavigationControl(ros::NodeHandle *rosNode_);
    ~NavigationControl();

};

#endif // NAVIGATION_CONTROL_H
