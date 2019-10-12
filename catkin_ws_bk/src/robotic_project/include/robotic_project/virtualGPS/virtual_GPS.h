#ifndef GPS_H
#define GPS_H

#include <ros/ros.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "virtualGPS/image_processing.h"

#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/video/tracking.hpp"

#include <boost/thread.hpp>

#define PI 3.14159265

class virtualGPS
{
private:
    	
	ImageProcessing imgProc;

    	image_transport::Subscriber image_sub;
	ros::Publisher pose2D_pub;
    	ros::NodeHandle *rosNode;
	
	geometry_msgs::Pose2D pose, prev_pose;
	
	float rho;
	int count;
	//cv::Mat g_last_image;
	cv::vector<cv::Mat> mean;	
	cv::vector<cv::Mat> variance;
	
	cv::Rect roi;
	int tile;
	
	cv::KalmanFilter *KF;
	cv::Mat_<float> *measurement;

	boost::mutex g_image_mutex;
	std::string g_window_name;
	std::string windowName;
	std::string windowName1; 
	std::string windowName2; 
	std::string windowName3; 
	std::string window_name;
	

	void image_receive(const sensor_msgs::ImageConstPtr& msg);
	void image_processing(const cv::Mat &image);
	void imageShow(const cv::Mat &image);
	double anguloCalculate(double x0, double y0, double x1, double y1);
	
	void drawDetection(cv::Mat &src,vector<Object> &objects, double v_x, double v_y );

	void colorProcessing(Mat cameraFeed, vector<Object> &objects);
	static void on_trackbar( int newValue, void* object);
    	void showValue( int newValue);
	void createTrackbars();
	
	void background_simples(const cv::Mat &image, cv::Mat &mean_, cv::Mat &variance_, cv::Mat &img_bw);
	void background_Gaussian(const cv::Mat &image, cv::Mat &mean_, cv::Mat &variance_, cv::Mat &img_bw);
public:
    
    virtualGPS(ros::NodeHandle *rosNode_);
    ~virtualGPS();
    
};

#endif // GPS_H
