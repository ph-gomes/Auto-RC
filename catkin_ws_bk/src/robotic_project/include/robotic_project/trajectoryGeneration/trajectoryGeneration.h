#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include <ros/ros.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <iterator>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/thread.hpp>
#include <math.h>

#define PI 3.14159265	

class trajectoryGeneration
{
private:
    	    
    	image_transport::Subscriber image_sub;
	ros::Publisher poseArray_pub;
    	ros::NodeHandle *rosNode;
	
	std::vector<cv::Point2d> points;
	geometry_msgs::PoseArray path;

	cv::Mat g_last_image;	
	boost::mutex g_image_mutex;
	std::string g_window_name;
	
	

	void image_receive(const sensor_msgs::ImageConstPtr& msg);
	void image_processing(const cv::Mat &image);
	void imageShow(const cv::Mat &image);
	void onMouse(int event, int x, int y, int flag);	
	static void onMouse(int event, int x, int y, int flag, void* userdata);
	double anguloCalculate(double x0, double y0, double x1, double y1);
	void trajectoryReader(std::string fileName);
	

public:
    trajectoryGeneration(ros::NodeHandle *rosNode_);
    ~trajectoryGeneration();
};

//std::istream& operator>>(std::istream& is, cv::Point2d& coordinates);
std::vector<std::string> split_string(const std::string& str, const std::string& delimiter);

#endif // GPS_H
