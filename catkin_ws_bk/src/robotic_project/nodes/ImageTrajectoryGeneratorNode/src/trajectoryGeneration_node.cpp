//////////////////////////////////////////////////////////////////////////////////
//  created:    2016/11/01 - 18:40
//  filename:   trajectoryGeneration_node.cpp
//
//  author:     Giovani Bernardes Vitor
//              Copyright DEG / UFLA
// 
//  version:    $Id: $
//
//  purpose:	
//
//////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <stdlib.h> 

#include<trajectoryGeneration/trajectoryGeneration.h>

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "Trajectory_Generation");
  	if (ros::names::remap("image") == "image") 
	{
    		ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
             		 "\t$ rosrun robotic_project trajectoryGeneration image:=<image topic>");
  	}

  	ros::NodeHandle nh;  	
	trajectoryGeneration path(&nh);
  	ros::spin();
  
  	return 0;
}
