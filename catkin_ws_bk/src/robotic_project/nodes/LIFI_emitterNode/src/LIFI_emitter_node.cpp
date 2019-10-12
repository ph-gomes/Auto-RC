//////////////////////////////////////////////////////////////////////////////////
//  created:    2016/11/01 - 18:40
//  filename:   LIFI_receiver_node.cpp
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

#include <LIFI_emitter/LIFI_emitter.h>

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "LIFI_emitter");
  	ros::NodeHandle nh;  	
	LIFI_emitter obj(&nh);
  	ros::spin();
  
  	return 0;
}
