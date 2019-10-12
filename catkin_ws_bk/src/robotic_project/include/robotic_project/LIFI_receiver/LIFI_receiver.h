//////////////////////////////////////////////////////////////////////////////////
//  created:    2016/11/10 - 18:40
//  filename:   LIFI_receiver.h
//
//  author:     Giovani Bernardes Vitor
//              Copyright DEG / UFLA
// 
//  version:    $Id: $
//
//  purpose:
//
//////////////////////////////////////////////////////////////////////////////////
#ifndef LIFI_RECEIVER_H
#define LIFI_RECEIVER_H


#include <iostream>
#include <ros/ros.h>
#include <stdio.h>

#include "std_msgs/Float32MultiArray.h"


class LIFI_receiver
{
private:
    	
	bool is_info_received;

	ros::NodeHandle *rosNode;	
	ros::Publisher command_pub;
	
	std_msgs::Float32MultiArray command;
	float info;
	
	void start_reception();
	void codification_process();
	bool send_info();

public:
	LIFI_receiver(ros::NodeHandle *rosNode_);
	~LIFI_receiver();
    
	
	
	
};



#endif // NAVIGATION_CONTROL_H
