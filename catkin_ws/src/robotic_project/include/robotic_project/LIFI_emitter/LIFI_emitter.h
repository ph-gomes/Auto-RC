//////////////////////////////////////////////////////////////////////////////////
//  created:    2016/11/10 - 18:40
//  filename:   LIFI_emitter.h
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


class LIFI_emitter
{
private:
    	
	
	ros::NodeHandle *rosNode;	
	

	void start_emitter();
	void codification_process();
	bool send_info();

public:
	LIFI_emitter(ros::NodeHandle *rosNode_);
	~LIFI_emitter();
    
	
};



#endif // NAVIGATION_CONTROL_H
