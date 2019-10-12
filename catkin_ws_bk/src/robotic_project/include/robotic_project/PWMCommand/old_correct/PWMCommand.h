//////////////////////////////////////////////////////////////////////////////////
//  created:    2016/11/10 - 18:40
//  filename:   PWMCommand.h
//
//  author:     Giovani Bernardes Vitor
//              Copyright DEG / UFLA
// 
//  version:    $Id: $
//
//  purpose:
//
//////////////////////////////////////////////////////////////////////////////////
#ifndef PWM_COMMAND_H
#define PWM_COMMAND_H


#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "std_msgs/Float32MultiArray.h"


#define STEERING_SERVO	1	// GPIO 11
#define SPEED_SERVO	0	// GPIO 7

class PWMCommand
{
private:
    	

	ros::NodeHandle *rosNode;	
	ros::Subscriber command_sub;
	
	//std_msgs::Float32MultiArray command;
	float steering;
	float prev_steering;
	float speed;
	float prev_speed;
	
	int PWM_speed;
	int PWM_steering;

	int fd;
	char buf[32];

	void control_receive(const std_msgs::Float32MultiArray::ConstPtr& msg);	
	void convertControlToPWM(float steering=0.0, float speed=0.0);	
	void control_smooth();

public:
	PWMCommand(ros::NodeHandle *rosNode_);
	~PWMCommand();
    
	float getSteering();
    	float getSpeed();
	int getPWMsteering();
	int getPWMspeed();
	bool send_PWM(int servo, int value);
	double signalspeed2PWM(double speed);
	double signalangle2PWM(double angle);
	
};

#endif // NAVIGATION_CONTROL_H
