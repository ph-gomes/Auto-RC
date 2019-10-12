//////////////////////////////////////////////////////////////////////////////////
//  created:    2016/11/01 - 18:40
//  filename:   PWMCommand_node.cpp
//
//  author:     Giovani Bernardes Vitor
//              Copyright DEG / UFLA
// 
//  version:    $Id: $
//
//  purpose:	[timestamps, steering wheel "rad", speed "m/s"]
//
//////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <stdlib.h> 

#include <PWMCommand/PWMCommand.h>


void infoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "PWMCommand_Rpi");
  	
  	ros::NodeHandle nh;
	
	ros::Subscriber command_sub = nh.subscribe<std_msgs::Float32MultiArray>("/LIFI_receiver/commands", 2,infoCallback);

	PWMCommand servoDriver(&nh);

	//Sets the loop to send PWM at a rate of 10Hz
     	ros::Rate rate(10);

	while(ros::ok()) {
          
		double now = ros::Time::now().toSec();
		double dt = now - servoDriver.getReceived_InfoTime();
		
		//std::cout << " now: " << now << " received: " << servoDriver.getReceived_InfoTime() << " Delta time: " << dt << std::endl;
		if(dt >= 2.0) servoDriver.setSpeed(0.0); // Stop Speed after 2 seconds...

		std::cout << " :: PWM :: Delta Time [ " << dt << " ] steering [ " << servoDriver.getSteering() << " ] speed [ " << servoDriver.getSpeed() << " ] " << std::endl;	
	  	
		servoDriver.send_PWM(STEERING_SERVO, servoDriver.getPWMsteering() );
		servoDriver.send_PWM(   SPEED_SERVO, servoDriver.getPWMspeed()    );

		ros::spinOnce();

          //Delays untill it is time to send another message
          rate.sleep();
         }


  	return 0;
}


void infoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	ROS_INFO(" INFO LIFI received... ");
	
	return;
}


