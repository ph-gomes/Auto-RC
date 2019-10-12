//////////////////////////////////////////////////////////////////////////////////
//  created:    2016/01/01 - 18:40
//  filename:   ManualOperation_node.cpp
//
//  author:     Giovani Bernardes Vitor
//              Copyright DEG / UFLA
// 
//  version:    $Id: $
//
//  purpose:	Send manual commands to car
//////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>


#include "std_msgs/Float32MultiArray.h"

#define KEY_UP    65
#define KEY_DOWN  66
#define KEY_RIGHT 67
#define KEY_LEFT  68


using namespace std;


ros::Publisher command_pub;
std_msgs::Float32MultiArray command;

///////////////////////////////////////////////////////////////////////////////////////////////
char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

///////////////////////////////////////////////////////////////////////////////////////////////
void send_command(float steering, float speed)
{
	cout << "send_command: Steering [ " << steering << " ] speed [ " << speed << " ] "<< endl;
	command.data[0] = ros::Time::now().toSec(); // Timestamp
	command.data[1] = steering; // steering [rad]
	command.data[2] = speed; // speed [m/s]
	
	command_pub.publish(command);
}


///////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "ManualOperation_node");
     ros::NodeHandle nh;
	
    command_pub = nh.advertise<std_msgs::Float32MultiArray>("/navigationControl/commands", 1);
     
     command.data.reserve(3);
     command.data.resize(3,0.0);

     //Sets the loop to publish at a rate of 20Hz
     ros::Rate rate(20);

      float speed = 2	; // [m/s]
      float steering = 0.0; // [rad]
      cout << " :: CAR Teleoperation ::  \n  Press arrow keys to send command..." << endl;
	int c=0;
       while(ros::ok()) 
	{
		  
		  c=0;	  
		  fflush(stdin);
		  switch((c=getch())) 
		  {
			case KEY_UP:
				speed += 0.1;		
			    	//cout << "speed up: " << speed << endl;//key up
				
			    break;
			case KEY_DOWN:
				speed -= 0.1;
			    	//cout << "speed down: " << speed << endl;//key down
			    break;
			case KEY_LEFT:
				steering -= 0.05;		
				steering = std::max(std::min(steering,(float)0.6),(float)-0.6);		    	
				//cout << "Left steering: " << steering << endl;// key left
			    break;
			case KEY_RIGHT:
				steering += 0.05;	
				steering = std::max(std::min(steering,(float)0.6),(float)-0.6);	
			    	//cout << "Right steering: " << steering << endl;// key right				
			    break;
			default:		    
			    break;
		}
			
		send_command(steering, speed);
		
		
		ros::spinOnce();
		
		//Delays untill it is time to send another message
		rate.sleep();
     }
	

	send_command(0.0, 0.0);

	return 0;

}


