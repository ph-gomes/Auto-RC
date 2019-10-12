#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "Road_Line_Processing.hpp"

ros::Publisher command_pub;
std_msgs::Float32MultiArray command;

void send_command(float steering, float speed)
{
	// std::cout << "send_command: Steering [ " << steering << " ][rad] ["<< steering/M_PI*180 <<"][deg] speed [ " << speed << " ] "<< std::endl;
	command.data[0] = ros::Time::now().toSec(); // Timestamp
	command.data[1] = steering; // steering [rad]
	command.data[2] = speed; // speed [m/s]
	
	command_pub.publish(command);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "road_line_detection");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  RoadLineProcessing myRoad(nh);

  image_transport::Subscriber sub;
  sub = it.subscribe("/raspicam_node/image_raw", 1, boost::bind(&RoadLineProcessing::imageCallback, &myRoad, _1));

  ros::Rate loop_rate(20);

  command_pub = nh.advertise<std_msgs::Float32MultiArray>("/navigationControl/commands", 1);
  
  command.data.reserve(3);
  command.data.resize(3,0.0);

  while (ros::ok())
  {        
    for (int i=0; i<4; i++)
      send_command((float)myRoad.steering, (float)myRoad.velocity);

    // ROS_INFO(" Right_line found (y=mx+b): x[%2.2f], y[%2.2f], m[%2.2f], b[%2.2f]\n\n", 
    //         msg.data[0],
    //         msg.data[1],
    //         msg.data[2],
    //         msg.data[3]);

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
