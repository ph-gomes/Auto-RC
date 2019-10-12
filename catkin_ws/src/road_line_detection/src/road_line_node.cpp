#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "Road_Line_Processing.hpp"

#include <termios.h>

#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_RIGHT 67
#define KEY_LEFT 68
#define KEY_SPACE 32

ros::Publisher command_pub;
std_msgs::Float32MultiArray command;

char getch()
{
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  struct termios old = {0};
  if (tcgetattr(filedesc, &old) < 0)
    ROS_ERROR("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &old) < 0)
    ROS_ERROR("tcsetattr ICANON");

  if (rv != -1 && rv != 0)
    read(filedesc, &buff, len);

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
    ROS_ERROR("tcsetattr ~ICANON");
  return (buff);
}

void send_command(float steering, float speed)
{
  command.data[0] = ros::Time::now().toSec(); // Timestamp
  command.data[1] = steering;                 // steering [rad]
  command.data[2] = speed;                    // speed [m/s]

  command_pub.publish(command);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "road_line_detection");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  RoadLineProcessing myRoad(nh);

  image_transport::Subscriber sub;
  // sub = it.subscribe("/camera/image", 1, boost::bind(&RoadLineProcessing::imageCallback, &myRoad, _1));
  sub = it.subscribe("/raspicam_node/image_raw", 1, boost::bind(&RoadLineProcessing::imageCallback, &myRoad, _1));

  ros::Rate loop_rate(20);

  command_pub = nh.advertise<std_msgs::Float32MultiArray>("/navigationControl/commands", 1);

  command.data.reserve(3);
  command.data.resize(3, 0.0);

  int c = 0;
  double var = peso1;
  double var2 = peso2;

  peso1 = 113;
  peso2 = 100;
  double initial = ros::Time::now().toSec();
  double now = ros::Time::now().toSec();
  double dt = now - initial;

  bool run = true;
  while (ros::ok())
  {
    var = peso1;
    var2 = peso2;

    c = getch();
    fflush(stdin);

    if (c == KEY_UP)
      var += 1;
    else if (c == KEY_DOWN)
      var -= 1;
    else if (c == KEY_RIGHT)
      var2 += 1;
    else if (c == KEY_LEFT)
      var2 -= 1;
    else if (c == KEY_SPACE)
      run = (run) ? false : true;

    if (var < 0)
      var = 0;
    else if (var > 200)
      var = 200;
    if (var2 < 0)
      var2 = 0;
    else if (var2 > 200)
      var2 = 200;

    peso1 = var;
    peso2 = var2;

    now = ros::Time::now().toSec();
    dt = now - initial;
    if (run)
      for (int i = 0; i < 4; i++)
      {
        if (std::fmod(dt, 1) > 0.8)
          send_command((float)myRoad.steering, 1.66);
        else
          send_command((float)myRoad.steering, 1.66);
      }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
