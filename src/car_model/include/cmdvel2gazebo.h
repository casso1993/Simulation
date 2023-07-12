#include "ros/ros.h"
#include <stdlib.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "cmath"
#include <algorithm>
#include <iostream>

using namespace std;

#define wheelbase 1.868
#define Tread 1.284
#define maxsteerInside 0.6

class cmdvel2gazebo
{
private:
  ros::NodeHandle n;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher pub_steerL, pub_steerR, pub_rearL, pub_rearR;
  void Cmd_Vel_Callback(const geometry_msgs::Twist &twist);
  double velocity, angle, velocity_rearL, velocity_rearR, angle_frontL, angle_frontR, radius, radius_rearL, radius_rearR, radius_frontL, radius_frontR;
  double rMax, rIdeal, maxsteer;
  ros::Time last_Time;
  float delta_Time;
  float timeout = 0.2;
  void publish();
  bool msgs_too_old;

public:
  cmdvel2gazebo();
  ~cmdvel2gazebo();
  void control();
};
