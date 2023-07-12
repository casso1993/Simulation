#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelState.h"
#include "tf/tf.h"
#include "styx_msgs/Lane.h"
#include "styx_msgs/Waypoint.h"
#include <iostream>

#define HORIZON 0.05
#define k 0.2
#define L 0.16
#define T 0.16

using namespace std;

class stanley_persuit
{
private:
  ros::NodeHandle n;
  ros::Subscriber rear_pose, velocity, final_waypoints;
  ros::Publisher cmd_vel;
  geometry_msgs::PoseStamped currentPose;
  geometry_msgs::TwistStamped currentVelocity;
  styx_msgs::Lane currentWaypoints;
  void pose_cb(const geometry_msgs::PoseStamped &data1);
  void vel_cb(const geometry_msgs::TwistStamped &data2);
  void lane_cb(const styx_msgs::Lane &data3);
  geometry_msgs::Twist calculateTwistCommand();
  int final_targetX = 0;
  int final_targetY = 4;
  int flag = 0;
  int flag_alpha = 0;

public:
  stanley_persuit();
  ~stanley_persuit();
  void process();
};