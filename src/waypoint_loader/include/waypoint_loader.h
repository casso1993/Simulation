#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <fstream>
#include <string>
#include <cstdlib>
#include <ros/package.h>
#include "tf/tf.h"
#include <iostream>
#include <nav_msgs/Path.h>
#include <styx_msgs/Lane.h>
#include <styx_msgs/Waypoint.h>
#include <cmath>

using namespace std;

#define MAX_DECEL 1.0

class waypoint_loader
{
private:
  ros::NodeHandle n;
  ros::Publisher path_pub, path, state_pub_;
  void getPose(std::string s, double *v);
  vector<styx_msgs::Waypoint> decelerate(const vector<styx_msgs::Waypoint> &data);
  double two_points_distance(const geometry_msgs::Point &current_position, const geometry_msgs::Point &last_position);
  int waypoint_count;

public:
  waypoint_loader();
  ~waypoint_loader();
  int process();
};
