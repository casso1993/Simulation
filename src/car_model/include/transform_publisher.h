#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include <iostream>

using namespace std;

class transform_publisher
{
private:
  ros::NodeHandle n;
  ros::Subscriber center_pose_sub;
  void pose_callback(const geometry_msgs::PoseStamped &pose_msgs);

public:
  transform_publisher();
  ~transform_publisher();
  void control();
};