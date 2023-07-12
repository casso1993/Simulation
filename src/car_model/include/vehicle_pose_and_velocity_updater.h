#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "math.h"
#include "tf/tf.h"
#include <iostream>

using namespace std;

class vehicle_pose_and_velocity_updater
{
private:
  ros::NodeHandle n;
  ros::Publisher rear_pose_pub, center_pose_pub, vel_pub;
  ros::Subscriber model_states_sub;
  void model_states_callback(const gazebo_msgs::ModelStatesConstPtr &states);
  ros::V_string vehicle_model_index;
  geometry_msgs::Pose vehicle_position;
  geometry_msgs::Twist vehicle_velocity;
  geometry_msgs::Quaternion orientation;
  ros::Time time_stamp;

public:
  vehicle_pose_and_velocity_updater();
  ~vehicle_pose_and_velocity_updater();
  void control();
};