#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Pose2D.h>
#include "gazebo_msgs/ModelState.h"
#include "tf/tf.h"
#include "styx_msgs/Lane.h"
#include "styx_msgs/Waypoint.h"
#include <iostream>
#include <mpc_track/trajectory.h>
#include <MPC_track/mpc_srv.h>

using namespace std;

#define HORIZON 20
#define L 1.868

class MPC
{
private:
  ros::NodeHandle n;
  ros::Subscriber rear_pose, velocity, final_waypoints, base_waypoints;
  ros::Publisher cmd_vel;
  ros::ServiceClient mpc_optimization;
  geometry_msgs::PoseStamped currentPose;
  geometry_msgs::TwistStamped currentVelocity;
  styx_msgs::Lane currentWaypoints, globalWaypoints;
  void pose_cb(const geometry_msgs::PoseStamped &data1);
  void vel_cb(const geometry_msgs::TwistStamped &data2);
  void lane_cb(const styx_msgs::Lane &data3);
  void global_waypoint_cb(const styx_msgs::Lane &data4);
  geometry_msgs::Twist calculateTwistCommand();
  int Find_target_index();
  styx_msgs::Lane get_local_path(int index);
  styx_msgs::Lane get_state_reference(int index);
  vector<geometry_msgs::Twist> get_control_reference(int index);
  void mpc_optimize(const styx_msgs::Lane currentWaypoints, const vector<geometry_msgs::Twist> control_reference);

  double CurrentRoll, CurrentPitch, CurrentYaw;
  U control;
  vector<geometry_msgs::Pose2D> state_prediction;

public:
  MPC();
  ~MPC();
  void process();
};
