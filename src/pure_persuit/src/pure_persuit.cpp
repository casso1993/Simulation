/*********************************************
pure_persuit:
纯追踪算法实现

*********************************************/

#include "pure_persuit.h"

/**********************************************
lad：前瞻距离,初始化为0
targetIndex: 获取局部路径的长度
this当前位置，next下一个点位置，计算两点之间的距离记录下来进行累加。当累加大于horizon就认为是目标点

**********************************************/
geometry_msgs::Twist pure_persuit::calculateTwistCommand()
{
  double lad = 0.0;
  int targetIndex = currentWaypoints.waypoints.size() - 1;
  for (int i = 0; i < currentWaypoints.waypoints.size(); i++)
  {
    if (i + 1 < currentWaypoints.waypoints.size())
    {
      double this_x = currentWaypoints.waypoints[i].pose.pose.position.x;
      double this_y = currentWaypoints.waypoints[i].pose.pose.position.y;
      double next_x = currentWaypoints.waypoints[i + 1].pose.pose.position.x;
      double next_y = currentWaypoints.waypoints[i + 1].pose.pose.position.y;
      lad = lad + hypot(next_x - this_x, next_y - this_y);
      if (lad > HORIZON)
      {
        targetIndex = i + 1;
        break;
      }
    }
  }

  geometry_msgs::Twist twistCmd;
  styx_msgs::Waypoint targetWaypoint;
  double targetSpeed, targetX, targetY, currentX, currentY;
  targetWaypoint = currentWaypoints.waypoints[targetIndex];
  targetSpeed = currentWaypoints.waypoints[0].twist.twist.linear.x;
  targetX = targetWaypoint.pose.pose.position.x;
  targetY = targetWaypoint.pose.pose.position.y;
  currentX = currentPose.pose.position.x;
  currentY = currentPose.pose.position.y;

  // 四元数转欧拉角
  tf::Quaternion q(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z,
                   currentPose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // alpha：计算小车当前的yaw角与目标点之间的角度偏差
  // l: 计算小车当前位置点与目标路径点的距离
  // L: 轴距1.868
  // 当距离l小与0.5时，认为是到达终点
  //  δ = atan(2* L* sin(α) / l)
  auto alpha = atan2(targetY - currentY, targetX - currentX) - yaw;
  auto l = sqrt(pow(currentX - targetX, 2) + pow(currentY - targetY, 2));
  if (l > 0.5)
  {
    auto theta = atan(2 * L * sin(alpha) / l);
    twistCmd.linear.x = targetSpeed;
    twistCmd.angular.z = theta;
  }
  else
  {
    twistCmd.linear.x = 0;
    twistCmd.angular.z = 0;
  }
  return twistCmd;
}

void pure_persuit::process()
{
  while (ros::ok())
  {
    ros::spinOnce();
    if (currentWaypoints.waypoints.size() != 0)
    {
      geometry_msgs::Twist twistCommand = calculateTwistCommand();
      cmd_vel.publish(twistCommand);
    }
  }
}

void pure_persuit::pose_cb(const geometry_msgs::PoseStamped &data1)
{
  currentPose = data1;
}

void pure_persuit::vel_cb(const geometry_msgs::TwistStamped &data2)
{
  currentVelocity = data2;
}

void pure_persuit::lane_cb(const styx_msgs::Lane &data3)
{
  currentWaypoints = data3;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_persuit");
  pure_persuit pure_persuit;
  pure_persuit.process();
  return 0;
}

/**********************************************

rear_pose：接收gazebo发送的/smart/rear_pose  位姿
velocity： 接收gazebo发送的/smart/velocity   速度
final_waypoints：接收waypoint_updater发送过来的/final_waypoints
cmd_vel： 向cmdvel2gazebo发送 /smart/cmd_vel

**********************************************/

pure_persuit::pure_persuit()
{
  rear_pose = n.subscribe("/smart/rear_pose", 10, &pure_persuit::pose_cb, this);
  velocity = n.subscribe("/smart/velocity", 10, &pure_persuit::vel_cb, this);
  final_waypoints = n.subscribe("/final_waypoints", 10, &pure_persuit::lane_cb, this);

  cmd_vel = n.advertise<geometry_msgs::Twist>("/smart/cmd_vel", 10);
}

pure_persuit::~pure_persuit()
{
}