#include "stanley_persuit.h"

geometry_msgs::Twist stanley_persuit::calculateTwistCommand()
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


  /*********************************************
  TargetYaw: 获取该局部路径点对应的目标航向角
  CurrentYaw: 获取小车当前的航向角
  *********************************************/
  tf::Quaternion waypoint_quanternion(targetWaypoint.pose.pose.orientation.x, targetWaypoint.pose.pose.orientation.y,
                                      targetWaypoint.pose.pose.orientation.z, targetWaypoint.pose.pose.orientation.w);
  double TargetRoll, TargetPitch, TargetYaw;
  tf::Matrix3x3(waypoint_quanternion).getRPY(TargetRoll, TargetPitch, TargetYaw);

  tf::Quaternion quanternion(currentPose.pose.orientation.x, currentPose.pose.orientation.y,
                             currentPose.pose.orientation.z, currentPose.pose.orientation.w);
  double CurrentRoll, CurrentPitch, CurrentYaw;
  tf::Matrix3x3(quanternion).getRPY(CurrentRoll, CurrentPitch, CurrentYaw);


  //计算得到目标航向角与当前实际航向角之间的航向误差alpha
  double alpha;
  if (TargetYaw >= 0)
  {
    alpha = TargetYaw - CurrentYaw;
  }
  else
  {
    if (TargetYaw * CurrentYaw < 0)
    {
      if (TargetYaw < 0 && CurrentYaw > 0)
      {
        alpha = (M_PI - CurrentYaw) + (M_PI + TargetYaw);
      }
      else
      {
        alpha = -((M_PI + CurrentYaw) + (M_PI - TargetYaw));
      }
    }
    else
    {
      alpha = TargetYaw - CurrentYaw;
    }
  }

  //计算得到小车前轮中心轴位置与目标路径点之间的横向误差error
  double error, vel, delta;
  if (alpha >= 0)
    error = abs(sqrt(pow(currentX - targetX, 2) + pow(currentY - targetY, 2)));
  else
    error = -abs(sqrt(pow(currentX - targetX, 2) + pow(currentY - targetY, 2)));

  //获取小车在当前航向角方向上的速度值
  vel = sqrt(pow(currentVelocity.twist.linear.x, 2) + pow(currentVelocity.twist.linear.y, 2));

  //产生一个直接指向期望路径的前轮偏角delta，并且收敛受车速v(t)限制
  if (vel < 0.001)
    delta = 0;
  else
    delta = atan(k * error / vel);

  //获取小车当前位置与目标位置之间的距离，若小车当前位置与终点位置(0,4)的距离小于0.1m，则小车到达终点，此时停止运动
  double l, l2, theta;
  l = sqrt(pow(currentX - targetX, 2) + pow(currentY - targetY, 2));
  l2 = sqrt(pow(currentX - final_targetX, 2) + pow(currentY - final_targetY, 2));

  if (l > 0.001)
  {
    if (l2 > 0.1)
    {
      theta = alpha + delta;
      twistCmd.linear.x = targetSpeed;
      twistCmd.angular.z = theta;
    }
    else
    {
      twistCmd.linear.x = 0;
      twistCmd.angular.z = 0;
    }
  }
  else
  {
    twistCmd.linear.x = 0;
    twistCmd.angular.z = 0;
  }
  return twistCmd;
}

void stanley_persuit::process()
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

void stanley_persuit::pose_cb(const geometry_msgs::PoseStamped &data1)
{
  currentPose = data1;
}

void stanley_persuit::vel_cb(const geometry_msgs::TwistStamped &data2)
{
  currentVelocity = data2;
}

void stanley_persuit::lane_cb(const styx_msgs::Lane &data3)
{
  currentWaypoints = data3;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stanley_persuit");
  stanley_persuit stanley_persuit;
  stanley_persuit.process();
  return 0;
}

stanley_persuit::stanley_persuit()
{
  rear_pose = n.subscribe("/smart/rear_pose", 10, &stanley_persuit::pose_cb, this);
  velocity = n.subscribe("/smart/velocity", 10, &stanley_persuit::vel_cb, this);
  final_waypoints = n.subscribe("/final_waypoints", 10, &stanley_persuit::lane_cb, this);

  cmd_vel = n.advertise<geometry_msgs::Twist>("/smart/cmd_vel", 10);
}

stanley_persuit::~stanley_persuit()
{
}