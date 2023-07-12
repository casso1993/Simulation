/**************************************
vehicle_pose_and_velocity_updater：
用于发布gazebo提供的定位
**************************************/

#include "vehicle_pose_and_velocity_updater.h"

void vehicle_pose_and_velocity_updater::model_states_callback(const gazebo_msgs::ModelStatesConstPtr &states)
{
  int modelCount = states->name.size();
  for (int modelInd = 0; modelInd < modelCount; ++modelInd)
  {
    if (states->name[modelInd] == "smart")
    {
      vehicle_position = states->pose[modelInd];
      vehicle_velocity = states->twist[modelInd];
      orientation = vehicle_position.orientation;

      // 四元数转换为欧拉角， 求yaw
      tf::Quaternion quaternion(
          orientation.x,
          orientation.y,
          orientation.z,
          orientation.w);
      double roll, pitch, yaw;                            // 定义存储r\p\y的容器
      tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 进行转换

      // center_pose, rear_pose和velocity的time_stamp是同时生成的，为了防止产生两个值，使用time_stamp保存当前时间
      time_stamp = ros::Time::now();

      geometry_msgs::PoseStamped center_pose;
      center_pose.header.frame_id = "/world";
      center_pose.header.stamp = time_stamp;
      center_pose.pose.position = vehicle_position.position;
      center_pose.pose.orientation = vehicle_position.orientation;
      center_pose_pub.publish(center_pose);

      geometry_msgs::PoseStamped rear_pose;
      rear_pose.header.frame_id = "/world";
      rear_pose.header.stamp = time_stamp;
      double center_x = vehicle_position.position.x;
      double center_y = vehicle_position.position.y;
      double rear_x = center_x - cos(yaw) * 0.945;
      double rear_y = center_y - sin(yaw) * 0.945;
      rear_pose.pose.position.x = rear_x;
      rear_pose.pose.position.y = rear_y;
      rear_pose.pose.orientation = vehicle_position.orientation;
      rear_pose_pub.publish(rear_pose);

      geometry_msgs::TwistStamped velocity;
      velocity.header.frame_id = "world";
      velocity.header.stamp = time_stamp;
      velocity.twist.linear = vehicle_velocity.linear;
      velocity.twist.angular = vehicle_velocity.angular;
      vel_pub.publish(velocity);
    }
  }
}

void vehicle_pose_and_velocity_updater::control()
{
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicle_pose_and_velocity_updater");
  vehicle_pose_and_velocity_updater vehicle_pose_and_velocity_updater;
  vehicle_pose_and_velocity_updater.control();
  return 0;
}

vehicle_pose_and_velocity_updater::vehicle_pose_and_velocity_updater()
{
  rear_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/smart/rear_pose", 10);
  center_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/smart/center_pose", 10);
  vel_pub = n.advertise<geometry_msgs::TwistStamped>("/smart/velocity", 10);

  model_states_sub = n.subscribe("/gazebo/model_states", 10, &vehicle_pose_and_velocity_updater::model_states_callback, this);
}

vehicle_pose_and_velocity_updater::~vehicle_pose_and_velocity_updater()
{
}
