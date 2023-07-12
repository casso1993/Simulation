/**************************************
transform_publisher：
用于发布坐标转换
发布小车base_link与RVIZ中世界坐标之间的转换
**************************************/

#include "transform_publisher.h"

void transform_publisher::pose_callback(const geometry_msgs::PoseStamped &pose_msgs)
{
  auto pose = pose_msgs.pose.position;
  auto orientation = pose_msgs.pose.orientation;
  static tf::TransformBroadcaster br;                                                                // 创建tf广播
  tf::Transform transform;                                                                           // 创建tf转换
  transform.setOrigin(tf::Vector3(pose.x, pose.y, pose.z));                                          // 向tf转换的对象赋值pose
  transform.setRotation(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)); // 向tf转换的对象赋值orientation

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link")); // 向tf 广播输入坐标变换
}

void transform_publisher::control()
{
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_publisher");
  transform_publisher transform_publisher;
  transform_publisher.control();
  return 0;
}

transform_publisher::transform_publisher()
{
  center_pose_sub = n.subscribe("/smart/center_pose", 10, &transform_publisher::pose_callback, this);
}

transform_publisher::~transform_publisher()
{
}
