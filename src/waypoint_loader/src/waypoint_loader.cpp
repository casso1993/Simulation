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


void getPose(std::string s, double *v)
{
  int p = 0;
  int q = 0;
  for (int i = 0; i < s.size(); i++)
  {
    if (s[i] == ',' || i == s.size() - 1)
    {
      char tab2[16];
      strcpy(tab2, s.substr(p, i - p).c_str());
      v[q] = std::strtod(tab2, NULL);
      p = i + 1;
      q++;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_loader");
  ros::NodeHandle n;
  ros::Publisher path_pub = n.advertise<geometry_msgs::PoseArray>("/trajectory", 10, true);
  ros::Publisher path = n.advertise<styx_msgs::Lane>("/base_waypoints", 10, true);
  ros::Publisher state_pub_ = n.advertise<nav_msgs::Path>("/path", 10);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(1);
  std::ifstream f(ros::package::getPath("waypoint_loader") + "/waypoints/" + "waypoints.csv");
  //f.open(ros::package::find(plan_pkg)+"/paths/path.csv"); //ros::package::find(plan_pkg)
  if (!f.is_open())
  {
    ROS_ERROR("failed to open file");
    return 0;
  }
  std::string line;
  std::vector<styx_msgs::Waypoint> waypoints;
  styx_msgs::Lane lane;
  double scale = 100;
  int count = -1;
  nav_msgs::Path ros_path_;

  geometry_msgs::PoseArray track;
  track.header.stamp = ros::Time::now();
  track.header.frame_id = "/world";
  while (std::getline(f, line))
  {
    std::cout << "Hello w000orld!!!" << std::endl;

    count++;
    double pose[3];
    getPose(line, pose);
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pose[2]);
    geometry_msgs::Pose pose1;

    geometry_msgs::PoseStamped pose2;
    styx_msgs::Waypoint pose3;
    ros_path_.header.frame_id = "/world";
    ros_path_.header.stamp = ros::Time::now();  
    pose2.header = ros_path_.header;

    pose2.pose.position.x = pose[0];
    pose2.pose.position.y = pose[1];
    pose2.pose.position.z = 0;

    pose1.orientation.x = q.x();
    pose1.orientation.y = q.y();
    pose1.orientation.z = q.z();
    pose1.orientation.w = q.w();
    pose1.position.x = pose[0];
    pose1.position.y = pose[1];
    pose1.position.z = 0;

    pose3.pose.pose.position.x = pose[0];
    pose3.pose.pose.position.y = pose[1];
    pose3.pose.pose.position.z = 0;
    pose3.pose.pose.orientation.x = q.x();
    pose3.pose.pose.orientation.y = q.y();
    pose3.pose.pose.orientation.z = q.z();
    pose3.pose.pose.orientation.w = q.w();
    pose3.twist.twist.linear.x = 2.78;
    pose3.forward = 1;

    std::cout << "path_pose1.position.x" << pose1.position.x <<std::endl;

    ROS_INFO("Adding waypoint x,y,z =  [%.2f, %0.2f, %0.2f]", pose1.position.x, pose1.position.y, pose1.position.z);

    waypoints.push_back(pose3);
    track.poses.push_back(pose1);
    ros_path_.poses.push_back(pose2);


  }

  lane.header.frame_id = "world";
  lane.header.stamp = ros::Time::now();
  lane.waypoints = waypoints;

  sleep(1);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    path_pub.publish(track);
    state_pub_.publish(ros_path_);
    path.publish(lane);
  }
  return 0;
}

