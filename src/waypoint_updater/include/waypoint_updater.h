#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "styx_msgs/Lane.h"
#include "nav_msgs/Path.h"
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

#define LOOKAHEAD_WPS 20
#define K 10

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

class waypoint_updater
{
private:
  ros::NodeHandle n;
  ros::Subscriber rear_pose, base_waypoints_sub;
  ros::Publisher final_waypoints_pub, final_path_pub;
  void pose_cb(const geometry_msgs::PoseStamped &msg);
  void waypoint_cb(const styx_msgs::Lane &waypoints);
  geometry_msgs::PoseStamped pose;
  styx_msgs::Lane base_waypoints;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  int waypoint_count;
  int get_closest_waypoint_idx();
  void publish_waypoints(const int &idx);

public:
  waypoint_updater();
  ~waypoint_updater();
  void process();
};
