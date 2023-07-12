/*****************************************
waypoint_updater：

*****************************************/
#include "waypoint_updater.h"

void waypoint_updater::publish_waypoints(const int &idx)
{
  styx_msgs::Lane Lane_pub;
  Lane_pub.header = base_waypoints.header;

  // 前视20个点,当不够20个点，代表快到终点，使用终点点
  if (idx + LOOKAHEAD_WPS < waypoint_count)
  {
    for (int i = 0; i < LOOKAHEAD_WPS; i++)
    {
      Lane_pub.waypoints.push_back(base_waypoints.waypoints[idx + i]);
    }
  }
  else
  {
    for (int i = 0; i < waypoint_count - idx; i++)
    {
      Lane_pub.waypoints.push_back(base_waypoints.waypoints[idx + i]);
    }
  }

  nav_msgs::Path path;
  path.header.frame_id = "/world";
  for (int i = 0; i < Lane_pub.waypoints.size(); i++)
  {
    geometry_msgs::PoseStamped path_element;
    path_element.pose.position.x = Lane_pub.waypoints[i].pose.pose.position.x;
    path_element.pose.position.y = Lane_pub.waypoints[i].pose.pose.position.y;
    path_element.pose.position.z = Lane_pub.waypoints[i].pose.pose.position.z;
    path.poses.push_back(path_element);
  }
  final_waypoints_pub.publish(Lane_pub);
  final_path_pub.publish(path);
}

int waypoint_updater::get_closest_waypoint_idx()
{
  pcl::PointXYZ searchPoint;
  searchPoint.x = pose.pose.position.x;
  searchPoint.y = pose.pose.position.y;
  searchPoint.z = pose.pose.position.z;

  vector<int> pointIdxNKNSearch(K);
  vector<float> pointNKNSquaredDistance(K);
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    // for (size_t i=0; i<pointIdxNKNSearch.size (); ++i){
    //   cout << "    点" << (*cloud)[pointIdxNKNSearch[i]].x
    //     << " " << (*cloud)[pointIdxNKNSearch[i]].y
    //     << " " << (*cloud)[pointIdxNKNSearch[i]].z
    //     << " (平方距离: " << pointNKNSquaredDistance[i] << ")" << endl;
    //   cout << "Index :" << pointIdxNKNSearch[i] << endl;
    // }
    return pointIdxNKNSearch[0]; // 返回最近点
  }
}

void waypoint_updater::pose_cb(const geometry_msgs::PoseStamped &msg)
{
  pose = msg;
}

void waypoint_updater::waypoint_cb(const styx_msgs::Lane &waypoints)
{
  waypoint_count = waypoints.waypoints.size(); // 记录一共有多少个waypoint点
  base_waypoints = waypoints;

  // 建立kdtree
  if ((*cloud).size() == 0)
  {
    cloud->width = waypoint_count;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (int i = 0; i < waypoints.waypoints.size(); i++)
    {
      (*cloud)[i].x = waypoints.waypoints[i].pose.pose.position.x;
      (*cloud)[i].y = waypoints.waypoints[i].pose.pose.position.y;
      (*cloud)[i].z = waypoints.waypoints[i].pose.pose.position.z;
    }
    kdtree.setInputCloud(cloud);
  }
}

void waypoint_updater::process()
{
  while (ros::ok())
  {
    ros::spinOnce();
    if (base_waypoints.waypoints.size() != 0)
    {
      int closest_waypoint_idx = get_closest_waypoint_idx();
      publish_waypoints(closest_waypoint_idx);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_updater");
  waypoint_updater waypoint_updater;
  waypoint_updater.process();
  return 0;
}

/*****************************************
final_waypoints_pub：给pure_persuit输入 /final_waypoints，主要用于后续pure_persuit计算
final_path_pub：给rviz显示局部路径
*****************************************/

waypoint_updater::waypoint_updater()
{
  rear_pose = n.subscribe("/smart/rear_pose", 10, &waypoint_updater::pose_cb, this);
  base_waypoints_sub = n.subscribe("/base_waypoints", 10, &waypoint_updater::waypoint_cb, this);
  final_waypoints_pub = n.advertise<styx_msgs::Lane>("final_waypoints", 10);
  final_path_pub = n.advertise<nav_msgs::Path>("final_path", 10);
}

waypoint_updater::~waypoint_updater()
{
}