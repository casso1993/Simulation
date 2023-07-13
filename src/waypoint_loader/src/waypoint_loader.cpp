/************************************
waypoint_loader：
读取csv的数据，并发布出
************************************/

#include "waypoint_loader.h"

double waypoint_loader::two_points_distance(const geometry_msgs::Point &current_position, const geometry_msgs::Point &last_position)
{
  double x, y, z;
  x = current_position.x - last_position.x;
  y = current_position.y - last_position.y;
  z = current_position.z - last_position.z;
  return sqrt(x * x + y * y + z * z);
}

vector<styx_msgs::Waypoint> waypoint_loader::decelerate(const vector<styx_msgs::Waypoint> &data)
{
  vector<styx_msgs::Waypoint> Return_data = data;
  styx_msgs::Waypoint last;
  double dist, vel;
  last = data[waypoint_count - 1];
  last.twist.twist.linear.x = 0;
  for (int i = 0; i < data.size(); i++)
  {
    dist = two_points_distance(data[i].pose.pose.position, last.pose.pose.position);
    vel = sqrt(2 * MAX_DECEL * dist);
    if (vel < 1)
    {
      vel = 0;
    }
    Return_data[i].twist.twist.linear.x = min(vel, data[i].twist.twist.linear.x);
  }
  return Return_data;
}

/************************************
substr：从字符串中截取p到i-p的子串
c_str()：返回当前字符串的首字符地址
strtod：将字符串转换成浮点数
************************************/

void waypoint_loader::getPose(std::string s, double *v)
{
  int p = 0;
  int q = 0;
  for (int i = 0; i < s.size(); i++) // s.size()有多少个string，即有多少行
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

int waypoint_loader::process()
{
  std::ifstream f(ros::package::getPath("waypoint_loader") + "/waypoints/" + "waypoints.csv"); // 获取csv的路径
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

  /********************************
  getline此函数可读取整行，包括前导和嵌入的空格，并将其存储在字符串对象中。
  其中 f 是正在读取的输入流，而 line 是接收输入字符串的 string 变量的名称
  ********************************/
  while (std::getline(f, line))
  {
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

    pose3.header.seq = count;
    pose3.pose.pose.position.x = pose[0];
    pose3.pose.pose.position.y = pose[1];
    pose3.pose.pose.position.z = 0;
    pose3.pose.pose.orientation.x = q.x();
    pose3.pose.pose.orientation.y = q.y();
    pose3.pose.pose.orientation.z = q.z();
    pose3.pose.pose.orientation.w = q.w();
    pose3.twist.twist.linear.x = 2.78;
    pose3.forward = true;

    waypoints.push_back(pose3);
    track.poses.push_back(pose1);
    ros_path_.poses.push_back(pose2);
  }

  waypoint_count = waypoints.size();
  waypoints = decelerate(waypoints);

  lane.header.frame_id = "world";
  lane.header.stamp = ros::Time::now();
  lane.waypoints = waypoints;

  while (ros::ok())
  {
    path_pub.publish(track);
    state_pub_.publish(ros_path_);
    path.publish(lane);
  }
  return 0;
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "waypoint_loader");
  waypoint_loader waypoint_loader;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(1);
  waypoint_loader.process();
  return 0;
}

/*****************************************
path_pub：
path：给waypoint_updater输入 /base_waypoints，主要用于后续pure_persuit计算
state_pub_：给rviz显示全局路径
*****************************************/

waypoint_loader::waypoint_loader()
{
  path_pub = n.advertise<geometry_msgs::PoseArray>("/trajectory", 10, true);
  path = n.advertise<styx_msgs::Lane>("/base_waypoints", 10, true);
  state_pub_ = n.advertise<nav_msgs::Path>("/path", 10);
}

waypoint_loader::~waypoint_loader()
{
}