#include "MPC.h"

int MPC::Find_target_index()
{
  double min = abs(sqrt(pow(currentPose.pose.position.x - globalWaypoints.waypoints[0].pose.pose.position.x, 2) + pow(currentPose.pose.position.y - globalWaypoints.waypoints[0].pose.pose.position.y, 2)));
  int index = 0;
  for (int i = 0; i < globalWaypoints.waypoints.size(); i++)
  {
    double d = abs(sqrt(pow(currentPose.pose.position.x - globalWaypoints.waypoints[i].pose.pose.position.x, 2) + pow(currentPose.pose.position.y - globalWaypoints.waypoints[i].pose.pose.position.y, 2)));
    if (d < min)
    {
      min = d;
      index = i;
    }
  }

  // 索引到终点前，当（机器人与下一个目标点的距离Lf）小于（当前目标点到下一个目标点距离L)时，索引下一个目标点
  if ((index + 1) < globalWaypoints.waypoints.size())
  {
    double current_x = globalWaypoints.waypoints[index].pose.pose.position.x;
    double current_y = globalWaypoints.waypoints[index].pose.pose.position.y;
    double next_x = globalWaypoints.waypoints[index + 1].pose.pose.position.x;
    double next_y = globalWaypoints.waypoints[index + 1].pose.pose.position.y;
    double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
    double L_1 = abs(sqrt(pow(currentPose.pose.position.x - next_x, 2) + pow(currentPose.pose.position.y - next_y, 2)));
    // ROS_INFO("L is %f,Lf is %f",L,Lf);
    if (L_1 < L_)
    {
      index += 1;
    }
  }
  return index;
}

styx_msgs::Lane MPC::get_local_path(int index)
{
  styx_msgs::Lane local_path;
  int num = globalWaypoints.waypoints.size();
  // ROS_INFO("%d", num);
  for (int i = 0; i < HORIZON; i++)
  {
    if (index + i < num)
    {
      local_path.waypoints.push_back(globalWaypoints.waypoints[index + i]);
    }
    else
    {
      local_path.waypoints.push_back(globalWaypoints.waypoints[num - 1]);
    }
  }
  return local_path;
}

styx_msgs::Lane MPC::get_state_reference(int index)
{
  styx_msgs::Lane local_path = get_local_path(index);
  return local_path;
}

// //参考控制量获取函数
vector<geometry_msgs::Twist> MPC::get_control_reference(int index)
{
  vector<geometry_msgs::Twist> control_reference;
  styx_msgs::Lane local_path = get_local_path(index);

  for (int i = 0; i < local_path.waypoints.size(); i++)
  {
    geometry_msgs::Twist tmp;
    tmp.linear.x = local_path.waypoints[i].twist.twist.linear.x;

    // 接着计算对应的前轮转角参考量

    double K = cal_K(globalWaypoints, currentWaypoints.waypoints[i].header.seq); // 计算曲率

    tmp.angular.z = atan2(L * K, 1);

    control_reference.push_back(tmp);
  }
  // ROS_INFO("control_reference length is %d",control_reference.size());
  return control_reference;
}

void MPC::mpc_optimize(const styx_msgs::Lane state_reference, const vector<geometry_msgs::Twist> control_reference)
{
  double solve_time;

  MPC_track::mpc_srv srv;
  srv.request.state_ref = state_reference;
  srv.request.control_ref = control_reference;
  geometry_msgs::Pose2D position;
  position.x = currentPose.pose.position.x;
  position.y = currentPose.pose.position.y;
  position.theta = CurrentYaw;

  srv.request.state = position;
  if (mpc_optimization.call(srv))
  {
    control.v = srv.response.opt_control.linear.x;
    control.kesi = srv.response.opt_control.angular.z;
    state_prediction = srv.response.state_pre;
    solve_time = srv.response.solve_time;
    ROS_INFO("optimization solve time is %fs", solve_time);
  }
  else
  {
    ROS_ERROR("can't connect to optimization!");
    control.v = control.kesi = 0;
    state_prediction.clear();
    solve_time = 0;
  }
}

geometry_msgs::Twist MPC::calculateTwistCommand()
{
  geometry_msgs::Twist twistCmd;
  int currentIndex = Find_target_index();

  tf::Quaternion quanternion(currentPose.pose.orientation.x, currentPose.pose.orientation.y,
                             currentPose.pose.orientation.z, currentPose.pose.orientation.w);
  tf::Matrix3x3(quanternion).getRPY(CurrentRoll, CurrentPitch, CurrentYaw);

  styx_msgs::Lane state_reference = get_state_reference(currentIndex);

  vector<geometry_msgs::Twist> control_reference = get_control_reference(currentIndex);

  mpc_optimize(state_reference, control_reference);

  twistCmd.linear.x = control.v;
  twistCmd.angular.z = control.v * tan(control.kesi) / L;

  return twistCmd;
}

void MPC::process()
{
  ros::Rate rate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    if (globalWaypoints.waypoints.size() != 0)
    {
      if (currentWaypoints.waypoints.size() != 0)
      {
        geometry_msgs::Twist twistCommand = calculateTwistCommand();
        cmd_vel.publish(twistCommand);
      }
    }
    rate.sleep();
  }
}

void MPC::pose_cb(const geometry_msgs::PoseStamped &data1)
{
  currentPose = data1; // 由gazebo获得POSE
}

void MPC::vel_cb(const geometry_msgs::TwistStamped &data2)
{
  currentVelocity = data2; // 由gazebo获得TWIST
}

void MPC::lane_cb(const styx_msgs::Lane &data3)
{
  currentWaypoints = data3; // 前视20点组成都waypoint path
}

void MPC::global_waypoint_cb(const styx_msgs::Lane &data4)
{
  globalWaypoints = data4;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MPC");
  MPC MPC;
  MPC.process();
  return 0;
}

MPC::MPC()
{
  rear_pose = n.subscribe("/smart/rear_pose", 10, &MPC::pose_cb, this);
  velocity = n.subscribe("/smart/velocity", 10, &MPC::vel_cb, this);
  final_waypoints = n.subscribe("/final_waypoints", 10, &MPC::lane_cb, this);
  base_waypoints = n.subscribe("/base_waypoints", 10, &MPC::global_waypoint_cb, this);
  mpc_optimization = n.serviceClient<MPC_track::mpc_srv>("mpc_optimization");

  cmd_vel = n.advertise<geometry_msgs::Twist>("/smart/cmd_vel", 10);
}

MPC::~MPC()
{
}
