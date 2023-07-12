#include "MPC.h"

void MPC::mpc_optimize(const styx_msgs::Lane currentWaypoints, const vector<geometry_msgs::Twist> control_reference){
  double solve_time;
  
  MPC_track::mpc_srv srv;
  srv.request.state_ref = currentWaypoints;
  srv.request.control_ref = control_reference;
  geometry_msgs::Pose2D position;
  position.x = currentPose.pose.position.x; position.y = currentPose.pose.position.y; position.theta = CurrentYaw;
  srv.request.state = position;
  if(mpc_optimization.call(srv)){
    control.v = srv.response.opt_control.linear.x;
    control.kesi = srv.response.opt_control.angular.z;
    state_prediction = srv.response.state_pre;
    solve_time = srv.response.solve_time;
    ROS_INFO("optimization solve time is %fs", solve_time);
  }
  else{
    ROS_ERROR("can't connect to optimization!");
    control.v = control.kesi = 0;
    state_prediction.clear();
    solve_time = 0;
  }
}


// //参考控制量获取函数
vector<geometry_msgs::Twist> MPC::get_control_reference(int index, double distance){
  vector<geometry_msgs::Twist> control_reference;
  // vector<waypoint> local_path = path->get_local_path(index);  current_waypints = local path
  for (int i = 0; i < currentWaypoints.waypoints.size();i++){
    geometry_msgs::Twist tmp;
    tmp.linear.x = currentWaypoints.waypoints[i].twist.twist.linear.x;

    //接着计算对应的前轮转角参考量
    double K = cal_K(globalWaypoints,index+i);//计算曲率  ?????是否index+i？？？？

    tmp.angular.z = atan2(L * K, 1);

    control_reference.push_back(tmp);

    //ROS_INFO("the v_desired is %f\nthe kesi_desired is %f\n",uu.v, uu.kesi);
  }
  //ROS_INFO("control_reference length is %d",control_reference.size());
  return control_reference;
}

geometry_msgs::Twist MPC::calculateTwistCommand()
{
  double lad = 0.0;
  int targetIndex = currentWaypoints.waypoints.size() - 1;
  int currentIndex = 0;
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
        currentIndex = i+1;
        break;
      }
    }
  }

  cout << "targetIndex: " << targetIndex << endl;
  cout << "currentIndex: " << currentIndex << endl;

  geometry_msgs::Twist twistCmd;
  styx_msgs::Waypoint targetWaypoint;
  //target: 预测的点， current：现在的点
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
  tf::Matrix3x3(quanternion).getRPY(CurrentRoll, CurrentPitch, CurrentYaw);

  double v_distance = abs(sqrt(pow(currentX - targetX, 2) + pow(currentY - targetY, 2)));
  ROS_INFO("the distance is %f", v_distance);

  vector<geometry_msgs::Twist> control_reference = get_control_reference(currentIndex, v_distance);

  mpc_optimize(currentWaypoints, control_reference);

  twistCmd.linear.x = control.v;
  cout << "twistCmd.angular.x: " << twistCmd.angular.x << endl;
  twistCmd.angular.z = control.v * tan(control.kesi) / L;
  cout << "twistCmd.angular.z: " << twistCmd.angular.z << endl;

  return twistCmd;
}

void MPC::process()
{
  while (ros::ok())
  {
    ros::spinOnce();
    if(globalWaypoints.waypoints.size()!=0)
    {
      if (currentWaypoints.waypoints.size() != 0)
      {
        geometry_msgs::Twist twistCommand = calculateTwistCommand();
        cmd_vel.publish(twistCommand);
      }
    }
  }
}

void MPC::pose_cb(const geometry_msgs::PoseStamped &data1)
{
  currentPose = data1; //由gazebo获得POSE
}

void MPC::vel_cb(const geometry_msgs::TwistStamped &data2)
{
  currentVelocity = data2; //由gazebo获得TWIST
}

void MPC::lane_cb(const styx_msgs::Lane &data3)
{
  currentWaypoints = data3; //前视20点组成都waypoint path
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
