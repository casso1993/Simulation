/******************************************
cmdvel2gazebo:
用于接收cmd_vel,并向gazebo发送vel
此代码为阿克曼运动模型
*******************************************/

#include "cmdvel2gazebo.h"

void cmdvel2gazebo::publish()
{
  std_msgs::Float64 msg_vel_L, msg_vel_R, msg_ang_L, msg_ang_R;
  delta_Time = (ros::Time::now() - last_Time).toSec();
  msgs_too_old = delta_Time > timeout ? 1 : 0; // 判断是不是延迟太严重，延迟太严重，不动
  if (msgs_too_old)
  {
    velocity = 0;
    angle = 0;
    msg_vel_L.data = velocity;
    msg_vel_R.data = velocity;
    msg_ang_L.data = angle;
    msg_ang_R.data = angle;

    pub_rearL.publish(msg_vel_L);
    pub_rearR.publish(msg_vel_R);
    pub_steerL.publish(msg_ang_L);
    pub_steerR.publish(msg_ang_R);
  }

  if (angle != 0) // 角度有数据(开始拐弯)，开始计算拐弯时的速度和转弯方向
  {
    // 计算各轮转弯半径
    radius = wheelbase / fabs(tan(angle));
    radius_rearL = radius - (copysign(1, angle) * (Tread / 2.0));
    radius_rearR = radius + (copysign(1, angle) * (Tread / 2.0));
    radius_frontL = radius - (copysign(1, angle) * (Tread / 2.0));
    radius_frontR = radius + (copysign(1, angle) * (Tread / 2.0));

    // 计算后轮速度
    velocity_rearL = velocity * radius_rearL / radius;
    velocity_rearR = velocity * radius_rearR / radius;

    msg_vel_L.data = velocity_rearL;
    msg_vel_R.data = velocity_rearR;

    pub_rearL.publish(msg_vel_L);
    pub_rearR.publish(msg_vel_R);

    // 计算前轮转角
    angle_frontL = atan2(wheelbase, radius_frontL) * copysign(1, angle);
    angle_frontR = atan2(wheelbase, radius_frontR) * copysign(1, angle);

    msg_ang_L.data = angle_frontL;
    msg_ang_R.data = angle_frontR;

    pub_steerL.publish(msg_ang_L);
    pub_steerR.publish(msg_ang_R);
  }
  else // 角度没数据,只给速度就行
  {
    msg_vel_L.data = velocity;
    msg_vel_R.data = velocity;
    msg_ang_L.data = angle;
    msg_ang_R.data = angle;

    pub_rearL.publish(msg_vel_L);
    pub_rearR.publish(msg_vel_R);
    pub_steerL.publish(msg_ang_L);
    pub_steerR.publish(msg_ang_R);
  }
}

void cmdvel2gazebo::Cmd_Vel_Callback(const geometry_msgs::Twist &twist)
{
  velocity = twist.linear.x / 0.3;
  angle = max(-maxsteer, min(maxsteer, twist.angular.z)); // -maxsteer< wist.angular.z < maxsteer
  last_Time = ros::Time::now();
  publish();
}

void cmdvel2gazebo::control()
{
  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmdvel2gazebo");
  cmdvel2gazebo cmdvel2gazebo;
  cmdvel2gazebo.control();
  return 0;
}

/******************************************
*pub_steerL: 前左轮方向
*pub_steerR: 前右轮方向
*pub_rearL:  后左轮速度
*pub_rearR:  后右轮速度

*cmd_vel_sub: 接收速度与方向
*******************************************/
cmdvel2gazebo::cmdvel2gazebo()
{
  pub_steerL = n.advertise<std_msgs::Float64>("/smart/front_left_steering_position_controller/command", 10);
  pub_steerR = n.advertise<std_msgs::Float64>("/smart/front_right_steering_position_controller/command", 10);
  pub_rearL = n.advertise<std_msgs::Float64>("/smart/rear_left_velocity_controller/command", 10);
  pub_rearR = n.advertise<std_msgs::Float64>("/smart/rear_right_velocity_controller/command", 10);
  cmd_vel_sub = n.subscribe("/smart/cmd_vel", 100, &cmdvel2gazebo::Cmd_Vel_Callback, this);

  last_Time = ros::Time::now();

  rMax = wheelbase / tan(maxsteerInside);
  rIdeal = rMax + (Tread / 2.0);
  maxsteer = atan2(wheelbase, rIdeal);
}

cmdvel2gazebo::~cmdvel2gazebo()
{
}