#pragma once
#include <iostream>
#include <math.h>
#include <vector>
#include "styx_msgs/Lane.h"
using namespace std;
#define pi acos(-1)
 
//定义路径点
typedef struct waypoint {
	int ID;
	double x, y, yaw;//x,y
}waypoint;
 
//定义小车状态
typedef struct vehicleState {
	double x, y, yaw, v, kesi;//x,y,yaw,前轮偏角kesi
}vehicleState;
 
//定义控制量
typedef struct U {
	double v;
	double kesi;//速度v,前轮偏角kesi
}U;

double cal_K(styx_msgs::Lane lane, int index);//计算曲率K
