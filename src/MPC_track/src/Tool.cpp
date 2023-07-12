#include<iostream>
#include<mpc_track/Tool.h>

double cal_K(styx_msgs::Lane lane, int index){
	double res;
	//差分法求一阶导和二阶导
	double dx, dy, ddx, ddy;
	if (index == 0) {
		dx = lane.waypoints[1].pose.pose.position.x - lane.waypoints[0].pose.pose.position.x;
		dy = lane.waypoints[1].pose.pose.position.y - lane.waypoints[0].pose.pose.position.y;
		ddx = lane.waypoints[2].pose.pose.position.x + lane.waypoints[0].pose.pose.position.x - 2 * lane.waypoints[1].pose.pose.position.x;
		ddy = lane.waypoints[2].pose.pose.position.y + lane.waypoints[0].pose.pose.position.y - 2 * lane.waypoints[1].pose.pose.position.y;
	}
	else if (index == (lane.waypoints.size() - 1)) {
		dx = lane.waypoints[index].pose.pose.position.x - lane.waypoints[index - 1].pose.pose.position.x;
		dy = lane.waypoints[index].pose.pose.position.y - lane.waypoints[index - 1].pose.pose.position.y;
		ddx = lane.waypoints[index].pose.pose.position.x + lane.waypoints[index - 2].pose.pose.position.x - 2 * lane.waypoints[index].pose.pose.position.x;
		ddy = lane.waypoints[index].pose.pose.position.y + lane.waypoints[index - 2].pose.pose.position.y - 2 * lane.waypoints[index].pose.pose.position.y;
	}
	else {
		dx = lane.waypoints[index + 1].pose.pose.position.x - lane.waypoints[index].pose.pose.position.x;
		dy = lane.waypoints[index + 1].pose.pose.position.y - lane.waypoints[index].pose.pose.position.y;
		ddx = lane.waypoints[index + 1].pose.pose.position.x + lane.waypoints[index - 1].pose.pose.position.x - 2 * lane.waypoints[index].pose.pose.position.x;
		ddy = lane.waypoints[index + 1].pose.pose.position.y + lane.waypoints[index - 1].pose.pose.position.y - 2 * lane.waypoints[index].pose.pose.position.y;
	}
	//res.yaw = atan2(dy, dx);//yaw
	//计算曲率：设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
	res = (ddy * dx - ddx * dy) / (sqrt(pow((pow(dx, 2) + pow(dy, 2)), 3)));
	return res;
}
