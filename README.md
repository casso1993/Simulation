# Simulation

## This code is used for pure persuit and stanley method simulation in ROS gazebo; 
### 0.catkin_make
catkin_make -DCATKIN_WHITELIST_PACKAGES="styx_msgs"

catkin_make -DCATKIN_WHITELIST_PACKAGES=""

### You may need to run "source devel/setup.bash" when opening a new terminal.
### 1.open gazebo
roslaunch gazebo_ros empty_world.launch

### 2.run the set environment
roslaunch car_model spawn_car.launch

### 3.run rviz
rviz
#### File -> open config -> "src/car_model/rviz_config/samrt.rviz"

### 4_1.run pure persuit
roslaunch pure_persuit pure_persuit.launch

### 4_2.run stanley method
roslaunch stanley_persuit stanley_persuit.launch

### 4_3.run MPC
roslaunch MPC_track MPC_track.launch
