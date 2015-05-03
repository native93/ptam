#include<ros/ros.h>
#include<ptam/move_robot.h>
#include<nav_msgs/OccupancyGrid.h>
#include<fstream>
#include<iostream>
#include"engine.h"
#include<geometry_msgs/Twist.h>
#include<cvd/thread.h>
class move_robot :protected CVD::Thread
{
public:
move_robot(Engine &ep_waypoint);
ros::Publisher pub_vel;
std::string value;
std::string numbers;

virtual void run();
unsigned found;
int counter;
float v,w;
double start_x,start_y,start_theta,goal_x,goal_y,goal_theta;
geometry_msgs::Twist velocity;
Engine *ep_traj;
double *start_x_pointer,*start_y_pointer,*start_theta_pointer,*goal_x_pointer,*goal_y_pointer,*goal_theta_pointer;
private:
ros::ServiceServer service;
ros::NodeHandle nh;
bool cal_traj(
                ptam::move_robot::Request &req,
                ptam::move_robot::Response &res);
};






