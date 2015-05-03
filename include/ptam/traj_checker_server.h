#include<ros/ros.h>
#include<ptam/trajectory_checker.h>
#include<nav_msgs/OccupancyGrid.h>
#include<fstream>
#include<iostream>
#include<geometry_msgs/PoseStamped.h>
class traj_check
{
public:
traj_check();
nav_msgs::OccupancyGrid map_msg;
bool map_received;
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
ros::Subscriber map_sub;
ros::Publisher pub_bad;
ros::Publisher pub_good;
std::string value;
std::string numbers;
unsigned found;
geometry_msgs::PoseStamped pose;
int cur_point_x;
int cur_point_y;
int counter;


private:
ros::ServiceServer service;
ros::NodeHandle nh;
bool check_trajectory(
                ptam::trajectory_checker::Request &req,
                ptam::trajectory_checker::Response &res);
};



