#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "engine.h"
#include<iostream>
#include<fstream>
#include<tf/tf.h>
#include<string>
#include<ros/ros.h>
#include<ptam/pos_robot.h>
#include<geometry_msgs/Twist.h>
#include<ptam/move_robot.h>
#include<ptam/trajectory_checker.h>
#include<ptam/FrontierExtractor.h>
#include<ptam/VisibilityCalculator.h>
#include<ptam/Frontier_check.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btQuaternion.h>
#include<vector>
#include<cvd/thread.h>
using namespace std;
class waypoints 

{
public:
waypoints(Engine &ep_waypoint);

void robot_pose_callback(const ptam::pos_robot::ConstPtr &msg);

void run();
private:

ros::ServiceClient traj_srv_client;

ptam::trajectory_checker traj_check_srv;
btQuaternion angle_correction_positive;
btQuaternion angle_correction_negative;
float positive_angle_increment,negative_angle_increment;
geometry_msgs::PoseArray pose_corrected;
vector<int>points_outside;
ros::NodeHandle nh;
vector<double>utility;
vector<double>angles;
vector<float>numb_points;
double pitch,roll;
btMatrix3x3 q;
Engine* ep_waypoint;
double robot_angle,robot_x,robot_y,goal_x,goal_y,goal_theta;
ros::Subscriber robot_pose_sub;
ptam::move_robot waypoints_srv;
ros::ServiceClient srv_client;
ros::ServiceClient frontier_srv_client;
ptam::FrontierExtractor frontier_srv;
ptam::Frontier_check frontier_check_srv;
ros::ServiceClient frontier_check_srv_client;
geometry_msgs::PoseArray pose_check;
geometry_msgs::PoseStamped pose_test;
ros::Publisher pub;
double *robot_angle_pointer,*robot_x_pointer,*robot_y_pointer,*goal_x_pointer,*goal_y_pointer,*goal_theta_pointer;
ptam::VisibilityCalculator visib_calc_srv;
ros::ServiceClient visib_calc_srv_client;

};
