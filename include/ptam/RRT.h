#include<iostream>
#include<vector>
#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include "engine.h"
#include<string.h>
#include<ros/ros.h>
#include<ptam/pos_robot.h>
#include<ptam/move_robot.h>
#include<ptam/trajectory_checker.h>
#include<cvd/thread.h>

using namespace std;
class RRT :protected CVD::Thread
{
public:
RRT(Engine &ep_waypoint);
virtual void run();
void robot_pose_callback(const ptam::pos_robot::ConstPtr &msg);

private:

ros::Subscriber robot_pose_sub; 
ros::NodeHandle nh;
ros::ServiceClient srv_client;
bool pose_received;
bool service_callback(
                ptam::move_robot::Request &req,
                ptam::move_robot::Response &res);

ros::ServiceServer service;
vector<double>start_pos;
vector<double>goal;
vector<double>acc_v_primitive_forward;
vector<double>acc_v_primitive_backward;
vector<double>acc_w_primitive_right;
vector<double>acc_w_primitive_left;
double v_max;
double v_min;
double w_max;
double w_min;
double time_inc;
double vel_t;
double vel_w;
double u_i;
double w_i,total_time;
bool path_possible;

Engine *ep;
ptam::trajectory_checker traj_check_srv;
ros::ServiceClient move_srv_client;

ptam::move_robot move_robot_srv;

double v,w,a,alpha,pos_x,pos_y,pos_theta,t_inc,time,goal_theta,goal_x,goal_y; 
int counter;
double *v_pointer,*w_pointer,*a_pointer,*alpha_pointer,*pos_x_pointer,*pos_y_pointer,*pos_theta_pointer,*t_inc_pointer,*time_pointer,*goal_theta_pointer,*goal_x_pointer,*goal_y_pointer; 

//mxArray *v_val,*w_val,*a_val,*alpha_val,*pos_x_val,*pos_y_val,*pos_theta_val,*t_inc_val,*time_val; 
vector<double>acc_v_primitive;
vector<double>acc_w_primitive;




};
