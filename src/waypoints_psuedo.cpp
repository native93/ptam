/* $Revision: 1.1.6.4 $ */
/*
 *	engdemo.cpp
 *
 *	A simple program to illustrate how to call MATLAB
 *	Engine functions from a C++ program.
 *
 * Copyright 1984-2011 The MathWorks, Inc.
 * All rights reserved
 */
#include<ros/ros.h>
#include<ptam/move_robot.h>
#define  BUFSIZE 256
#include<nav_msgs/Odometry.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btQuaternion.h>
#include<ptam/pos_robot.h>
using namespace std;
float robot_x,robot_y,robot_angle;
bool pose_received;
btQuaternion q;
double yaw,pitch,roll;


void robot_pose_callback(const ptam::pos_robot::ConstPtr &msg)
{
robot_x=msg->x;
robot_y=msg->y;

//q=btQuaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
//btMatrix3x3 mat (q);
//mat.getEulerYPR(yaw,pitch,roll);

robot_angle=msg->theta;


printf("%f,%f,%f\n",robot_x,robot_y,robot_angle);
pose_received=true;
}


int main(int argc,char **argv)
{

ros::init(argc,argv,"ways");
//waypoints way;
//way.run();

ros::NodeHandle nh;
ros::ServiceClient srv_client = nh.serviceClient<ptam::move_robot>("/move");
ptam::move_robot waypoints_srv;
ros::Subscriber sub=nh.subscribe("/robot_pose",1,robot_pose_callback);
getchar();

//printf("%f",yaw);

for(int i=0;i<4;i++)
{
pose_received=false;
while(!pose_received)
ros::spinOnce();

fprintf(stderr,"started");
waypoints_srv.request.start_x=robot_x;
waypoints_srv.request.start_y=robot_y;
waypoints_srv.request.start_theta=robot_angle;
if(i==0)
{
waypoints_srv.request.goal_x=2.22;
waypoints_srv.request.goal_y=1.36;
waypoints_srv.request.goal_theta=0.63;
}
if(i==1)
{
waypoints_srv.request.goal_x=-0.49;
waypoints_srv.request.goal_y=-1.77;
waypoints_srv.request.goal_theta=1.18;
}

if(i==2)
{

waypoints_srv.request.goal_x=1.2;
waypoints_srv.request.goal_y = 0.6;
waypoints_srv.request.goal_theta=0.42;
}
if(i==3)
{

waypoints_srv.request.goal_x=-0.82;
waypoints_srv.request.goal_y=0.56;
waypoints_srv.request.goal_theta=-0.82;
}
if(i==4)
{
waypoints_srv.request.goal_x=0;
waypoints_srv.request.goal_y=0;
waypoints_srv.request.goal_theta=0;
}

if(i==4)
{

waypoints_srv.request.goal_x=-1.3;
waypoints_srv.request.goal_y=-1.2;
waypoints_srv.request.goal_theta=1.3;
}
if(i==5)
{

waypoints_srv.request.goal_x=-1.3;
waypoints_srv.request.goal_y=-1.2;
waypoints_srv.request.goal_theta=1.3;
}

if(i==6)
{

waypoints_srv.request.goal_x=-1;
waypoints_srv.request.goal_y=1;
waypoints_srv.request.goal_theta=1.8;
}

if(srv_client.call(waypoints_srv))
printf("waypoint %d moved\n",i);
ros::Duration(5.0).sleep();
}

return;



}











