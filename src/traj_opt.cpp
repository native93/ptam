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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "engine.h"
#include<iostream>
#include<fstream>

#include<string>
#include<ros/ros.h>
#include<ptam/pos_robot.h>
#include<geometry_msgs/Twist.h>
#include<ptam/trajectory_checker.h>
#define  BUFSIZE 256
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
 
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
     
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
    ch = getchar();
 
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
 
    if(ch !=EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
    }

using namespace std;
double pos_x,pos_y,theta,rho;
double* pos_x_pointer;
double* pos_y_pointer;
double* theta_pointer;
double* rho_pointer;

int check = 0;

void robot_pose_callback(const ptam::pos_robot::ConstPtr &msg)
{
*pos_x_pointer=msg->x;
*pos_y_pointer=msg->y;
*theta_pointer=msg->theta;
printf("%f,%f,%f\n",pos_x,pos_y,theta);
check = 1;
}

int main(int argc,char **argv)
{

ros::init(argc,argv,"readin");
ros::NodeHandle nh;
pos_x_pointer=&pos_x;
pos_y_pointer=&pos_y;
theta_pointer=&theta;
rho_pointer=&rho;
ptam::trajectory_checker traj_check_srv;
ros::ServiceClient srv_client;
srv_client = nh.serviceClient<ptam::trajectory_checker>("traj_check");

*rho_pointer=3.5;


printf("rho %lf\n",rho);
ros::Subscriber robot_pose_sub=nh.subscribe<ptam::pos_robot>("/robot_pose",1,robot_pose_callback);

	Engine *ep = NULL;
	mxArray *T = NULL, *result = NULL,*pos_x_val=NULL,*pos_y_val=NULL,*theta_val=NULL,*rho_val=NULL;
//	char buffer[BUFSIZE+1];
//	double time[10] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
double* data;
size_t rows;
size_t cols;
std::cerr<<"all ";
	if (!(ep = engOpen(""))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
		return EXIT_FAILURE;
	}
std::cerr<<"good";

std::string value;
std::string numbers;
float v,w;
ros::Publisher pub;
pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
geometry_msgs::Twist velocity;
getchar();
bool path_possible=false;

std::stringstream ss;

while(1){
ss<<1;
std::cerr<<ss.str();
std::ofstream f(ss.str().c_str());

while(1){
	ros::spinOnce();
	if(check){
		f<<*pos_x_pointer<<","<<*pos_y_pointer<<"\n";
		check = 0;
if (kbhit()) break;
	}
}

f.close();
getchar();
}
for(int i=0;i<0;i++)
{
pos_x_val = mxCreateDoubleScalar( mxREAL);
pos_y_val = mxCreateDoubleScalar( mxREAL);
theta_val = mxCreateDoubleScalar( mxREAL);
rho_val=mxCreateDoubleScalar( mxREAL);

//while(ros::ok())
//{
ros::spinOnce();
//memcpy((void*)mxGetPr(pos_x_val),(void*)pos_x_pointer,sizeof(pos_x_pointer));
memcpy((void *)mxGetPr(pos_x_val), (void *)pos_x_pointer, sizeof(pos_x_pointer));
memcpy((void *)mxGetPr(pos_y_val), (void *)pos_y_pointer, sizeof(pos_y_pointer));
memcpy((void *)mxGetPr(theta_val), (void *)theta_pointer, sizeof(theta_pointer));
memcpy((void *)mxGetPr(rho_val), (double *)rho_pointer, sizeof(rho_pointer));

engPutVariable(ep,"pos_x_val",pos_x_val);
engPutVariable(ep,"pos_y_val",pos_y_val);
engPutVariable(ep,"theta_val",theta_val);
engPutVariable(ep,"rho_val",rho_val);

//engEvalString(ep,"k=test_cpp(rho_val,rho_val)");

//T=engGetVariable(ep,"k");
//rows=mxGetM(T);
//cols=mxGetN(T);
//data=mxGetPr(T);
//printf("%ld,%d\n",rows,cols);
//for(int i=0;i<rows*cols;i++)
//printf("%lf,%lf\n",data[0],data[1]);
//getchar();
//}
#if 0
if(i==0)
engEvalString(ep,"k=motionprof_fov([pos_x_val pos_y_val 3.6 1.2 theta_val .34833 3.5])");
else if(i==1)
engEvalString(ep,"k=motionprof_fov([pos_x_val pos_y_val 1.2 0.0 theta_val 1.04 3.5])");
else if(i==2)
engEvalString(ep,"k=motionprof_fov([pos_x_val pos_y_val -0.8 -0.8 theta_val .34833 3.5])");
else if(i==3)
engEvalString(ep,"k=motionprof_fov([pos_x_val pos_y_val 3.0 -1.2 theta_val -0.215 3.5])");
else if(i==4)
engEvalString(ep,"k=motionprof_fov([pos_x_val pos_y_val 1.2 -0.4 theta_val 0.0 3.5])");
#else 

if(i==0)
engEvalString(ep,"k=motionprof_fov([0 0 1.8 -.7883 0.0 -0.65 3.5])");
else if(i==1)
engEvalString(ep,"k=motionprof_fov([1.8 -0.7883 3.6 1.2 -0.65 0.34 3.5])");
else if(i==2)
engEvalString(ep,"k=motionprof_fov([3.6 1.2 1.2 -0.8 .34833 1.04 3.5])");
else if(i==3)
engEvalString(ep,"k=motionprof_fov([1.2 -0.8 .64 .34 1.04 -0.45 3.5])");
else if(i==4)
engEvalString(ep,"k=motionprof_fov([pos_x_val pos_y_val 1.2 -0.4 theta_val 0.0 3.5])");
#endif


engEvalString(ep,"plot(k(:,2),k(:,3))");
getchar();
/*
while(!path_possible)
{

engEvalString(ep,"[k rho]=motionprof_fov([0.0 0.0 1.7716 -0.8 0.0 -0.61 rho_val])");


T=engGetVariable(ep,"rho");
double complete=mxGetScalar(T);

printf("complete:%lf\n",complete);
//getchar();

if(srv_client.call(traj_check_srv))
{
if(traj_check_srv.response.possible)
{
printf("trajectory possible\n");
path_possible=true;
getchar();

}
else
{
printf("trajectory not possible\n");
getchar();
}
}
else
{
printf("server did not work\n");
getchar();
}
*rho_pointer=*rho_pointer-10.0;
memcpy((void *)mxGetPr(rho_val), (double *)rho_pointer, sizeof(rho_pointer));

engPutVariable(ep,"rho_val",rho_val);
}
*/



//engEvalString(ep,"k=motionprof_fov([0.0;0.0;2.4;0.4;0.0;.3422;3.5])");
//engEvalString(ep,"k=motionprof_fov([2.4;0.4;2.0;0.0;.3422;1.04;1.5])");
//engEvalString(ep,"plot(k(:,2),k(:,3))");
//double complete=mxGetScalar(T);
//printf("complete:%lf\n",complete);
//getchar();

//if(srv_client.call(traj_check_srv))
//{
//if(traj_check_srv.response.possible)
//printf("trajectory possible\n");
//}






//	return EXIT_SUCCESS;

std::ifstream file("/home/sarthak/Desktop/velocity_profile.csv");
velocity.linear.y=0.0;

velocity.linear.z=0.0;
velocity.angular.y=0.0;

unsigned found;
velocity.angular.x=0.0;

std::string str2 (",");
#if 1
while ( getline ( file, value ))
{
 
for(int k=0;k<2;k++)
{


found = value.find(str2);

numbers=value.substr(0,found);

//cout<<numbers<<endl;
if(k==1)
{
v=atof(numbers.c_str());
velocity.linear.x=v;
}
if(k==0)
{
w=atof(numbers.c_str());
velocity.angular.z=w;
}
value=value.substr(found+1);

}
//printf("%f,%f\n",v,w);
//getchar();

pub.publish(velocity);
ros::Duration(0.1).sleep();

}
//getchar();
#endif
file.close();
engEvalString(ep,"close all");
printf("Done!\n");
}
	mxDestroyArray(result);
	engClose(ep);
	
}








