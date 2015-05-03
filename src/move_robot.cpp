#include<iostream>
#include<ptam/move_robot_server.h>
#include<string>
using namespace std;
using namespace CVD;
move_robot::move_robot(Engine &ep): ep_traj(&ep)
{
service=nh.advertiseService("/move",&move_robot::cal_traj,this);
pub_vel=nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);


start_x_pointer=&start_x;
start_y_pointer=&start_y;
start_theta_pointer=&start_theta;
goal_x_pointer=&goal_x;
goal_y_pointer=&goal_y;
goal_theta_pointer=&goal_theta;
//	if (!(ep_traj = engOpen(""))) {
//		fprintf(stderr, "\nCan't start MATLAB engine\n");
//	}

start();
}
bool move_robot::cal_traj(
                ptam::move_robot::Request &req,
                ptam::move_robot::Response &res)
{



mxArray *x_input=NULL,*y_input=NULL,*theta_input=NULL,*x_out=NULL,*y_out=NULL,*theta_out=NULL;

x_input= mxCreateDoubleScalar( mxREAL);
y_input= mxCreateDoubleScalar( mxREAL);
theta_input= mxCreateDoubleScalar( mxREAL);
x_out= mxCreateDoubleScalar( mxREAL);
y_out= mxCreateDoubleScalar( mxREAL);
theta_out= mxCreateDoubleScalar( mxREAL);

printf("%lf,%lf,%lf,%lf,%lf,%lf\n",req.start_x,req.start_y,req.start_theta,req.goal_x,req.goal_y,req.goal_theta);
//getchar();
*start_x_pointer=req.start_x;
*start_y_pointer=req.start_y;
*start_theta_pointer=req.start_theta;
*goal_x_pointer=req.goal_x;
*goal_y_pointer=req.goal_y;
*goal_theta_pointer=req.goal_theta;

memcpy((void *)mxGetPr(x_input),(void *)start_x_pointer,sizeof(start_x_pointer));
memcpy((void *)mxGetPr(y_input), (void *)start_y_pointer, sizeof(start_y_pointer));
memcpy((void *)mxGetPr(theta_input), (void *)start_theta_pointer, sizeof(start_theta_pointer));
memcpy((void *)mxGetPr(x_out),(void *)goal_x_pointer,sizeof(goal_x_pointer));
memcpy((void *)mxGetPr(y_out), (void *)goal_y_pointer, sizeof(goal_y_pointer));
memcpy((void *)mxGetPr(theta_out), (void *)goal_theta_pointer, sizeof(goal_theta_pointer));

engPutVariable(ep_traj,"pos_x_val",x_input);
engPutVariable(ep_traj,"pos_y_val",y_input);
engPutVariable(ep_traj,"pos_theta_val",theta_input);
engPutVariable(ep_traj,"goal_x_val",x_out);
engPutVariable(ep_traj,"goal_y_val",y_out);
engPutVariable(ep_traj,"goal_theta_val",theta_out);


engEvalString(ep_traj,"k=motionprof_fov([pos_x_val pos_y_val goal_x_val goal_y_val pos_theta_val goal_theta_val 3.5])");

//engEvalString(ep_traj,"plot(k(:,2),k(:,3))");
//getchar();
std::ifstream file("/home/sarthak/Downloads/RRT/velocity_profile.csv");
velocity.linear.y=0.0;

velocity.linear.z=0.0;
velocity.angular.y=0.0;
unsigned found;
velocity.angular.x=0.0;

std::string str2 (",");
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

pub_vel.publish(velocity);
ros::Duration(0.1).sleep();

}
file.close();

//	engClose(ep_traj);
return true;
}

void move_robot::run()
{
while(ros::ok)
{
ros::spinOnce();
//printf("thread");
}
}
/*
int main(int argc,char **argv)
{

ros::init(argc,argv,"move_robots");
move_robot move;
while(ros::ok)
ros::spinOnce();

return 0;


}

*/

