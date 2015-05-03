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
#include<ptam/waypoints.h>
#define  BUFSIZE 256
using namespace CVD;
using namespace std;
waypoints::waypoints(Engine &ep) : ep_waypoint(&ep)
{
robot_pose_sub=nh.subscribe<ptam::pos_robot>("/robot_pose",1,&waypoints::robot_pose_callback,this);
srv_client = nh.serviceClient<ptam::move_robot>("/waypoints");
frontier_srv_client = nh.serviceClient<ptam::FrontierExtractor>("/FrontierExtractor");
traj_srv_client = nh.serviceClient<ptam::trajectory_checker>("traj_check");

visib_calc_srv_client=nh.serviceClient<ptam::VisibilityCalculator>("VisibilityCalculator");
frontier_check_srv_client = nh.serviceClient<ptam::Frontier_check>("/Frontier_check");
pub=nh.advertise<geometry_msgs::PoseStamped>("/goal",1);
pose_test.header.frame_id="world";
pose_check.header.frame_id="world";
robot_angle_pointer=&robot_angle;
robot_x_pointer=&robot_x;
robot_y_pointer=&robot_y;
goal_theta_pointer=&goal_theta;
goal_x_pointer=&goal_x;
goal_y_pointer=&goal_y;
//	if (!(ep_waypoint = engOpen(""))) {
//		fprintf(stderr, "\nCan't start MATLAB engine\n");
//		return EXIT_FAILURE;
//	}
//start();
}

void waypoints::robot_pose_callback(const ptam::pos_robot::ConstPtr &msg)
{
robot_x=msg->x;
robot_y=msg->y;
robot_angle=msg->theta;
//printf("%f,%f,%f\n",robot_x,robot_y,robot_angle);
}

void waypoints::run()
{
getchar();

/*
pose_test.pose.position.x=3.7;pose_test.pose.position.y=1.3;pose_test.pose.position.z=0.0;
btQuaternion rot_test(3.0,0,0);
pose_test.pose.orientation.x=rot_test[0];
pose_test.pose.orientation.y=rot_test[1];

pose_test.pose.orientation.z=rot_test[2];


pose_test.pose.orientation.w=rot_test[3];

frontier_check_srv.request.Frontiers.poses.push_back(pose_test.pose);

if(frontier_check_srv_client.call(frontier_check_srv))
{
printf("%d\n",frontier_check_srv.response.points_visible[0]);

}
getchar();
*/

while(ros::ok())
{

frontier_check_srv.request.Frontiers.poses.clear();
visib_calc_srv.request.ugv_frontiers.poses.clear();
points_outside.clear();
utility.clear();
angles.clear();
numb_points.clear();
pose_check.poses.clear();
mxArray *x_input=NULL,*y_input=NULL,*theta_input=NULL,*goal_theta_input=NULL,*goal_x_val=NULL,*goal_y_val=NULL,*T=NULL;


x_input= mxCreateDoubleScalar( mxREAL);
y_input= mxCreateDoubleScalar( mxREAL);
theta_input= mxCreateDoubleScalar( mxREAL);
goal_theta_input= mxCreateDoubleScalar( mxREAL);
goal_x_val= mxCreateDoubleScalar( mxREAL);
goal_y_val= mxCreateDoubleScalar( mxREAL);


ros::spinOnce();
//getchar();

//printf("inside run\n");


float distance;
btQuaternion rot_frontier;
if(frontier_srv_client.call(frontier_srv))
{
positive_angle_increment=(1.3-(robot_angle))/3.0;

negative_angle_increment=(-1.3-(robot_angle))/3.0;
printf("angle increment %f,%f\n",positive_angle_increment,negative_angle_increment);
float angle_change_positive=0.0;

float angle_change_negative=0.0;

for(int l=1;l<4;l++)
{

angle_change_positive+=positive_angle_increment;
angle_change_negative+=negative_angle_increment;


angle_correction_positive=btQuaternion(robot_angle+angle_change_positive,0.0,0.0);
angle_correction_negative=btQuaternion(robot_angle+angle_change_negative,0.0,0.0);
for (int i=0;i<frontier_srv.response.ugv_frontiers.poses.size();i++)
{
pose_test.pose=frontier_srv.response.ugv_frontiers.poses[i];
pose_test.pose.orientation.x=angle_correction_positive[0];
pose_test.pose.orientation.y=angle_correction_positive[1];
pose_test.pose.orientation.z=angle_correction_positive[2];
pose_test.pose.orientation.w=angle_correction_positive[3];
frontier_check_srv.request.Frontiers.poses.push_back(pose_test.pose);
visib_calc_srv.request.ugv_frontiers.poses.push_back(pose_test.pose);
pose_test.pose.orientation.x=angle_correction_negative[0];
pose_test.pose.orientation.y=angle_correction_negative[1];
pose_test.pose.orientation.z=angle_correction_negative[2];
pose_test.pose.orientation.w=angle_correction_negative[3];
frontier_check_srv.request.Frontiers.poses.push_back(pose_test.pose);
visib_calc_srv.request.ugv_frontiers.poses.push_back(pose_test.pose);

}


//printf("%f,%f\n",robot_angle+angle_change_positive,robot_angle+angle_change_negative);
}
//frontier_check_srv.request.Frontiers.header.frame_id="world";
//getchar();
//if(frontier_check_srv_client.call(frontier_check_srv))

if(visib_calc_srv_client.call(visib_calc_srv))
{

if(frontier_check_srv_client.call(frontier_check_srv))
{

}

/*
for(int i=0;i<visib_calc_srv.response.ugv_frontier_visibility.size();i++)
{

printf("visib:%f,%d\n",visib_calc_srv.response.ugv_frontier_visibility[i],i);
pose_test.pose=visib_calc_srv.request.ugv_frontiers.poses[i];
pose_test.header.frame_id="world";
pub.publish(pose_test);
getchar();

}
*/
for(int i=0;i<visib_calc_srv.response.ugv_frontier_visibility.size();i++)
{

//printf("visib:%f\n",visib_calc_srv.response.ugv_frontier_visibility[i]);
//pose_test.pose=visib_calc_srv.request.ugv_frontiers.poses[i];
//pose_test.header.frame_id="world";
rot_frontier=btQuaternion(btScalar(visib_calc_srv.request.ugv_frontiers.poses[i].orientation.x),btScalar(visib_calc_srv.request.ugv_frontiers.poses[i].orientation.y),btScalar(visib_calc_srv.request.ugv_frontiers.poses[i].orientation.z),btScalar(visib_calc_srv.request.ugv_frontiers.poses[i].orientation.w));
q=btMatrix3x3(rot_frontier);
q.getEulerZYX(goal_theta,pitch,roll);


//getchar();
printf("%f,%f,%lf,%f\n",visib_calc_srv.request.ugv_frontiers.poses[i].position.x,visib_calc_srv.request.ugv_frontiers.poses[i].position.y,goal_theta,(float)frontier_check_srv.response.points_visible[i]/(float)frontier_check_srv.response.total_points);
//printf("%d,%d\n",frontier_check_srv.response.points_visible[i],frontier_check_srv.response.total_points);
if((float)frontier_check_srv.response.points_visible[i]/(float)frontier_check_srv.response.total_points>0.2&&(float)frontier_check_srv.response.points_visible[i]/(float)frontier_check_srv.response.total_points<0.6)
{

//printf("%d\n",frontier_check_srv.response.points_visible[i]);

//printf("angle %lf,%lf,%f,%f\n",rot_frontier.getAngle(),fabs(rot_frontier.getAngle()-robot_angle),frontier_srv.response.ugv_frontiers.poses[i].position.x,frontier_srv.response.ugv_frontiers.poses[i].position.y);
if(fabs(goal_theta-robot_angle)<0.25||fabs(goal_theta)>1.5||fabs(goal_theta-robot_angle)>1.8)///angle constraint
continue;

distance=sqrt(((robot_x-visib_calc_srv.request.ugv_frontiers.poses[i].position.x)*(robot_x-visib_calc_srv.request.ugv_frontiers.poses[i].position.x))+((robot_y-visib_calc_srv.request.ugv_frontiers.poses[i].position.y)*(robot_y-visib_calc_srv.request.ugv_frontiers.poses[i].position.y)));
//printf("distance %f\n",distance);

if(distance<1.5)///distance constraint
continue;


//pose_test.pose=frontier_srv.response.ugv_frontiers.poses[i];
pose_check.poses.push_back(visib_calc_srv.request.ugv_frontiers.poses[i]);
//pub.publish(pose_test);
//getchar();
//printf("visib %f\n",visib_calc_srv.response.ugv_frontier_visibility[i]);
numb_points.push_back(visib_calc_srv.response.ugv_frontier_visibility[i]);
}
}
}
}

//pub.publish(pose_check);
//getchar();
memcpy((void *)mxGetPr(x_input), (void *)robot_x_pointer, sizeof(robot_x_pointer));
memcpy((void *)mxGetPr(y_input), (void *)robot_y_pointer, sizeof(robot_y_pointer));
memcpy((void *)mxGetPr(theta_input), (void *)robot_angle_pointer, sizeof(robot_angle_pointer));
engPutVariable(ep_waypoint,"pos_x_val",x_input);
engPutVariable(ep_waypoint,"pos_y_val",y_input);
engPutVariable(ep_waypoint,"pos_theta_val",theta_input);

for(int i=0;i<pose_check.poses.size();i++)
{
goal_x=pose_check.poses[i].position.x;
goal_y=pose_check.poses[i].position.y;

q=btMatrix3x3(btQuaternion(pose_check.poses[i].orientation.x,pose_check.poses[i].orientation.y,pose_check.poses[i].orientation.z,pose_check.poses[i].orientation.w));
q.getEulerYPR(goal_theta,pitch,roll);

printf("%lf,%lf,%lf\n",goal_x,goal_y,goal_theta);
memcpy((void *)mxGetPr(goal_x_val), (void *)goal_x_pointer, sizeof(goal_x_pointer));
memcpy((void *)mxGetPr(goal_y_val), (void *)goal_y_pointer, sizeof(goal_y_pointer));
memcpy((void *)mxGetPr(goal_theta_input), (void *)goal_theta_pointer, sizeof(goal_theta_pointer));
engPutVariable(ep_waypoint,"goal_x_val",goal_x_val);
engPutVariable(ep_waypoint,"goal_y_val",goal_y_val);
engPutVariable(ep_waypoint,"goal_theta_val",goal_theta_input);
//printf("%lf,%lf,%lf,%lf,%lf,%lf\n",*robot_x_pointer,*robot_y_pointer,*robot_angle_pointer,*goal_x_pointer,*goal_y_pointer,*goal_theta_pointer);


angles.push_back(goal_theta);



engEvalString(ep_waypoint,"[k distance]=motionprof_fov([pos_x_val pos_y_val goal_x_val goal_y_val pos_theta_val goal_theta_val 3.5])");
//engEvalString(ep_waypoint,"plot(k(:,2),k(:,3))");

T=engGetVariable(ep_waypoint,"distance");
printf("dist%lf,visib %f\n",mxGetScalar(T),numb_points[i]);
if(traj_srv_client.call(traj_check_srv))
{

printf("points outside:%d\n",traj_check_srv.response.points);
points_outside.push_back(traj_check_srv.response.points);
if(traj_check_srv.response.points>150)
utility.push_back(-100.0);
else
utility.push_back(numb_points[i]/mxGetScalar(T));
}


}
double max=-1000;
int index=-1;
for(int i=0;i<utility.size();i++)
{

//printf("utility:%lf\n",utility[i]);
if(utility[i]>max)
{
max=utility[i];
index=i;
}
}
if (max==-100)
{
max=10000;
index=-1;


for(int i=0;i<points_outside.size();i++)
{
//printf("utility:%lf\n",points_outside[i]);
if(points_outside[i]<max)
{
max=points_outside[i];
index=i;
}
}
}

printf("visibility %f,%f\n",numb_points[index],utility[index]);


printf("max:%lf,index:%d\n",max,index);
pose_test.pose=pose_check.poses[index];
pose_test.header.frame_id="world";
pub.publish(pose_test);



//getchar();


waypoints_srv.request.start_x=robot_x;waypoints_srv.request.start_y=robot_y;waypoints_srv.request.start_theta=robot_angle;
waypoints_srv.request.goal_x=pose_check.poses[index].position.x;waypoints_srv.request.goal_y=pose_check.poses[index].position.y;waypoints_srv.request.goal_theta=angles[index];
if(srv_client.call(waypoints_srv))
printf("waypoint moved");

getchar();
}
}

//int main(int argc,char **argv)
//{

//ros::init(argc,argv,"waypoints");
//waypoints way;
//way.run();

/*
//pose_check.poses.resize(1);
geometry_msgs::Pose test;

test.position.x=0.0;

test.position.y=0.0;
test.position.z=0.0;

btQuaternion rot(1.25,0,0);
test.orientation.x=rot[0];

test.orientation.y=rot[1];
test.orientation.z=rot[2];
test.orientation.w=rot[3];
pose_check.poses.push_back(test);
test.position.x=3.0;

test.position.y=-1.3;
test.position.z=0.0;

 rot=btQuaternion(-2.34,0,0);
test.orientation.x=rot[0];

test.orientation.y=rot[1];
test.orientation.z=rot[2];
test.orientation.w=rot[3];
pose_check.poses.push_back(test);








pose_check.header.frame_id="world";
frontier_check_srv.request.Frontiers=pose_check;
if(frontier_check_srv_client.call(frontier_check_srv))
{
printf("%d,%d\n",frontier_check_srv.response.points_visible.size(),frontier_check_srv.response.total_points);
for(int i=0;i<frontier_check_srv.response.points_visible.size();i++)
printf("%d\n",frontier_check_srv.response.points_visible[i]);

getchar();


}
*/
//while(1)
//pub.publish(pose_check);

//test.orientation=tf::Quaternion(1.573,0,0);


/*
pose_check.poses[0].position.x=0.0;

pose_check.poses[0].position.y=0.0;
pose_check.poses[0].position.z=0.0;
pose_check.poses[0].orientation.z=0.0;
pose_check.poses[0].orientation.x=0.0;
pose_check.poses[0].orientation.y=0.0;
pose_check.poses[0].orientation.w=1.0;
pose_check.header.frame_id="/world";



*/







//pose_check.header.frame_id="world";
//while(1)
//pub.publish(pose_check);

//ros::Subscriber robot_pose_sub=nh.subscribe<ptam::pos_robot>("/robot_pose",1,robot_pose_callback);
/*
for(int i=0;i<4;i++)
{
if(i==0)
{
waypoints_srv.request.start_x=0.0;waypoints_srv.request.start_y=0.0;waypoints_srv.request.start_theta=0.0;
waypoints_srv.request.goal_x=3.6;waypoints_srv.request.goal_y=1.2;waypoints_srv.request.goal_theta=0.34;
if(srv_client.call(waypoints_srv))
printf("waypoint 1 moved");

}
if(i==1)
{
waypoints_srv.request.start_x=3.6;waypoints_srv.request.start_y=1.2;waypoints_srv.request.start_theta=0.34;
waypoints_srv.request.goal_x=1.2;waypoints_srv.request.goal_y=-0.8;waypoints_srv.request.goal_theta=1.02;
if(srv_client.call(waypoints_srv))
printf("waypoint 2 moved");
}
if(i==2)
{
waypoints_srv.request.start_x=1.2;waypoints_srv.request.start_y=-0.8;waypoints_srv.request.start_theta=1.02;
waypoints_srv.request.goal_x=0.64;waypoints_srv.request.goal_y=0.34;waypoints_srv.request.goal_theta=-0.45;
if(srv_client.call(waypoints_srv))
printf("waypoint 3 moved");
}
if(i==3)
{
waypoints_srv.request.start_x=2.5;waypoints_srv.request.start_y=0.34;waypoints_srv.request.start_theta=-0.45;
waypoints_srv.request.goal_x=2.5;waypoints_srv.request.goal_y=-0.3;waypoints_srv.request.goal_theta=-1.34;
if(srv_client.call(waypoints_srv))
printf("waypoint 4 moved");
}



*/



//}





//}








