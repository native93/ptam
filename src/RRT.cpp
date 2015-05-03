#include<ptam/RRT.h>
using namespace std;
using namespace CVD;
RRT::RRT(Engine &ep_RRT):ep(&ep_RRT)
{
start_pos.resize(3);
goal.resize(3);
total_time=0.0;
srv_client = nh.serviceClient<ptam::trajectory_checker>("traj_check");
move_srv_client=nh.serviceClient<ptam::move_robot>("/move");
//	if (!(ep = engOpen(""))) {
//		fprintf(stderr, "\nCan't start MATLAB engine\n");
//		return EXIT_FAILURE;
//	}
printf("RRT Constructor");
pose_received=false;

robot_pose_sub=nh.subscribe<ptam::pos_robot>("/robot_pose",1,&RRT::robot_pose_callback,this);


service=nh.advertiseService("/waypoints",&RRT::service_callback,this);

//printf("%f\n",acc_w_primitive_right[k]);
//for(int k=0;k<8;k++)
//printf("%f\n",acc_w_primitive_left[k]);
path_possible=false;
v_pointer=&v;
w_pointer=&w;
a_pointer=&a;
alpha_pointer=&alpha;
pos_x_pointer=&pos_x;
pos_y_pointer=&pos_y;
pos_theta_pointer=&pos_theta;
t_inc_pointer=&t_inc;
time_pointer=&time;
goal_theta_pointer=&goal_theta;
goal_x_pointer=&goal_x;
goal_y_pointer=&goal_y;

time=0.0;
//run();
start();
}
void RRT::robot_pose_callback(const ptam::pos_robot::ConstPtr &msg)
{
start_pos[0]=msg->x;
start_pos[1]=msg->y;
start_pos[2]=msg->theta;
//printf("%f,%f,%f\n",pos_x,pos_y,theta);

pose_received=true;
}



#if 0
void RRT::run()
{
	if (!(ep = engOpen(""))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
		return EXIT_FAILURE;
	}



mxArray *v_val=NULL,*w_val=NULL,*a_val=NULL,*alpha_val=NULL,*pos_x_val=NULL,*pos_y_val=NULL,*pos_theta_val=NULL,*t_inc_val=NULL,*time_val=NULL,*x_out=NULL,*y_out=NULL,*theta_out=NULL;

v_val= mxCreateDoubleScalar( mxREAL);
w_val= mxCreateDoubleScalar( mxREAL);
alpha_val= mxCreateDoubleScalar( mxREAL);
pos_x_val= mxCreateDoubleScalar( mxREAL);
pos_y_val= mxCreateDoubleScalar( mxREAL);
pos_theta_val= mxCreateDoubleScalar( mxREAL);
t_inc_val= mxCreateDoubleScalar( mxREAL);
time_val= mxCreateDoubleScalar( mxREAL);
a_val= mxCreateDoubleScalar( mxREAL);

double* pos_x_array=new double[64];
double* pos_y_array=new double[64];
double* pos_theta_array=new double[64];




acc_v_primitive.clear();
for(int i=0;i<acc_v_primitive_forward.size();i++)
acc_v_primitive.push_back(acc_v_primitive_forward[i]);

if(goal[2]-start_pos[2]>0.0)
{
acc_w_primitive.clear();
for(int i=0;i<acc_w_primitive_right.size();i++)
acc_w_primitive.push_back(acc_w_primitive_right[i]);
}
else
{
acc_w_primitive.clear();
for(int i=0;i<acc_w_primitive_right.size();i++)
acc_w_primitive.push_back(acc_w_primitive_left[i]);
}
//for(int k=0;k<8;k++)
//printf("%f\n",acc_w_primitive[k]);
//for(int k=0;k<8;k++)
//printf("%f\n",acc_v_primitive[k]);

*pos_x_pointer=start_pos[0];
*pos_y_pointer=start_pos[1];
*pos_theta_pointer=start_pos[2];
*t_inc_pointer=time_inc;

ros::Time start=ros::Time::now();
for(int i=0;i<400;i++)
{

*time_pointer=total_time;
/*
if(vel_t>=v_max)
{
acc_v_primitive.clear();
for(int i=0;i<acc_v_primitive_forward.size();i++)
acc_v_primitive.push_back(acc_v_primitive_backward[i]);
}
else
{

acc_v_primitive.clear();
for(int i=0;i<acc_v_primitive_forward.size();i++)
acc_v_primitive.push_back(acc_v_primitive_forward[i]);
}
if(fabs(u_i/w_i)<10.0)
{
if(goal[2]-start_pos[2]>0.0)
{
acc_w_primitive.clear();
for(int i=0;i<acc_w_primitive_right.size();i++)
acc_w_primitive.push_back(acc_w_primitive_left[i]);
}
else
{
acc_w_primitive.clear();
for(int i=0;i<acc_w_primitive_right.size();i++)
acc_w_primitive.push_back(acc_w_primitive_right[i]);
}
}
*/

u_i=0.0;
w_i=0.0;
counter=0;

memcpy((void *)mxGetPr(pos_x_val), (void *)pos_x_pointer, sizeof(pos_x_pointer));
memcpy((void *)mxGetPr(pos_y_val), (void *)pos_y_pointer, sizeof(pos_y_pointer));
memcpy((void *)mxGetPr(pos_theta_val), (void *)pos_theta_pointer, sizeof(pos_theta_pointer));
memcpy((void *)mxGetPr(t_inc_val), (void *)t_inc_pointer, sizeof(t_inc_pointer));
memcpy((void *)mxGetPr(time_val), (void *)time_pointer, sizeof(time_pointer));
engPutVariable(ep,"pos_x_val",pos_x_val);
engPutVariable(ep,"pos_y_val",pos_y_val);
engPutVariable(ep,"pos_theta_val",pos_theta_val);
engPutVariable(ep,"t_inc_val",t_inc_val);
engPutVariable(ep,"time_val",time_val);

cout<<(ros::Time::now()-start).toSec()<<endl;
for(int j=0;j<acc_v_primitive.size();j++)
{
*v_pointer=u_i+(acc_v_primitive[j]*time_inc);
*a_pointer=acc_v_primitive[j];
memcpy((void *)mxGetPr(a_val), (void *)a_pointer, sizeof(a_pointer));
memcpy((void *)mxGetPr(v_val), (void *)v_pointer, sizeof(v_pointer));
engPutVariable(ep,"a_val",a_val);
engPutVariable(ep,"v_val",v_val);


for(int k=0;k<acc_w_primitive.size();k++)
{
memcpy((void *)mxGetPr(w_val), (void *)w_pointer, sizeof(w_pointer));
memcpy((void *)mxGetPr(alpha_val), (void *)alpha_pointer, sizeof(alpha_pointer));
engPutVariable(ep,"w_val",w_val);
engPutVariable(ep,"alpha_val",alpha_val);


*w_pointer=w_i+(acc_w_primitive[k]*time_inc);
*alpha_pointer=acc_w_primitive[k];

engEvalString(ep,"[x_out y_out theta_out]=fresnel_plot(v_val,a_val,alpha_val,w_val,pos_theta_val,pos_x_val,pos_y_val,time_val,t_inc_val)");

x_out=engGetVariable(ep,"x_out");
y_out=engGetVariable(ep,"y_out");
theta_out=engGetVariable(ep,"theta_out");
pos_x_array[counter]=mxGetScalar(x_out);
pos_y_array[counter]=mxGetScalar(y_out);
pos_theta_array[counter]=mxGetScalar(theta_out);
//printf("%lf,%lf,%lf\n",mxGetScalar(x_out),mxGetScalar(y_out),mxGetScalar(theta_out));

counter+=1;
}

cout<<(ros::Time::now()-start).toSec()<<endl;
}

cout<<(ros::Time::now()-start).toSec()<<endl;
for(int l=0;l<64;l++)
printf("%lf,%lf,%lf\n",pos_x_array[l],pos_y_array[l],pos_theta_array[l]);
getchar();


total_time=total_time+time_inc;
}


}
#else 
bool RRT::service_callback(ptam::move_robot::Request &req,
                ptam::move_robot::Response &res)
{

while(!pose_received)
ros::spinOnce();

//start_pos[0]=req.start_x;start_pos[1]=req.start_y;start_pos[2]=req.start_theta;


goal[0]=req.goal_x;goal[1]=req.goal_y;goal[2]=req.goal_theta;


printf("%lf,%lf,%lf,%lf,%lf,%lf\n",start_pos[0],start_pos[1],start_pos[2],req.goal_x,req.goal_y,req.goal_theta);
//getchar();

mxArray *x_input=NULL,*v_val,*w_val,*y_input=NULL,*theta_input=NULL,*time_input=NULL,*x_out=NULL,*y_out=NULL,*theta_out=NULL,*goal_theta_input=NULL,*v_out=NULL,*w_out=NULL,*goal_x_val=NULL,*goal_y_val=NULL;

x_input= mxCreateDoubleScalar( mxREAL);
y_input= mxCreateDoubleScalar( mxREAL);
theta_input= mxCreateDoubleScalar( mxREAL);
time_input= mxCreateDoubleScalar( mxREAL);
goal_theta_input= mxCreateDoubleScalar( mxREAL);
v_val= mxCreateDoubleScalar( mxREAL);
w_val= mxCreateDoubleScalar( mxREAL);

goal_x_val= mxCreateDoubleScalar( mxREAL);
goal_y_val= mxCreateDoubleScalar( mxREAL);


*goal_x_pointer=goal[0];
*goal_y_pointer=goal[1];
*goal_theta_pointer=goal[2];

*pos_x_pointer=start_pos[0];
*pos_y_pointer=start_pos[1];
*pos_theta_pointer=start_pos[2];
*time_pointer=0.0;
*v_pointer=0.0;
*w_pointer=0.0;

bool done=false;
int sim_time=0;
counter=0;
memcpy((void *)mxGetPr(goal_x_val), (void *)goal_x_pointer, sizeof(goal_x_pointer));
memcpy((void *)mxGetPr(goal_y_val), (void *)goal_y_pointer, sizeof(goal_y_pointer));
engPutVariable(ep,"goal_x_val",goal_x_val);
engPutVariable(ep,"goal_y_val",goal_y_val);

while(!done)

{


memcpy((void *)mxGetPr(x_input), (void *)pos_x_pointer, sizeof(pos_x_pointer));
memcpy((void *)mxGetPr(y_input), (void *)pos_y_pointer, sizeof(pos_y_pointer));
memcpy((void *)mxGetPr(theta_input), (void *)pos_theta_pointer, sizeof(pos_theta_pointer));
memcpy((void *)mxGetPr(time_input), (void *)time_pointer, sizeof(time_pointer));
memcpy((void *)mxGetPr(goal_theta_input), (void *)goal_theta_pointer, sizeof(goal_theta_pointer));
memcpy((void *)mxGetPr(v_val), (void *)v_pointer, sizeof(v_pointer));
memcpy((void *)mxGetPr(w_val), (void *)w_pointer, sizeof(w_pointer));


engPutVariable(ep,"pos_x_val",x_input);
engPutVariable(ep,"pos_y_val",y_input);
engPutVariable(ep,"pos_theta_val",theta_input);
engPutVariable(ep,"time_val",time_input);
engPutVariable(ep,"goal_theta",goal_theta_input);
engPutVariable(ep,"v_val",v_val);
engPutVariable(ep,"w_val",w_val);

if(sim_time==0)
{
engEvalString(ep,"[x_out y_out theta_out v_out w_out]=RRT_cpp(pos_x_val,pos_y_val,pos_theta_val,goal_theta,time_val,v_val,w_val,10)");
sim_time=1;
}
else
engEvalString(ep,"[x_out y_out theta_out v_out w_out]=RRT_cpp(pos_x_val,pos_y_val,pos_theta_val,goal_theta,time_val,v_val,w_val,10)");

x_out=engGetVariable(ep,"x_out");
y_out=engGetVariable(ep,"y_out");
theta_out=engGetVariable(ep,"theta_out");
v_out=engGetVariable(ep,"v_out");
w_out=engGetVariable(ep,"w_out");

engEvalString(ep,"k=motionprof_fov([x_out y_out goal_x_val goal_y_val theta_out goal_theta 3.5])");


//engEvalString(ep,"plot(k(:,2),k(:,3))");
//getchar();
printf("%lf,%lf,%lf\n",mxGetScalar(x_out),mxGetScalar(y_out),mxGetScalar(theta_out));

//engEvalString(ep,"close all");

if(srv_client.call(traj_check_srv))
{
if(traj_check_srv.response.possible)
{
printf("trajectory possible\n");
path_possible=true;
//getchar();
done=true;
}
else
{
counter+=1;
printf("trajectory not possible\n");
//getchar();
}
}
else
{
printf("server did not work\n");
//getchar();
}

*pos_x_pointer=mxGetScalar(x_out);
*pos_y_pointer=mxGetScalar(y_out);
*pos_theta_pointer=mxGetScalar(theta_out);
*v_pointer=mxGetScalar(v_out);
*w_pointer=mxGetScalar(w_out);
time+=1.0;


}

//	engClose(ep);
if(counter!=0)
{



for(int i=0;i<2;i++)
{
if(i==0)
{
pose_received=false;
while(!pose_received)
ros::spinOnce();


move_robot_srv.request.start_x=start_pos[0];move_robot_srv.request.start_y=start_pos[1];move_robot_srv.request.start_theta=start_pos[2];
move_robot_srv.request.goal_x=*pos_x_pointer;move_robot_srv.request.goal_y=*pos_y_pointer;move_robot_srv.request.goal_theta=*pos_theta_pointer;
if(move_srv_client.call(move_robot_srv))
printf("robot_moved");
//getchar();

}
if(i==1)
{pose_received=false;
while(!pose_received)
ros::spinOnce();



move_robot_srv.request.goal_x=goal[0];move_robot_srv.request.goal_y=goal[1];move_robot_srv.request.goal_theta=goal[2];
move_robot_srv.request.start_x=*pos_x_pointer;move_robot_srv.request.start_y=*pos_y_pointer;move_robot_srv.request.start_theta=*pos_theta_pointer;
if(move_srv_client.call(move_robot_srv))
printf("robot_moved");
//getchar();

}
}
}
else
{

pose_received=false;
while(!pose_received)
ros::spinOnce();



printf("trajectory possible all along");
move_robot_srv.request.goal_x=goal[0];move_robot_srv.request.goal_y=goal[1];move_robot_srv.request.goal_theta=goal[2];
move_robot_srv.request.start_x=start_pos[0];move_robot_srv.request.start_y=start_pos[1];move_robot_srv.request.start_theta=start_pos[2];
if(move_srv_client.call(move_robot_srv))
printf("robot_moved");
//getchar();


}

return (true);

}
void RRT::run()
{
while(ros::ok)
{
ros::spinOnce();
}
}

#endif
/*
int main(int argc,char **argv)
{

ros::init(argc,argv,"RRT");
RRT rrt;
ros::spin();

//rrt.run();
return(1);
}
*/
