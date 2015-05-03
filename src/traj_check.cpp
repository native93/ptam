#include<iostream>
#include<ptam/traj_checker_server.h>
#include<string>
using namespace std;
traj_check::traj_check()
{
service=nh.advertiseService("traj_check",&traj_check::check_trajectory,this);
map_sub=nh.subscribe("/mapping",1,&traj_check::map_callback,this);
map_received=false;
pub_bad=nh.advertise<geometry_msgs::PoseStamped>("/points_bad",10);
pub_good=nh.advertise<geometry_msgs::PoseStamped>("/points_good",10);
pose.header.frame_id="world";
pose.pose.position.z=0.0;
pose.pose.orientation.x=0.0;
pose.pose.orientation.y=0.0;
pose.pose.orientation.z=0.0;
pose.pose.orientation.w=1.0;



}
bool traj_check::check_trajectory(
                ptam::trajectory_checker::Request &req,
                ptam::trajectory_checker::Response &res)
{
//printf("time to check motherfucker\n");

//map_received=false;
res.possible=false;
//while(!map_received)
ros::spinOnce();
counter=0;
//printf("%d,%d",map_msg.info.width,map_msg.info.height);
std::ifstream file("/home/sarthak/Downloads/RRT/traj_profile.csv");
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
pose.pose.position.y=atof(numbers.c_str());

}
if(k==0)
{
pose.pose.position.x=atof(numbers.c_str());
}
value=value.substr(found+1);

}
//printf("%f,%f\n",pose.pose.position.x,pose.pose.position.y);
            cur_point_x = (int)(pose.pose.position.x / map_msg.info.resolution) + map_msg.info.width/2;
            cur_point_y = (int)(pose.pose.position.y / map_msg.info.resolution) + map_msg.info.height/2;
       if(cur_point_x >= 0 && cur_point_x < map_msg.info.width && cur_point_y>=0 && cur_point_y< map_msg.info.height) 
{
if(map_msg.data[cur_point_y * map_msg.info.width + cur_point_x]!=0)
{
//pub_bad.publish(pose);
counter+=1;

}
}
//else
//pub_good.publish(pose);

//ros::Duration(0.1).sleep();
//getchar();


}

res.points=counter;
printf("points fucking us%d\n",counter);
if(counter>50)
res.possible=false;
else
res.possible=true;

file.close();

map_received=false;


return true;
}



void traj_check::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
map_received=true;
map_msg=*msg;
}
int main(int argc,char **argv)
{

ros::init(argc,argv,"readin_l1");
traj_check check;
while(ros::ok)
ros::spinOnce();

return 0;

/*
*/
}


