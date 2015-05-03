#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>

class map
{

public:
map();
ros::Subscriber sub; 
ros::Publisher pub;
ros::Subscriber sub_visib; 
ros::Publisher pub_visib;



bool map_received_visib_check;
bool map_received_check;
nav_msgs::OccupancyGrid map_received;
nav_msgs::OccupancyGrid map_received_visib;
void poseCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg);
void run();
 ros::NodeHandle n;
};
map::map()
{
 sub=n.subscribe("/map",10,&map::poseCallback,this); 
 pub=n.advertise<nav_msgs::OccupancyGrid>("/mapping",10);
sub_visib=n.subscribe("/map_visib",10,&map::mapCallback,this); 
 pub_visib=n.advertise<nav_msgs::OccupancyGrid>("/mapping_visib",10);


map_received_visib_check=false;
map_received_check=false;
}
void map::poseCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{


map_received.info=msg->info;
map_received.header=msg->header;
map_received.data=msg->data;

map_received_check=true;
//printf("map received\n");

//  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
 // transform.setRotation( tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w ));
 // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot1/odom_tf","/robot1/base"));




}
void map::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{


map_received_visib.info=msg->info;
map_received_visib.header=msg->header;
map_received_visib.data=msg->data;

map_received_visib_check=true;
//printf("map received\n");

//  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
 // transform.setRotation( tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w ));
 // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot1/odom_tf","/robot1/base"));




}


void map::run()
{
while(ros::ok())
{
if(map_received_check)
pub.publish(map_received); 
if(map_received_visib_check)
pub_visib.publish(map_received_visib); 


ros::spinOnce();



}
}
int main(int argc, char **argv)
{

 ros::init(argc, argv, "p3dx_subscriber");
      fprintf(stderr,"what\n");
map m;
m.run();

}
//}
