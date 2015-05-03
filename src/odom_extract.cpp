
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
 nav_msgs::Odometry::ConstPtr out;
int no=0;


void check(const nav_msgs::Odometry::ConstPtr &msg)
{
if(no == 0){
	no =1;
	out = msg;
}
if(out->pose.pose.position.x -  msg->pose.pose.position.x <0.05)
	no++;

if(no>5)
{
	no=1;
out = msg;
std::cerr<<msg->pose.pose.position.x;
 std::cerr<<msg->pose.pose.position.y;
 std::cerr<<msg->pose.pose.position.z;
 std::cerr<<msg->pose.pose.orientation.x;
 std::cerr<<msg->pose.pose.orientation.y;
 std::cerr<<msg->pose.pose.orientation.z;
 std::cerr<<msg->header.stamp;
}
else
	no = 1;
std::cerr<<no;
}


int main(int argc, char** argv) {
 
 ros::init(argc,argv, "o_test");
 ros::NodeHandle nh;
ros::Subscriber odom_sub;
  odom_sub = nh.subscribe("/odom",100,&check);
for(;;)
	ros::spinOnce();
}
