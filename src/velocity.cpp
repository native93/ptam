#include<iostream>
#include<fstream>
#include<ros/ros.h>
#include<string>
#include<geometry_msgs/Twist.h>
using namespace std;
int main(int argc,char **argv)
{

ros::init(argc,argv,"readin");

ros::NodeHandle nh;
std::ifstream file("/home/irl/RRT/file_vel.csv");
std::string value;
std::string numbers;
float v,w;
ros::Publisher pub;
pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
geometry_msgs::Twist velocity;
velocity.linear.y=0.0;

velocity.linear.z=0.0;
velocity.angular.y=0.0;

unsigned found;
velocity.angular.x=0.0;

std::string str2 (",");
while ( getline ( file, value ))
{
 
for(int i=0;i<2;i++)
{


found = value.find(str2);

numbers=value.substr(0,found);

cout<<numbers<<endl;
if(i==0)
{
v=stof(numbers);
velocity.linear.x=v;
}
if(i==1)
{
w=stof(numbers);
velocity.angular.z=w;
}
value=value.substr(found+1);

}
//printf("%f,%f\n",v,w);
//getchar();

pub.publish(velocity);
ros::Duration(0.1).sleep();


}

}


