#include<iostream>
#include<fstream>
#include <pcl/io/pcd_io.h>
#include<ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/ros/conversions.h>
#include "pcl_ros/point_cloud.h"
#include<string>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btQuaternion.h>
#include <tf/transform_broadcaster.h>
#include<TooN/TooN.h>
#include<string>
#include<TooN/so3.h>

#include<TooN/se3.h>
using namespace std;
using namespace TooN;
int main(int argc,char **argv)
{

ros::init(argc,argv,"readin");

ros::NodeHandle nh;
std::string value;
std::string numbers;

std::ifstream file1("/home/irl/workspace/ptam/bin/maps_test/RandT_1.info");
float x,y,z;
int color;
unsigned found;
float point_x;
pcl::PointXYZRGB point;
pcl::PointCloud<pcl::PointXYZRGB> cloud_planar;
ros::Publisher pub;
pub=nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pointcloud_bitches",1000);
float rot_array[12];
int count=0;
while ( getline ( file1, value ))
{

      // read a string until next comma: http://www.cplusplus.com/reference/string/getline/

//   cout << string( value)<<endl; // display value removing the first and the last character from it

  // different member versions of find in the same order as above:

std::string str2 (",");

for(int i=0;i<4;i++)
{


found = value.find(str2);

numbers=value.substr(0,found);
#if 0
//cout<<numbers<<endl;
if(i==0)
point_trans[0]=stof(numbers);
if(i==1)
point_trans[1]=stof(numbers);
if(i==2)
point_trans[2]=stof(numbers);

point_trans[3]=1.0;

point_transformed=transform*point_trans;

point.x=point_transformed[0]/point_transformed[3];

point.y=point_transformed[1]/point_transformed[3];
point.z=point_transformed[2]/point_transformed[3];

value=value.substr(found+1);
#endif
if(i==0)
{
rot_array[count*4+i]=stof(numbers);
}
//point_trans[0]=stof(numbers);
if(i==1)
{
rot_array[count*4+i]=stof(numbers);

}
//point_trans[1]=stof(numbers);
//point_trans[0]=stof(numbers);
if(i==2)
{
rot_array[count*4+i]=stof(numbers);

}
if(i==3)
{
rot_array[count*4+i]=stof(numbers);

//printf("%d,%f\n",count*4+i,stof(numbers));
}


//point_trans[1]=stof(numbers);
//point_trans[2]=stof(numbers);

//point_trans[3]=1.0;

//point_transformed=transform*point_trans;

//point.x=point_transformed[0]/point_transformed[3];

//point.y=point_transformed[1]/point_transformed[3];
//point.z=point_transformed[2]/point_transformed[3];

value=value.substr(found+1);
}
if(count==2)
break;

count+=1;
//point.r=point.g=point.b=stoi(value.substr(0,found));

//getchar();
}
for(int i=0;i<12;i++)
printf("%f\n",rot_array[i]);
getchar();
btMatrix3x3 mat = btMatrix3x3(rot_array[0],rot_array[1],rot_array[2],rot_array[4],rot_array[5],rot_array[6],rot_array[8],rot_array[9],rot_array[10]);
btVector3 translation = btVector3(btScalar(rot_array[4]),btScalar(rot_array[7]),btScalar(rot_array[11]));

//mat.getEulerYPR(yaw, pitch, roll);
//cout << "the roll is "<<yaw<<" pitch "<< pitch<< "roll "<<roll <<endl;
//trans=se3camfromworld.get_translation();
static tf::TransformBroadcaster br;
tf::Transform transform;

btTransform transrot(mat,translation);
transform.setOrigin(translation);
btQuaternion q = transrot.getRotation();
transform.setRotation(q);

//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world","base_link"));
//Matrix<3,3> rot;
//Vector<3>trans;
//rot(0,0)=-0.0573;rot(0,1)=-0.00568207;rot(0,2)=0.997937;
//rot(1,0)=-0.9943281;rot(1,1)=-0.0025257;rot(1,2)=-0.0569126;
//rot(2,0)=0.00251411;rot(2,1)=-0.9971511;rot(2,2)=-0.005649;
//btVector3 translation = btVector3(btScalar(0.31526),btScalar(0.0842366),btScalar(1.411));
//btTransform transrot(mat,translation);
//btQuaternion q = transrot.getRotation();
//trans[0]=-0.04;trans[1]=0.1742366;trans[2]=.3187;

//SO3<>trans_so3(rot);

//SE3<>transform(trans_so3,trans);



//Matrix<3,3>rot_check=transform.get_rotation().get_matrix();
//fprintf(stderr,"rotation:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",rot_check(0,0),rot_check(0,1),rot_check(0,2),rot_check(1,0),rot_check(1,1),rot_check(1,2),rot_check(2,0),rot_check(2,1),rot_check(2,2),trans[0],trans[1],trans[2]);
//std::string why;

//transform.setOrigin(translation);
//transform.setRotation(q); // tf::Quaternion(msg->theta, 0, 0) );


std::ifstream file("/home/irl/dataset_ptam/data_20_night/ptam_points.csv");


while ( getline ( file, value ))
{
      // read a string until next comma: http://www.cplusplus.com/reference/string/getline/

//   cout << string( value)<<endl; // display value removing the first and the last character from it

  // different member versions of find in the same order as above:
Vector<4>point_trans;  
Vector<4>point_transformed;  
std::string str2 (",");

for(int i=0;i<3;i++)
{


found = value.find(str2);

numbers=value.substr(0,found);
#if 0
//cout<<numbers<<endl;
if(i==0)
point_trans[0]=stof(numbers);
if(i==1)
point_trans[1]=stof(numbers);
if(i==2)
point_trans[2]=stof(numbers);

point_trans[3]=1.0;

point_transformed=transform*point_trans;

point.x=point_transformed[0]/point_transformed[3];

point.y=point_transformed[1]/point_transformed[3];
point.z=point_transformed[2]/point_transformed[3];

value=value.substr(found+1);
#endif
if(i==0)
point.x=stof(numbers);
//point_trans[0]=stof(numbers);
if(i==1)
point.y=stof(numbers);
//point_trans[1]=stof(numbers);
if(i==2)
point.z=stof(numbers);
//point_trans[2]=stof(numbers);

//point_trans[3]=1.0;

//point_transformed=transform*point_trans;

//point.x=point_transformed[0]/point_transformed[3];

//point.y=point_transformed[1]/point_transformed[3];
//point.z=point_transformed[2]/point_transformed[3];

value=value.substr(found+1);


}

//point.r=point.g=point.b=stoi(value.substr(0,found));

cloud_planar.points.push_back(point);
//getchar();
}

cout<<cloud_planar.points.size()<<endl;
cloud_planar.is_dense=true;
cloud_planar.width=cloud_planar.points.size();
cloud_planar.height=1;
cloud_planar.header.frame_id="/camera";
while(ros::ok())
{

//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world","camera"));
pub.publish(cloud_planar);
}

}


