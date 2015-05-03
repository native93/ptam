#include <ptam/read_point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iostream>

#include <pcl_ros/transforms.h>

using namespace std;
read_pointcloud::read_pointcloud()
{
// ros::NodeHandle nh_private("~");
  //  nh_private.param("max_no_of_ptam_maps", max_no_of_ptam_maps, 10)
//map_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/map/cloud",1);
std::stringstream ss;
int max_no_of_ptam_maps=1;
uav_cloud_sub.resize(max_no_of_ptam_maps);
pointcloud_received=false;
cloud.resize(max_no_of_ptam_maps);
//for(int i=0;i<max_no_of_ptam_maps;i++)
//{
//pcd_ready.push_back(false);
 // nh.getParam("/ptam_no", ptam_no);
//ss.str("");
//ss<<"/ptam/"<<i<<"/map_aligned";
uav_cloud_sub[0] = nh.subscribe("/ptam/map_points", 1, &read_pointcloud::uav_cloud_callback, this);

//}

ros::Publisher pub;

    for(int i = 0; i < 2; i++)
    {
        ss.str("");
        ss << "/ptam/" << i << "/map";
        pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(ss.str(), 1000);
        map_pubs.push_back(pub);
    }




}
void read_pointcloud::uav_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{

   cloud[0] = *msg;
pointcloud_received=true;
calculate_plane();
//printf("entered the callback\n");

/*
std::stringstream ss;
std::string stt;

int pose = msg -> header.frame_id.find('/',0 );

fprintf(stderr,"inside callback with msg no %d",pose);
int ptam_no_of_msg = atoi(msg -> header.frame_id.substr(pose+1,1).c_str());

    if(pcd_ready[ptam_no_of_msg-1] != true)
    {
        cloud[ptam_no_of_msg-1] = *msg;
//ss.str("");
//ss<<"/media/home/ayushd/ros/ptam_improved/launch/test"<<ptam_no_of_msg<<".pcd";
//stt=ss.str();
sum+=1;

        pcd_ready[ptam_no_of_msg-1] = true;
fprintf(stderr,"sum%d\n",sum);
       ROS_INFO("%d pcd ready. frameid: %s", ptam_no_of_msg, msg->header.frame_id.c_str());
    }

*/
#if 0
pcl::PointCloud<pcl::PointXYZ> cloud_planar;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ>);
   cloud[0] = *msg;
for(int i = 0; i < (int)cloud[0].points.size(); i+=1)
    {

//fprintf(stderr,"z coordinates:%f",fabs(map_cloud->points[i].z));
if(fabs(cloud[0].points[i].z) < 0.5)
{
        cloud_pub->points.push_back(cloud[0].points[i]);
}
    }
cloud_pub->is_dense=true;
cloud_pub->width=cloud_pub->points.size();
cloud_pub->height=1;

cloud_pub->header.frame_id="/world";

//printf("publish pcl with points%d\n",cloud_pub->points.size());
//map_pubs[0].publish(cloud_pub);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_pub->makeShared ());
  seg.segment (*inliers, *coefficients);
if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
   // return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;




  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)

{
//fprintf(stderr,"%d\n",i);
cloud_planar.points.push_back(cloud_pub->points[inliers->indices[i]]);

}
cloud_planar.is_dense=true;
cloud_planar.width=cloud_planar.points.size();
cloud_planar.height=1;

cloud_planar.header.frame_id="/world";

map_pubs[1].publish(cloud_planar);
//  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  //  std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
    //                                           << cloud.points[inliers->indices[i]].y << " "
      //                                         << cloud.points[inliers->indices[i]].z << std::endl;



//writer.write<pcl::PointXYZ> ("/media/home/ayushd/data_iros/cloud3.pcd",cloud[0],false);
//printf("saved %d points in a file",msg->points.size());
//	getchar();

#endif
  //  is_uav_cloud_ready = true;
}
void read_pointcloud::calculate_plane()
{


printf("inside calculate plane\n");
pcl::PointCloud<pcl::PointXYZ> cloud_planar;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ>);
for(int i = 0; i < (int)cloud[0].points.size(); i+=1)
    {

//fprintf(stderr,"z coordinates:%f",fabs(map_cloud->points[i].z));
if(fabs(cloud[0].points[i].z) < 0.5)
{
        cloud_pub->points.push_back(cloud[0].points[i]);
}
    }
cloud_pub->is_dense=true;
cloud_pub->width=cloud_pub->points.size();
cloud_pub->height=1;

cloud_pub->header.frame_id="/world";

//printf("publish pcl with points%d\n",cloud_pub->points.size());
//map_pubs[0].publish(cloud_pub);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud[0].makeShared ());
  seg.segment (*inliers, *coefficients);
if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
   // return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;




  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)

{
//fprintf(stderr,"%d\n",i);
cloud_planar.points.push_back(cloud[0].points[inliers->indices[i]]);

}
cloud_planar.is_dense=true;
cloud_planar.width=cloud_planar.points.size();
cloud_planar.height=1;

cloud_planar.header.frame_id="/world";
/*
while(ros::ok())
{

map_pubs[1].publish(cloud_planar);

ros::Duration(.1).sleep();
}
*/



//map_pubs[1].publish(cloud_planar);
//  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  //  std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
    //                                           << cloud.points[inliers->indices[i]].y << " "
      //                                         << cloud.points[inliers->indices[i]].z << std::endl;



//writer.write<pcl::PointXYZ> ("/media/home/ayushd/data_iros/cloud3.pcd",cloud[0],false);
//printf("saved %d points in a file",msg->points.size());
//	getchar();
 
}


void read_pointcloud::readpcd()

{


pcl::PCDReader reader;
std::stringstream ss;
fprintf(stderr,"inside read");
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_read (new pcl::PointCloud<pcl::PointXYZ>);

reader.read("/home/irl/workspace/ptam/dt.pcd",*cloud_read);
cloud[0]=*cloud_read;
cloud[0].is_dense=true;
cloud[0].header.frame_id="/world";
cloud[0].width=cloud_read->points.size();
cloud[0].height=1;


/*
fprintf(stderr,"cloud read point size %d",cloud[0].width);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setInputCloud (cloud[0].makeShared ());
  seg.segment (*inliers, *coefficients);
if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
   // return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;




pcl::PointCloud<pcl::PointXYZ> cloud_planar;
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)

{
//fprintf(stderr,"%d\n",i);
cloud_planar.points.push_back(cloud[0].points[inliers->indices[i]]);

}
cloud_planar.is_dense=true;
cloud_planar.width=cloud_planar.points.size();
cloud_planar.height=1;

cloud_planar.header.frame_id="/world";
*/


ofstream myfile;
myfile.open ("/home/irl/workspace/ptam/pcd_matlab_3.csv",ios::app);




for(int i=0;i<cloud[0].points.size();i++)
{
if(cloud[0].points[i].x>10.0||cloud[0].points[i].y>10.0||cloud[0].points[i].z>10.0)
continue;
if(cloud[0].points[i].x<-10.0||cloud[0].points[i].y<-10.0||cloud[0].points[i].z<-10.0)
continue;

myfile<<cloud[0].points[i].x<<","<<cloud[0].points[i].y<<","<<cloud[0].points[i].z<<endl;

}

myfile.close();
/*
for(int i=1;i<=5;i++)
{
ss.str("");
//fprintf(stderr,"width :%d\n",ptam_maps[i].width);
//if (i==4)
//continue;
ss<<"/media/home/ayushd/ros/ptam_improved/launch/final_pcd/box"<<i<<".pcd";

reader.read(ss.str(),*cloud_read);

cloud[i]=*cloud_read;
cloud[i].is_dense=true;
//cloud_read->header.frame_id="cloud";
cloud[i].width=cloud_read->points.size();
cloud[i].height=1;


fprintf(stderr,"cloud size %d\n",cloud[i].width);
}
*/
/*
sum=0;

pcl::PointCloud<pcl::PointXYZ> cloud_merge;
while(sum!=6)
{

ros::spinOnce();
}

for(int i=0;i<sum;i++)
{
if(i==0||i==5)
continue;
for(int j=0;j<cloud[i].points.size();j++)
cloud_merge.points.push_back(cloud[i].points[j]);

}

fprintf(stderr,"merge cloud size %d",cloud_merge.points.size());
cloud_merge.width=cloud_merge.points.size();
cloud_merge.height=1;
cloud_merge.header.frame_id="/map";

cloud_merge.header.stamp=ros::Time::now();
  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud", 1);

writer.write<pcl::PointXYZ> ("/media/home/ayushd/ros/ptam_improved/launch/box4/cloud_final_done.pcd",cloud_merge,false);

while(1)
{
pub.publish(cloud_merge);
ros::Duration(0.1).sleep();

}


//sensor_msgs::PointCloud2 msg,msg_ret;

//pcl::toROSMsg(cloud[1],msg);
*/
/*
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ>);

for(int i = 0; i < (int)cloud[2].points.size(); i+=1)
    {

//fprintf(stderr,"z coordinates:%f",fabs(map_cloud->points[i].z));
//if(fabs(map_cloud->points[i].z) > 0.2)
        cloud_pub -> points.push_back(cloud[1].points[i]);
    }

//=*cloud_read;
cloud_pub->is_dense=true;
//cloud_read->header.frame_id="cloud";
cloud_pub->width=cloud_pub->points.size();
cloud_pub->height=1;

cloud_pub->header.frame_id="/map";
*/
//while(ros::ok())
//{


//           map_pubs[1].publish(cloud[0]);

//ros::Duration(.1).sleep();
/*
for(int i=1;i<=5;i++)
{


           std::stringstream ss;                                                          
         // <<i<<"/map";
           cloud[i].header.frame_id="/map";

//fprintf(stderr,"cloud size %d\n",cloud[i].width);

//pub.publish(cloud[i]);


           map_pubs[i].publish(cloud[i]);
ros::Duration(.1).sleep();
}
*/
//ros::Duration(100).sleep();
//}
//pcl::PointCloud<pcl::PointXYZ> cloud_ret;

/*
fprintf(stderr,"inside read\n");
pcl::PCDWriter writer;
cloud_read->is_dense=false;
cloud_read->header.frame_id="cloud";
cloud_read->width=cloud_read->points.size();
cloud_read->height=1;

//fprintf(stderr,"cloud size%ld\n",cloud->points.size());


sensor_msgs::PointCloud2 msg,msg_ret;

pcl::toROSMsg(*cloud_read,msg);
const tf::TransformListener tt;
tt.waitForTransform("map","cloud",
                             ros::Time::now(), ros::Duration(2.0));
bool val=pcl_ros::transformPointCloud("map",msg,msg_ret,tt);
if(val)
pcl::fromROSMsg(msg_ret,cloud_ret);
    for(int i = 0; i < cloud_ret.points.size(); i++)
    {
   
fprintf(stderr,"Points %f,%f,%f\n",cloud_ret.points[i].x,cloud_ret.points[i].y,cloud_ret.points[i].z);
}

writer.write<pcl::PointXYZ> ("/media/home/ayushd/ros/ptam_improved/launch/test_transformed.pcd",cloud_ret,false);
*/

//pcl::PCDWriter writer;
/*
while(ros::ok())
{
if(is_uav_cloud_ready&&cloud.width>0)
{
fprintf(stderr,"pointc cloud received with width %d",cloud.width);
writer.write<pcl::PointXYZ> ("/media/home/ayushd/ros/ptam_improved/launch/test.pcd",cloud,false);
fprintf(stderr,"done with writinh");
break;
}
//pcl::toROSMsg(cloud,msg);
//map_cloud_pub.publish(*cloud);
ros::spinOnce();
}
//ros::Duration(100.0).sleep();
*/
}

int main(int argc,char **argv)
{
ros::init(argc,argv,"readin");
read_pointcloud read;
//while(!read.pointcloud_received)
//{
//ros::spinOnce();
read.readpcd();
//}
//ros::Spin();
//read.calculate_plane();
//return 0;
}
