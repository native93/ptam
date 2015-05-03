
#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include<ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/ros/conversions.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


class read_pointcloud
{

private:
pcl::PCDWriter writer;
//int max_no_of_ptam_maps;
       std::vector< pcl::PointCloud<pcl::PointXYZ> >cloud;
std::vector<ros::Subscriber> uav_cloud_sub;
ros::NodeHandle nh;
std::vector<ros::Publisher> map_pubs;
public:
int ptam_no;
int sum;
bool pointcloud_received;
std::vector<bool> pcd_ready;
void uav_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);
read_pointcloud();
void calculate_plane(); 
void readpcd();
};
