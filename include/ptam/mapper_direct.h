#ifndef _MAPPER_DIRECT_H_
#define _MAPPER_DIRECT_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include<std_msgs/Bool.h>
#include <queue>
#include <ptam/VoxelGrid2D.h>

#include <ptam/map_info.h>
#include <ptam/map.h>

//#include <ugv_uav_collaborative_exploration/navigation/explore_node.h>
#include <geometry_msgs/PoseArray.h>

#include <ros/console.h>

class mapper_direct
{
    public:
 struct Point2D {
            int i;
            int j;
        };
        
        mapper_direct();
	void map_making(const ptam::map::ConstPtr &msg);
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void kinectCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
        void sonarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        static const int OCCUPIED;
        static const int UNOCCUPIED;
        static const int UNKNOWN;

	void bool_callback(const std_msgs::Bool::ConstPtr &msg);
         void findUGVFrontiers();
    private:

    ptam::map_info dat;
        ros::Publisher pop_pub;
	 std::vector<int>data;
    std::string map_name;
        geometry_msgs::PoseArray _ugv_frontier_poses;
        nav_msgs::OccupancyGrid _ugv_map;
        ros::NodeHandle nh;
        int no_of_ugv;
        int no_of_uav;
	std::vector<ros::Subscriber>sicklms;
	ros::Subscriber move_done_val;
        ros::Subscriber *sicklms_subs;
        ros::Subscriber *kinect_subs;
        ros::Subscriber *sonar_subs;
	ros::Subscriber map_sub;
	ptam::map frontier;
        ros::Publisher ugv_frontier_pub_disp;
        ros::Publisher ugv_frontier_pub;
       	int THRESHOLD_FRONTIER_SIZE;
        ros::Publisher map_pub;
        ros::Publisher map_pub_visib;
        ros::Publisher pcd_map_pub;
        ros::Publisher uav_frontiermap_points_pub;
        ros::Publisher map_cloud_pub;
        nav_msgs::OccupancyGrid map;
        nav_msgs::OccupancyGrid pcd_map;
        geometry_msgs::PoseArray uav_frontier_map_points;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        tf::TransformListener listener;
        std::string CLOUD_FRAME_ID;
        std::string MAP_FRAME_ID;
        const float UAV_SIZE;
	bool move_done;
//explore exp;
        tf::TransformListener tf_listener;
};

const int mapper_direct::OCCUPIED = 100;
const int mapper_direct::UNOCCUPIED = 0;
const int mapper_direct::UNKNOWN = -1;
#endif
