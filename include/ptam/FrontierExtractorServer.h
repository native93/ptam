#ifndef _FRONTIER_EXTRACTOR_SERVER_H_
#define _FRONTIER_EXTRACTOR_SERVER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <LinearMath/btVector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetMap.h>
#include <ptam/mapper_direct.h>
#include <ptam/FrontierExtractor.h>
#include <queue>
#include <ptam/cluster.h>

class FrontierExtractorServer
{
    public:
        struct Point2D {
            int i;
            int j;
        };
        FrontierExtractorServer();
        bool extract_frontier(
                ptam::FrontierExtractor::Request &req,
                ptam::FrontierExtractor::Response &res);
        void ugv_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void cloud_occupancy_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void uav_frontiermap_callback(const geometry_msgs::PoseArray::ConstPtr &msg);
        void uav_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);
    protected:

ros::ServiceClient srv_client;
ptam::cluster _cluster_srv;
//     virtual void findPassiveFrontiers();
//        virtual void findFrontiers(costmap_2d::Costmap2DROS& costmap_);
         virtual void findUGVFrontiers();
        //virtual void findPCDFrontiers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    private:
        ros::NodeHandle nh;
        ros::ServiceServer service;
        tf::TransformListener tf_listener;

        ros::Subscriber ugv_map_sub;
        ros::Subscriber cloud_occupancy_map_sub;
        ros::Subscriber uav_cloud_sub;
        ros::Subscriber uav_frontiermap_sub;

        ros::Publisher ugv_frontier_pub;
        ros::Publisher uav_frontier_pub;
        ros::Publisher passive_uav_frontier_pub;

        bool is_ugv_map_ready;
        bool is_cloud_occupancy_map_ready;
        bool is_uav_cloud_ready;
        bool is_passive_uav_frontier_poses_ready;

        double _uav_height;

        const int THRESHOLD_FRONTIER_SIZE;

        nav_msgs::OccupancyGrid _ugv_map;
        nav_msgs::OccupancyGrid _cloud_occupancy_map;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _uav_map_cloud;

        pcl::PointCloud<pcl::PointXYZ>cloud_rece;
        geometry_msgs::PoseArray _ugv_frontier_poses;
        geometry_msgs::PoseArray _pcd_contour;
        geometry_msgs::PoseArray _passive_uav_frontier_poses;
        //std::vector<pcl::PointXYZ> _pcd_contour;
//        nav_msgs::OccupancyGrid _map;
};

#endif
