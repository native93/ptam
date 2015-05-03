#ifndef _VISIBILITY_CALCULATOR_SERVER_H_
#define _VISIBILITY_CALCULATOR_SERVER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <ptam/VisibilityCalculator.h>
#include <ptam/mapper_direct.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <ptam/map.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>
//#include <ptam/Front_dist.h>
class VisibilityCalculatorServer {
    private:
        ros::NodeHandle nh;
        ros::ServiceServer service;
        tf::TransformListener tf_listener;
	ros::Publisher pub;
       // ros::Subscriber frontier_sub;
        ros::Subscriber ugv_map_sub;
       // ros::Subscriber uav_cloud_sub;
      //  ros::Subscriber uav_frontiermap_sub;
      //  ros::Subscriber uav_occupancy_sub;
     //   bool _is_uav_map_received;
      //  bool _is_passive_uav_map_received;
      //  bool _is_uav_occupancymap_ready;
int index,robot_no;

        geometry_msgs::PoseArray _ugv_frontier_poses;
        nav_msgs::OccupancyGrid _ground_map;
    //    nav_msgs::OccupancyGrid _uav_occupancymap;
     //   geometry_msgs::PoseArray _passive_uav_frontier_poses;
     //   pcl::PointCloud<pcl::PointXYZ>::Ptr _uav_map_cloud;
    public:
        struct Point2D {
            int i;
            int j;
        };
        VisibilityCalculatorServer();
void front_callback(const geometry_msgs::PoseArray::ConstPtr &msg);

bool front_received;
        bool _is_ground_map_received;
        bool calculate_visibility(
                ptam::VisibilityCalculator::Request &req,
                ptam::VisibilityCalculator::Response &res);
	void cal();
        void ugv_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
       // void uav_frontiermap_callback(const geometry_msgs::PoseArray::ConstPtr &msg);
       // void uav_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);
       // void uav_occupancymap_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    protected:
        float calVisibilityUGV(geometry_msgs::Pose &p);
//        float calVisibilityUAV(geometry_msgs::Pose &p);
  //      float calVisibilityPassiveUAV(geometry_msgs::Pose &p);
};

#endif
