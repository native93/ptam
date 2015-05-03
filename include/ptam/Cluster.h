#ifndef _UGV_UAV_COLLABO_EXPLORE_CLUSTER_H_
#define _UGV_UAV_COLLABO_EXPLORE_CLUSTER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <ptam/cluster.h>
#include<vector>
#include<geometry_msgs/PoseArray.h>
#include <ptam/point.h>


class Cluster{
    public:
 Cluster();
 bool clustering(
                ptam::cluster::Request &req,
                ptam::cluster::Response &res);
std::vector<ptam::point>distance(geometry_msgs::PoseArray frontier,int *max_index);
        
    private:
        //ros related structures
        ros::NodeHandle nh;
        ros::ServiceServer service;
       std::vector<ptam::point>point_info;
       std:: vector<int>cluster_center;
	float max_radius;
        int max_index;
double threshold;
        //data members for invoking the get plan services
        
};
#endif


