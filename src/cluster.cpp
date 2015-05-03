#include<iostream>
#include<stdio.h>
//#include<vector>
#include<cmath>
#include<ptam/Cluster.h>
using namespace std;
Cluster::Cluster()
{
ros::NodeHandle nh_private("~");
  //  nh_private.getParam("threshold", threshold);
   // nh_private.getParam("optimization_data_filename", _optimization_data_filename);
service = nh.advertiseService("clustering", &Cluster::clustering, this);

}

std::vector<ptam::point>Cluster::distance(geometry_msgs::PoseArray points,int *max_index) 
{
vector<ptam::point>dist_calc;
   ptam::point p;
int max=-10000;
int dist;
for(int i=0;i<points.poses.size();i++)
{


   // vector<point>dist_calc;
    dist=(points.poses[i].position.x-points.poses[0].position.x)*(points.poses[i].position.x-points.poses[0].position.x)+
         (points.poses[i].position.y-points.poses[0].position.y)*(points.poses[i].position.y-points.poses[0].position.y);
p.dist=dist;
p.c_i=0;
    dist_calc.push_back(p);
    if(dist>max)
    {
        max=dist;
       *max_index=i;
    }

//cout<<dist_calc.size();
}
return(dist_calc);
}


bool Cluster::clustering(
        ptam::cluster::Request &req,
        ptam::cluster::Response &res) {

    
vector<ptam::point>point_info;///every point's cluster index and its distance from cluster center
res.clustered_frontiers.poses.clear();
cluster_center.clear();
threshold=req.threshold;

fprintf(stderr,"threshold%f\n",threshold);

//getchar();
//vector<int>cluster_center;
cluster_center.push_back(0);
int max_index;
point_info.clear();
res.point_info.clear();
point_info=distance(req.frontiers,&max_index);
//cluster_center.push_back(max_index);

//for(int i=0;i<point_info.size();i++)
//cout<<point_info[i].c_i<<"   "<<point_info[i].dist<<endl;
float max_radius;
float dist;
float max_val;
max_radius=point_info[max_index].dist;
while(max_radius>threshold)
{

    max_val=-1000;
cluster_center.push_back(max_index);    
int cluster_no=cluster_center.size()-1;    
int c_index=cluster_center[cluster_no];
//cout<<"c_index:"<<max_index<<endl;
    for(int i=0;i<req.frontiers.poses.size();i++)
    {    
      dist=(req.frontiers.poses[i].position.x-req.frontiers.poses[c_index].position.x)*(req.frontiers.poses[i].position.x-req.frontiers.poses[c_index].position.x)+
           (req.frontiers.poses[i].position.y-req.frontiers.poses[c_index].position.y)*(req.frontiers.poses[i].position.y-req.frontiers.poses[c_index].position.y);   
        if(dist<point_info[i].dist)
        {
        point_info[i].c_i=cluster_no;
        point_info[i].dist=dist;
                if(dist>max_val)
                {
                    max_val=dist;
                    max_index=i;
                }
        }
        else
        {
                     if(point_info[i].dist>max_val)
                     {
                    max_val=point_info[i].dist;
                    max_index=i;
                     }
        }

//cout<<point_info[i].c_i<<"   "<<point_info[i].dist<<endl;

       

    }


    
max_radius=max_val;
// cout<<max_radius<<endl;       
//getchar();

    }
geometry_msgs::Pose p;
p.position.x=0.0;
p.position.y=0.0;
p.position.z=0.0;

fprintf(stderr,"%d\n",cluster_center.size());

 for(int i=0;i<cluster_center.size();i++)
res.clustered_frontiers.poses.push_back(req.frontiers.poses[cluster_center[i]]);



res.cluster_center=cluster_center;
/*
vector<int>counter(cluster_center.size(),0);
  
for(int i=0;i<point_info.size();i++)
{
counter[point_info[i].c_i]+=1;
res.clustered_frontiers.poses[point_info[i].c_i].position.x+=req.frontiers.poses[i].position.x;
res.clustered_frontiers.poses[point_info[i].c_i].position.y+=req.frontiers.poses[i].position.y;
res.clustered_frontiers.poses[point_info[i].c_i].orientation.x=req.frontiers.poses[i].orientation.x;
res.clustered_frontiers.poses[point_info[i].c_i].orientation.y=req.frontiers.poses[i].orientation.y;
res.clustered_frontiers.poses[point_info[i].c_i].orientation.z=req.frontiers.poses[i].orientation.z;
res.clustered_frontiers.poses[point_info[i].c_i].orientation.w=req.frontiers.poses[i].orientation.w;

}
for(int i=0;i<cluster_center.size();i++)
{

//fprintf(stderr,"x:%f counter:%d\n",res.clustered_frontiers.poses[i].position.x,counter[i]);
res.clustered_frontiers.poses[i].position.x=res.clustered_frontiers.poses[i].position.x/counter[i];
res.clustered_frontiers.poses[i].position.x=res.clustered_frontiers.poses[i].position.y/counter[i];


}
*/
res.point_info=point_info;
return true;

}

int main(int argc, char* argv[])
{
//    printf("HI");
    ros::init(argc, argv, "cluster");
     Cluster c;
ros::spin();
  //  counter+=1;
}

