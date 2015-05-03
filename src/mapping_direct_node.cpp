#include <ptam/mapper_direct.h>

#include <LinearMath/btVector3.h>

#include <math.h>

mapper_direct::mapper_direct()  : map_cloud(new pcl::PointCloud<pcl::PointXYZ>() ), CLOUD_FRAME_ID("/world"), MAP_FRAME_ID("/world"), UAV_SIZE(0.5)
{
    nh.getParam("/no_of_ugv", no_of_ugv);
    nh.getParam("/no_of_uav", no_of_uav);
    ROS_INFO("Initializing mapper to map using %d UGV's, %d UAV's", no_of_ugv, no_of_uav);
    map.header.frame_id = MAP_FRAME_ID;
    map.info.map_load_time = ros::Time::now();
    map.info.resolution = 0.05;
    map.info.width = 4000;
    map.info.height = 4000;
    map.info.origin.position.x = -((1.0 * map.info.width)/2) * map.info.resolution;
    map.info.origin.position.y = -((1.0 * map.info.height)/2) * map.info.resolution;
    map.info.origin.position.z = 0.0;

    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    map.data.resize(map.info.width * map.info.height, UNKNOWN);
    


    _ugv_map.header.frame_id = MAP_FRAME_ID;
    _ugv_map.info.map_load_time = ros::Time::now();
    _ugv_map.info.resolution = 0.05;
    _ugv_map.info.width = 4000;
    _ugv_map.info.height = 4000;
    _ugv_map.info.origin.position.x = -((1.0 * _ugv_map.info.width)/2) * _ugv_map.info.resolution;
    _ugv_map.info.origin.position.y = -((1.0 * _ugv_map.info.height)/2) * _ugv_map.info.resolution;
    _ugv_map.info.origin.position.z = 0.0;

    _ugv_map.info.origin.orientation.x = 0.0;
    _ugv_map.info.origin.orientation.y = 0.0;
    _ugv_map.info.origin.orientation.z = 0.0;
    _ugv_map.info.origin.orientation.w = 1.0;

    _ugv_map.data.resize(_ugv_map.info.width * _ugv_map.info.height, 50);


    /*
    uav_frontier_map.header.frame_id = MAP_FRAME_ID;
    uav_frontier_map.info = map.info;
    uav_frontier_map.data.resize(uav_frontier_map.info.width
            * uav_frontier_map.info.height, -1);*/
    pcd_map.header.frame_id = MAP_FRAME_ID;
    pcd_map.info.map_load_time = ros::Time::now();
    pcd_map.info.resolution = 0.10;
    pcd_map.info.width = 512;
    pcd_map.info.height = 512;
    pcd_map.info.origin.position.x = -((1.0 * pcd_map.info.width)/2) * pcd_map.info.resolution;
    pcd_map.info.origin.position.y = -((1.0 * pcd_map.info.height)/2) * pcd_map.info.resolution;
    pcd_map.info.origin.position.z = 0.0;

    pcd_map.info.origin.orientation.x = 0.0;
    pcd_map.info.origin.orientation.y = 0.0;
    pcd_map.info.origin.orientation.z = 0.0;
    pcd_map.info.origin.orientation.w = 1.0;

    pcd_map.data.resize(pcd_map.info.width * pcd_map.info.height, UNKNOWN);


    std::stringstream ss, ss1, ss2;

    sicklms_subs = new ros::Subscriber[1];
 //   for(int i = 0; i < no_of_ugv; i++)
   // {
     //   ss.seekp(0);
       // ss << "/sicklms" << i << "/scan";
       // sicklms_subs[i] = nh.subscribe(ss.str(), 100, &mapper_direct::laserCallback, this);
   // }
sicklms_subs[0]=nh.subscribe("/scan",100,&mapper_direct::laserCallback,this);
/*
    sonar_subs = new ros::Subscriber[no_of_uav];
    for(int i = 0; i < no_of_uav; i++)
    {
        ss1.seekp(0);
        ss1 << "/sonar" << i << "/scan";
        sonar_subs[i] = nh.subscribe(ss1.str(), 100, &mapper_direct::sonarCallback, this);
    }

    kinect_subs = new ros::Subscriber[no_of_uav];
    for(int i = 0; i < no_of_uav; i++)
    {
        ss2.seekp(0);
        ss2 << "/kinect" << i << "/points";
        kinect_subs[i] = nh.subscribe(ss2.str(), 10, &mapper_direct::kinectCallback, this);
    }
*/
    map_pub_visib = nh.advertise<nav_msgs::OccupancyGrid>("/map_visib", 1);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map1", 1);
    uav_frontiermap_points_pub = nh.advertise<geometry_msgs::PoseArray>("/map/uavfrontier", 1);
    map_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/map/cloud", 1);
    pcd_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map/cloud_occupancy", 1);
}

void mapper_direct::kinectCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{

//fprintf(stderr,"inside kinect");
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < (int)cloud -> points.size(); i+=20)
    {
//fprintf(stderr,"z coordinates:%f",fabs(map_cloud->points[i].z));
//if(fabs(map_cloud->points[i].z) > 0.2)
        map_cloud -> points.push_back(cloud -> points[i]);
    }
    //fprintf(stderr,"map_cloud.size() = %ld before filtering", map_cloud -> points.size());
    // Create the filtering object
    sor.setInputCloud (map_cloud);
    sor.setLeafSize (0.02f, 0.02f, 0.02f);
    sor.filter (*filtered_cloud);
 //  fprintf(stderr,"map_cloud.size() = %ld after filtering", filtered_cloud -> points.size());
//getchar();
    filtered_cloud -> header.stamp = ros::Time::now();
    filtered_cloud -> header.frame_id = CLOUD_FRAME_ID;
    map_cloud_pub.publish(*filtered_cloud);

    *map_cloud = *filtered_cloud;

    //ROS_INFO("published uav cloud map");

    tf::StampedTransform transform;
    try {
        tf_listener.lookupTransform(cloud -> header.frame_id, "/world", ros::Time(0), transform);
        int cur_point_x;
        int cur_point_y;
        for(int i = 0; i < (int)cloud -> points.size(); i+=3) {
            cur_point_x = (int)(cloud -> points[i].x / pcd_map.info.resolution + pcd_map.info.width / 2);
            cur_point_y = (int)(cloud -> points[i].y / pcd_map.info.resolution + pcd_map.info.height / 2);
            if(fabs(cloud -> points[i].z) < 0.1) {
                pcd_map.data[cur_point_y * pcd_map.info.width + cur_point_x] = OCCUPIED;
            }
            else {
                pcd_map.data[cur_point_y * pcd_map.info.width + cur_point_x] = UNOCCUPIED;
            }
        }
        pcd_map_pub.publish(pcd_map);
    }


    catch (tf::TransformException ex) {
        ROS_ERROR("Error publishing pcd occupancy map: %s", ex.what());
    }

//fprintf(stderr,"outside kinect");
}

void mapper_direct::sonarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("world", msg ->header.frame_id,
               msg -> header.stamp, ros::Duration(0.1));
        listener.lookupTransform("world", msg -> header.frame_id,  
               msg -> header.stamp, transform);
    }
    catch (tf::TransformException ex){
  //      ROS_ERROR("Error looking up transformation\n%s",ex.what());
        return;
    }

//    ROS_INFO("Transform ready");

    tf::Vector3 cur_point;
    tf::Vector3 zero_point;
    zero_point.setX(0.0);
    zero_point.setY(0.0);
    zero_point.setZ(0.0);
    tf::Vector3 cur_point_transformed;
    tf::Vector3 zero_point_transformed;

    int cur_point_x;
    int cur_point_y;
//fprintf(stderr,"uav frontiers initial size%ld",uav_frontier_map_points.poses.size());
    float theta = msg -> angle_min;
//    ROS_INFO("Theta starting from %f", theta);
    geometry_msgs::Pose p;
    btQuaternion rot;

//fprintf(stderr,"inside sonar");
    zero_point_transformed = transform * zero_point;
    for(int i = 0; i < (int)(msg -> ranges.size()); i++)
    {
//        ROS_INFO("Treating %dth laser from scan", i);
        float r;
        float dist;
        bool redundant_point = false;
        int flag = 0;
        for(r = 0; r < msg -> ranges[i] - 2*UAV_SIZE; r+= map.info.resolution)
        {
            cur_point.setX(r * cos(theta));
            cur_point.setY(r * sin(theta));
            cur_point.setZ(0.0);
            cur_point_transformed = transform * cur_point;
            cur_point_x = (int)(cur_point_transformed.getX() / map.info.resolution) + map.info.width/2;
            cur_point_y = (int)(cur_point_transformed.getY() / map.info.resolution) + map.info.height/2;
	    if(cur_point_y * map.info.width + cur_point_x>=map.data.size()-1)
		continue;
	    else
	    {
//            ROS_INFO("%d, %d for r=%f", cur_point_x, cur_point_y, r);
            if(map.data[cur_point_y * map.info.width + cur_point_x] > OCCUPIED/2)
            {
                if(flag == 2) {
                    p.position.x = cur_point_transformed.getX();
                    p.position.y = cur_point_transformed.getY();
                    p.position.z = 0.0;
                    for(int j = 0; j < (int)uav_frontier_map_points.poses.size(); j++) {
                        dist = (uav_frontier_map_points.poses[j].position.x - p.position.x)*(uav_frontier_map_points.poses[j].position.x - p.position.x)
                            +(uav_frontier_map_points.poses[j].position.y - p.position.y)*(uav_frontier_map_points.poses[j].position.y - p.position.y)
                            +(uav_frontier_map_points.poses[j].position.z - p.position.z)*(uav_frontier_map_points.poses[j].position.z - p.position.z);
//fprintf(stderr,"thresh%f\n",dist);
                        if(dist < 0.01) { 
                            redundant_point = true;
                        }
                    }
                    if(!redundant_point) {
                        rot.setEulerZYX(atan2(p.position.y - zero_point_transformed.getX(), p.position.x - zero_point_transformed.getY()), 0.0, 0.0);
                        p.orientation.x = rot.x();
                        p.orientation.y = rot.y();
                        p.orientation.z = rot.z();
                        p.orientation.w = rot.w();
                        uav_frontier_map_points.poses.push_back(p);
                    }
                    break;
                }
                else {
                    flag++;
                    continue;
                }
            }
	    
            else if(flag != 0) {
                flag = 0;
            }
        }
	}
        //next range corresponds to next angle. Hence, increment theta
        theta += msg -> angle_increment;
    }

//fprintf(stderr,"outside sonar");
    uav_frontier_map_points.header.seq = 0;
    uav_frontier_map_points.header.frame_id = "/world";
    uav_frontiermap_points_pub.publish(uav_frontier_map_points);
}

void mapper_direct::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //  ROS_INFO("New scan");
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("world", msg -> header.frame_id,
                msg -> header.stamp, ros::Duration(0.1));
        listener.lookupTransform("world", msg -> header.frame_id,  
                msg -> header.stamp, transform);
    }
    catch (tf::TransformException ex){
//        ROS_ERROR("Error looking up transformation\n%s",ex.what());
        return;
    }

//    ROS_INFO("Transform ready");

    tf::Vector3 cur_point;
    tf::Vector3 cur_point_transformed;

    int cur_point_x;
    int cur_point_y;
//fprintf(stderr,"inside laser");
    float theta = msg -> angle_min;
//    ROS_INFO("Theta starting from %f", theta);
    for(int i = 0; i < (int)(msg -> ranges.size()); i++)
    {
//        ROS_INFO("Treating %dth laser from scan", i);
        float r;

        for(r = 0; r < msg -> ranges[i]; r+= map.info.resolution)
        {
            cur_point.setX(r * cos(theta));
            cur_point.setY(r * sin(theta));
            cur_point.setZ(0.0);
            cur_point_transformed = transform * cur_point;

            
            
            
            cur_point_x = (int)(cur_point_transformed.getX() / map.info.resolution) + map.info.width/2;
            cur_point_y = (int)(cur_point_transformed.getY() / map.info.resolution) + map.info.height/2;
        //    ROS_INFO("%d, %d for r=%f", cur_point_x, cur_point_y, r);
            
map.data[cur_point_y * map.info.width + cur_point_x] -=20;
if(map.data[cur_point_y * map.info.width + cur_point_x]<20)
map.data[cur_point_y * map.info.width + cur_point_x] = UNOCCUPIED;

_ugv_map.data[cur_point_y * _ugv_map.info.width + cur_point_x] = UNOCCUPIED;
     
        }
//        ROS_INFO("Marked unoccupied");
        if(msg -> ranges[i] < msg -> range_max)
        {

printf("range%f\n",msg->ranges[i]);
            cur_point.setX(r * cos(theta));
            cur_point.setY(r * sin(theta));
            cur_point.setZ(0.0);
            cur_point_transformed = transform * cur_point;
            cur_point_x = (int)(cur_point_transformed.getX() / map.info.resolution) + map.info.width/2;
            cur_point_y = (int)(cur_point_transformed.getY() / map.info.resolution) + map.info.height/2;

map.data[cur_point_y *map.info.width + cur_point_x] +=30;
if(map.data[cur_point_y * _ugv_map.info.width + cur_point_x]<80)
{
            map.data[cur_point_y * map.info.width + cur_point_x] = OCCUPIED;
            //assuming that I am mapping well without map boundaries
          
  map.data[cur_point_y * map.info.width + (cur_point_x-1)] = OCCUPIED;
            map.data[(cur_point_y-1) * map.info.width + cur_point_x] = OCCUPIED;
            map.data[(cur_point_y-1) * map.info.width + (cur_point_x-1)] = OCCUPIED;
            map.data[cur_point_y * map.info.width + (cur_point_x+1)] = OCCUPIED;
            map.data[(cur_point_y+1) * map.info.width + cur_point_x] = OCCUPIED;
            map.data[(cur_point_y+1) * map.info.width + (cur_point_x+1)] = OCCUPIED;
            map.data[(cur_point_y+1) * map.info.width + (cur_point_x-1)] = OCCUPIED;
            map.data[(cur_point_y-1) * map.info.width + (cur_point_x+1)] = OCCUPIED;

}
 _ugv_map.data[cur_point_y * _ugv_map.info.width + cur_point_x] = OCCUPIED;
            //assuming that I am mapping well without map boundaries
          
  _ugv_map.data[cur_point_y * _ugv_map.info.width + (cur_point_x-1)] = OCCUPIED;
            _ugv_map.data[(cur_point_y-1) * _ugv_map.info.width + cur_point_x] = OCCUPIED;
            _ugv_map.data[(cur_point_y-1) * _ugv_map.info.width + (cur_point_x-1)] = OCCUPIED;
            _ugv_map.data[cur_point_y * _ugv_map.info.width + (cur_point_x+1)] = OCCUPIED;
            _ugv_map.data[(cur_point_y+1) * _ugv_map.info.width + cur_point_x] = OCCUPIED;
            _ugv_map.data[(cur_point_y+1) * _ugv_map.info.width + (cur_point_x+1)] = OCCUPIED;
            _ugv_map.data[(cur_point_y+1) * _ugv_map.info.width + (cur_point_x-1)] = OCCUPIED;
            _ugv_map.data[(cur_point_y-1) * _ugv_map.info.width + (cur_point_x+1)] = OCCUPIED;



  for(r =msg -> ranges[i]+0.2 ; r <10; r+= map.info.resolution)
        {
            cur_point.setX(r * cos(theta));
            cur_point.setY(r * sin(theta));
            cur_point.setZ(0.0);
            cur_point_transformed = transform * cur_point;

            
            
            
            cur_point_x = (int)(cur_point_transformed.getX() / map.info.resolution) + map.info.width/2;
            cur_point_y = (int)(cur_point_transformed.getY() / map.info.resolution) + map.info.height/2;
        //    ROS_INFO("%d, %d for r=%f", cur_point_x, cur_point_y, r);
            map.data[cur_point_y * map.info.width + cur_point_x] = UNKNOWN;
        }


          //        ROS_INFO("Marked occupied");
        }
 //     
        //next range corresponds to next angle. Hence, increment theta
        theta += msg -> angle_increment;
    }

    map.data[150*map.info.width + 358] = 0;
    map.data[149*map.info.width + 358] = 0;

//fprintf(stderr,"outside laser");
    map_pub.publish(map);
map_pub_visib.publish(_ugv_map);
}   

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mapper_direct");
    mapper_direct mapper;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
