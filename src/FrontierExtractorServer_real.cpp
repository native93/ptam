#include "ptam/FrontierExtractorServer.h"
#include <pcl/io/pcd_io.h>

FrontierExtractorServer::FrontierExtractorServer() : THRESHOLD_FRONTIER_SIZE(5)
{
    is_ugv_map_ready = false;
    is_cloud_occupancy_map_ready = false;
    is_uav_cloud_ready = false; 
    is_passive_uav_frontier_poses_ready = false; 

    nh.getParam("/uav_height", _uav_height);
_uav_height=2.8;
    ROS_INFO("loaded uav height: %lf", _uav_height);

// _uav_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ugv_map_sub = nh.subscribe("/mapping", 10, &FrontierExtractorServer::ugv_map_callback, this);
  cloud_occupancy_map_sub = nh.subscribe("/map/cloud_occupancy", 10, &FrontierExtractorServer::cloud_occupancy_map_callback, this);
  //  uav_frontiermap_sub = nh.subscribe("/map/uavfrontier", 10, &FrontierExtractorServer::uav_frontiermap_callback, this);
    uav_cloud_sub = nh.subscribe("/map/cloud", 1, &FrontierExtractorServer::uav_cloud_callback, this);
srv_client=nh.serviceClient<ptam::cluster>("clustering");
    service = nh.advertiseService("/FrontierExtractor", &FrontierExtractorServer::extract_frontier, this);

    ugv_frontier_pub = nh.advertise<geometry_msgs::PoseArray>(std::string("/frontier/ground"), 1);
    uav_frontier_pub = nh.advertise<geometry_msgs::PoseArray>(std::string("/frontier/uav"), 1);
    passive_uav_frontier_pub = nh.advertise<geometry_msgs::PoseArray>(std::string("/frontier/passive_uav"), 1);
}

void FrontierExtractorServer::cloud_occupancy_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    _cloud_occupancy_map = *msg;
    is_cloud_occupancy_map_ready = true;
}


void FrontierExtractorServer::uav_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{

    *_uav_map_cloud= *msg;

//fprintf(stderr,"message received");
    is_uav_cloud_ready = true;
}
void FrontierExtractorServer::ugv_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    _ugv_map = *msg;
    is_ugv_map_ready = true;
}



void FrontierExtractorServer::findUGVFrontiers() {
    int i, j;

    int idx;
    int width = _ugv_map.info.width;
    int height = _ugv_map.info.height;
    int max_idx = _ugv_map.data.size();

    printf("loaded params of map: width: %d, height: %d, max_idx: %d\n", 
            width, height, max_idx);

    bool is_frontier;
    std::vector<bool> isfrontier_data(width * height);
    int direction_x;
    int direction_y;
    std::vector<int> direction_x_data(width * height);
    std::vector<int> direction_y_data(width * height);
    FILE* raw_frontiers_file = fopen("raw_frontiers_file.pcd", "w");
    if(!raw_frontiers_file) {
        ROS_ERROR("Unable to open raw_frontiers_file.pcd for writing");
    }
    fprintf(raw_frontiers_file, "goodday");
    for(i = 0; i < height; i++) {
        for(j = 0; j < width; j++) {
            if(j == 689 && i == 599) {
                printf("Gotcha %d\n", _ugv_map.data[i * _ugv_map.info.width + j]);
            }
            isfrontier_data[i*width + j] = false;
            is_frontier = false;
            direction_x = direction_y = 0;
            idx = i * width + j;
            if(_ugv_map.data[idx] != mapper_direct::UNKNOWN && _ugv_map.data[idx] < mapper_direct::OCCUPIED/2) {
                if(idx - 1 >= 0 && _ugv_map.data[idx - 1] == mapper_direct::UNKNOWN) {
                    is_frontier = true;
                    direction_x -= 1;
                }
                else if(idx + 1 < max_idx && _ugv_map.data[idx + 1] == mapper_direct::UNKNOWN) {
                    is_frontier = true;
                    direction_x += 1;
                }
                else if(idx - width >= 0 && _ugv_map.data[idx - width] == mapper_direct::UNKNOWN) {
                    is_frontier = true;
                    direction_y -= 1;
                }
                else if(idx + width < max_idx && _ugv_map.data[idx + width] == mapper_direct::UNKNOWN) {
                    is_frontier = true;
                    direction_y += 1;
                }
            }
            if(is_frontier) {
                isfrontier_data[idx] = true;
                direction_x_data[idx] = direction_x;
                direction_y_data[idx] = direction_y;
            }
            
            if(is_frontier) {
                fprintf(raw_frontiers_file, "%f %f 0.0\n", j * _ugv_map.info.resolution + _ugv_map.info.origin.position.x,
                        i * _ugv_map.info.resolution + _ugv_map.info.origin.position.y);
            }
        }
    }
    fclose(raw_frontiers_file);
   // ROS_INFO("Preprocessing done");
    //now that preprocessing is done. Lets begin aggregating the frontiers

    std::queue<FrontierExtractorServer::Point2D> qpt;
    FrontierExtractorServer::Point2D start_pt;
    FrontierExtractorServer::Point2D cur_pt;
    FrontierExtractorServer::Point2D new_pt;
    std::vector<std::vector<FrontierExtractorServer::Point2D> > frontiers;
    std::vector<FrontierExtractorServer::Point2D> cur_frontier;
    std::vector<bool> marked(width * height);
    for(i = 0; i < width * height; i++) { marked[i] = false; }
    bool found_start = false;
    for(i = 0; i < height; i++) {
        for(j = 0; j < width; j++) {
            if(isfrontier_data[i*width + j]) {
                start_pt.i = i;
                start_pt.j = j;
                found_start = true;
                break;
            }
        }
        if(found_start == true) {
            marked[i*width + j] = true;
            qpt.push(start_pt);
            break;
        }
    }
    while(found_start) {
        //ROS_INFO("start found at %d, %d", start_pt.i, start_pt.j);
        while(!qpt.empty()) {
            cur_pt = qpt.front();
            qpt.pop();
            cur_frontier.push_back(cur_pt);
            //check neighbours and enqueue them if they are unmarked and mark them.
            if(cur_pt.i - 1 >= 0 && 
                    isfrontier_data[(cur_pt.i - 1) * width + cur_pt.j] && 
                    marked[(cur_pt.i - 1) * width + cur_pt.j] == false) {
                new_pt.i = cur_pt.i - 1; new_pt.j = cur_pt.j;
                //ROS_INFO("Adding %d, %d", new_pt.i, new_pt.j);
                marked[(cur_pt.i - 1) * width + cur_pt.j] = true;
                qpt.push(new_pt);
            }
            if(cur_pt.i + 1 < height && 
                    isfrontier_data[(cur_pt.i + 1) * width + cur_pt.j] &&
                    marked[(cur_pt.i + 1) * width + cur_pt.j] == false) {
                new_pt.i = cur_pt.i + 1; new_pt.j = cur_pt.j;
                //ROS_INFO("Adding %d, %d", new_pt.i, new_pt.j);
                marked[(cur_pt.i + 1) * width + cur_pt.j] = true;
                qpt.push(new_pt);
            }
            if(cur_pt.j - 1 >= 0 && 
                    isfrontier_data[cur_pt.i * width + cur_pt.j - 1] &&
                    marked[cur_pt.i * width + cur_pt.j - 1] == false) {
                new_pt.i = cur_pt.i; new_pt.j = cur_pt.j - 1;
                //ROS_INFO("Adding %d, %d", new_pt.i, new_pt.j);
                marked[cur_pt.i * width + cur_pt.j - 1] = true;
                qpt.push(new_pt);
            }
            if(cur_pt.j + 1 < width &&
                    isfrontier_data[cur_pt.i * width + cur_pt.j + 1] &&
                    marked[cur_pt.i * width + cur_pt.j + 1] == false) {
                new_pt.i = cur_pt.i; new_pt.j = cur_pt.j + 1;
                //ROS_INFO("Adding %d, %d", new_pt.i, new_pt.j);
                marked[cur_pt.i * width + cur_pt.j + 1] = true;
                qpt.push(new_pt);
            }
            if(cur_pt.j + 1 < width && cur_pt.i + 1 < height && 
                    isfrontier_data[(cur_pt.i + 1) * width + cur_pt.j + 1] &&
                    marked[(cur_pt.i + 1) * width + cur_pt.j + 1] == false) {
                new_pt.i = cur_pt.i + 1; new_pt.j = cur_pt.j + 1;
                //ROS_INFO("Adding %d, %d", new_pt.i, new_pt.j);
                marked[(cur_pt.i + 1) * width + cur_pt.j + 1] = true;
                qpt.push(new_pt);
            }
            if(cur_pt.j - 1 >= 0 && cur_pt.i - 1 >= 0 &&
                    isfrontier_data[(cur_pt.i - 1) * width + cur_pt.j - 1] &&
                    marked[(cur_pt.i - 1) * width + cur_pt.j - 1] == false) {
                new_pt.i = cur_pt.i - 1; new_pt.j = cur_pt.j - 1;
                //ROS_INFO("Adding %d, %d", new_pt.i, new_pt.j);
                marked[(cur_pt.i - 1) * width + cur_pt.j - 1] = true;
                qpt.push(new_pt);
            }
            if(cur_pt.j - 1 >= 0 && cur_pt.i + 1 < height &&
                    isfrontier_data[(cur_pt.i - 1) * width + cur_pt.j + 1] &&
                    marked[(cur_pt.i - 1) * width + cur_pt.j + 1] == false) {
                new_pt.i = cur_pt.i - 1; new_pt.j = cur_pt.j + 1;
                //ROS_INFO("Adding %d, %d", new_pt.i, new_pt.j);
                marked[(cur_pt.i - 1) * width + cur_pt.j + 1] = true;
                qpt.push(new_pt);
            }
            if(cur_pt.j + 1 < width && cur_pt.i - 1 >= 0 && 
                    isfrontier_data[(cur_pt.i + 1) * width + cur_pt.j - 1] &&
                    marked[(cur_pt.i + 1) * width + cur_pt.j - 1] == false) {
                new_pt.i = cur_pt.i + 1; new_pt.j = cur_pt.j - 1;
               // ROS_INFO("Adding %d, %d", new_pt.i, new_pt.j);
                marked[(cur_pt.i + 1) * width + cur_pt.j - 1] = true;
                qpt.push(new_pt);
            }
        }
        //ROS_INFO("Extracted frontier of size %d", cur_frontier.size());
        frontiers.push_back(cur_frontier);
        cur_frontier.clear();
        found_start = false;
        //printf("Finding start\n");
        for(i = start_pt.i; i < height; i++) {
            for(j = 0; j < width; j++) {
                //printf("%d %d\n", i, j);
                if(isfrontier_data[i*width + j] && !marked[i*width + j]) {
                    found_start = true;
                    start_pt.i = i;
                    start_pt.j = j;
                    marked[i*width + j] = true;
                    break;
                }
            }
            if(found_start) { break; }
        }
        if(found_start) {
           qpt.push(start_pt);
        }
    }
    //now we have the points that make the frontiers
    //lets sample points and directions from these.
   /* 
    //before that lets just print out everypoint and its direction for debugging
    FILE* fout = fopen("frontier_debug.pcd", "w");
    if(!fout) { ROS_ERROR("Error opening debug file"); }
    
    float rgb;
    int rgb_int;

    for(i = 0; i < (int)frontiers.size(); i++) {
        for(j = 0; j < (int)frontiers[i].size(); j++) {
            geometry_msgs::Pose p;
            p.position.x = frontiers[i][j].j * _ugv_map.info.resolution + _ugv_map.info.origin.position.x;
            p.position.y = frontiers[i][j].i * _ugv_map.info.resolution + _ugv_map.info.origin.position.y;
            rgb_int = i * (255/frontiers.size());
            rgb_int <<= 8;
            rgb_int = rgb_int | ((frontiers.size() - i) * (255/frontiers.size()));
            rgb_int <<= 8;
            rgb_int = rgb_int | (i * (255/frontiers.size()));
            rgb = *reinterpret_cast<float*>(&rgb_int);
            fprintf(fout, "%f %f 0.0 %e\n", p.position.x, p.position.y, rgb);
        }
    }
    fclose(fout);
*/











    for(i = 0; i < (int)frontiers.size(); i++) {
        if((int)frontiers[i].size() > THRESHOLD_FRONTIER_SIZE) {
            for(j = 2; j < (int)frontiers[i].size() - 2; j+=70) {
//printf("j:%d\n",j);
                //at each i,j we will extract a frontier point and pose
                //printf("%d, %d\n", direction_x_data[frontiers[i][j].i * width + frontiers[i][j].j]*5, direction_y_data[frontiers[i][j].i * width + frontiers[i][j].j]*5);
                geometry_msgs::Pose p;
                p.position.x = (frontiers[i][j].j - direction_x_data[frontiers[i][j].i * width + frontiers[i][j].j]*7) * _ugv_map.info.resolution + _ugv_map.info.origin.position.x;
                p.position.y = (frontiers[i][j].i - direction_y_data[frontiers[i][j].i * width + frontiers[i][j].j]*7) * _ugv_map.info.resolution + _ugv_map.info.origin.position.y;
                float direction_xf = 0.0;
                float direction_yf = 0.0;
                float yaw = 0.0;
                btQuaternion rot;
                for(int k = 0; k < 4; k++) {
                    direction_xf += direction_x_data[frontiers[i][j+k].i * width + frontiers[i][j+k].j];
                    direction_yf += direction_y_data[frontiers[i][j+k].i * width + frontiers[i][j+k].j];
                }
                yaw = atan2f(direction_yf, direction_xf);
                rot.setEulerZYX(yaw, 0.0, 0.0);
                p.orientation.x = rot.x();
                p.orientation.y = rot.y();
                p.orientation.z = rot.z();
                p.orientation.w = rot.w();
                _ugv_frontier_poses.poses.push_back(p);
            }
        }
    }
}

/*
void FrontierExtractorServer::findFrontiers(costmap_2d::Costmap2DROS& costmap_) {
    _frontiers.clear();

    costmap_2d::Costmap2D costmap;
    costmap_.getCostmapCopy(costmap);

    int idx;
    int w = costmap.getSizeInCellsX();
    int h = costmap.getSizeInCellsY();
    int size = (w * h);

    _map.info.width = w;
    _map.info.height = h;
    _map.data.resize(size);
    _map.info.resolution = costmap.getResolution();
    _map.info.origin.position.x = costmap.getOriginX();
    _map.info.origin.position.y = costmap.getOriginY();

    // Find all frontiers (open cells next to unknown cells).
    const unsigned char* map = costmap.getCharMap();
    for (idx = 0; idx < size; idx++) {
        //    //get the world point for the index
        //    unsigned int mx, my;
        //    costmap.indexToCells(idx, mx, my);
        //    geometry_msgs::Point p;
        //    costmap.mapToWorld(mx, my, p.x, p.y);
        //
        //check if the point has valid potential and is next to unknown space
        //    bool valid_point = planner_->validPointPotential(p);
        bool valid_point = (map[idx] < costmap_2d::LETHAL_OBSTACLE);

        if ((valid_point && map) &&
                (((idx+1 < size) && (map[idx+1] == costmap_2d::NO_INFORMATION)) ||
                 ((idx-1 >= 0) && (map[idx-1] == costmap_2d::NO_INFORMATION)) ||
                 ((idx+w < size) && (map[idx+w] == costmap_2d::NO_INFORMATION)) ||
                 ((idx-w >= 0) && (map[idx-w] == costmap_2d::NO_INFORMATION))))
        {
            _map.data[idx] = -128;
        } else {
            _map.data[idx] = -127;
        }
    }

    // Clean up frontiers detected on separate rows of the map
    idx = _map.info.height - 1;
    for (unsigned int y=0; y < _map.info.width; y++) {
        _map.data[idx] = -127;
        idx += _map.info.height;
    }

    // Group adjoining _map pixels
    int segment_id = 127;
    std::vector< std::vector<explore::FrontierPoint> > segments;
    for (int i = 0; i < size; i++) {
        if (_map.data[i] == -128) {
            std::vector<int> neighbors;
            std::vector<explore::FrontierPoint> segment;
            neighbors.push_back(i);

            // claim all neighbors
            while (neighbors.size() > 0) {
                int idx = neighbors.back();
                neighbors.pop_back();
                _map.data[idx] = segment_id;

                btVector3 tot(0,0,0);
                int c = 0;
                if ((idx+1 < size) && (map[idx+1] == costmap_2d::NO_INFORMATION)) {
                    tot += btVector3(1,0,0);
                    c++;
                }
                if ((idx-1 >= 0) && (map[idx-1] == costmap_2d::NO_INFORMATION)) {
                    tot += btVector3(-1,0,0);
                    c++;
                }
                if ((idx+w < size) && (map[idx+w] == costmap_2d::NO_INFORMATION)) {
                    tot += btVector3(0,1,0);
                    c++;
                }
                if ((idx-w >= 0) && (map[idx-w] == costmap_2d::NO_INFORMATION)) {
                    tot += btVector3(0,-1,0);
                    c++;
                }
                assert(c > 0);
                segment.push_back(explore::FrontierPoint(idx, tot / c));

                // consider 8 neighborhood
                if (((idx-1)>0) && (_map.data[idx-1] == -128))
                    neighbors.push_back(idx-1);
                if (((idx+1)<size) && (_map.data[idx+1] == -128))
                    neighbors.push_back(idx+1);
                if (((idx-_map.info.width)>0) && (_map.data[idx-_map.info.width] == -128))
                    neighbors.push_back(idx-_map.info.width);
                if (((idx-_map.info.width+1)>0) && (_map.data[idx-_map.info.width+1] == -128))
                    neighbors.push_back(idx-_map.info.width+1);
                if (((idx-_map.info.width-1)>0) && (_map.data[idx-_map.info.width-1] == -128))
                    neighbors.push_back(idx-_map.info.width-1);
                if (((idx+(int)_map.info.width)<size) && (_map.data[idx+_map.info.width] == -128))
                    neighbors.push_back(idx+_map.info.width);
                if (((idx+(int)_map.info.width+1)<size) && (_map.data[idx+_map.info.width+1] == -128))
                    neighbors.push_back(idx+_map.info.width+1);
                if (((idx+(int)_map.info.width-1)<size) && (_map.data[idx+_map.info.width-1] == -128))
                    neighbors.push_back(idx+_map.info.width-1);
            }

            segments.push_back(segment);
            segment_id--;
            if (segment_id < -127)
                break;
        }
    }

    int num_segments = 127 - segment_id;
    if (num_segments <= 0)
        return;

    for (unsigned int i=0; i < segments.size(); i++) {
        explore::Frontier frontier;
        std::vector<explore::FrontierPoint>& segment = segments[i];
        uint size = segment.size();

        //we want to make sure that the frontier is big enough for the robot to fit through
        if (size * costmap.getResolution() < costmap.getInscribedRadius())
            continue;

        float x = 0, y = 0;
        btVector3 d(0,0,0);

        for (uint j=0; j<size; j++) {
            d += segment[j].d;
            int idx = segment[j].idx;
            x += (idx % _map.info.width);
            y += (idx / _map.info.width);
        }
        d = d / size;
        frontier.pose.position.x = _map.info.origin.position.x + _map.info.resolution * (x / size);
        frontier.pose.position.y = _map.info.origin.position.y + _map.info.resolution * (y / size);
        frontier.pose.position.z = 0.0;

        frontier.pose.orientation = tf::createQuaternionMsgFromYaw(btAtan2(d.y(), d.x()));
        frontier.size = size;

        _frontiers.push_back(frontier);
    }

} */

bool FrontierExtractorServer::extract_frontier(
        ptam::FrontierExtractor::Request &req,
        ptam::FrontierExtractor::Response &res)
{
    res.ugv_frontiers.poses.clear();
    res.uav_frontiers.poses.clear();
    res.passive_uav_frontiers.poses.clear();

   
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);

double in_sec=ros::Time::now().toSec();

//while(!is_uav_cloud_ready)
//{


//fprintf(stderr,"inside yeahhh");
ros::spinOnce();
//}
is_ugv_map_ready = 0;
while(!is_ugv_map_ready)
{
ros::spinOnce();

}
    if(is_ugv_map_ready) {
//getchar();
//getchar();
        _ugv_frontier_poses.poses.clear();
       ROS_INFO("beginning to find UGV frontiers");
        findUGVFrontiers();
       fprintf(stderr,"UGV Frontier: Num points: %d\n", _ugv_frontier_poses.poses.size());
//getchar();
_cluster_srv.request.frontiers.poses.clear();
_cluster_srv.request.frontiers=_ugv_frontier_poses;
_cluster_srv.request.threshold=2.5;
if(srv_client.call(_cluster_srv))
{
res.ugv_frontiers =_cluster_srv.response.clustered_frontiers;
}

//        res.ugv_frontiers = _ugv_frontier_poses;
        res.ugv_frontiers.header.frame_id = "/world";
        ugv_frontier_pub.publish(res.ugv_frontiers);
fprintf(stderr,"frontier size %d,%d ",_ugv_frontier_poses.poses.size(),res.ugv_frontiers.poses.size());

//findPassiveFrontiers();
res.passive_uav_frontiers = _passive_uav_frontier_poses;
res.passive_uav_frontiers.header.frame_id = "/world";
ugv_frontier_pub.publish(res.ugv_frontiers);

}
else
{
fprintf(stderr,"map aint here nigga");
getchar();
}
//printf("UGV done\n");
//getchar();
//getchar();
/*
//printf("UgV frontiers %lf:\n",ros::Time::now().toSec()-in_sec);
    if(is_passive_uav_frontier_poses_ready) {

//printf("time to passify");
//getchar();
//getchar();



//std::vector<bool>::iterator iterator;
//iterator=req.passive_checking.begin();

for(int i=0;req.passive_checking.size();i++)
{
if(req.passive_checking[i])




}

fprintf(stderr,"front_no:%d\n", (int)_passive_uav_frontier_poses.poses.size());
        //now to process the passive uav frontier poses
        //first we cluster them and reject small clusters
        pcl::PointCloud<pcl::PointXYZ>::Ptr passive_uav_frontier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ pt;
        FILE* file_passive_uav_pcd = fopen("/media/home/ayushd/ros/iiith_ros_stack/ptam/passive_uav_frontiers.pcd", "w");
        for(int i = 0; i < (int)_passive_uav_frontier_poses.poses.size(); i++) {
            fprintf(file_passive_uav_pcd, "%f %f 0.0\n", _passive_uav_frontier_poses.poses[i].position.x, 
                         _passive_uav_frontier_poses.poses[i].position.y);
            pt.x = _passive_uav_frontier_poses.poses[i].position.x;
            pt.y = _passive_uav_frontier_poses.poses[i].position.y;
            pt.z = 0.0;
            passive_uav_frontier_cloud -> points.push_back(pt);
        }
        fclose(file_passive_uav_pcd);
        passive_uav_frontier_cloud -> height = passive_uav_frontier_cloud -> points.size();
        passive_uav_frontier_cloud -> width = 1;

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (passive_uav_frontier_cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
      //  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

        if(cloud_filtered -> points.size() == 0) {
            res.passive_uav_frontiers.poses.clear();
        }
        else {
        // Creating the KdTree object for the search method of the extraction
        //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_filtered);

        pcl::PCDWriter writer;

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.20); // 20cm
        ec.setMinClusterSize (20);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

  //      ROS_INFO("Clustering of passive uav frontiers complete");
fprintf(stderr,"frontier in result%f",res.passive_uav_frontiers.poses.size());
//getchar();
        int j = 0;
        geometry_msgs::Pose p;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster->points.push_back (cloud_filtered->points[*pit]); /
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

  //          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        //    std::stringstream ss;
    //        ss << "cloud_cluster_" << j << ".pcd";
      //      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); /
            j++;
            for(int k = 5; k < (int)cloud_cluster -> points.size(); k+=20) {
                p.position.x = cloud_cluster -> points[k].x;
                p.position.y = cloud_cluster -> points[k].y;
                p.position.z = _uav_height; //cloud_cluster -> points[k].z;
                p.orientation.x = 0.0;
                p.orientation.y = 0.0;
                p.orientation.z = 0.707;
                p.orientation.w = 0.707;
                res.passive_uav_frontiers.poses.push_back(p);
            }
        }

        
        //then we sample from the large clusters
        btScalar roll, pitch, yaw;
        float direction_xf;
        float direction_yf;
        geometry_msgs::Pose p;
        btQuaternion rot;
        for(int i = 5; i < (int)_passive_uav_frontier_poses.poses.size() - 5; i+=10) {
        direction_xf = 0.0;
        direction_yf = 0.0;
        for(int m = -5; m < 5; m++) {
        rot.setX(_passive_uav_frontier_poses.poses[i+m].orientation.x);
        rot.setY(_passive_uav_frontier_poses.poses[i+m].orientation.y);
        rot.setZ(_passive_uav_frontier_poses.poses[i+m].orientation.z);
        rot.setW( _passive_uav_frontier_poses.poses[i+m].orientation.w);
        btMatrix3x3(rot).getEulerYPR(yaw, pitch, roll);
        direction_xf += cos(yaw);
        direction_yf += sin(yaw);
        }
        rot.setEulerZYX(atan2(direction_yf, direction_xf), 0.0, 0.0);
        p.position = _passive_uav_frontier_poses.poses[i].position;

        //check if the corresponding map cell is occupied. If yes then delete the frontier
        if(is_cloud_occupancy_map_ready) {
        int cur_point_x;
        int cur_point_y;
        cur_point_x = (int)(p.position.x / _cloud_occupancy_map.info.resolution + _cloud_occupancy_map.info.width / 2);
        cur_point_y = (int)(p.position.y / _cloud_occupancy_map.info.resolution + _cloud_occupancy_map.info.height / 2);
        if(_cloud_occupancy_map.data[cur_point_y * _cloud_occupancy_map.info.width + cur_point_x] != mapper_direct::UNKNOWN) {
        ROS_ERROR("Rejected bad uav frontier");
        continue;
        }
        }
        p.orientation.x = rot.x();
        p.orientation.y = rot.y();
        p.orientation.z = rot.z();
        p.orientation.w = rot.w();
        res.passive_uav_frontiers.poses.push_back(p);
        }*/
   //     res.passive_uav_frontiers.header.frame_id = "/world";
  // passive_uav_frontier_pub.publish(res.passive_uav_frontiers);

//printf("Passive UAV frontiers %lf:\n",ros::Time::now().toSec()-in_sec);
//ros::Duration(2.0).sleep();
  //      }
   // }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frontier_extractor_server");

    FrontierExtractorServer server;
    ROS_INFO("Ready to extract frontiers.");
    ros::spin();

    return 0;
}
