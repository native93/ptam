#include <ptam/VisibilityCalculatorServer.h>

VisibilityCalculatorServer::VisibilityCalculatorServer()
{
    _is_ground_map_received = false;
    ugv_map_sub = nh.subscribe("/mapping_visib", 10, &VisibilityCalculatorServer::ugv_map_callback, this);
    service = nh.advertiseService("VisibilityCalculator", &VisibilityCalculatorServer::calculate_visibility, this);
}

bool VisibilityCalculatorServer::calculate_visibility(
                ptam::VisibilityCalculator::Request &req,
                ptam::VisibilityCalculator::Response &res)
{
    res.ugv_frontier_visibility.clear();
    ROS_INFO("Calculating Visibility");
    int i;
_is_ground_map_received=false;
while(!_is_ground_map_received)
{

ros::spinOnce();
}
    ROS_INFO("UGV frontiers NOS: %d", req.ugv_frontiers.poses.size());
    if(_is_ground_map_received) {
        for(i = 0; i < (int)req.ugv_frontiers.poses.size(); i++) {
fprintf(stderr,"%d\n",i);
            res.ugv_frontier_visibility.push_back(calVisibilityUGV(req.ugv_frontiers.poses[i]));
        }
    }

    

fprintf(stderr,"returning\n");
    return true;
}

void VisibilityCalculatorServer::ugv_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    _ground_map = *msg;
    _is_ground_map_received = true;
}



float VisibilityCalculatorServer::calVisibilityUGV(geometry_msgs::Pose &p) {
    tf::Transform T;
    tf::Vector3 position;
    position.setX(p.position.x);
    position.setY(p.position.y);
    position.setZ(p.position.z);

    btQuaternion orientation;
    orientation.setX(p.orientation.x);
    orientation.setY(p.orientation.y);
    orientation.setZ(p.orientation.z);
    orientation.setW(p.orientation.w);
double yaw,pitch,roll;

   // T.setOrigin(position);
   // T.setRotation(orientation)
btMatrix3x3 mat(orientation);
mat.getEulerZYX(yaw,pitch,roll);


    T.setOrigin(position);
    T.setRotation(orientation);
    tf::Vector3 pt;
    tf::Vector3 pt_transformed;
    int x;
    int y;
    int width = _ground_map.info.width;
    int height = _ground_map.info.height;
    const float R_MAX = 4.0;
    float visibility = 0.0;
    for(float theta = yaw-0.534; theta <= yaw+0.534; theta+=0.15) {
        float r_effective = 0.0f;
        for(float r = 0; r < R_MAX; r+=0.1) {
            pt.setX(r * cos(theta));
            pt.setY(r * sin(theta));
            pt.setZ(0.0);
            pt.setW(1.0);
            pt_transformed = T * pt;
            x = pt_transformed.getX() / _ground_map.info.resolution + width/2;
            y = pt_transformed.getY() / _ground_map.info.resolution + height/2;
            if(x >= 0 && x < width && y>=0 && y< height) {
                if(_ground_map.data[y*width + x] >= 25) {
                    break;
                }
                else if(_ground_map.data[y*width + x] == mapper_direct::UNKNOWN) {
                    visibility += 0.1/R_MAX;
                }
            }
            else {
                break;
            }
        }
    }
    return visibility;
}



int main(int argc, char* argv[]) {
    ros::init(argc, argv, "visibility_calculator_server");

    VisibilityCalculatorServer server;
    ROS_INFO("Ready to calculate visibility");

    ros::spin();
    return 0;
}


