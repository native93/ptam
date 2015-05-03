#include <ptam/read_point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>

read_pointcloud::read_pointcloud()
{
std::stringstream ss;
int max_no_of_ptam_maps=1;
uav_cloud_sub.resize(max_no_of_ptam_maps);
pointcloud_received=false;
cloud.resize(max_no_of_ptam_maps);
uav_cloud_sub[0] = nh.subscribe("/ptam/map_points", 1, &read_pointcloud::uav_cloud_callback, this); // /ptam/map_points for point cloud
ros::Publisher pub;
    for(int i = 0; i < 3; i++)
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
  //  is_uav_cloud_ready = true;
}

void read_pointcloud::calculate_plane()
{
	printf("inside calculate plane\n");
pcl::PointCloud<pcl::PointXYZ> cloud_planar;
pcl::PointCloud<pcl::PointXYZ> cloud_inliers;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ>);

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

//  if(angle b/w planes =~90) { find eqation of line at intersection}
//  #set line as a corner{we are still maintaining the assumption that the room walls are orthogonal to each other.
//  if(holes are present)
//

std::cerr << "inlier values "<<std::endl;
int count=0;
for(count=0;count <  inliers->indices.size (); count++)
{
//std::cerr << inliers->indices[count]<< std::endl;
cloud_inliers.points.push_back(cloud[0].points[inliers->indices[count]]);
}
std::cerr << "the number of points are " << count << std::endl;




int points_bad=0;int points_good=0;
for(int i = 0; i < (int)cloud[0].points.size(); i+=1)
    {
//fprintf(stderr,"z coordinates:%f",fabs(map_cloud->points[i].z));
//if(fabs(cloud[0].points[i].z) < 0.5)
	if (fabs(coefficients->values[0]*cloud[0].points[i].x +coefficients->values[1]*cloud[0].points[i].y+coefficients->values[2]*cloud[0].points[i].z+coefficients->values[3]) < 0.2) 
	{
        cloud_inliers.points.push_back(cloud[0].points[i]);
        	points_good++;
	}
	else 
	{
		points_bad++;
		cloud_planar.points.push_back(cloud[0].points[i]);
	}
    }

std::cerr << " points that satisfy are " << points_good << "   and ones that are bad are  " << points_bad << std::endl;


cloud_planar.is_dense=true;
cloud_planar.width=cloud_planar.points.size();
cloud_planar.height=1;
cloud_planar.header.frame_id="/world";


cloud_inliers.is_dense=true;
cloud_inliers.width=cloud_inliers.points.size();
cloud_inliers.height=1;
cloud_inliers.header.frame_id="/map";
while(ros::ok())
{
map_pubs[1].publish(cloud_planar);  // for points that do not belong to the plane
map_pubs[2].publish(cloud_inliers); // for points that are inliers.
ros::Duration(.1).sleep();
}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"readin");
	while(1)
	{
		read_pointcloud read;
		while(!read.pointcloud_received)
		{
			ros::spinOnce();
			//read.readpcd();
		}
		read.calculate_plane();
	}
return 0;
}
