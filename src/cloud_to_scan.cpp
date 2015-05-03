#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include<ptam/points_visible.h>
#include<vector>

using namespace std;
class pcl_to_scan
{
private:

ros::NodeHandle nh;
  float  min_height_ ;
   float  max_height_ ;
     float angle_min_ ;
  float   angle_max_ ;
    float angle_increment_ ;
    float scan_time_ ;
    float range_min_ ;
    float range_max_ ;
int start_index;
float range_value;
vector<int>range_to_be_fixed;
    float time_increment_ ;
    float range_min_sq_ ;
ros::Publisher pub_;
ros::Subscriber pcl_sub;
ros::Subscriber points_sub;
int points;
public:
pcl_to_scan()
{
    angle_min_ = -0.5234;
     angle_max_ = 0.5234;
     angle_increment_=0.017255;
     time_increment_ = 0.0;
     scan_time_ = 1.0/30.0;
     range_min_ = 0.45;
     range_max_ = 5.0;
max_height_=4.0;

min_height_=0.0;
range_min_sq_ = range_min_ * range_min_;



 points_sub=nh.subscribe("/visible_points",10,&pcl_to_scan::points_callback,this);
 pub_=nh.advertise<sensor_msgs::LaserScan>("/scan",10);
 pcl_sub=nh.subscribe("/cloud_throttled",10,&pcl_to_scan::pcl_callback,this);
}
void pcl_to_scan::points_callback(const ptam::points_visible::ConstPtr& msg);
void pcl_to_scan::pcl_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
}; 

void pcl_to_scan::points_callback(const ptam::points_visible::ConstPtr& msg)
{
points=msg->points_visible;
}
void pcl_to_scan::pcl_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{

if(points<300)
return;

sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = cloud->header;
    output->header.frame_id = "base_link"; // Set output frame. Point clouds come from "optical" frame, scans come from corresponding mount frame
    output->angle_min = -0.5234;
    output->angle_max = 0.5234;
    output->angle_increment =0.017255;
    output->time_increment = 0.0;
    output->scan_time = 1.0/30.0;
    output->range_min = 0.45;
    output->range_max = 5.0;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->begin(); it != cloud->end(); ++it)
    {
      const float &x = it->x;
      const float &y = it->y;
      const float &z = it->z;

      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
     //   NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
        continue;
      }
      if (-y > max_height_ || -y < min_height_)
      {
    //    fprintf(stderr,"rejected for height %f not in range (%f, %f)\n", x, min_height_, max_height_);
        continue;
      }

      double range_sq = z*z+x*x;
      if (range_sq < range_min_sq_) {
    //   fprintf(stderr,"rejected for range %f below minimum value %f. Point: (%f, %f, %f)\n", range_sq, range_min_sq_, x, y, z);
        continue;
      }

      double angle = -atan2(x, z);
      if (angle < output->angle_min || angle > output->angle_max)
      {
      //  fprintf(stderr,"rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
        continue;
      }
      int index = (angle - output->angle_min) / output->angle_increment;


      if (output->ranges[index] * output->ranges[index] > range_sq)
        output->ranges[index] = sqrt(range_sq);
      }
for(int i=0;i<output->ranges.size();i++)
{
if(output->ranges[i]<output->range_max)
{

if(range_to_be_fixed.size()>0&&range_to_be_fixed.size()<4)
{
for(int k=0;k<range_to_be_fixed.size();k++)
{
output->ranges[range_to_be_fixed[k]]=output->ranges[start_index];
}
range_to_be_fixed.clear();
}
else
start_index=i;

}
else
{

if(range_to_be_fixed.size()>3&&start_index!=-1)
range_to_be_fixed.clear();
else
range_to_be_fixed.push_back(i);
}

}

range_to_be_fixed.clear();
start_index=0;
range_value=output->ranges[0];
for(int i=1;i<output->ranges.size();i++)
{
if(fabs(output->ranges[i]-range_value)<1.0)
{

if(range_to_be_fixed.size()>0&&range_to_be_fixed.size()<4)
{
for(int k=0;k<range_to_be_fixed.size();k++)
{
output->ranges[range_to_be_fixed[k]]=output->ranges[start_index];
}
range_to_be_fixed.clear();
}
else
start_index=i;
range_value=output->ranges[i];

}
else
{

if(range_to_be_fixed.size()>3&&start_index!=-1)
range_to_be_fixed.clear();
else


range_value=output->ranges[i];
range_to_be_fixed.push_back(i);
}

}


  pub_.publish(output);
  }



int main(int argc,char **argv)
{

    ros::init(argc, argv, "scan");
pcl_to_scan s;

ros::spin();
}































/*



namespace pointcloud_to_laserscan
{
class CloudToScan : public nodelet::Nodelet
{
public:
  //Constructor
  CloudToScan(): min_height_(0.10),
                 max_height_(0.15),
                 angle_min_(-M_PI/2),
                 angle_max_(M_PI/2),
                 angle_increment_(M_PI/180.0/2.0),
                 scan_time_(1.0/30.0),
                 range_min_(0.45),
                 range_max_(10.0),
                 output_frame_id_("/kinect_depth_frame")
  {
  };

  ~CloudToScan()
  {
    delete srv_;
  }

private:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  boost::mutex connect_mutex_;
  // Dynamic reconfigure server
  dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>* srv_;

  virtual void onInit()
  {
    nh_ = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("min_height", min_height_);
    private_nh.getParam("max_height", max_height_);

    private_nh.getParam("angle_min", angle_min_);
    private_nh.getParam("angle_max", angle_max_);
    private_nh.getParam("angle_increment", angle_increment_);
    private_nh.getParam("scan_time", scan_time_);
    private_nh.getParam("range_min", range_min_);
    private_nh.getParam("range_max", range_max_);

    range_min_sq_ = range_min_ * range_min_;

    private_nh.getParam("output_frame_id", output_frame_id_);

    srv_ = new dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>(private_nh);
    dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>::CallbackType f = boost::bind(&CloudToScan::reconfigure, this, _1, _2);
    srv_->setCallback(f);

    // Lazy subscription to point cloud topic
    ros::AdvertiseOptions scan_ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      "scan", 10,
      boost::bind( &CloudToScan::connectCB, this),
      boost::bind( &CloudToScan::disconnectCB, this), ros::VoidPtr(), nh_.getCallbackQueue());

    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_ = nh_.advertise(scan_ao);
  };

  void connectCB() {
      boost::lock_guard<boost::mutex> lock(connect_mutex_);
      if (pub_.getNumSubscribers() > 0) {
          NODELET_DEBUG("Connecting to point cloud topic.");
          sub_ = nh_.subscribe<PointCloud>("cloud", 10, &CloudToScan::callback, this);
      }
  }

  void disconnectCB() {
      boost::lock_guard<boost::mutex> lock(connect_mutex_);
      if (pub_.getNumSubscribers() == 0) {
          NODELET_DEBUG("Unsubscribing from point cloud topic.");
          sub_.shutdown();
      }
  }

  void reconfigure(pointcloud_to_laserscan::CloudScanConfig &config, uint32_t level)
  {
    min_height_ = config.min_height;
    max_height_ = config.max_height;
    angle_min_ = config.angle_min;
    angle_max_ = config.angle_max;
    angle_increment_ = config.angle_increment;
    scan_time_ = config.scan_time;
    range_min_ = config.range_min;
    range_max_ = config.range_max;

    range_min_sq_ = range_min_ * range_min_;
  }

  void callback(const PointCloud::ConstPtr& cloud)
  {

//fprintf(stderr,"%f,%f\n",min_height_,max_height_);

    

  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_, range_min_sq_;
  std::string output_frame_id_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

PLUGINLIB_DECLARE_CLASS(pointcloud_to_laserscan, CloudToScan, pointcloud_to_laserscan::CloudToScan, nodelet::Nodelet);
}
*/
