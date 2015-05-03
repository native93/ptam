#include "RandomSampler.h"
#include "JLinkage.h"
#include <sensor_msgs/PointCloud2.h>
#include<ros/ros.h>
 #include <pcl/ros/conversions.h>

#include "sensor_msgs/LaserScan.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btQuaternion.h>
#include <tf/transform_broadcaster.h>


#include <ctime>
#include <cvd/thread.h>
#include "densify.h"

using namespace TooN;

struct dataPoint{
	std::vector<float> *val;
	MapPoint *mp;
	int indX;
	sPtLnk *clP;
	int processed;
};

class JL : protected CVD::Thread{

	public:

		JL(Map &iMap, ATANCamera &iCam, int inSample, int isamplingType, float isamplingCoef, float iinlierThreshold, int iKdTreeRAnge, int minpts, double iKdTreeCloseProb = 1.0, double iKdTreeFarProb = 1.0 );
		~JL();
virtual void run(); 

		int keyframeno, lastkeyframeno,ikeyframeno;
	private:
		                      			//thread    
		         
		/// COMMON
		int padded;
		Map &jMap;
		ATANCamera &Cam;
		int minPts;
		pcl::PointCloud<pcl::PointXYZ> cloud,cloud_local;
		void reset();
		
		std::vector<dataPoint*> mDataPoints; // Data Points
		void add_points();
		void update_point(dataPoint*);
		void load_points(RandomSampler &mRandomSampler, JLinkage &mJLinkage);
		void delete_point(dataPoint *dP, int i);
		/// SAMPLE		
		
		int samplingType;				//sampling type
		int nSample;     			        //   no of samples
		int mMSS;                                   //   PLANE
		float samplingCoef; 				//sampling variables

		double KdTreeCloseProb;				// kdtree search
		double KdTreeFarProb;				// kdtree search
		
		std::vector<std::vector<float> *> *sample(RandomSampler &mRandomSampler); 
		
		densify denser;

		
		//// CLUSTER				
		
		int KdTreeRange;				//clustering variable
		int planepointsthresh;				//min point	s in cluster
		float inlierThreshold;				//threshold for inliers


		void laserscan(bool calc);
	/// HELPER FUNCTIONS		
    sensor_msgs::LaserScanPtr output;
		ros::NodeHandle nh;				
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		ros::Publisher map_pub1;



tf::TransformBroadcaster br;
tf::Transform transform;

btMatrix3x3 mat; 
btVector3 translation;

Matrix<3,3> rot;
Vector<3>trans;

tf::TransformBroadcaster camera_base;
tf::Transform camera_base_tf;

double lasttime ;


	pcl::PCDWriter writer;
};
