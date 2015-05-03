#include "JL1.h"
#include "PrimitiveFunctions.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>


#include "../PTAM/KeyFrame.h"
#include "../PTAM/TrackerData.h"

#include<ros/ros.h>

#include "TooN/TooN.h"
#include "TooN/se3.h"



void JL::laserscan(bool calc=1)
  {
if(calc == 0 && (ros::Time::now().toSec() - lasttime < 0.05))
	return;

if(calc){
ros::Time begin = ros::Time::now();
lasttime = begin.toSec();
rot=jMap.vpKeyFrames[lastkeyframeno]->se3CfromW.inverse().get_rotation().get_matrix();
trans=jMap.vpKeyFrames[lastkeyframeno]->se3CfromW.inverse().get_translation();
translation = btVector3(btScalar(trans[0]), btScalar(trans[1]), btScalar(trans[2]));
mat = btMatrix3x3(rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
btTransform transrot(mat,translation);
transform.setOrigin(translation);
btQuaternion q = transrot.getRotation();
transform.setRotation(q); // tf::Quaternion(msg->theta, 0, 0) );


camera_base_tf.setOrigin( tf::Vector3(0,0.30,0) );
camera_base_tf.setRotation( tf::Quaternion(0,1.540,1.54) );

//fprintf(stderr,"%f,%f\n",min_height_,max_height_);
double min_height_=(-0.20)  , max_height_ = (4.0),
                 angle_min_ = (-M_PI/6),
                 angle_max_ = (M_PI/6),
                 angle_increment_ = (M_PI/180.0/2.0),
                 scan_time_ = (1.0/30.0),
                 range_min_= (0.35),
                 range_max_ = (9.0),    range_min_sq_ = range_min_ * range_min_;


	output->header = cloud.header;
	output->header.frame_id = "base_link_c"; // Set output frame. Point clouds come from "optical" frame, scans come from corresponding mount frame
	output->angle_min = angle_min_;
	output->angle_max = angle_max_;
	output->angle_increment = angle_increment_;
	output->time_increment = 0.0;
	output->scan_time = scan_time_;
	output->range_min = range_min_;
	output->range_max = range_max_;



        uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max );

    for (int it = 0; it < cloud.size(); ++it)
    {
		Vector<3> v;
		v[0] = cloud.points[it].x;
		v[1] = cloud.points[it].y;
		v[2] = cloud.points[it].z;
		v = jMap.vpKeyFrames[lastkeyframeno]->se3CfromW * v;


      const float &x = v[0];
      const float &y = v[1];
      const float &z = v[2];

      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
//        NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
        continue;
      }

      if (-y > max_height_ || -y < min_height_)
      {
    //    fprintf(stderr,"rejected for height %f not in range (%f, %f)\n", x, min_height_, max_height_);
        continue;
      }

      double range_sq = z*z+x*x;
      if (range_sq < range_min_sq_) {
//       fprintf(stderr,"rejected for range %f below minimum value %f. Point: (%f, %f, %f)\n", range_sq, range_min_sq_, x, y, z);
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
////////////////////////////////////////////////////


	for(int j = 1; j < output->ranges.size()-1; j++){
		float b = output->ranges[j-1] - output->ranges[j];
		b = fabs(b);
		if( b > 0.25 && output->ranges[j-1] <8.0){
			for( int k = 1; k < 8 ; k++){
				float kl = fabs(output->ranges[j-1] - output->ranges[j+k]);
				if( kl < 0.3)
					for( int l = 0; l<k ; l++)
						output->ranges[j+l] =output->ranges[j-1] + (kl*(l+1))/(k+1);
			}
		}
	}


///////////////////////////////////////////////////////////
}

br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ptam","current"));

camera_base.sendTransform(tf::StampedTransform(camera_base_tf, ros::Time::now(), "current","base_link_c"));

map_pub1.publish(output);

}


std::vector<float> get_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  	
	std::vector<float> plane;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  // Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients (true);
 	 // Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);

	seg.setInputCloud (cloud);
  	seg.segment (*inliers, *coefficients);

	plane.push_back(coefficients->values[0]);
	plane.push_back(coefficients->values[1]);
	plane.push_back(coefficients->values[2]);
	plane.push_back(coefficients->values[3]);
	return plane;
}

JL::JL(Map &iMap, ATANCamera &iCam, int inSample, int isamplingType, float isamplingCoef, float iinlierThreshold, int iKdTreeRange,int minpts, double iKdTreeCloseProb = 1.0, double iKdTreeFarProb = 1.0 ):jMap(iMap), Cam(iCam), denser(iMap, iCam, 0.25, 0.2, 40, 640,480,3),output(new sensor_msgs::LaserScan()){

	map_pub1 = nh.advertise<sensor_msgs::LaserScan >("/scan",10);
	nSample =inSample;
	samplingType = isamplingType;
	samplingCoef = isamplingCoef;
	inlierThreshold = iinlierThreshold;
	KdTreeRange = iKdTreeRange;	
	KdTreeCloseProb = iKdTreeCloseProb;
	KdTreeFarProb = iKdTreeFarProb;
	mMSS = 3;
	minPts = minpts;
	keyframeno = 0;
	lastkeyframeno = 0;
	ikeyframeno = 0;
//viewer->setBackgroundColor (0,0,0);
//viewer->addCoordinateSystem (1.0);
//viewer->initCameraParameters ();
	start();

std::cerr<<" JL started \n";
}

std::vector<std::vector<float> *> *JL::sample(RandomSampler &mRandomSampler){
	RS_NFSAMPLINGTYPE mNFSamplingType = (RS_NFSAMPLINGTYPE)samplingType;
		
	if(NFST_EXP == mNFSamplingType)
		mRandomSampler.SetNFSamplingTypeExp(samplingCoef);
	if(NFST_NN == mNFSamplingType)
		mRandomSampler.SetNFSamplingTypeNN(samplingCoef,  KdTreeCloseProb, KdTreeFarProb, false);
	if(NFST_NN_ME == mNFSamplingType)
		mRandomSampler.SetNFSamplingTypeNN(samplingCoef, KdTreeCloseProb, KdTreeFarProb, true);
	
	return mRandomSampler.GetNSample(nSample, 0, NULL);
}

void JL::update_point(dataPoint *dP){
	(*(dP->val))[0] = dP->mp->v3WorldPos[0];
	(*(dP->val))[1] = dP->mp->v3WorldPos[1];
	(*(dP->val))[2] = dP->mp->v3WorldPos[2];
}
/*
void JL::delete_point(dataPoint *dP, int i){
	mRandomSampler.RemovePoint(dP->indX );
std::cerr<<"1del\n";
	mJLinkage.RemovePoint(dP->clP);
std::cerr<<"2del\n";
	mDataPoints.erase(mDataPoints.begin()+i);
std::cerr<<i<<"del\n";
}
*/
void JL::add_points(){
	int a=0;
	 for(unsigned int i=0; i<jMap.vpPoints.size(); i++){
	        MapPoint &p= *(jMap.vpPoints[i]);
        	// Ensure that this map point has an associated TrackerData struct.
        	if(!p.pTData) p.pTData = new TrackerData(&p);
        	TrackerData &TData = *p.pTData;
        	// Project according to current view, and if it's not in the image, skip.
        	TData.Project(jMap.vpKeyFrames[keyframeno]->se3CfromW, Cam);//equation 2, function in TrackerData.cc 
    		if(!TData.bInImage)
           		continue;
		
		dataPoint *dp = new dataPoint;
		dp->mp = jMap.vpPoints[i];

		dp->val = new std::vector<float>(3);

		dp->processed = 0;

		(*(dp->val))[0] = dp->mp->v3WorldPos[0];
		(*(dp->val))[1] = dp->mp->v3WorldPos[1];
		(*(dp->val))[2] = dp->mp->v3WorldPos[2];
		cloud_local.push_back(pcl::PointXYZ(dp->mp->v3WorldPos[0], dp->mp->v3WorldPos[1], dp->mp->v3WorldPos[2]));
		mDataPoints.push_back(dp);
	}
}

void JL::load_points(RandomSampler &mRandomSampler, JLinkage &mJLinkage){
	for(int i = 0; i < mDataPoints.size(); i++){
		if(mDataPoints[i]->processed == 0){
			mDataPoints[i]->indX = mRandomSampler.AddPoint(mDataPoints[i]->val);
			mDataPoints[i]->clP = mJLinkage.AddPoint(mDataPoints[i]->val, mDataPoints[i]->mp);
			mDataPoints[i]->processed = 1;
		}
	}
}		



float magnitude(Vector<3> P){
	float mag = sqrt(pow(P[0],2) + pow(P[1],2) + pow(P[2],2));
	return mag;
}


void JL::run(){
// busy = 1
// busy =0	
 while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
{
	if((ikeyframeno < 4) || (jMap.vpPoints.size()<50))
		continue;

	reset();
if(keyframeno == ikeyframeno){
	laserscan(0);
	continue;
}
//cerr<<"got new cloud \n";
keyframeno = ikeyframeno;

//std::cerr<<keyframeno<<"\n";

if(keyframeno == lastkeyframeno){
	laserscan(0);
	continue;
	cerr<<"never \n";
}


//if(keyframeno==0){
//chck = 1;
//continue;
//laserscan(0);
//}
//if( keyframeno != lastkeyframeno)
//{
//keyframeno = jMap.vpKeyFrames.size()-5;


	RandomSampler mRandomSampler( GetFunction_Plane, DistanceFunction_Plane, 3, 3, 0 , false );
	JLinkage mJLinkage( DistanceFunction_Plane, inlierThreshold, nSample, false, 3, KdTreeRange ); 
	add_points();										// add or remove

	if(mDataPoints.size() < 100){
		if(lastkeyframeno != 0)
			laserscan(0);
		continue;
	}

	load_points(mRandomSampler, mJLinkage);							// load to sampler and jlinkage
	
	std::vector<std::vector<float> *> *mModels = sample(mRandomSampler);						// get new model hypothesis 
	for(unsigned int nModelCount = 0; nModelCount < mModels->size(); nModelCount++){
		mJLinkage.AddModel(((*mModels)[nModelCount]));
	}
	std::list<sClLnk *> mClustersList = mJLinkage.DoJLClusterization();
//if(chck == 1)
//laserscan(0);
	if(jMap.densepoints.size() > 0)
		jMap.densepoints.clear();

	unsigned int counterCl = 1;
	for(std::list<sClLnk *>::iterator iterCl = mClustersList.begin(); iterCl != mClustersList.end(); ++iterCl){
		pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>);
		if ((*iterCl)->mBelongingPts.size() > minPts){
			for(std::list<sPtLnk *>::iterator iterPt = (*iterCl)->mBelongingPts.begin(); iterPt != (*iterCl)->mBelongingPts.end(); ++iterPt){
				tcloud->push_back(pcl::PointXYZ((*iterPt)->pMapPoint->v3WorldPos[0], (*iterPt)->pMapPoint->v3WorldPos[1], (*iterPt)->pMapPoint->v3WorldPos[2]));
//				(*iterPt)->pMapPoint->label = counterCl;
			}
			
			std::vector<float> plane = get_plane(tcloud);
			Vector<3> norm,zero;
			norm[0] =  plane[0];
			norm[1] =  plane[1];
			norm[2] =  plane[2];
			//norm = jMap.vpKeyFrames[keyframeno]->se3CfromW*norm ;
			//zero = jMap.vpKeyFrames[keyframeno]->se3CfromW*zero ;
			//norm = norm - zero;
			float angle = (acos(fabs(norm[1]/(magnitude(norm)))) * 180) /(2*3.14);
			if(angle<30){
				continue;
			}				
			if(jMap.vpKeyFrames.size()>4){
				if(lastkeyframeno != 0)
				laserscan(0);
				cloud = cloud + *denser.make_dense(tcloud, keyframeno);
			}
		}
		++counterCl;
	}


if(cloud.size() > 50){
	jMap.densepoints = cloud;
	lastkeyframeno = keyframeno;
	laserscan();

//cerr<<" scan  -------------- >   "<<keyframeno<<"         "<<cloud.size()<<endl;

std::stringstream ss;

//ss<<"dense/"<<keyframeno<<".pcd";
//writer.write (ss.str(), cloud_local, false);
//ss.str("");
//ss<<"dense/"<<keyframeno<<"d.pcd";
//writer.write (ss.str(), denser.colcloudT, false);

}
else{
	cerr<<"skip\n";
	if(lastkeyframeno!=0)
		laserscan(0);
}

//laserscan(0);
/*if( cloud.size() >100){
	pcl::PointCloud<pcl::PointXYZ> newcloud;
	try{
	pcl::PCDReader reader;
	reader.read("planedense/test.pcd",newcloud);
	}catch(int e){
	}
//	if(newcloud.size() == 0)
		newcloud = newcloud + cloud;

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
icp.setInputCloud(cloud.makeShared());
icp.setInputTarget(newcloud.makeShared());
pcl::PointCloud<pcl::PointXYZ> Final;
icp.align(Final);

pcl::VoxelGrid<pcl::PointXYZ > sor;
sor.setInputCloud (newcloud.makeShared());
sor.setLeafSize (0.03f, 0.03f, 0.03f);
sor.filter (newcloud);
*/
/*}

    cloud.is_dense = false;
    cloud.header.seq = keyframeno;
    cloud.header.frame_id ="/camera_frame";
    cloud.header.stamp = ros::Time::now();
cloud.width = cloud.points.size();
    cloud.height = 1;

	map_pub1.publish(cloud.makeShared());
*/
//stringstream sss;
//sss<<keyframeno;
//viewer->addPointCloud<pcl::PointXYZ> (cptr,sss.str());
//std::cerr<<"\033[A\033[2K";
//}
//else{
//laserscan(0);
//}
}

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
void JL::reset(){
	for(unsigned int i=0; i<mDataPoints.size(); i++)
		delete (mDataPoints)[i];
	mDataPoints.clear();
	if(cloud.size()>0)
		cloud.clear();
	if(denser.colcloudT.size()>0)
		denser.colcloudT.clear();
	if(cloud_local.size()>0)
		cloud_local.clear();




//stringstream sss;
//sss<<keyframeno;
	//viewer->removePointCloud(sss.str());
//cerr<<"reset\n";
}

JL::~JL(){
	for(unsigned int i=0; i<mDataPoints.size(); i++)
		delete (mDataPoints)[i];
}
