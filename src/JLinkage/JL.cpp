#include "JL.h"
#include "PrimitiveFunctions.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

std::vector<float> *get_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  	
	std::vector<float> *plane = new std::vector<float>(3);
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

	plane->push_back(coefficients->values[0]);
	plane->push_back(coefficients->values[1]);
	plane->push_back(coefficients->values[2]);
	plane->push_back(coefficients->values[3]);
	return plane;
}

JL::JL(Map *iMap, ATANCamera *iCam, pcl::PointCloud<pcl::PointXYZ> icloud, int inSample, int isamplingType, float isamplingCoef, float iinlierThreshold, int iKdTreeRange,int minpts, double iKdTreeCloseProb = 1.0, double iKdTreeFarProb = 1.0 ):mRandomSampler(GetFunction_Plane, DistanceFunction_Plane, 3, 3,0,false), mJLinkage(DistanceFunction_Plane, iinlierThreshold, inSample, false, 3, iKdTreeRange),denser(iMap, iCam, 0.2, 0.1, 100, 640,480,5) {
	jMap = iMap;
	Cam = iCam;
	dCloud = icloud;
	nSample =inSample;
	samplingType = isamplingType;
	samplingCoef = isamplingCoef;
	inlierThreshold = iinlierThreshold;
	KdTreeRange = iKdTreeRange;	
	KdTreeCloseProb = iKdTreeCloseProb;
	KdTreeFarProb = iKdTreeFarProb;
	mMSS = 3;
	minPts = minpts;
	start();
std::cerr<<" JL started \n";
}

std::vector<std::vector<float> *> *JL::sample(){
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

void JL::delete_point(dataPoint *dP, int i){
	mRandomSampler.RemovePoint(dP->indX );
std::cerr<<"1del\n";
	mJLinkage.RemovePoint(dP->clP);
std::cerr<<"2del\n";
	mDataPoints.erase(mDataPoints.begin()+i);
std::cerr<<i<<"del\n";
}

void JL::add_points(){
	int a=0;
	for(size_t i=0; i<jMap->vpPoints.size(); i++){
		if(i+a < mDataPoints.size()){
			if(jMap->vpPoints[i] == mDataPoints[i+a]->mp){
				update_point(mDataPoints[i+a]);
				continue;
			}
			else{
				a++;
				//delete_point(mDataPoints[i+a],i);
				i--;
				continue;
			}		
		}

		dataPoint *dp = new dataPoint;
		dp->mp = jMap->vpPoints[i];

		dp->val = new std::vector<float>(3);

		dp->processed = 0;

		(*(dp->val))[0] = dp->mp->v3WorldPos[0];
		(*(dp->val))[1] = dp->mp->v3WorldPos[1];
		(*(dp->val))[2] = dp->mp->v3WorldPos[2];

		mDataPoints.push_back(dp);
	}
}

void JL::load_points(){
	for(int i = 0; i < mDataPoints.size(); i++){
		if(mDataPoints[i]->processed == 0){
			mDataPoints[i]->indX = mRandomSampler.AddPoint(mDataPoints[i]->val);
			mDataPoints[i]->clP = mJLinkage.AddPoint(mDataPoints[i]->val, mDataPoints[i]->mp);
			mDataPoints[i]->processed = 1;
		}
	}
}		
			
void JL::run(){
// busy = 1
// busy =0	
 while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
	{
std::cerr<<"here\n";
	padded=0;
std::cerr<<"-----\n";
if (jMap->vpPoints.size() - mDataPoints.size()>100){
	add_points();						// add or remove

	load_points();							// load to sampler and jlinkage


	

	std::vector<std::vector<float> *> *mModels = sample();						// get new model hypothesis 
	for(unsigned int nModelCount = 0; nModelCount < mModels->size(); nModelCount++){
		mJLinkage.AddModel(((*mModels)[nModelCount]));
	}
	std::list<sClLnk *> mClustersList = mJLinkage.DoJLClusterization();
	unsigned int counterCl = 1;
	for(std::list<sClLnk *>::iterator iterCl = mClustersList.begin(); iterCl != mClustersList.end(); ++iterCl){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if ((*iterCl)->mBelongingPts.size() > minPts){
			for(std::list<sPtLnk *>::iterator iterPt = (*iterCl)->mBelongingPts.begin(); iterPt != (*iterCl)->mBelongingPts.end(); ++iterPt){
				cloud->push_back(pcl::PointXYZ((*iterPt)->pMapPoint->v3WorldPos[0], (*iterPt)->pMapPoint->v3WorldPos[0], (*iterPt)->pMapPoint->v3WorldPos[0]));
				(*iterPt)->pMapPoint->label = counterCl;
			}
		//std::vector<float> *plane = get_plane(cloud);
		//Plane.push_back(plane);
			if(jMap->vpKeyFrames.size()>10)
				denser.make_dense(cloud, jMap->vpKeyFrames.size()-1);
		}
		++counterCl;
	}
}
std::cerr<<"\033[A\033[2K\033[A\033[2K";
}

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

JL::~JL(){
	for(unsigned int i=0; i<mDataPoints.size(); i++)
		delete (mDataPoints)[i];
	
	
}
