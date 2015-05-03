// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and 
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the 
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>
#include <sensor_msgs/PointCloud2.h>
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>

#include<ptam/points_visible.h>
#include<ptam/Frontier_check.h>
  // PCL specific includes
#include<ptam/pos_robot.h>
 #include <pcl/ros/conversions.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
#include<ptam/RandT.h>
#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
//#include "DenseMapMaker.h"
#include <queue>
#include "TrackerData.h"

//#include <cv.h>
//#include <cvaux.h>
//#include <highgui.h>
class TrackerData;
struct MapMakerData
{
  std::set<KeyFrame*> sMeasurementKFs;   // Which keyframes has this map point got measurements in?
  std::set<KeyFrame*> sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
  inline int GoodMeasCount()            
  {  return sMeasurementKFs.size(); }
};

// MapMaker dervives from CVD::Thread, so everything in void run() is its own thread.
class MapMaker : protected CVD::Thread
{

private:

ros::NodeHandle nh;
      ros::NodeHandle nh_private;
 //     std::string map_frame_id;
      ros::Publisher map_pub;
ros::Publisher points_pub;
ros::ServiceServer service;

ptam::points_visible pvs_pub;
public:

int keyframeadded;

bool frontier_check(
                ptam::Frontier_check::Request &req,
                ptam::Frontier_check::Response &res);

SE3<>camera_world;
ros::Publisher pos_publish;
ptam::pos_robot position_robot;
float ratio_pvs;
int count_pvs;

nav_msgs::Path path;
ros::Publisher trajectory;
ptam::RandT randt;
SE3<>transform_world_ptam;    
SE3<>transform_world;
void save_points(int count,SE3<>se3camfromworld);
 void PublishMap(SE3<>se3camfromworld);
  MapMaker(Map &m, const ATANCamera &cam);
  ~MapMaker();
  //DenseMapMaker mDmm;
  //bool mbAddDense;
  //bool mbAGP;



int number_of_points;
//init stereo for ultrasound reading
  bool InitFromStereo_altitude(KeyFrame &kFirst, KeyFrame &kSecond, 
		      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
		      SE3<> &se3CameraPos,float scale);
  
//This is the actual function, commented to  bring in sonar reading from drone-ayush
  // Make a map from scratch. Called by the tracker.
 // bool InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond, 
//		      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
//		      SE3<> &se3CameraPos);
  
  bool InitFromStereoPlus(KeyFrame &kFirst, KeyFrame &kSecond, 
		      	  std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
		      	  SE3<> &se3CameraPos);

  void AddKeyFrame(KeyFrame &k);   // Add a key-frame to the map. Called by the tracker.
  void RequestReset();   // Request that the we reset. Called by the tracker.
  bool ResetDone();      // Returns true if the has been done.
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} // How many KFs in the queue waiting to be added?
  bool NeedNewKeyFrame(KeyFrame &kCurrent);            // Is it a good camera pose to add another KeyFrame?
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)
  int LatestKeyFrameId();

  //E
  //void TriangulateDense(KeyFrame &kSrc, KeyFrame &kTarget, std::vector<std::pair<TooN::Vector<2>,TooN::Vector<2> > > vDenseMatches);
  //std::vector<TooN::Vector<3> >  MaskPoints(KeyFrame &kMaskFrame, const char *maskPath);
  //std::vector<Vector<3> > FindMaskPlane(std::vector<TooN::Vector<3> > vMaskPt);
	
	
	enum enumStatus {STAT_IDLE, STAT_LOCAL_BA, STAT_REFIND_NEW_PTS, STAT_GLOBAL_BA, STAT_REFIND_OUTLIERS,
				STAT_HANDLE_BAD_PTS, STAT_ADD_KEYFRAME,STAT_CONST_BA};
	int  GetStatus() { return mbMapMakerStatus ;} // Get the current Status of the MapMaker
	void PrintStatus(); // Prints the current Status String of the MapMaker
	
	void CalcGroundPlane();		// Compute Ground Plane
	double GetWiggleScale() { return mdWiggleScale; }	// get Map Init Scale
  
protected:
  
  Map &mMap;               // The map
  ATANCamera mCamera;      // Same as the tracker's camera: N.B. not a reference variable!
  virtual void run();      // The MapMaker thread code lives here

//    pcl::PointCloud<pcl::PointXYZ> cloud;
  // Functions for starting the map from scratch:
  SE3<> CalcPlaneAligner();
  void ApplyGlobalTransformationToMap(SE3<> se3NewFromOld);
  void ApplyGlobalScaleToMap(double dScale);
  
  // Compute Dominant Plane Used by CalcPlaneAligner() and CalcGroundPlane()
  int FindDominantPlane();
  
  // Map expansion functions:
  void AddKeyFrameFromTopOfQueue();  
  void ThinCandidates(KeyFrame &k, int nLevel);
  void AddSomeMapPoints(int nLevel);
  void AddOFPoints();
  bool AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate);
  bool AddOFPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, Vector<2> point);
  std::vector<TooN::Vector<2> > FindExtraPointsh(int nFirstFrame, int nSecondFrame);

  //E
  //bool AddPt(int segment);//Method to add dense points to the map.
  //bool AddGroundPoints(int segment);
  //void RotatePlane();//Method to rotate the dense plane.
  //void WriteObjectsToFile(KeyFrame kFrame);

  // Returns point in ref frame B
  Vector<3> ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B);
  
  // Bundle adjustment functions:
  void BundleAdjust(std::set<KeyFrame*>, std::set<KeyFrame*>, std::set<MapPoint*>, bool);
  void BundleAdjustAll();
  void BundleAdjustRecent();

  void BundleAdjustConstraint();
  // Data association functions:
  int ReFindInSingleKeyFrame(KeyFrame &k);
  void ReFindFromFailureQueue();
  void ReFindNewlyMade();
  void ReFindAll();
  bool ReFind_Common(KeyFrame &k, MapPoint &p);
  void SubPixelRefineMatches(KeyFrame &k, int nLevel);
  
  // General Maintenance/Utility:
  void Reset();
  void HandleBadPoints();
  double DistToNearestKeyFrame(KeyFrame &kCurrent);
  double KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2);
  KeyFrame* ClosestKeyFrame(KeyFrame &k);
  std::vector<KeyFrame*> NClosestKeyFrames(KeyFrame &k, unsigned int N);
  void RefreshSceneDepth(KeyFrame *pKF);
  

  // GUI Interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
  

  // Member variables:
  std::vector<KeyFrame*> mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
  std::vector<std::pair<KeyFrame*, MapPoint*> > mvFailureQueue; // Queue of failed observations to re-find
  std::queue<MapPoint*> mqNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames
  
  double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
                         // This sets the scale of the map
  GVars3::gvar3<double> mgvdWiggleScale;   // GVar for above
  double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth, 
                                        // this controls keyframe separation
  
  bool mbBundleConverged_Full;    // Has global bundle adjustment converged?
  bool mbBundleConverged_Recent;  // Has local bundle adjustment converged?
  
  // Thread interaction signalling stuff
  bool mbResetRequested;   // A reset has been requested
  bool mbResetDone;        // The reset was done.
  bool mbBundleAbortRequested;      // We should stop bundle adjustment
  bool mbBundleRunning;             // Bundle adjustment is running
  bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.
  //int mnSegment;
	
	enumStatus mbMapMakerStatus;

  
};

#endif
