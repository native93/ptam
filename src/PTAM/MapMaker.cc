// Copyright 2008 Isis Innovation Limited
#include "MapMaker.h"
#include "MapPoint.h"
#ifdef _multi
#include "Bundle_multi.h"
#else
#include "Bundle.h"
#endif
#include<math.h>
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "HomographyInit.h"
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btQuaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//AuxTracking by opencv & suryansh code
//#include "AuxTrack/track.h"

#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/image_interpolate.h>
#include <cvd/timer.h>

#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

#include <gvars3/instances.h>
#include <fstream>
#include <algorithm>
#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
//#include "stopwatch.h"

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace CVD;
using namespace std;
using namespace GVars3;
//using namespace TooN;

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map& m, const ATANCamera &cam)
  : mCamera(cam), mMap(m)//, mDmm(m, cam)
{
count_pvs=0;
path.header.frame_id="world";
trajectory=nh.advertise<nav_msgs::Path>("Trajectory",1);
pos_publish=nh.advertise<ptam::pos_robot>("/robot_pose",1);
points_pub=nh.advertise<ptam::points_visible>("/visible_points",1);

  //TrackerData::irImageSize = mirSize;
map_pub=nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/ptam/map_points",1);
number_of_points=0;

service=nh.advertiseService("/Frontier_check",&MapMaker::frontier_check,this);
  cout<<"MapMaker Thread starting..."<<endl;
  mbResetRequested = false;
  Reset();
  start(); // This CVD::thread func starts the map-maker thread with function run()
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GV3::Register(mgvdWiggleScale, "MapMaker.WiggleScale", 0.1, SILENT);
randt.translation.resize(3);
randt.rotation.resize(9);
 // Default to 10cm between keyframes
  //mnSegment = 0;
  //mbAddDense=false;
keyframeadded=0;
};

void MapMaker::Reset()
{
  // This is only called from within the mapmaker thread...
  mMap.Reset();
  mvFailureQueue.clear();
  while(!mqNewQueue.empty()) mqNewQueue.pop();
  mMap.vpKeyFrames.clear(); // TODO: actually erase old keyframes
  mvpKeyFrameQueue.clear(); // TODO: actually erase old keyframes
  mbBundleRunning = false;
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  mbResetDone = true;
  mbResetRequested = false;
  mbBundleAbortRequested = false;
	mbMapMakerStatus = STAT_IDLE;
}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

void MapMaker::run()
{

#ifdef WIN32
  // For some reason, I get tracker thread starvation on Win32 when
  // adding key-frames. Perhaps this will help:
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif

  while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
	{
		CHECK_RESET;
		sleep(5); // Sleep not really necessary, especially if mapmaker is busy
		CHECK_RESET;
		
		// Handle any GUI commands encountered..
		while(!mvQueuedCommands.empty())
		{
			GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
			mvQueuedCommands.erase(mvQueuedCommands.begin());
		}
#if 0
		if(mbAddDense)
		{
		    mbAddDense=false;
		    AddPt(mnSegment);
		}
#endif
		
		if(!mMap.IsGood())  // Nothing to do if there is no map yet!
			continue;
		
		// From here on, mapmaker does various map-maintenance jobs in a certain priority
		// Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
		// then that takes high priority.
		
		CHECK_RESET;
		// Should we run local bundle adjustment?
		if(!mbBundleConverged_Recent && QueueSize() == 0)  
			BundleAdjustRecent();   
	
		CHECK_RESET;
		// Are there any newly-made map points which need more measurements from older key-frames?
		if(mbBundleConverged_Recent && QueueSize() == 0)
			ReFindNewlyMade();  
		
		CHECK_RESET;
		// Run global bundle adjustment?
		

                if(mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0)
{
                if(ratio_pvs>0.85)
		BundleAdjustAll();
		else if(ratio_pvs<0.7)
		BundleAdjustConstraint();
}
		
		CHECK_RESET;
		// Very low priorty: re-find measurements marked as outliers
		if(mbBundleConverged_Recent && mbBundleConverged_Full && rand()%20 == 0 && QueueSize() == 0)
			ReFindFromFailureQueue();
		
		CHECK_RESET;
		// HandleBadPoints() Does some heuristic checks on all points in the map to see if 
		// they should be flagged as bad, based on tracker feedback.
		HandleBadPoints();
		
		CHECK_RESET;
		// Any new key-frames to be added?
ros::spinOnce();
//PublishMap();
		if(QueueSize() > 0)
			AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process
			
		if(mbBundleConverged_Recent && mbBundleConverged_Full && QueueSize() == 0)
			mbMapMakerStatus = STAT_IDLE;
			
	}
}

bool MapMaker::frontier_check(ptam::Frontier_check::Request &req,
                              ptam::Frontier_check::Response &res)
{
//geometry_msgs::Pose pose;
res.points_visible.clear();
Vector<3>pose_trans;
btMatrix3x3 rotation;
Matrix<3,3>rot;
btVector3 row;
SO3<>orth;
SE3<>trans;
Matrix<3,3> rot_world;
Vector<3>trans_world;
tf::TransformListener tf_listen;

rot_world(0,0)=0.0;rot_world(0,1)=0.000;rot_world(0,2)=1.0;
rot_world(1,0)=-1.0;rot_world(1,1)=0.0;rot_world(1,2)=0.000;
rot_world(2,0)=0.0;rot_world(2,1)=-1.0;rot_world(2,2)=0.0;
//btVector3 translation = btVector3(btScalar(0.31526),btScalar(0.0842366),btScalar(1.411));
//btTransform transrot(mat,translation);
//btQuaternion q = transrot.getRotation();

Matrix<3,3>rot_camera;

SO3<>trans_so3(rot_world);
SO3<>camera_so3;
//geometry_msgs::PoseStamped pose_in;
//pose_in.header.frame_id="world";

//SE3<>camera_ptam=transform_camera.inverse()*transform_world_ptam;

SE3<>transform_camera;
geometry_msgs::PoseStamped pose_out;
Vector<4>pose_in;
Vector<4>pose_transformed;
int count_points=0;
btMatrix3x3 rot_frontier;
Matrix<3,3>rot_pos_robot;
Vector<3>trans_check;
btQuaternion rot_check;
double angle;
double yaw,pitch,roll;
for(int i=0;i<req.Frontiers.poses.size();i++)
{

trans_world[0]=req.Frontiers.poses[i].position.x;trans_world[1]=req.Frontiers.poses[i].position.y;trans_world[2]=0.30;

rot_frontier=btMatrix3x3(btQuaternion(btScalar(req.Frontiers.poses[i].orientation.x),btScalar(req.Frontiers.poses[i].orientation.y),btScalar(req.Frontiers.poses[i].orientation.z),btScalar(req.Frontiers.poses[i].orientation.w)));


rot_frontier.getEulerYPR(angle,pitch,roll);
rot_camera(0,0)=cos(angle);rot_camera(0,1)=-1*sin(angle);rot_camera(0,2)=0.0;
rot_camera(1,0)=sin(angle);rot_camera(1,1)=cos(angle);rot_camera(1,2)=0.000;
rot_camera(2,0)=0.0;rot_camera(2,1)=0.0;rot_camera(2,2)=1.0;
camera_so3=SO3<>(rot_camera);
transform_camera=SE3<>(camera_so3*trans_so3,trans_world);
//rot_pos_robot=transform_camera.get_rotation().get_matrix();
//trans_check=transform_camera.get_translation();
//btMatrix3x3 mat_robot_pose = btMatrix3x3(rot_pos_robot(0,0),rot_pos_robot(0,1),rot_pos_robot(0,2),rot_pos_robot(1,0),rot_pos_robot(1,1),rot_pos_robot(1,2),rot_pos_robot(2,0),rot_pos_robot(2,1),rot_pos_robot(2,2));
//mat_robot_pose.getRotation(rot_check);
//cout<<trans_check<<endl;
//cout<<rot_check[0]<<","<<rot_check[1]<<","<<rot_check[2]<<","<<rot_check[3]<<endl;





count_points=0;
/*
pose_trans[0]=req.Frontiers.poses[i].position.x;
pose_trans[1]=req.Frontiers.poses[i].position.y;
pose_trans[2]=req.Frontiers.poses[i].position.z;
//trans(btQuaternion(btScalar(req.Frontiers.poses.[i].orientation.x),btScalar(req.Frontiers.poses.[i].orientation.y),btScalar(req.Frontiers.poses.[i].orientation.z),btScalar(req.Frontiers.poses.[i].orientation.w)),btVector3(btScalar(req.Frontiers.poses.[i].position.x),btScalar(req.Frontiers.poses.[i].position.y),btScalar(req.Frontiers.poses.[i].position.z)));

//trans.setOrigin(btVector3(btScalar(req.Frontiers.poses.[i].position.x),btScalar(req.Frontiers.poses.[i].position.y),btScalar(req.Frontiers.poses.[i].position.z)));
//trans.setRotation(btQuaternion(btScalar(req.Frontiers.poses.[i].orientation.x),btScalar(req.Frontiers.poses.[i].orientation.y),btScalar(req.Frontiers.poses.[i].orientation.z),btScalar(req.Frontiers.poses.[i].orientation.w)));
//rotation=trans.getBasis();
//mat = btMatrix3x3(rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
rotation=btMatrix3x3(btQuaternion(btScalar(req.Frontiers.poses[i].orientation.x),btScalar(req.Frontiers.poses[i].orientation.y),btScalar(req.Frontiers.poses[i].orientation.z),btScalar(req.Frontiers.poses[i].orientation.w)));
row=rotation[0];
rot(0,0)=row[0];rot(0,1)=row[1];rot(0,2)=row[2];
row=rotation[1];
rot(1,0)=row[0];rot(1,1)=row[1];rot(1,2)=row[2];
row=rotation[2];
rot(2,0)=row[0];rot(2,1)=row[1];rot(2,2)=row[2];
orth=SO3<>(rot);
trans=SE3<>(orth,pose_trans);
*/


for(int i = 0; i < (int)mMap.vpPoints.size(); i++)
    {
	MapPoint &p= *(mMap.vpPoints[i]); 
	// Ensure that this map point has an associated TrackerData struct.
	if(!p.pTData)
{ 
p.pTData = new TrackerData(&p);   
}	

TrackerData &TData = *p.pTData;
TData.Project(transform_camera.inverse()*transform_world_ptam, mCamera);//equation 2, function in TrackerData.cc 
if(!TData.bInImage)
continue;
else
count_points+=1;
}

res.points_visible.push_back(count_points);
}
res.total_points=(int)mMap.vpPoints.size();

}


void MapMaker::save_points(int count,SE3<>se3camfromworld)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
ofstream myfile;
stringstream ss,ss1;


ss1.str();
ss.str();
pcl::PointXYZ point;
ofstream write;

ss1<<"/home/irl/workspace/ptam/bin/maps_test/RandT_"<<count<<".info";
ss<<"/home/irl/workspace/ptam/bin/maps_test/map_"<<count<<".csv";

write.open(ss1.str().c_str());
myfile.open(ss.str().c_str());
write<<camera_world<<endl;

write.close();

Vector<4>points;
Vector<4>points_trans;
    for(int i = 0; i < (int)mMap.vpPoints.size(); i++)
    {
	MapPoint &p= *(mMap.vpPoints[i]); 
	// Ensure that this map point has an associated TrackerData struct.
	if(!p.pTData)
{ 
p.pTData = new TrackerData(&p);   
}	

TrackerData &TData = *p.pTData;
TData.Project(se3camfromworld, mCamera);//equation 2, function in TrackerData.cc 
if(!TData.bInImage)
continue;

	 points[0]=mMap.vpPoints[i] -> v3WorldPos[0];
	 points[1]=mMap.vpPoints[i] -> v3WorldPos[1];
	 points[2]=mMap.vpPoints[i] -> v3WorldPos[2];
	 points[3]=1;
	 points_trans=transform_world_ptam*points;
point.x=points_trans[0]/points_trans[3];

point.y=points_trans[1]/points_trans[3];
point.z=points_trans[2]/points_trans[3];
cloud->points.push_back(point);

myfile<<points_trans[0]/points_trans[3]<<","<<points_trans[1]/points_trans[3]<<","<<points_trans[2]/points_trans[3]<<","<<endl;
}
cloud->points.clear();
myfile.close();
ss.str("");
ss<<"/home/irl/workspace/ptam/bin/maps_test/map_camera"<<count<<".csv";
myfile.open(ss.str().c_str());

    for(int i = 0; i < (int)mMap.vpPoints.size(); i++)
    {
	MapPoint &p= *(mMap.vpPoints[i]); 
	// Ensure that this map point has an associated TrackerData struct.
	if(!p.pTData)
{ 
p.pTData = new TrackerData(&p);   
}	

TrackerData &TData = *p.pTData;
TData.Project(se3camfromworld, mCamera);//equation 2, function in TrackerData.cc 
if(!TData.bInImage)
continue;

	 points[0]=mMap.vpPoints[i] -> v3WorldPos[0];
	 points[1]=mMap.vpPoints[i] -> v3WorldPos[1];
	 points[2]=mMap.vpPoints[i] -> v3WorldPos[2];
	 points[3]=1;
	 points_trans=se3camfromworld*points;
point.x=points_trans[0]/points_trans[3];

point.y=points_trans[1]/points_trans[3];
point.z=points_trans[2]/points_trans[3];
cloud->points.push_back(point);

myfile<<points_trans[0]/points_trans[3]<<","<<points_trans[1]/points_trans[3]<<","<<points_trans[2]/points_trans[3]<<","<<endl;
}

//pcl::VoxelGrid<pcl::PointXYZ> sor;
 // sor.setInputCloud (cloud);
 // sor.setLeafSize (0.1f, 0.1f,0.1f);
//  sor.setStddevMulThresh (1.0);
 // sor.filter (*cloud_filtered);

//for(int k=0;k<cloud_filtered->points.size();k++)
//{

//myfile<<cloud_filtered->points[k].x<<","<<cloud_filtered->points[k].y<<","<<cloud_filtered->points[k].z<<endl;

//}



printf("points_saved %d,%d\n",(int)cloud->points.size(),(int)cloud_filtered->points.size());
myfile.close();
}
void MapMaker::PublishMap(SE3<>se3camfromworld)
{


   static int seq_no = 0;
    pcl::PointCloud<pcl::PointXYZ> cloud;
KeyFrame *point_kf = new KeyFrame();
ofstream myfile;
myfile.open("/home/irl/workspace/ptam/ptam_points.csv");
    pcl::PointCloud<pcl::PointXYZ> cloud_ret;
    cloud.is_dense = false;
    cloud.header.seq = seq_no++;
    cloud.header.frame_id ="camera";
    cloud.header.stamp = ros::Time::now();
    cloud_ret.is_dense = false;
    cloud_ret.header.seq = seq_no++;
    cloud_ret.header.frame_id ="world";
    cloud_ret.header.stamp = ros::Time::now();
pcl::PCDWriter writer;
    pcl::PointXYZ pt;



Matrix<3,3> rot;
Vector<3>trans;
Vector<4> pose;
Matrix<3,3> rot_world;
Vector<3>trans_world;


Matrix<3,3>rot_ptam_world;
Vector<3>trans_ptam_world;

rot_world(0,0)=0.0;rot_world(0,1)=0.000;rot_world(0,2)=1.0;
rot_world(1,0)=-1.0;rot_world(1,1)=0.0;rot_world(1,2)=0.000;
rot_world(2,0)=0.0;rot_world(2,1)=-1.0;rot_world(2,2)=0.0;
//btVector3 translation = btVector3(btScalar(0.31526),btScalar(0.0842366),btScalar(1.411));
//btTransform transrot(mat,translation);
//btQuaternion q = transrot.getRotation();
trans_world[0]=1.5;trans_world[1]=0.0;trans_world[2]=0.30;

SO3<>trans_so3(rot_world);

transform_world=SE3<>(trans_so3,trans_world);
transform_world_ptam=transform_world*mMap.vpKeyFrames[0]->se3CfromW;

path.poses.clear();
//printf("inside trajectory\n");
geometry_msgs::PoseStamped pose_t;

Vector<3>v3Pos;

Vector<4>v3Pos_new;
Vector<4>v3Pos_t;
Vector<4> pose_trans;
Vector<3>pos_robot;
//Vector<4> pos_robot_homo;
//Vector<4>pos_robot_trans;
Matrix<3,3>rot_pos_robot;

//SE3<>camera_world;

camera_world=transform_world_ptam*se3camfromworld.inverse();
pos_robot=camera_world.get_translation();


//pos_robot_homo[0]=pos_robot[0];
//pos_robot_homo[1]=pos_robot[1];
//pos_robot_homo[2]=pos_robot[2];
//pos_robot_homo[3]=1.0;
//pos_robot_homo[0]=pos_robot[0];
//pos_robot_trans=transform_world_ptam*pos_robot_homo;
//position_robot.x=pos_robot_trans[0]/pos_robot_trans[3];
//position_robot.y=pos_robot_trans[1]/pos_robot_trans[3];
position_robot.x=pos_robot[0];
position_robot.y=pos_robot[1];

float dist_to_plane=((.9389*pos_robot[0])+(-0.0271*pos_robot[1])+(-0.3431*pos_robot[2])-4.3085)/((.9389*.9389)+(-.0271*-0.271)+(-.3431*-.3431));
//printf("%f\n",dist_to_plane);
rot_pos_robot=camera_world.get_rotation().get_matrix();

btMatrix3x3 mat_robot_pose = btMatrix3x3(rot_pos_robot(0,0),rot_pos_robot(0,1),rot_pos_robot(0,2),rot_pos_robot(1,0),rot_pos_robot(1,1),rot_pos_robot(1,2),rot_pos_robot(2,0),rot_pos_robot(2,1),rot_pos_robot(2,2));

btScalar yaw, pitch, roll;

mat_robot_pose.getEulerZYX(yaw, pitch, roll);
position_robot.theta=yaw+1.571;
pos_publish.publish(position_robot);







//Vector<4> pose_homogeneous;
static tf::TransformBroadcaster br;
tf::Transform transform;
static tf::TransformBroadcaster br_new;
tf::Transform transform_new;
static tf::TransformBroadcaster camera_base;
tf::Transform camera_base_tf;
 ratio_pvs;
 count_pvs=0;

    for(int i = 0; i < (int)mMap.vpPoints.size(); i++)
    {


//myfile<<mMap.vpPoints[i] -> v3WorldPos[0]<<","<<mMap.vpPoints[i] -> v3WorldPos[1]<<","<<mMap.vpPoints[i] -> v3WorldPos[2]<<endl;
//        pose[0] = mMap.vpPoints[i] -> v3WorldPos[0];
//        pose[1] = mMap.vpPoints[i] -> v3WorldPos[1];
//        pose[2] = mMap.vpPoints[i] -> v3WorldPos[2];
//pose[3]=1.0;

	MapPoint &p= *(mMap.vpPoints[i]); 
	// Ensure that this map point has an associated TrackerData struct.
	if(!p.pTData)
{ 
p.pTData = new TrackerData(&p);   
}	
int number_kf=(int)mMap.vpKeyFrames.size();

TrackerData &TData = *p.pTData;
TData.Project(mMap.vpKeyFrames[number_kf-1]->se3CfromW, mCamera);//equation 2, function in TrackerData.cc 
if(!TData.bInImage)
continue;
count_pvs+=1;
//pose_trans=se3camfromworld*pose;

//printf("%f,%f,%f\n",pose[0],pose[1],pose[2]);
//rot=point_kf->se3CfromW.get_rotation().get_matrix();
//trans=point_kf->se3CfromW.get_translation();
//btMatrix3x3 mat = btMatrix3x3(rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
//mat.getEulerYPR(yaw, pitch, roll);
//cout <<yaw<<","<< pitch<<","<<roll <<","<<trans[0]<<","<<trans[1]<<","<<trans[2]<<endl;

//printf("%d \n",point_kf->nFrameIdAbs);
//fprintf(stderr,"rotation:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2),trans[0],trans[1],trans[2]);
//printf("after trans%f,%f,%f\n",pose_trans[0],pose_trans[1],pose_trans[2],pose_trans[3]);
//pt.x=pose[0];pt.y=pose[1];pt.z=pose[2];


//pt.x=pose_trans[0]/pose_trans[3];pt.y=pose_trans[1]/pose_trans[3];pt.z=pose_trans[2]/pose_trans[3];
//        cloud.points.push_back(pt);
}
//myfile.close();


ratio_pvs=(float)count_pvs/mMap.vpPoints.size();
	for(size_t i=0; i<mMap.vpKeyFrames.size(); i++)
	{
	    v3Pos = mMap.vpKeyFrames[i]->se3CfromW.inverse().get_translation();

v3Pos_new[0]=v3Pos[0];

v3Pos_new[1]=v3Pos[1];
v3Pos_new[2]=v3Pos[2];
v3Pos_new[3]=1.0;
v3Pos_t=transform_world_ptam*v3Pos_new;

pose_t.pose.position.x=v3Pos_t[0]/v3Pos_t[3];
pose_t.pose.position.y=v3Pos_t[1]/v3Pos_t[3];
pose_t.pose.position.z=v3Pos_t[2]/v3Pos_t[3];
//fprintf(stderr,"position z %f\n",pose.pose.position.z);
pose_t.pose.orientation.x=0;
pose_t.pose.orientation.y=0;
pose_t.pose.orientation.z=0;
pose_t.pose.orientation.w=1;
path.poses.push_back(pose_t);

	   // camTraj.push_back(v3Pos);
	}


trajectory.publish(path);	




//number_of_points=cloud.points.size();
//printf("%d,%d\n",cloud.points.size(),(int)mMap.vpPoints.size());
//rot(0,0)=.996042;rot(0,1)=0.038863;rot(0,2)=0.079938;
//rot(1,0)=0.059238;rot(1,1)=-0.960744;rot(1,2)=-0.271038;
//rot(2,0)=0.066267;rot(2,1)=0.274700;rot(2,2)=-0.959244;
//pt.x=pose[0]*rot(0,0) + pose[1]*rot(0,1) + pose[2]*rot(0,2); 

//pt.y=pose[1]*rot(1,0) + pose[1]*rot(1,1) + pose[2]*rot(1,2); 

//pt.z=pose[2]*rot(2,0) + pose[1]*rot(2,1) + pose[2]*rot(2,2) +3.0;



/*
*/




//rot=transform_world.get_rotation().get_matrix();
//trans=transform_world.get_translation();

//mat.getEulerZYX(yaw, pitch, roll);
//cout << "the roll is "<<yaw<<" pitch "<< pitch<< "roll "<<roll <<endl;
//trans=se3camfromworld.get_translation();
//btQuaternion q = transrot.getRotation();
 // transform_new.setOrigin( tf::Vector3(0.0,0.0,3.0) );
 // transform_new.setRotation( tf::Quaternion(-1.54,0,-1.54) );
//  br_new.sendTransform(tf::StampedTransform(transform_new, ros::Time::now(), "world", "camera"));


//std::string why;

//transform.setRotation(tf::Quaternion(-1.54,0,-1.54));

rot=se3camfromworld.inverse().get_rotation().get_matrix();
trans=se3camfromworld.inverse().get_translation();
btVector3 translation = btVector3(btScalar(trans[0]), btScalar(trans[1]), btScalar(trans[2]));
btMatrix3x3 mat = btMatrix3x3(rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
btTransform transrot(mat,translation);
transform.setOrigin(translation);
btQuaternion q = transrot.getRotation();
transform.setRotation(q); // tf::Quaternion(msg->theta, 0, 0) );
br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ptam","camera"));

rot_ptam_world=transform_world_ptam.get_rotation().get_matrix();
trans_ptam_world=transform_world_ptam.get_translation();
btVector3 translation_ptam_world = btVector3(btScalar(trans_ptam_world[0]), btScalar(trans_ptam_world[1]), btScalar(trans_ptam_world[2]));
btMatrix3x3 mat_ptam_world = btMatrix3x3(rot_ptam_world(0,0),rot_ptam_world(0,1),rot_ptam_world(0,2),rot_ptam_world(1,0),rot_ptam_world(1,1),rot_ptam_world(1,2),rot_ptam_world(2,0),rot_ptam_world(2,1),rot_ptam_world(2,2));
btTransform transrot_ptam_world(mat_ptam_world,translation_ptam_world);
transform_new.setOrigin(translation_ptam_world);
btQuaternion q_new = transrot_ptam_world.getRotation();

transform_new.setRotation(q_new); //  );
//tf::Matrix3x3 rot_check=transform_new.getBasis();
//tf::Vector3 row1=rot_check.getRow(0);

//tf::Vector3 row2=rot_check.getRow(1);
//tf::Vector3 row3=rot_check.getRow(2);

//fprintf(stderr,"rotation:%f,%f,%f,%f,%f,%f,%f,%f,%f\n",row1[0],row1[1],row1[2],row2[0],row2[1],row2[2],row3[0],row3[1],row3[2]);
br_new.sendTransform(tf::StampedTransform(transform_new, ros::Time::now(), "world","ptam"));

camera_base_tf.setOrigin( tf::Vector3(0,0.30,0) );
camera_base_tf.setRotation( tf::Quaternion(0,1.540,1.54) );
camera_base.sendTransform(tf::StampedTransform(camera_base_tf, ros::Time::now(), "camera","base_link"));



//cloud.width = cloud.points.size();
//cloud.height = 1;

//writer.write<pcl::PointXYZ> ("/media/home/ayushd/ros/ptam_improved/launch/test.pcd",cloud,false);
  //  sensor_msgs::PointCloud2 msg;

  //  sensor_msgs::PointCloud2 msg_ret;
//    pcl::toROSMsg(cloud, msg);
//const tf::TransformListener tt;
//tt.waitForTransform("map","world",
  //                           ros::Time::now(), ros::Duration(5.0));
//bool val=pcl_ros::transformPointCloud("map",msg,msg_ret,tt);
//if(val)
//pcl::fromROSMsg(msg_ret,cloud_ret);
  //  for(int i = 0; i < cloud_ret.points.size(); i++)
   // {
   
//fprintf(stderr,"Points %f,%f,%f\n",cloud_ret.points[i].x,cloud_ret.points[i].y,cloud_ret.points[i].z);

      //  cloud.points.push_back(pt);
  //  }
//printf("points published\n"); 


//pvs_pub.points_visible=count_pvs;
//points_pub.publish(pvs_pub);
    map_pub.publish(cloud);
}



// Tracker calls this to demand a reset
void MapMaker::RequestReset()
{
  mbResetDone = false;
  mbResetRequested = true;
}

bool MapMaker::ResetDone()
{
  return mbResetDone;
}

// HandleBadPoints() Does some heuristic checks on all points in the map to see if 
// they should be flagged as bad, based on tracker feedback.
void MapMaker::HandleBadPoints()
{
	mbMapMakerStatus = STAT_HANDLE_BAD_PTS;
	
  // Did the tracker see this point as an outlier more often than as an inlier?
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      MapPoint &p = *mMap.vpPoints[i];
      if(p.nMEstimatorOutlierCount > 20 && p.nMEstimatorOutlierCount > p.nMEstimatorInlierCount)
	p.bBad = true;
    }
  
  // All points marked as bad will be erased - erase all records of them
  // from keyframes in which they might have been measured.
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    if(mMap.vpPoints[i]->bBad)
      {
	MapPoint *p = mMap.vpPoints[i];
	for(unsigned int j=0; j<mMap.vpKeyFrames.size(); j++)
	  {
	    KeyFrame &k = *mMap.vpKeyFrames[j];
	    if(k.mMeasurements.count(p))
	      k.mMeasurements.erase(p);
	  }
      }
  // Move bad points to the trash list.
  mMap.MoveBadPointsToTrash();
}

MapMaker::~MapMaker()
{
  mbBundleAbortRequested = true;
  stop(); // makes shouldStop() return true
  cout << "Waiting for mapmaker to die.." << endl;
  join();
  cout << " .. mapmaker has died." << endl;
}


// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> MapMaker::ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
  Matrix<3,4> PDash;
  PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();
  
  Matrix<4> A;
  A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
  A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
  A[2] = v2A[0] * PDash[2] - PDash[0];
  A[3] = v2A[1] * PDash[2] - PDash[1];

  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
  return project(v4Smallest);
}



// InitFromStereo() generates the initial match from two keyframes
// and a vector of image correspondences. Uses the 
bool MapMaker::InitFromStereo_altitude(KeyFrame &kF,
			      KeyFrame &kS,
			      vector<pair<ImageRef, ImageRef> > &vTrailMatches,
			      SE3<> &se3TrackerPose,float scale)

{
//mdWiggleScale=scale;
mdWiggleScale=0.1;
// mdWiggleScale = *mgvdWiggleScale; // Cache this for the new map.

fprintf(stderr,"wigglescale%f\n",mdWiggleScale);
  mCamera.SetImageSize(kF.aLevels[0].im.size());

  
  vector<HomographyMatch> vMatches;
  for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
      HomographyMatch m;
      m.v2CamPlaneFirst = mCamera.UnProject(vTrailMatches[i].first);//Used for creating a ray from pixel location
      m.v2CamPlaneSecond = mCamera.UnProject(vTrailMatches[i].second);
      m.m2PixelProjectionJac = mCamera.GetProjectionDerivs();
      vMatches.push_back(m);
    }

  SE3<> se3;//rotation and translation between first and second frame, estimated by decomposing homography matrix between 2 frames.
  bool bGood;
  HomographyInit HomographyInit;
  bGood = HomographyInit.Compute(vMatches, 5.0, se3);
  if(!bGood)
    {
      cout << "  Could not init from stereo pair, try again." << endl;
      return false;
    }
  
  //cout<<se3;
  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
  if(dTransMagn == 0)
	{
		cout << "  Estimated zero baseline from stereo pair, try again." << endl;
		return false;
	}
  // change the scale of the map so the second camera is wiggleScale away from the first
  se3.get_translation() *= mdWiggleScale/dTransMagn;
  
  KeyFrame *pkFirst = new KeyFrame();
  KeyFrame *pkSecond = new KeyFrame();
  *pkFirst = kF;
  *pkSecond = kS;
  
  pkFirst->bFixed = true;
  pkFirst->se3CfromW = SE3<>();
  
  pkSecond->bFixed = false;
  pkSecond->se3CfromW = se3;
  
  // Construct map from the stereo matches.
  PatchFinder finder;

  for(unsigned int i=0; i<vMatches.size(); i++)
    {
      MapPoint *p = new MapPoint();
      
      // Patch source stuff:
      p->pPatchSourceKF = pkFirst;
      p->nSourceLevel = 0;
      p->v3Normal_NC = makeVector( 0,0,-1);
      p->irCenter = vTrailMatches[i].first;
      p->v3Center_NC = unproject(mCamera.UnProject(p->irCenter));
      p->v3OneDownFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + ImageRef(0,1)));
      p->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + ImageRef(1,0)));
      normalize(p->v3Center_NC);
      normalize(p->v3OneDownFromCenter_NC);
      normalize(p->v3OneRightFromCenter_NC);
      p->RefreshPixelVectors();

      // Do sub-pixel alignment on the second image
      finder.MakeTemplateCoarseNoWarp(*p);
      finder.MakeSubPixTemplate();
      finder.SetSubPixPos(vec(vTrailMatches[i].second));
      bool bGood = finder.IterateSubPixToConvergence(*pkSecond,10);
      if(!bGood)
			{ 
				delete p; continue;
			}
      
      // Triangulate point:
      Vector<2> v2SecondPos = finder.GetSubPixPos();
      p->v3WorldPos = ReprojectPoint(se3, mCamera.UnProject(v2SecondPos), vMatches[i].v2CamPlaneFirst);
      if(p->v3WorldPos[2] < 0.0)
      {
				delete p; continue;
			}
      
      // Not behind map? Good, then add to map.
      p->pMMData = new MapMakerData();
      mMap.vpPoints.push_back(p);
      //mMap.interestPoint.push_back(false);//E
      
      // Construct first two measurements and insert into relevant DBs:
      Measurement mFirst;
      mFirst.nLevel = 0;
      mFirst.Source = Measurement::SRC_ROOT;
      mFirst.v2RootPos = vec(vTrailMatches[i].first);
      mFirst.bSubPix = true;
      pkFirst->mMeasurements[p] = mFirst;
      p->pMMData->sMeasurementKFs.insert(pkFirst);
      
      Measurement mSecond;
      mSecond.nLevel = 0;
      mSecond.Source = Measurement::SRC_TRAIL;
      mSecond.v2RootPos = finder.GetSubPixPos();
      mSecond.bSubPix = true;
      pkSecond->mMeasurements[p] = mSecond;
      p->pMMData->sMeasurementKFs.insert(pkSecond);
    }
  
  cout<<"nMapPoints = "<<mMap.vpPoints.size()<<endl;
  mMap.vpKeyFrames.push_back(pkFirst);
  mMap.vpKeyFrames.push_back(pkSecond);
  pkFirst->MakeKeyFrame_Rest();
  pkSecond->MakeKeyFrame_Rest();
Vector<3>v3Pos;
v3Pos=pkFirst->se3CfromW.inverse().get_translation();
fprintf(stderr,"Key frame1 %f,%f,%f\n",v3Pos[0],v3Pos[1],v3Pos[2]);
 v3Pos=pkSecond->se3CfromW.inverse().get_translation();
fprintf(stderr,"Key frame2 %f,%f,%f\n",v3Pos[0],v3Pos[1],v3Pos[2]);  
 
  for(int i=0; i<5; i++)
    BundleAdjustAll();

  //cout<<"se3 = "<<endl<<se3<<endl;
  //cout<<"pks->se3 = "<<endl<<pkSecond->se3CfromW<<endl;

  // Estimate the feature depth distribution in the first two key-frames
  // (Needed for epipolar search)
  RefreshSceneDepth(pkFirst);
  RefreshSceneDepth(pkSecond);
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;


  AddSomeMapPoints(0);
  AddSomeMapPoints(3);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);
#if 0
  AddPt();
#endif
  //AddOFPoints();
  
  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;
  
  while(!mbBundleConverged_Full)
	{
		BundleAdjustAll();
		if(mbResetRequested)
			return false;
	}
	
	// Rotate and translate the map so the dominant plane is at z=0:
 ApplyGlobalTransformationToMap(CalcPlaneAligner());
  mMap.bGood = true;
  se3TrackerPose = pkSecond->se3CfromW;
	
	//Add to camera trajectory the intial 2 CamPoses
	CamPose cp1(pkFirst->nFrameIdAbs, pkFirst->se3CfromW, true);
	CamPose cp2(pkSecond->nFrameIdAbs, pkSecond->se3CfromW, true);
	mMap.vCamPoses.push_back(cp1);
	mMap.vCamPoses.push_back(cp2);
	cout<<"AddOFPOints"<<endl;
  //AddOFPoints();
	
  
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;
  return true; 
}

// InitFromStereo() generates the initial match from two keyframes
// and a vector of image correspondences. Uses the 
bool MapMaker::InitFromStereoPlus(KeyFrame &kF,
			      KeyFrame &kS,
			      vector<pair<ImageRef, ImageRef> > &vTrailMatches,
			      SE3<> &se3TrackerPose)
{
  mdWiggleScale = *mgvdWiggleScale; // Cache this for the new map.

  mCamera.SetImageSize(kF.aLevels[0].im.size());

  //cout<<"trail[10] = "<<vTrailMatches[10].first<<", "<<vTrailMatches[10].second<<endl;
  //cout<<"vTrail.s = "<<vTrailMatches.size()<<", "<<endl;
  /*Vector<2> irFirst, irSecond;
  ifstream ifs("../codes/Homography/matches.txt");
  if(ifs.is_open())
  {
      cout<<"What!!!"<<endl;
      while(!ifs.eof())
      {
	  ifs>>irFirst; ifs>>irSecond;
	  cout<<irFirst<<", "<<irSecond<<endl;
	  vTrailMatches.push_back(pair<ImageRef, ImageRef>(ImageRef(irFirst[0],irFirst[1]), ImageRef(irSecond[0], irSecond[1])));
      }
      vTrailMatches.pop_back();
      cout<<"vTrails.news = "<<vTrailMatches.size()<<endl;
      ifs.close();
  }
*/
  
  vector<HomographyMatch> vMatches;
  for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
      HomographyMatch m;
      m.v2CamPlaneFirst = mCamera.UnProject(vTrailMatches[i].first);
      m.v2CamPlaneSecond = mCamera.UnProject(vTrailMatches[i].second);
      m.m2PixelProjectionJac = mCamera.GetProjectionDerivs();
      vMatches.push_back(m);
    }
  cout<<"vMatches.s = "<<vMatches.size()<<endl;

  SE3<> se3;
  bool bGood;
  HomographyInit HomographyInit;
  bGood = HomographyInit.Compute(vMatches, 5.0, se3);
  if(!bGood)
    {
      cout << "  Could not init from stereo pair, try again." << endl;
      return false;
    }
  
  //cout<<se3;
  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
  if(dTransMagn == 0)
	{
		cout << "  Estimated zero baseline from stereo pair, try again." << endl;
		return false;
	}
  // change the scale of the map so the second camera is wiggleScale away from the first
  se3.get_translation() *= mdWiggleScale/dTransMagn;
  
  KeyFrame *pkFirst = new KeyFrame();
  KeyFrame *pkSecond = new KeyFrame();
  *pkFirst = kF;
  *pkSecond = kS;
  
  pkFirst->bFixed = true;
  pkFirst->se3CfromW = SE3<>();
  
  pkSecond->bFixed = false;
  pkSecond->se3CfromW = se3;
  
  // Construct map from the stereo matches.
  PatchFinder finder;

  for(unsigned int i=0; i<vMatches.size(); i++)
    {
      MapPoint *p = new MapPoint();
      
      // Patch source stuff:
      p->pPatchSourceKF = pkFirst;
      p->nSourceLevel = 0;
      p->v3Normal_NC = makeVector( 0,0,-1);
      p->irCenter = vTrailMatches[i].first;
      p->v3Center_NC = unproject(mCamera.UnProject(p->irCenter));
      p->v3OneDownFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + ImageRef(0,1)));
      p->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(p->irCenter + ImageRef(1,0)));
      normalize(p->v3Center_NC);
      normalize(p->v3OneDownFromCenter_NC);
      normalize(p->v3OneRightFromCenter_NC);
      p->RefreshPixelVectors();

      // Do sub-pixel alignment on the second image
      finder.MakeTemplateCoarseNoWarp(*p);
      finder.MakeSubPixTemplate();
      finder.SetSubPixPos(vec(vTrailMatches[i].second));
      bool bGood = finder.IterateSubPixToConvergence(*pkSecond,10);
      if(!bGood)
			{ 
				delete p; continue;
			}
      
      // Triangulate point:
      Vector<2> v2SecondPos = finder.GetSubPixPos();
      p->v3WorldPos = ReprojectPoint(se3, mCamera.UnProject(v2SecondPos), vMatches[i].v2CamPlaneFirst);
      if(p->v3WorldPos[2] < 0.0)
      {
				delete p; continue;
			}
      
      // Not behind map? Good, then add to map.
      p->pMMData = new MapMakerData();
      mMap.vpPoints.push_back(p);
      //mMap.interestPoint.push_back(false);//E
      
      // Construct first two measurements and insert into relevant DBs:
      Measurement mFirst;
      mFirst.nLevel = 0;
      mFirst.Source = Measurement::SRC_ROOT;
      mFirst.v2RootPos = vec(vTrailMatches[i].first);
      mFirst.bSubPix = true;
      pkFirst->mMeasurements[p] = mFirst;
      p->pMMData->sMeasurementKFs.insert(pkFirst);
      
      Measurement mSecond;
      mSecond.nLevel = 0;
      mSecond.Source = Measurement::SRC_TRAIL;
      mSecond.v2RootPos = finder.GetSubPixPos();
      mSecond.bSubPix = true;
      pkSecond->mMeasurements[p] = mSecond;
      p->pMMData->sMeasurementKFs.insert(pkSecond);
    }
  
  cout<<"nMapPoints = "<<mMap.vpPoints.size()<<endl;
  mMap.vpKeyFrames.push_back(pkFirst);
  mMap.vpKeyFrames.push_back(pkSecond);
  pkFirst->MakeKeyFrame_Rest();
  pkSecond->MakeKeyFrame_Rest();
  
  for(int i=0; i<5; i++)
    BundleAdjustAll();

  //cout<<"se3 = "<<endl<<se3<<endl;
  //cout<<"pks->se3 = "<<endl<<pkSecond->se3CfromW<<endl;

  // Estimate the feature depth distribution in the first two key-frames
  // (Needed for epipolar search)
  RefreshSceneDepth(pkFirst);
  RefreshSceneDepth(pkSecond);
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;


  AddSomeMapPoints(0);
  AddSomeMapPoints(3);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);
#if 0
  AddPt();
#endif
  
  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;
  
  while(!mbBundleConverged_Full)
	{
		BundleAdjustAll();
		if(mbResetRequested)
			return false;
	}
	
	// Rotate and translate the map so the dominant plane is at z=0:
  ApplyGlobalTransformationToMap(CalcPlaneAligner());
  mMap.bGood = true;
  se3TrackerPose = pkSecond->se3CfromW;
	
	//Add to camera trajectory the intial 2 CamPoses
	CamPose cp1(pkFirst->nFrameIdAbs, pkFirst->se3CfromW, true);
	CamPose cp2(pkSecond->nFrameIdAbs, pkSecond->se3CfromW, true);
	mMap.vCamPoses.push_back(cp1);
	mMap.vCamPoses.push_back(cp2);
	
  
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;
  return true; 
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt 
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMaker::ThinCandidates(KeyFrame &k, int nLevel)
{
  vector<Candidate> &vCSrc = k.aLevels[nLevel].vCandidates;
  vector<Candidate> vCGood;
  vector<ImageRef> irBusyLevelPos;
  // Make a list of `busy' image locations, which already have features at the same level
  // or at one level higher.
  for(meas_it it = k.mMeasurements.begin(); it!=k.mMeasurements.end(); it++)
	{
		if(!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
			continue;
		irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
	}
  
  // Only keep those candidates further than 10 pixels away from busy positions.
  unsigned int nMinMagSquared = 10*10;
  for(unsigned int i=0; i<vCSrc.size(); i++)
	{
		ImageRef irC = vCSrc[i].irLevelPos;
		bool bGood = true;
		for(unsigned int j=0; j<irBusyLevelPos.size(); j++)
		{
			ImageRef irB = irBusyLevelPos[j];
			if((irB - irC).mag_squared() < nMinMagSquared)
			{
				bGood = false;
				break;
			}
		}
		if(bGood)
			vCGood.push_back(vCSrc[i]);
	} 
  vCSrc = vCGood;
}

// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void MapMaker::AddSomeMapPoints(int nLevel)
{
  KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The new keyframe
  KeyFrame &kTarget = *(ClosestKeyFrame(kSrc));//Second point of view, required for triangulating points 
  //cout<<"kS = "<<kSrc.nFrameIdAbs<<", "<<"kT = "<<kTarget.nFrameIdAbs<<endl;//E
  Level &l = kSrc.aLevels[nLevel];

  ThinCandidates(kSrc, nLevel);
  
  for(unsigned int i = 0; i<l.vCandidates.size(); i++)
      AddPointEpipolar(kSrc, kTarget, nLevel, i);
};

// Rotates/translates the whole map and all keyframes
void MapMaker::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();
  
  SO3<> so3Rot = se3NewFromOld.get_rotation();
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      mMap.vpPoints[i]->v3WorldPos = 
	se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
      mMap.vpPoints[i]->RefreshPixelVectors();
    }
  //E
  //for(unsigned int i=0; i<mMap.vpOFPoints.size(); i++)
  //  {
  //    mMap.vpOFPoints[i] = se3NewFromOld * mMap.vpOFPoints[i];
  //  }
}

// Applies a global scale factor to the map
void MapMaker::ApplyGlobalScaleToMap(double dScale)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW.get_translation() *= dScale;
  
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      (*mMap.vpPoints[i]).v3WorldPos *= dScale;
      (*mMap.vpPoints[i]).v3PixelRight_W *= dScale;
      (*mMap.vpPoints[i]).v3PixelDown_W *= dScale;
      (*mMap.vpPoints[i]).RefreshPixelVectors();
    }
}

// The tracker entry point for adding a new keyframe;
// the tracker thread doesn't want to hang about, so 
// just dumps it on the top of the mapmaker's queue to 
// be dealt with later, and return.
void MapMaker::AddKeyFrame(KeyFrame &k)
{
  //cout<<"wiggleScale = "<<mdWiggleScaleDepthNormalized<<endl;
  KeyFrame *pK = new KeyFrame;
  *pK = k;
  //cout<<"\r"<<"kf="<<pK->nFrameIdAbs<<flush;
  pK->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  mvpKeyFrameQueue.push_back(pK);
  if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mbBundleAbortRequested = true;
}

int MapMaker::LatestKeyFrameId()
{
    return mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]->nFrameIdAbs;
}

// Mapmaker's code to handle incoming key-frames.
void MapMaker::AddKeyFrameFromTopOfQueue()
{
    //cout<<"addDense="<<mbAddDense<<endl;
  mbMapMakerStatus = STAT_ADD_KEYFRAME;
	
	if(mvpKeyFrameQueue.size() == 0)
    return;
  
  KeyFrame *pK = mvpKeyFrameQueue[0];
  mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
  pK->MakeKeyFrame_Rest();
  mMap.vpKeyFrames.push_back(pK);

keyframeadded = mMap.vpKeyFrames.size() - 2;

  for(meas_it it = pK->mMeasurements.begin();
      it!=pK->mMeasurements.end();
      it++)
    {
      it->first->pMMData->sMeasurementKFs.insert(pK);
      it->second.Source = Measurement::SRC_TRACKER;
    }
  
  // And maybe we missed some - this now adds to the map itself, too.
  ReFindInSingleKeyFrame(*pK);
  
  AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
  AddSomeMapPoints(0);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);
  //if(mbAGP)
  //{
  //    cout<<" AGP called at KF"<<flush;
  //    //mDmm.AGP(mMap.vpKeyFrames.size()-1);//mDmm.AddGroundPoints(*pK); 
  //}
#if 0
  //if(pK->nFrameIdAbs==79) //slope_26_image2
  //if(pK->nFrameIdAbs==49) //slope_26_image2
  //if(pK->nFrameIdAbs==41)//77) //slope_26_image1
  //if(pK->nFrameIdAbs==110) //slope_29
  //if(pK->nFrameIdAbs==171) //slope_29_late
  //if(pK->nFrameIdAbs==197) //plane1_images2_late
  //if(pK->nFrameIdAbs==150 || pK->nFrameIdAbs==156 || pK->nFrameIdAbs==162) //plane1_images2
  //if(pK->nFrameIdAbs==/*163)*/139) //plane25
  //if(pK->nFrameIdAbs==218) //corridor
  //if(pK->nFrameIdAbs==137) //plane31
  //if(pK->nFrameIdAbs==112) //plane31_good
  //if(pK->nFrameIdAbs==213) //plane31_check
  //if(pK->nFrameIdAbs==605) //plane31_box
  //if(pK->nFrameIdAbs>=26 && pK->nFrameIdAbs<=32) //plane31_lt
  //if(pK->nFrameIdAbs==101) //plane31_low
  //if(pK->nFrameIdAbs==105) //plane31_high
  //if(pK->nFrameIdAbs==199) //lab_video_16
  //if(pK->nFrameIdAbs==211) //lab_video_16
  //if(pK->nFrameIdAbs==373) //lab_video_16
  //if(pK->nFrameIdAbs==54) //side_9
  //if(pK->nFrameIdAbs==96) //side_9 for "make BUNDLE=_multi"
  //if(pK->nFrameIdAbs==97) //side_9 for "make"
  //if(pK->nFrameIdAbs==33) //low_02
  if(pK->nFrameIdAbs==315)//148)//76) //low_02
  //if(pK->nFrameIdAbs==76)//82) //low_11
  {
      cout<<endl<<"AddPt() called"<<endl;
      //AddOFPoints();
      //AddPt(0);
      AddPt(1);
      //AddPt(2);
  }
#endif
#if 0
  static bool isFirst=true, isSecond=true;//false;
  if(isFirst)
  {
      if(isSecond)
      {
	  AddPt(0);
	  isFirst=false;
      }
      isSecond=true;
  }
#endif
  
  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;
}

//E
#if 0
bool MapMaker::AddPt(int segment)
{
    //Getting the keyframes for triangulation
    KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The new keyframe
    KeyFrame &kTarget = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size()-2]);
    cout<<"AddPt Ksrc.id = "<<kSrc.nFrameIdAbs<<", Ktarget.id = "<<kTarget.nFrameIdAbs<<endl;
    mDmm.AddPt(kSrc, kTarget, segment);
    if(!segment)
	mbAGP=true;
    //mDmm.AddPt(kSrc, kTarget, 0);
    //mDmm.AddPt(kSrc, kTarget, 1);
}

//E
bool MapMaker::AddGroundPoints(int segment)
{
    //Getting the keyframes for triangulation
    //KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The new keyframe
    //cout<<endl<<"AGP Ksrc.id = "<<kSrc.nFrameIdAbs<<endl;
    mDmm.AGP(mMap.vpKeyFrames.size() - 1);
}

//E
//Triangulte the Dense points in Tracker.
void MapMaker::TriangulateDense(KeyFrame &kSrc,
			       KeyFrame &kTarget,
			       vector<pair<Vector<2>,Vector<2> > > vDenseMatches)
{
    for(unsigned int i=0; i<vDenseMatches.size(); i++)
    {
	Vector<3> v3New;
	v3New = kTarget.se3CfromW.inverse() *  
	    ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
		    mCamera.UnProject(vDenseMatches[i].first), 
		    mCamera.UnProject(vDenseMatches[i].second));

	mMap.vpDensePoints.push_back(v3New);
    }
    cout<<"dSz = "<<mMap.vpDensePoints.size()<<endl;
}

//E
vector<TooN::Vector<3> > MapMaker::MaskPoints(KeyFrame &kMaskFrame, const char *maskPath)
{
    using namespace cv;
    Mat mask = imread(maskPath, CV_LOAD_IMAGE_GRAYSCALE);
    vector<TooN::Vector<3> > vMaskPt;
    if(mask.empty())
    {
	cout<<"the mask is not there, man"<<endl;
	return vMaskPt;
    }

    vector<MapPoint*> vMapPoints = mMap.vpPoints;
    TooN::Vector<2> v2ImPoint;
    for(unsigned int i=0; i<vMapPoints.size(); i++)
    {
	printf("\ri=%d", i);
	fflush(stdout);
	if(kMaskFrame.mMeasurements.count(vMapPoints[i])==0)
	    continue;

	v2ImPoint = kMaskFrame.mMeasurements[vMapPoints[i]].v2RootPos;
	if(mask.at<uchar>(v2ImPoint[1], v2ImPoint[0])==255)
	{
	    mMap.interestPoint[i]=true;
	    vMaskPt.push_back(vMapPoints[i]->v3WorldPos);
	}
    }
    printf("\n");
    return vMaskPt;
}

//E
vector<Vector<3> > MapMaker::FindMaskPlane(vector<TooN::Vector<3> > vMaskPt)
{
  unsigned int nPoints = vMaskPt.size();
  vector<Vector<3> > retVal;
  if(nPoints < 3)
	{
		cout << "  MapMaker: too few points to find Dominant plane." << endl;
		retVal.push_back(makeVector(0,0,1));
		retVal.push_back(makeVector(0,0,0));
		return retVal;
	};
  
  int nRansacs = GV2.GetInt("MapMaker.PlaneAlignerRansacs", 1000, HIDDEN|SILENT);
  Vector<3> v3BestMean;
  Vector<3> v3BestNormal;
  double dBestDistSquared = 9999999999999999.9;

  //Matrix<> M(3, vMaskPt.size());
  Matrix<> M(vMaskPt.size(),3);
  SVD<> svd(M);

#if 1
  for(unsigned int i=0; i<vMaskPt.size(); i++)
  {
      v3BestMean += vMaskPt[i];
  }
  v3BestMean *= (1.0/vMaskPt.size());
  for(unsigned int i=0; i<vMaskPt.size(); i++)
  {
      vMaskPt[i] = vMaskPt[i] - v3BestMean;
      //M.T()[i] = vMaskPt[i];
      M[i] = vMaskPt[i];
  }

  svd.compute(M);
  Vector<> vSmallest = svd.get_VT()[2];
  cout<<"vss="<<vSmallest<<endl;
  mMap.vpPlanePoints.push_back(v3BestMean);
  mMap.vpPlanePoints.push_back(vSmallest);
#endif

#if 0  
  //RANSAC LOOP
  int nA, nB, nC;
  for(int i=0; i<nRansacs; i++)
	{
		nA = rand()%nPoints;
		nB = nA;
		nC = nA;
		while(nB == nA)
			nB = rand()%nPoints;
		while(nC == nA || nC==nB)
			nC = rand()%nPoints;
		
		Vector<3> v3Mean = 0.33333333 * (vMaskPt[nA] + 
			       vMaskPt[nB] + 
			       vMaskPt[nC]);
		
		Vector<3> v3CA = vMaskPt[nC]  - vMaskPt[nA];
		Vector<3> v3BA = vMaskPt[nB]  - vMaskPt[nA];
		Vector<3> v3Normal = v3CA ^ v3BA;
		if(v3Normal * v3Normal  == 0)
			continue;
		normalize(v3Normal);
		
		double dSumError = 0.0;
		for(unsigned int i=0; i<nPoints; i++)
		{
		  Vector<3> v3Diff = vMaskPt[i] - vMaskPt[nA];
		  double dDistSq = v3Diff * v3Diff;
		  if(dDistSq == 0.0)
		    continue;
		  double dNormDist = fabs(v3Diff * v3Normal);
		  
		  if(dNormDist > 0.05)
		    dNormDist = 0.05;
		  dSumError += dNormDist;
		}
		if(dSumError < dBestDistSquared)
		{
		  dBestDistSquared = dSumError;
		  v3BestMean = v3Mean;
		  v3BestNormal = v3Normal;
		}
	}
  
  mMap.vpPlanePoints.push_back(vMaskPt[nA]);
  mMap.vpPlanePoints.push_back(vMaskPt[nB]);
  mMap.vpPlanePoints.push_back(vMaskPt[nC]);
  mMap.vpPlanePoints.push_back(v3BestMean);
#endif

  retVal.push_back(v3BestNormal);
  retVal.push_back(v3BestMean);
  return retVal; 
}

//E
void MapMaker::RotatePlane()
{
    vector<Vector<3> > vGroundPoints = mMap.vpDensePoints;
    Matrix<3> m3Cov = Zeros;
    Vector<3> v3Diff;
    Vector<3> v3Mean = Zeros;

    for(unsigned int i=0; i<vGroundPoints.size(); i++)
	v3Mean += vGroundPoints[i];
    v3Mean *= (1.0 / vGroundPoints.size());

    for(unsigned int i=0; i<vGroundPoints.size(); i++)
    {
	v3Diff = vGroundPoints[i] - v3Mean;
	m3Cov += v3Diff.as_col() * v3Diff.as_row();
    }
    SymEigen<3> sym(m3Cov);
    Vector<3> v3Normal = sym.get_evectors()[0];

    /*if(v3Normal[2] > 0)
    {
	cout<<"turning..."<<endl;
	v3Normal *= -1.0;
    }*/

    Matrix<3> m3Rot = Identity;
    m3Rot[2] = v3Normal;
    m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal) );
    normalize(m3Rot[0]);
    m3Rot[1] = m3Rot[2] ^ m3Rot[0];

    SE3<> se3Rot;
    se3Rot.get_rotation() = m3Rot;
    v3Mean[2]=0;
    cout<<"v3M = "<<v3Mean<<endl;
    se3Rot.get_translation() = /*makeVector(0,0,0);*//*-1*(se3Rot * v3Mean) +*/ v3Mean;
    for(unsigned int i=0; i<mMap.vpDensePoints.size(); i++)
    {
	mMap.vpDensePoints[i] = se3Rot * (mMap.vpDensePoints[i] - v3Mean);
    }
}
#endif

// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddPointEpipolar(KeyFrame &kSrc, 
				KeyFrame &kTarget, 
				int nLevel,
				int nCandidate)
{
  static Image<Vector<2> > imUnProj;
  static bool bMadeCache = false;
  if(!bMadeCache)
	{
		imUnProj.resize(kSrc.aLevels[0].im.size());
		ImageRef ir;
		do imUnProj[ir] = mCamera.UnProject(ir);
		while(ir.next(imUnProj.size()));
		bMadeCache = true;
	}
  
  int nLevelScale = LevelScale(nLevel);
  Candidate &candidate = kSrc.aLevels[nLevel].vCandidates[nCandidate];
  ImageRef irLevelPos = candidate.irLevelPos;
  Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);
  
  Vector<3> v3Ray_SC = unproject(mCamera.UnProject(v2RootPos));
  normalize(v3Ray_SC);
  Vector<3> v3LineDirn_TC = kTarget.se3CfromW.get_rotation() * (kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC);

  // Restrict epipolar search to a relatively narrow depth range
  // to increase reliability
  double dMean = kSrc.dSceneDepthMean;
  double dSigma = kSrc.dSceneDepthSigma;
  double dStartDepth = max(mdWiggleScale, dMean - dSigma);
  double dEndDepth = min(40 * mdWiggleScale, dMean + dSigma);
  
  Vector<3> v3CamCenter_TC = kTarget.se3CfromW * kSrc.se3CfromW.inverse().get_translation(); // The camera end
  Vector<3> v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC;                               // the far-away end
  Vector<3> v3RayEnd_TC = v3CamCenter_TC + dEndDepth * v3LineDirn_TC;                               // the far-away end

  
  if(v3RayEnd_TC[2] <= v3RayStart_TC[2])  // it's highly unlikely that we'll manage to get anything out if we're facing backwards wrt the other camera's view-ray
    return false;
  if(v3RayEnd_TC[2] <= 0.0 )  return false;
  if(v3RayStart_TC[2] <= 0.0)
    v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);
  
  Vector<2> v2A = project(v3RayStart_TC);
  Vector<2> v2B = project(v3RayEnd_TC);
  Vector<2> v2AlongProjectedLine = v2A-v2B;
  
  if(v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001)
    {
      //cout << "v2AlongProjectedLine too small." << endl;
      return false;
    }
  normalize(v2AlongProjectedLine);
  Vector<2> v2Normal;
  v2Normal[0] = v2AlongProjectedLine[1];
  v2Normal[1] = -v2AlongProjectedLine[0];
  
  double dNormDist = v2A * v2Normal;
  if(fabs(dNormDist) > mCamera.LargestRadiusInImage() )
    return false;
  
  double dMinLen = min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
  double dMaxLen = max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
  if(dMinLen < -2.0)  dMinLen = -2.0;
  if(dMaxLen < -2.0)  dMaxLen = -2.0;
  if(dMinLen > 2.0)   dMinLen = 2.0;
  if(dMaxLen > 2.0)   dMaxLen = 2.0;

  // Find current-frame corners which might match this
  PatchFinder Finder;
  Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
  if(Finder.TemplateBad())  return false;
  
  vector<Vector<2> > &vv2Corners = kTarget.aLevels[nLevel].vImplaneCorners;
  vector<ImageRef> &vIR = kTarget.aLevels[nLevel].vCorners;
  if(!kTarget.aLevels[nLevel].bImplaneCornersCached)
	{
		for(unsigned int i=0; i<vIR.size(); i++)   // over all corners in target img..
			vv2Corners.push_back(imUnProj[ir(LevelZeroPos(vIR[i], nLevel))]);
		kTarget.aLevels[nLevel].bImplaneCornersCached = true;
	}
  
  int nBest = -1;
  int nBestZMSSD = Finder.mnMaxSSD + 1;
  double dMaxDistDiff = mCamera.OnePixelDist() * (4.0 + 1.0 * nLevelScale);
  double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;
  
  for(unsigned int i=0; i<vv2Corners.size(); i++)   // over all corners in target img..
	{
		Vector<2> v2Im = vv2Corners[i];
		double dDistDiff = dNormDist - v2Im * v2Normal;
		if(dDistDiff * dDistDiff > dMaxDistSq)	continue; // skip if not along epi line
		if(v2Im * v2AlongProjectedLine < dMinLen)	continue; // skip if not far enough along line
		if(v2Im * v2AlongProjectedLine > dMaxLen)	continue; // or too far
		int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aLevels[nLevel].im, vIR[i]);
		if(nZMSSD < nBestZMSSD)
		{
			nBest = i;
			nBestZMSSD = nZMSSD;
		}
	} 
  
  if(nBest == -1)   return false;   // Nothing found.
  
  //  Found a likely candidate along epipolar ray
  Finder.MakeSubPixTemplate();
  Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
  bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget,10);
  if(!bSubPixConverges)
    return false;
		
  
  // Now triangulate the 3d point...
  Vector<3> v3New;
  v3New = kTarget.se3CfromW.inverse() *  
    ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
		   mCamera.UnProject(v2RootPos), 
		   mCamera.UnProject(Finder.GetSubPixPos()));
#if 0
  if(v2RootPos[0]<350 && v2RootPos[0]>300)
  {
      //ofstream ofPose("sparse_pose.txt");
      //ofPose<<kSrc.nFrameIdAbs<<" "<<kTarget.nFrameIdAbs<<endl;
      //ofPose<<"kSrc="<<endl<<kSrc.se3CfromW<<endl;
      //ofPose<<"kTarget="<<endl<<kTarget.se3CfromW;
      //ofPose.close();
      Vector<2> camUnPTemp1 = mCamera.UnProject(v2RootPos);
      Vector<2> camUnPTemp2 = mCamera.UnProject(Finder.GetSubPixPos());
      FILE *fp = fopen("ptam_pt.txt", "a");
      fprintf(fp, "%d %d => ", kSrc.nFrameIdAbs, kTarget.nFrameIdAbs);
      fprintf(fp, "%.2lf %.2lf; %.2lf %.2lf :: ", v2RootPos[0], v2RootPos[1], Finder.GetSubPixPos()[0], Finder.GetSubPixPos()[1]);
      //fprintf(fp, "%lf %lf; %lf %lf :: ", camUnPTemp1[0], camUnPTemp1[1], camUnPTemp2[0], camUnPTemp2[1]);
      fprintf(fp, "%lf %lf %lf\n", v3New[0], v3New[1], v3New[2]);
      fclose(fp);
  }
#endif
  
  MapPoint *pNew = new MapPoint;
  pNew->v3WorldPos = v3New;
  pNew->pMMData = new MapMakerData();
  
  // Patch source stuff:
  pNew->pPatchSourceKF = &kSrc;
  pNew->nSourceLevel = nLevel;
  pNew->v3Normal_NC = makeVector( 0,0,-1);
  pNew->irCenter = irLevelPos;
  pNew->v3Center_NC = unproject(mCamera.UnProject(v2RootPos));
  pNew->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromCenter_NC  = unproject(mCamera.UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));
  
  normalize(pNew->v3Center_NC);
  normalize(pNew->v3OneDownFromCenter_NC);
  normalize(pNew->v3OneRightFromCenter_NC);
  
  pNew->RefreshPixelVectors();
    
  mMap.vpPoints.push_back(pNew);
  //mMap.interestPoint.push_back(false); //E
  mqNewQueue.push(pNew);
  Measurement m;
  m.Source = Measurement::SRC_ROOT;
  m.v2RootPos = v2RootPos;
  m.nLevel = nLevel;
  m.bSubPix = true;
  kSrc.mMeasurements[pNew] = m;

  m.Source = Measurement::SRC_EPIPOLAR;
  m.v2RootPos = Finder.GetSubPixPos();
  kTarget.mMeasurements[pNew] = m;
  pNew->pMMData->sMeasurementKFs.insert(&kSrc);
  pNew->pMMData->sMeasurementKFs.insert(&kTarget);
  return true;
}

//E
#if 0
void MapMaker::AddOFPoints()
{
  KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The new keyframe
  KeyFrame &kTarget = *(ClosestKeyFrame(kSrc));   
  //ifstream ifs("../codes/Homography/matches.txt");
  ifstream ifs("matches.txt");
  Vector<2> irFirst, irSecond;
  int count=0;
  if(ifs.is_open())
  {
      cout<<"AddOFPoints"<<endl;
      ifs>>irFirst;
      while(!ifs.eof())
      {
	  ifs>>irFirst; ifs>>irSecond;
	  //cout<<irFirst<<", "<<irSecond<<endl;
	  if(AddOFPointEpipolar(kSrc, kTarget, irSecond))
	      count++;
      }
      cout<<"count ="<<count<<endl;
      ifs.close();
  }
  
}

// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddOFPointEpipolar(KeyFrame &kSrc, 
				KeyFrame &kTarget, 
				Vector<2> point)
{
    int nLevel =0;
    static Image<Vector<2> > imUnProj;
    static bool bMadeCache = false;
    if(!bMadeCache)
    {
	imUnProj.resize(kSrc.aLevels[0].im.size());
	ImageRef ir;
	do imUnProj[ir] = mCamera.UnProject(ir);
	while(ir.next(imUnProj.size()));
	bMadeCache = true;
    }

    Vector<2> v2RootPos = point;
    ImageRef irLevelPos(point[0], point[1]);
  int nLevelScale = LevelScale(nLevel);

    Vector<3> v3Ray_SC = unproject(mCamera.UnProject(v2RootPos));
    normalize(v3Ray_SC);
    Vector<3> v3LineDirn_TC = kTarget.se3CfromW.get_rotation() * (kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC);

    // Restrict epipolar search to a relatively narrow depth range
    // to increase reliability
    double dMean = kSrc.dSceneDepthMean;
    double dSigma = kSrc.dSceneDepthSigma;
    double dStartDepth = max(mdWiggleScale, dMean - dSigma);
    double dEndDepth = min(40 * mdWiggleScale, dMean + dSigma);

    Vector<3> v3CamCenter_TC = kTarget.se3CfromW * kSrc.se3CfromW.inverse().get_translation(); // The camera end
    Vector<3> v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC;                               // the far-away end
    Vector<3> v3RayEnd_TC = v3CamCenter_TC + dEndDepth * v3LineDirn_TC;                               // the far-away end


    if(v3RayEnd_TC[2] <= v3RayStart_TC[2])  // it's highly unlikely that we'll manage to get anything out if we're facing backwards wrt the other camera's view-ray
	return false;
    if(v3RayEnd_TC[2] <= 0.0 )  return false;
    if(v3RayStart_TC[2] <= 0.0)
	v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);

    Vector<2> v2A = project(v3RayStart_TC);
    Vector<2> v2B = project(v3RayEnd_TC);
    Vector<2> v2AlongProjectedLine = v2A-v2B;

    if(v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001)
    {
	//cout << "v2AlongProjectedLine too small." << endl;
	return false;
    }
    normalize(v2AlongProjectedLine);
    Vector<2> v2Normal;
    v2Normal[0] = v2AlongProjectedLine[1];
    v2Normal[1] = -v2AlongProjectedLine[0];

    double dNormDist = v2A * v2Normal;
    if(fabs(dNormDist) > mCamera.LargestRadiusInImage() )
	return false;

    double dMinLen = min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
    double dMaxLen = max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
    if(dMinLen < -2.0)  dMinLen = -2.0;
    if(dMaxLen < -2.0)  dMaxLen = -2.0;
    if(dMinLen > 2.0)   dMinLen = 2.0;
    if(dMaxLen > 2.0)   dMaxLen = 2.0;

    // Find current-frame corners which might match this
    PatchFinder Finder;
    Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
    if(Finder.TemplateBad())  return false;

    vector<Vector<2> > &vv2Corners = kTarget.aLevels[nLevel].vImplaneCorners;
    vector<ImageRef> &vIR = kTarget.aLevels[nLevel].vCorners;
    if(!kTarget.aLevels[nLevel].bImplaneCornersCached)
    {
	for(unsigned int i=0; i<vIR.size(); i++)   // over all corners in target img..
	    vv2Corners.push_back(imUnProj[ir(LevelZeroPos(vIR[i], nLevel))]);
	kTarget.aLevels[nLevel].bImplaneCornersCached = true;
    }

    int nBest = -1;
    int nBestZMSSD = Finder.mnMaxSSD + 1;
    double dMaxDistDiff = mCamera.OnePixelDist() * (4.0 + 1.0 * nLevelScale);
    double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;

    for(unsigned int i=0; i<vv2Corners.size(); i++)   // over all corners in target img..
    {
	Vector<2> v2Im = vv2Corners[i];
	double dDistDiff = dNormDist - v2Im * v2Normal;
	if(dDistDiff * dDistDiff > dMaxDistSq)	continue; // skip if not along epi line
	if(v2Im * v2AlongProjectedLine < dMinLen)	continue; // skip if not far enough along line
	if(v2Im * v2AlongProjectedLine > dMaxLen)	continue; // or too far
	int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aLevels[nLevel].im, vIR[i]);
	if(nZMSSD < nBestZMSSD)
	{
	    nBest = i;
	    nBestZMSSD = nZMSSD;
	}
    } 

    if(nBest == -1)   return false;   // Nothing found.

    //  Found a likely candidate along epipolar ray
    Finder.MakeSubPixTemplate();
    Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
    bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget,10);
    if(!bSubPixConverges)
	return false;


    // Now triangulate the 3d point...
    Vector<3> v3New;
    v3New = kTarget.se3CfromW.inverse() *  
	ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
		mCamera.UnProject(v2RootPos), 
		mCamera.UnProject(Finder.GetSubPixPos()));
#if 0
    if(v2RootPos[0]<350 && v2RootPos[0]>300)
    {
	//ofstream ofPose("sparse_pose.txt");
	//ofPose<<kSrc.nFrameIdAbs<<" "<<kTarget.nFrameIdAbs<<endl;
	//ofPose<<"kSrc="<<endl<<kSrc.se3CfromW<<endl;
	//ofPose<<"kTarget="<<endl<<kTarget.se3CfromW;
	//ofPose.close();
	Vector<2> camUnPTemp1 = mCamera.UnProject(v2RootPos);
	Vector<2> camUnPTemp2 = mCamera.UnProject(Finder.GetSubPixPos());
	FILE *fp = fopen("ptam_pt.txt", "a");
	fprintf(fp, "%d %d => ", kSrc.nFrameIdAbs, kTarget.nFrameIdAbs);
	fprintf(fp, "%.2lf %.2lf; %.2lf %.2lf :: ", v2RootPos[0], v2RootPos[1], Finder.GetSubPixPos()[0], Finder.GetSubPixPos()[1]);
	//fprintf(fp, "%lf %lf; %lf %lf :: ", camUnPTemp1[0], camUnPTemp1[1], camUnPTemp2[0], camUnPTemp2[1]);
	fprintf(fp, "%lf %lf %lf\n", v3New[0], v3New[1], v3New[2]);
	fclose(fp);
    }
#endif

    mMap.vpOFPoints.push_back(v3New);
    MapPoint *pNew = new MapPoint;
    pNew->v3WorldPos = v3New;
    pNew->pMMData = new MapMakerData();

    // Patch source stuff:
    pNew->pPatchSourceKF = &kSrc;
    pNew->nSourceLevel = nLevel;
    pNew->v3Normal_NC = makeVector( 0,0,-1);
    pNew->irCenter = irLevelPos;
    pNew->v3Center_NC = unproject(mCamera.UnProject(v2RootPos));
    pNew->v3OneRightFromCenter_NC = unproject(mCamera.UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
    pNew->v3OneDownFromCenter_NC  = unproject(mCamera.UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));

    normalize(pNew->v3Center_NC);
    normalize(pNew->v3OneDownFromCenter_NC);
    normalize(pNew->v3OneRightFromCenter_NC);

    pNew->RefreshPixelVectors();

    mMap.vpPoints.push_back(pNew);
    mMap.interestPoint.push_back(false); //E
    mqNewQueue.push(pNew);
    Measurement m;
    m.Source = Measurement::SRC_ROOT;
    m.v2RootPos = v2RootPos;
    m.nLevel = nLevel;
    m.bSubPix = true;
    kSrc.mMeasurements[pNew] = m;

    m.Source = Measurement::SRC_EPIPOLAR;
    m.v2RootPos = Finder.GetSubPixPos();
    kTarget.mMeasurements[pNew] = m;
    pNew->pMMData->sMeasurementKFs.insert(&kSrc);
    pNew->pMMData->sMeasurementKFs.insert(&kTarget);
    return true;
}
#endif


double MapMaker::KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2)
{
  Vector<3> v3KF1_CamPos = k1.se3CfromW.inverse().get_translation();
  Vector<3> v3KF2_CamPos = k2.se3CfromW.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double dDist = sqrt(v3Diff * v3Diff);
  return dDist;
}

vector<KeyFrame*> MapMaker::NClosestKeyFrames(KeyFrame &k, unsigned int N)
{
  vector<pair<double, KeyFrame* > > vKFandScores;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mMap.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
      vKFandScores.push_back(make_pair(dDist, mMap.vpKeyFrames[i]));
    }
  if(N > vKFandScores.size())
    N = vKFandScores.size();
  partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());
  
  vector<KeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);
  return vResult;
}

KeyFrame* MapMaker::ClosestKeyFrame(KeyFrame &k)
{
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mMap.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
      if(dDist < dClosestDist)
	{
	  dClosestDist = dDist;
	  nClosest = i;
	}
    }
  assert(nClosest != -1);
  return mMap.vpKeyFrames[nClosest];
}

double MapMaker::DistToNearestKeyFrame(KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
  return dDist;
}

bool MapMaker::NeedNewKeyFrame(KeyFrame &kCurrent)
{


//printf("ratio pvs: %f\n",ratio_pvs);
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
  dDist *= (1.0 / kCurrent.dSceneDepthMean);
  
  if(dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",1.0,SILENT) * mdWiggleScaleDepthNormalized)//&&ratio_pvs<0.6)
    return true;
  return false;
}

// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAll()
{
  mbMapMakerStatus = STAT_GLOBAL_BA;
	
//	printf("BA Global\n");
	// construct the sets of kfs/points to be adjusted:
  // in this case, all of them
  set<KeyFrame*> sAdj;
  set<KeyFrame*> sFixed;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    if(mMap.vpKeyFrames[i]->bFixed)
      sFixed.insert(mMap.vpKeyFrames[i]);
    else
      sAdj.insert(mMap.vpKeyFrames[i]);
  
  set<MapPoint*> sMapPoints;
  for(unsigned int i=0; i<mMap.vpPoints.size();i++)
    sMapPoints.insert(mMap.vpPoints[i]);
  
  BundleAdjust(sAdj, sFixed, sMapPoints, false);

}
void MapMaker::BundleAdjustConstraint()
{
  mbMapMakerStatus = STAT_CONST_BA;
	if(mMap.vpKeyFrames.size() < 30)  
    { // Ignore this unless map is big enough
     BundleAdjustAll();
// mbBundleConverged_ = true;
      return;
    }

//	printf("BA Constraint\n");
  // First, make a list of the keyframes we want adjusted in the adjuster.
  // This will be the last keyframe inserted, and its four nearest neighbors
  set<KeyFrame*> sAdjustSet;
  KeyFrame *pkfNewest = mMap.vpKeyFrames.back();
  sAdjustSet.insert(pkfNewest);
  vector<KeyFrame*> vClosest = NClosestKeyFrames(*pkfNewest, 30);
  for(int i=0; i<30; i++)
    if(vClosest[i]->bFixed == false)
      sAdjustSet.insert(vClosest[i]);
  
  // Now we find the set of features which they contain.
  set<MapPoint*> sMapPoints;
  for(set<KeyFrame*>::iterator iter = sAdjustSet.begin();
      iter!=sAdjustSet.end();
      iter++)
    {
      map<MapPoint*,Measurement> &mKFMeas = (*iter)->mMeasurements;
      for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
	sMapPoints.insert(jiter->first);
    };
  
  // Finally, add all keyframes which measure above points as fixed keyframes
  set<KeyFrame*> sFixedSet;
  for(vector<KeyFrame*>::iterator it = mMap.vpKeyFrames.begin(); it!=mMap.vpKeyFrames.end(); it++)
    {
      if(sAdjustSet.count(*it))
	continue;
      bool bInclude = false;
      for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
	if(sMapPoints.count(jiter->first))
	  {
	    bInclude = true;
	    break;
	  }
      if(bInclude)
	sFixedSet.insert(*it);
    }
  
  BundleAdjust(sAdjustSet, sFixedSet, sMapPoints, false);
}


// Peform a local bundle adjustment which only adjusts
// recently added key-frames
void MapMaker::BundleAdjustRecent()
{


  mbMapMakerStatus = STAT_LOCAL_BA;
	
	if(mMap.vpKeyFrames.size() < 8)  
    { // Ignore this unless map is big enough
      mbBundleConverged_Recent = true;
      return;
    }

  // First, make a list of the keyframes we want adjusted in the adjuster.
  // This will be the last keyframe inserted, and its four nearest neighbors
  set<KeyFrame*> sAdjustSet;
  KeyFrame *pkfNewest = mMap.vpKeyFrames.back();
  sAdjustSet.insert(pkfNewest);
  vector<KeyFrame*> vClosest = NClosestKeyFrames(*pkfNewest, 4);
  for(int i=0; i<4; i++)
    if(vClosest[i]->bFixed == false)
      sAdjustSet.insert(vClosest[i]);
  
  // Now we find the set of features which they contain.
  set<MapPoint*> sMapPoints;
  for(set<KeyFrame*>::iterator iter = sAdjustSet.begin();
      iter!=sAdjustSet.end();
      iter++)
    {
      map<MapPoint*,Measurement> &mKFMeas = (*iter)->mMeasurements;
      for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
	sMapPoints.insert(jiter->first);
    };
  
  // Finally, add all keyframes which measure above points as fixed keyframes
  set<KeyFrame*> sFixedSet;
  for(vector<KeyFrame*>::iterator it = mMap.vpKeyFrames.begin(); it!=mMap.vpKeyFrames.end(); it++)
    {
      if(sAdjustSet.count(*it))
	continue;
      bool bInclude = false;
      for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
	if(sMapPoints.count(jiter->first))
	  {
	    bInclude = true;
	    break;
	  }
      if(bInclude)
	sFixedSet.insert(*it);
    }
  
  BundleAdjust(sAdjustSet, sFixedSet, sMapPoints, true);
}

// Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
void MapMaker::BundleAdjust(set<KeyFrame*> sAdjustSet, set<KeyFrame*> sFixedSet, set<MapPoint*> sMapPoints, bool bRecent)
{
  //Bundle b(mCamera);   // Our bundle adjuster

  //E
#ifdef _multi
  CameraModel *camMod = new CameraModel;
  Bundle b(camMod);   // Our bundle adjuster
#else
  Bundle b(mCamera);
#endif
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;
  
  // The bundle adjuster does different accounting of keyframes and map points;
  // Translation maps are stored:
  map<MapPoint*, int> mPoint_BundleID;
  map<int, MapPoint*> mBundleID_Point;
  map<KeyFrame*, int> mView_BundleID;
  map<int, KeyFrame*> mBundleID_View;
  
  // Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  for(set<KeyFrame*>::iterator it = sAdjustSet.begin(); it!= sAdjustSet.end(); it++)
    {
      int nBundleID = b.AddCamera((*it)->se3CfromW, (*it)->bFixed);
      mView_BundleID[*it] = nBundleID;
      mBundleID_View[nBundleID] = *it;
    }
  for(set<KeyFrame*>::iterator it = sFixedSet.begin(); it!= sFixedSet.end(); it++)
    {
      int nBundleID = b.AddCamera((*it)->se3CfromW, true);
      mView_BundleID[*it] = nBundleID;
      mBundleID_View[nBundleID] = *it;
    }
  
  // Add the points' 3D position
  for(set<MapPoint*>::iterator it = sMapPoints.begin(); it!=sMapPoints.end(); it++)
    {
      int nBundleID = b.AddPoint((*it)->v3WorldPos);
      mPoint_BundleID[*it] = nBundleID;
      mBundleID_Point[nBundleID] = *it;
    }
  
  // Add the relevant point-in-keyframe measurements
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mView_BundleID.count(mMap.vpKeyFrames[i]) == 0)
	continue;
      
      int nKF_BundleID = mView_BundleID[mMap.vpKeyFrames[i]];
      for(meas_it it= mMap.vpKeyFrames[i]->mMeasurements.begin();
	  it!= mMap.vpKeyFrames[i]->mMeasurements.end();
	  it++)
	{
	  if(mPoint_BundleID.count(it->first) == 0)
	    continue;
	  int nPoint_BundleID = mPoint_BundleID[it->first];
	  b.AddMeas(nKF_BundleID, nPoint_BundleID, it->second.v2RootPos, LevelScale(it->second.nLevel) * LevelScale(it->second.nLevel));
	}
    }
  
  // Run the bundle adjuster. This returns the number of successful iterations
  int nAccepted = b.Compute(&mbBundleAbortRequested);
  
  if(nAccepted < 0)
    {
      // Crap: - LM Ran into a serious problem!
      // This is probably because the initial stereo was messed up.
      // Get rid of this map and start again! 
      cout << "!! MapMaker: Cholesky failure in bundle adjust. " << endl
	   << "   The map is probably corrupt: Ditching the map. " << endl;
      mbResetRequested = true;
      return;
    }

  // Bundle adjustment did some updates, apply these to the map
  if(nAccepted > 0)
    {
      
      for(map<MapPoint*,int>::iterator itr = mPoint_BundleID.begin();
	  itr!=mPoint_BundleID.end();
	  itr++)
	itr->first->v3WorldPos = b.GetPoint(itr->second);
      
      for(map<KeyFrame*,int>::iterator itr = mView_BundleID.begin();
	  itr!=mView_BundleID.end();
	  itr++)
	itr->first->se3CfromW = b.GetCamera(itr->second);
      if(bRecent)
	mbBundleConverged_Recent = false;
      mbBundleConverged_Full = false;
    };
  
  if(b.Converged())
    {
      mbBundleConverged_Recent = true;
      if(!bRecent)
	mbBundleConverged_Full = true;
    }
  
  mbBundleRunning = false;
  mbBundleAbortRequested = false;
  
  // Handle outlier measurements:
  vector<pair<int,int> > vOutliers_PC_pair = b.GetOutlierMeasurements();
  for(unsigned int i=0; i<vOutliers_PC_pair.size(); i++)
    {
      MapPoint *pp = mBundleID_Point[vOutliers_PC_pair[i].first];
      KeyFrame *pk = mBundleID_View[vOutliers_PC_pair[i].second];
      Measurement &m = pk->mMeasurements[pp];
      if(pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT)   // Is the original source kf considered an outlier? That's bad.
	pp->bBad = true;
      else
	{
	  // Do we retry it? Depends where it came from!!
	  if(m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
	    mvFailureQueue.push_back(pair<KeyFrame*,MapPoint*>(pk,pp));
	  else
	    pp->pMMData->sNeverRetryKFs.insert(pk);
	  pk->mMeasurements.erase(pp);
	  pp->pMMData->sMeasurementKFs.erase(pk);
	}
    }
}

// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in 
// TrackerData.h.
bool MapMaker::ReFind_Common(KeyFrame &k, MapPoint &p)
{
  // abort if either a measurement is already in the map, or we've
  // decided that this point-kf combo is beyond redemption
  if(p.pMMData->sMeasurementKFs.count(&k)
     || p.pMMData->sNeverRetryKFs.count(&k))
    return false;
  
  static PatchFinder Finder;
  Vector<3> v3Cam = k.se3CfromW*p.v3WorldPos;
  if(v3Cam[2] < 0.001)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  Vector<2> v2ImPlane = project(v3Cam);
  if(v2ImPlane* v2ImPlane > mCamera.LargestRadiusInImage() * mCamera.LargestRadiusInImage())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  Vector<2> v2Image = mCamera.Project(v2ImPlane);
  if(mCamera.Invalid())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  ImageRef irImageSize = k.aLevels[0].im.size();
  if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  Matrix<2> m2CamDerivs = mCamera.GetProjectionDerivs();
  Finder.MakeTemplateCoarse(p, k.se3CfromW, m2CamDerivs);
  
  if(Finder.TemplateBad())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  bool bFound = Finder.FindPatchCoarse(ir(v2Image), k, 4);  // Very tight search radius!
  if(!bFound)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  // If we found something, generate a measurement struct and put it in the map
  Measurement m;
  m.nLevel = Finder.GetLevel();
  m.Source = Measurement::SRC_REFIND;
  
  if(Finder.GetLevel() > 0)
    {
      Finder.MakeSubPixTemplate();
      Finder.IterateSubPixToConvergence(k,8);
      m.v2RootPos = Finder.GetSubPixPos();
      m.bSubPix = true;
    }
  else
    {
      m.v2RootPos = Finder.GetCoarsePosAsVector();
      m.bSubPix = false;
    };
  
  if(k.mMeasurements.count(&p))
    {
      assert(0); // This should never happen, we checked for this at the start.
    }
  k.mMeasurements[&p] = m;
  p.pMMData->sMeasurementKFs.insert(&k);
  return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMaker::ReFindInSingleKeyFrame(KeyFrame &k)
{
  vector<MapPoint*> vToFind;
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    vToFind.push_back(mMap.vpPoints[i]);
  
  int nFoundNow = 0;
  for(unsigned int i=0; i<vToFind.size(); i++)
    if(ReFind_Common(k,*vToFind[i]))
      nFoundNow++;

  return nFoundNow;
};

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void MapMaker::ReFindNewlyMade()
{
  mbMapMakerStatus = STAT_REFIND_NEW_PTS;
	
	if(mqNewQueue.empty())
    return;
  int nFound = 0;
  int nBad = 0;
  while(!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0)
	{
		MapPoint* pNew = mqNewQueue.front();
		mqNewQueue.pop();
		if(pNew->bBad)
		{
			nBad++;
			continue;
		}
		for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
			if(ReFind_Common(*mMap.vpKeyFrames[i], *pNew))
				nFound++;
	}
};

// Dud measurements get a second chance.
void MapMaker::ReFindFromFailureQueue()
{
  mbMapMakerStatus = STAT_REFIND_OUTLIERS;
	
	if(mvFailureQueue.size() == 0)
    return;
  sort(mvFailureQueue.begin(), mvFailureQueue.end());
  vector<pair<KeyFrame*, MapPoint*> >::iterator it;
  int nFound=0;
  for(it = mvFailureQueue.begin(); it!=mvFailureQueue.end(); it++)
    if(ReFind_Common(*it->first, *it->second))
      nFound++;
  
  mvFailureQueue.erase(mvFailureQueue.begin(), it);
};

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent)
{
  return DistToNearestKeyFrame(kCurrent) > mdWiggleScale * 10.0;
}

// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
SE3<> MapMaker::CalcPlaneAligner()
{
  if(!FindDominantPlane())
		return SE3<>();
    
  vector<Vector<3> > vv3Inliers;
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
	{
		if(mMap.vpPoints[i]->nSceneLabel == 1)
			vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
	}
  
  // With these inliers, calculate mean and cov
  Vector<3> v3MeanOfInliers = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    v3MeanOfInliers+=vv3Inliers[i];
  v3MeanOfInliers *= (1.0 / vv3Inliers.size());
  
  Matrix<3> m3Cov = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    {
      Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
      m3Cov += v3Diff.as_col() * v3Diff.as_row();
    };
  
  // Find the principal component with the minimal variance: this is the plane normal
  SymEigen<3> sym(m3Cov);
  Vector<3> v3Normal = sym.get_evectors()[0];
  
  // Use the version of the normal which points towards the cam center
  if(v3Normal[2] > 0)
    v3Normal *= -1.0;
  
  Matrix<3> m3Rot = Identity;
  m3Rot[2] = v3Normal;
  m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];
  
  SE3<> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;
  
  return se3Aligner;
}

// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void MapMaker::RefreshSceneDepth(KeyFrame *pKF)
{
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int nMeas = 0;
  for(meas_it it = pKF->mMeasurements.begin(); it!=pKF->mMeasurements.end(); it++)
    {
      MapPoint &point = *it->first;
      Vector<3> v3PosK = pKF->se3CfromW * point.v3WorldPos;
      dSumDepth += v3PosK[2];
      dSumDepthSquared += v3PosK[2] * v3PosK[2];
      nMeas++;
    }
 
  assert(nMeas > 2); // If not then something is seriously wrong with this KF!!
  pKF->dSceneDepthMean = dSumDepth / nMeas;
  pKF->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean));
}

void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="SaveMap")
  {
		cout << "  MapMaker: Saving the map.... " << endl;
		ofstream ofs("map.dump");
		for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
		{
		  ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
		  ofs << mMap.vpPoints[i]->nSourceLevel << endl;
		}
    ofs.close();
      
    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
		{
		  ostringstream ost1;
		  ost1 << "keyframes/" << i << ".jpg";
			//img_save(mMap.vpKeyFrames[i]->aLevels[0].im, ost1.str());
		  
		  ostringstream ost2;
		  ost2 << "keyframes/" << i << ".info";
		  ofstream ofs2;
		  ofs2.open(ost2.str().c_str());
		  ofs2 << mMap.vpKeyFrames[i]->se3CfromW << endl;
		  ofs2.close();
		}
      cout << "  ... done saving map." << endl;
      return;
  }
  
  if(sCommand=="KeyPress")
  {
      //if(sParams=="d")
      //{
      //    mbAddDense=true;
      //}
      //else if(sParams=="g")
      //{
      //    mbAGP = true;
      //}
      //else if(int(sParams[0])<58 && int(sParams[0]>47))
      //{
      //    mnSegment = sParams[0]-48; //ASCII code for 0 is 48.
      //}
      //else if(sParams=="o")
      //{
      //    //WriteObjectsToFile(*(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]));
      //    WriteObjectsToFile(*(mMap.vpKeyFrames[0]));
      //}
      return;
  }
  cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
};



// Find a dominant plane in the map, Used by CalcPlaneAligner() and CalcGroundPlane()
int MapMaker::FindDominantPlane()
{
  unsigned int nPoints = mMap.vpPoints.size();
  if(nPoints < 10)
	{
		cout << "  MapMaker: too few points to find Dominant plane." << endl;
		return -1;
	};
  
  int nRansacs = GV2.GetInt("MapMaker.PlaneAlignerRansacs", 100, HIDDEN|SILENT);
  Vector<3> v3BestMean;
  Vector<3> v3BestNormal;
  double dBestDistSquared = 9999999999999999.9;
  
  //RANSAC LOOP
  for(int i=0; i<nRansacs; i++)
	{
		int nA = rand()%nPoints;
		int nB = nA;
		int nC = nA;
		while(nB == nA)
			nB = rand()%nPoints;
		while(nC == nA || nC==nB)
			nC = rand()%nPoints;
		
		Vector<3> v3Mean = 0.33333333 * (mMap.vpPoints[nA]->v3WorldPos + 
			       mMap.vpPoints[nB]->v3WorldPos + 
			       mMap.vpPoints[nC]->v3WorldPos);
		
		Vector<3> v3CA = mMap.vpPoints[nC]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
		Vector<3> v3BA = mMap.vpPoints[nB]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
		Vector<3> v3Normal = v3CA ^ v3BA;
		if(v3Normal * v3Normal  == 0)
			continue;
		normalize(v3Normal);
		
		double dSumError = 0.0;
		for(unsigned int i=0; i<nPoints; i++)
		{
		  Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3Mean;
		  double dDistSq = v3Diff * v3Diff;
		  if(dDistSq == 0.0)
		    continue;
		  double dNormDist = fabs(v3Diff * v3Normal);
		  
		  if(dNormDist > 0.05)
		    dNormDist = 0.05;
		  dSumError += dNormDist;
		}
		if(dSumError < dBestDistSquared)
		{
		  dBestDistSquared = dSumError;
		  v3BestMean = v3Mean;
		  v3BestNormal = v3Normal;
		}
	}
  
  // Done the ransacs, now collect the supposed inlier set
  for(unsigned int i=0; i<nPoints; i++)
	{
		Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
		double dDistSq = v3Diff * v3Diff;
		if(dDistSq == 0.0)
			continue;
		double dNormDist = fabs(v3Diff * v3BestNormal);
		if(dNormDist < 0.05)
			mMap.vpPoints[i]->nSceneLabel = 1;
		else
			mMap.vpPoints[i]->nSceneLabel = 0;
	}
	return 1;
}

void MapMaker::CalcGroundPlane()
{
	FindDominantPlane();
}

#if 0
vector<Vector<2> > MapMaker::FindExtraPointsh(int nFirstFrame, int nSecondFrame)
{
    vector<Vector<2> > vv2RetVal;
    //using namespace cv;
    char name[200];
    string sDataDir =  GV3::get<string>("dir");
    int nNod = GV3::get<int>("VideoSource::TotalImages");

    sprintf(name, "%s/%0*d.jpg", sDataDir.c_str(), nNod, nFirstFrame);
    cv::Mat img1 = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
    //Mat img_col1 = imread(name);

    sprintf(name, "%s/%0*d.jpg", sDataDir.c_str(), nNod, nSecondFrame);
    cv::Mat img2 = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
    //Mat img_col2 = imread(name);

    cv::Mat mask;// = imread(argv[4], CV_LOAD_IMAGE_GRAYSCALE);

    _track track;
    ostringstream oss;
    //cv::namedWindow("tracks", CV_WINDOW_AUTOSIZE);
    for(int i=0; i<100; i++)
    {
	oss.str("");
	oss<<"../pg_masks/"<<i<<".png";
	cout<<"\r"<<oss.str();
	mask = cv::imread(oss.str(), CV_LOAD_IMAGE_GRAYSCALE);
	vector<cv::Point2f> pts = track.detect_feature(img1, mask);
	//vector<Point2f> tracks = track.track_feature(img1, img2, pts);
	//cout<<", feat = "<<pts.size()<<", tracks = "<<tracks.size()<<flush;

	//track.draw_features(img_col1, pts, 0, 0, 255);
	//imshow("tracks", img_col1);
	//char k = (char)waitKey(30);
	//if(k==27)
	    //break;
	//track.draw_opticalflow(img_col1, pts, tracks);
	//imshow("tracks", img_col1);
	//waitKey(30);
	for(size_t j=0; j<pts.size(); j++)
	{
	    vv2RetVal.push_back(makeVector(pts[j].x, pts[j].y));
	}
    }
    //cout<<endl;

}

void MapMaker::WriteObjectsToFile(KeyFrame kFrame)
{
    if(mMap.mvoObjects.size() < 2)
    {
	cout<<"Nothing to write idiot, get the dense construction first idiot"<<endl;
	return;
    }
    ostringstream oss;
    oss<<GV3::get<string>("dir") << "/" << GV3::get<string>("md") << "/" << kFrame.nFrameIdAbs << ".obj";
    ofstream ofs(oss.str().c_str());
    ofs<<kFrame.se3CfromW.inverse();
    ofs<<mMap.mvoObjects.size()-1<<endl;
    cout<<"WOF: "<<mMap.mvoObjects.size();
    for ( size_t obj = 1; obj < mMap.mvoObjects.size(); ++obj )//Starts with 1 since there is no need to write ground.
    {
	vector<Vector<3> > vv3Points = mMap.mvoObjects[obj].mvv3Sparse;
	vector<Vector<2> > vv2Points = mMap.mvoObjects[obj].mvv2Sparse;
	ofs<<vv3Points.size()<<endl;
	ofs<<mMap.mvoObjects[obj].mv4Plane<<endl;
	for ( size_t count = 0; count < vv3Points.size(); ++count )
	{
	   ofs<<vv3Points[count]<<" "<<vv2Points[count]<<endl; 
	}
    }
    ofs.close();
    cout<<", WOF end"<<endl;
}
#endif