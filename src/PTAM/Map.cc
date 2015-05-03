// Copyright 2008 Isis Innovation Limited
#include "Map.h"
#include "MapPoint.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
Map::Map()
{
mapno = 0 ;
  Reset();
}

void Map::Reset()
{
  for(unsigned int i=0; i<vpPoints.size(); i++)
    delete vpPoints[i];
  vpPoints.clear();
  /*for(unsigned int i=0; i<vpDensePoints.size(); i++)
    delete vpDensePoints[i];
  vpDensePoints.clear();*/
  bGood = false;
  EmptyTrash();
}

void Map::MoveBadPointsToTrash()
{
  int nBad = 0;
  for(int i = vpPoints.size()-1; i>=0; i--)
    {
      if(vpPoints[i]->bBad)
	{
	  vpPointsTrash.push_back(vpPoints[i]);
	  vpPoints.erase(vpPoints.begin() + i);
	  nBad++;
	}
    };
};

void Map::EmptyTrash()
{
  for(unsigned int i=0; i<vpPointsTrash.size(); i++)
    delete vpPointsTrash[i];
  vpPointsTrash.clear();
};

void Map::save(SE3<> pose){
	if(vpPoints.size() >0){

	ostringstream ost2;
	ost2 << "data/keyframes"<<mapno<<".info";
	ofstream ofs2;
	ofs2.open(ost2.str().c_str());
 
	for(unsigned int i=0; i< vpKeyFrames.size(); i++)
		ofs2 << vpKeyFrames[i]->se3CfromW<< endl;

	ofs2.close();

	pcl::PointCloud<pcl::PointXYZ> cloud;
	Vector<3> point;
	for(size_t i=0; i<vpPoints.size(); i++){	
		point = lastpose*vpPoints[i]->v3WorldPos;
		cloud.push_back (pcl::PointXYZ ( point[0], point[1], point[2]));
	}

	cloud.width=vpPoints.size();
	cloud.height=1;
	cloud.resize(cloud.width*cloud.height);
	cloud.is_dense=false;


	pcl::PCDWriter writer;
	ostringstream ost1;
	ost1 << "data/points"<<mapno<<".info";
	writer.write(ost1.str(),cloud,false);


	ost2 << "data/pose_"<<mapno<<".info";
	ofs2.open(ost2.str().c_str());
	ofs2 <<pose<< endl;
	ofs2.close();

	lastpose = pose*lastpose;
	mapno++;
}}
