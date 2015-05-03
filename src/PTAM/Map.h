// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// This header declares the Map class.
// This is pretty light-weight: All it contains is
// a vector of MapPoints and a vector of KeyFrames.
//
// N.b. since I don't do proper thread safety,
// everything is stored as lists of pointers,
// and map points are not erased if they are bad:
// they are moved to the trash list. That way
// old pointers which other threads are using are not 
// invalidated!

#ifndef __MAP_H
#define __MAP_H
#include <vector>
#include <TooN/se3.h>
#include <cvd/image.h>
#include "CamPose.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include "MapPoint.h"
#include "KeyFrame.h"
#include <fstream>


struct MapPoint;
struct KeyFrame;

struct objects
{
    int mnBodyId;
    std::vector<Vector<3> > mvv3Points;
    std::vector<Vector<3> > mvv3Pixels;
    std::vector<Vector<3> > mvv3Sparse;
    std::vector<Vector<2> > mvv2Sparse;
    Vector<4> mv4Plane;
};

struct Map
{
  Map();
  SE3<> lastpose;
  inline bool IsGood() {return bGood;}
  void Reset();
  void save(SE3<>); 
  void MoveBadPointsToTrash();
  void EmptyTrash();
  int mapno;  
  pcl::PointCloud<pcl::PointXYZ> densepoints;
  std::vector<MapPoint*> vpPoints;
  std::vector<MapPoint*> vpPointsTrash;
  std::vector<KeyFrame*> vpKeyFrames;
	
	std::vector<CamPose> vCamPoses;
	std::vector<CamPose> baPose;

  bool bGood;

  //E
  //std::vector<bool> interestPoint;
  //std::vector<Vector<3> > vpDensePoints; //Added to make dense points possible.//change
  //std::vector<Vector<3> > vpPlanePoints;
  //std::vector<Vector<3> > vpBAPoints;
  //std::vector<Vector<3> > vpPixelValues; //Added to make dense points possible.//change
  //std::vector<Vector<3> > vpBAPixelValues; //Added to make dense points possible.//change
  //std::vector<Vector<3> > vpOFPoints; //Added to make dense points possible.//change
  //std::vector<objects> mvoObjects;

};




#endif
