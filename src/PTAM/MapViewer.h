// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// MapViewer.h
//
// Defines the MapViewer class
//
// This defines a simple map viewer widget, which can draw the 
// current map and the camera/keyframe poses within it.
//
#ifndef __MAP_VIEWER_H
#define __MAP_VIEWER_H

#include "Map.h"
#include <TooN/TooN.h>
using namespace TooN;

#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include <TooN/se3.h>
#include <sstream>
#include "GLWindow2.h"
#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
class Map;

class MapViewer
{

public:
ros::NodeHandle nh;
ros::Publisher trajectory;
nav_msgs::Path path;
  MapViewer(Map &map, GLWindow2 &glw);
  void DrawMap(SE3<> se3CamFromWorld, CVD::Image<CVD::byte> &imFrame);
  std::string GetMessageForUser();
	float mScale;
	double a,b,c,d,e,f;  
protected:
  Map &mMap;
  GLWindow2 &mGLWindow;
  
  void DrawGrid();
  void DrawMapDots();
  //void DrawDense(int fnum);//E
  //void DrawCamera(SE3<> se3, bool bSmall=false);
  void DrawCamera(SE3<> se3CfromW);//E Abhijit
  void SetupFrustum();
  void SetupModelView(SE3<> se3WorldFromCurrent = SE3<>());
	void DrawCameraTrajectory(SE3<> se3CamFromWorld);
  void DrawCameraTrajectory();

  void LoadTexture(CVD::Image<CVD::byte> &imFrame);
  
  Vector<3> mv3MassCenter;
  SE3<> mse3ViewerFromWorld;
  GLuint mTextureId;

  std::ostringstream mMessageForUser;
};

#endif
