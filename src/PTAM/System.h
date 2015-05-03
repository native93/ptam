// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "VideoSource.h"
#include "GLWindow2.h"
#include<fstream>
#include<iostream>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include<ros/ros.h>
#include "Map.h"
#include "MapPoint.h"
#include "JL1.h"
class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class ARDriver;
class MapViewer;
using namespace std;
class System
{
    public:
	System(ros::NodeHandle &nh);
	void Run();
	void FinishHim();

    private:

ofstream myfile;
int count_file;
bool save_points;

JL *mJL;

VideoSource mVideoSource;
	GLWindow2 mGLWindow;
	CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;
	CVD::Image<CVD::byte> mimFrameBW;
//int *altitude_ptr;
//int altitude;
float *pos_x_ptr;
float pos_x;
float *pos_y_ptr;
float pos_y;
bool first_kf;
bool* first_kf_ptr;
bool second_kf;
bool* second_kf_ptr;


	Map *mpMap; 
	MapMaker *mpMapMaker; 
	Tracker *mpTracker; 
	ATANCamera *mpCamera;
	ARDriver *mpARDriver;
	MapViewer *mpMapViewer;

	bool mbDone;
	bool mbDrawMap;

	static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};



#endif