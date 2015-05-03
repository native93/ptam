// Copyright 2008 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"
#include "Map.h"
#include "MapPoint.h"
#include "TrackerData.h"

using namespace CVD;
using namespace std;
using namespace GVars3;


System::System(ros::NodeHandle &nh)
  :mVideoSource(nh), mGLWindow(mVideoSource.Size(), "PTAM")
{
    GUI.RegisterCommand("exit", GUICommandCallBack, this);
    GUI.RegisterCommand("quit", GUICommandCallBack, this);
    GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);

    mimFrameBW.resize(mVideoSource.Size());
    mimFrameRGB.resize(mVideoSource.Size());
    // First, check if the camera is calibrated.
    // If not, we need to run the calibration widget.
    Vector<NUMTRACKERCAMPARAMETERS> vTest;

    vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
    mpCamera = new ATANCamera("Camera");
    Vector<2> v2;
    if(v2==v2) {}
    if(vTest == ATANCamera::mvDefaultParams)
    {
	cout << endl;
	cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
	cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
	exit(1);
    }
altitude_ptr=&altitude;
    mpMap = new Map;
    mpMapMaker = new MapMaker(*mpMap, *mpCamera);
    mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, *mpMap, *mpMapMaker,nh);
    mpARDriver = new ARDriver(*mpCamera, mVideoSource.Size(), mGLWindow);
    mpMapViewer = new MapViewer(*mpMap, mGLWindow);

    GUI.ParseLine("GLWindow.AddMenu Menu Menu");
    GUI.ParseLine("Menu.ShowMenu Root");
    GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
    GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
    GUI.ParseLine("DrawAR=0");
    GUI.ParseLine("DrawMap=0");
    GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
    GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

    mbDone = false;
    mbDrawMap=false;
};

void System::Run()
{
    while(!mbDone)
    {

	// We use two versions of each video frame:
	// One black and white (for processing by the tracker etc)
	// and one RGB, for drawing.

	// Grab new video frame...
	if(!mVideoSource.GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB,altitude_ptr))
	    break;
	//fprintf(stderr, "\ric=%d, oc=%d", mVideoSource.ic, mVideoSource.oc);
	//fflush(stdout);
	//continue;
//fprintf(stderr,"stamp%lf\n",time_sec);
	static bool bFirstFrame = true;
	if(bFirstFrame)
	{
	    mpARDriver->Init();
	    bFirstFrame = false;
	}

	mGLWindow.SetupViewport();
	mGLWindow.SetupVideoOrtho();
	mGLWindow.SetupVideoRasterPosAndZoom();

	if(!mpMap->IsGood())
	    mpARDriver->Reset();

	static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
	static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN|SILENT);

	bool bDrawMap = mpMap->IsGood() && mbDrawMap;
	bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;

	mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap,altitude);

mpMapMaker->PublishMap(mpTracker->GetCurrentPose());
	if(bDrawMap)
{
	    mpMapViewer->DrawMap(mpTracker->GetCurrentPose(), mimFrameBW);
}
	else if(bDrawAR)
	    mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose());

	//      mGLWindow.GetMousePoseUpdate();
	string sCaption;
	if(bDrawMap)
	    sCaption = mpMapViewer->GetMessageForUser();
	else
	    sCaption = mpTracker->GetMessageForUser();
	mGLWindow.DrawCaption(sCaption);
	mGLWindow.DrawMenus();
	mGLWindow.swap_buffers();
	mGLWindow.HandlePendingEvents();
    }
    printf("Done!!\n");
}
#if 0
void System::FinishHim()
{
    printf("avg hit = %lf\n", mpTracker->avgFeatHitRatio);

    //------------------------------
    int count=0;
    for(size_t i=0; i<mpMap->vpPoints.size(); i++)
	count += mpMap->vpPoints[i]->foundFrameCount;
    printf("avg seen = %lf\n", count/(double)mpMap->vpPoints.size());

    //-----------------------------
    double dist = 0.0;
    count=0;
    for(size_t i=0;i<mpMap->vpPoints.size(); i++)
    {
	mpMap->vpPoints[i]->pTData->Project(mpMap->vpKeyFrames[0]->se3CfromW, *mpCamera);
	Vector<2> v2LastPos = mpMap->vpPoints[i]->pTData->v2Image;
	for(size_t j=1; j<mpMap->vpKeyFrames.size(); j++)
	{
	    mpMap->vpPoints[i]->pTData->Project(mpMap->vpKeyFrames[j]->se3CfromW, *mpCamera);
	    Vector<2> v2CurrentPos = mpMap->vpPoints[i]->pTData->v2Image; 
	    if(!(mpMap->vpPoints[i]->pTData->bInImage))
		break;
	    dist += sqrt(v2LastPos*v2CurrentPos);
	    v2LastPos = v2CurrentPos;
	    count++;
	}
    }
    printf("avg moved = %lf, count=%d\n", (dist/(double)count), count);
}
#endif

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
    if(sCommand=="quit" || sCommand == "exit")
	static_cast<System*>(ptr)->mbDone = true;

    if(sCommand=="KeyPress")
    {
	if(sParams == "t")
	{
	    printf("\nFinishHim(): \n");
	    //static_cast<System*>(ptr)->FinishHim();
	}
	if(sParams == "m")
	{
	    if(	static_cast<System*>(ptr)->mbDrawMap)
		static_cast<System*>(ptr)->mbDrawMap =false;
	    else
	    {
#ifndef RM
		if( static_cast<System*>(ptr)->mpMap->IsGood() )
#endif
		    static_cast<System*>(ptr)->mbDrawMap =true;
	    }
	}
    }
}








