// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"
#include "TrackerData.h"
//#include <cv.h>
//#include <cvaux.h>
//#include <highgui.h>
#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <cvd/image_io.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <TooN/wls.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <fstream>
#include <fcntl.h>

int cnt=0;

using namespace CVD;
using namespace std;
using namespace GVars3;

// The constructor mostly sets up interal reference variables
// to the other classes..
Tracker::Tracker(ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm,ros::NodeHandle &nh_): //, char *dsPath) : 
  mMap(m),
  mMapMaker(mm),
  mCamera(c),
nh(nh_),
  mRelocaliser(mMap, mCamera),
  mirSize(irVideoSize)
{
  //cDatasetPath(dsPath) //put a "," on the above line if uncommenting this 
//ros::NodeHandle nh_private(~);
//altitude_sub=nh.subscribe("/ardrone/navdata_altitude",10,&Tracker::altitude_callback,this);
  mCurrentKF.bFixed = false;
  GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);
  TrackerData::irImageSize = mirSize;

  mpSBILastFrame = NULL;
  mpSBIThisFrame = NULL;
	
	mnFrameAbs = 0;	//This is not initialized in Reset()

  // Most of the initialisation is done in Reset()
  Reset();
  //E
  //Code to deal with Dense.
  //H1n = cv::Mat::eye(3,3, CV_64FC1);
  //cv::namedWindow( "image", CV_WINDOW_AUTOSIZE );
  //cvMoveWindow("image", 640, 100);
}

// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset()
{


  mMap.save(mse3CamFromWorld);
  mbDidCoarse = false;
  mbUserPressedSpacebar = false;
  mTrackingQuality = GOOD;
  mnLostFrames = 0;
  mdMSDScaledVelocityMagnitude = 0;
  mCurrentKF.dSceneDepthMean = 1.0;
  mCurrentKF.dSceneDepthSigma = 1.0;
  mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mCamera.SetImageSize(mirSize);
  mCurrentKF.mMeasurements.clear();
  //mnLastKeyFrameDropped = -20;
  mnLastKeyFrameDropped = -10;
  mnFrame=0;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = false;
  mdMapInitScale = 1;
  
  // Tell the MapMaker to reset itself.. 
  // this may take some time, since the mapmaker thread may have to wait
  // for an abort-check during calculation, so sleep while waiting.
  // MapMaker will also clear the map.
  mMapMaker.RequestReset();
  while(!mMapMaker.ResetDone())
#ifndef WIN32
	  usleep(10);
#else
	  Sleep(1);
#endif
}
/*
void Tracker::altitude_callback(const ardrone_autonomy::navdata_altitude::ConstPtr &msg)
{

alti_raw=*msg;
//:fprintf(stderr,"altitude:%d\n",altitude);
}
*/
// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void Tracker::TrackFrame(Image<Rgb <byte> > &imFrameRGB, Image<byte> &imFrame, bool bDraw,float p_x,float p_y,bool fkf_select,bool skf_select)
{

//ros::spinOnce();
//altitude=alti;
fkf_sel=fkf_select;
skf_sel=skf_select;
pos_x=p_x;
pos_y=p_y;
  //usleep(200000);
  mbDraw = bDraw;
  mMessageForUser.str("");   // Wipe the user message clean
  
  // Take the input video image, and convert it into the tracker's keyframe struct
  // This does things like generate the image pyramid and find FAST corners
  mCurrentKF.mMeasurements.clear();
  mCurrentKF.MakeKeyFrame_Lite(imFrameRGB, imFrame);

  // Update the small images for the rotation estimator
  static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
  static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
  mbUseSBIInit = *gvnUseSBI;
  if(!mpSBIThisFrame)
    {
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
      mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
  else
    {
      delete  mpSBILastFrame;
      mpSBILastFrame = mpSBIThisFrame;
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
  
  mnFrame++;
  mnFrameAbs++;
  mCurrentKF.nFrameIdAbs = mnFrameAbs;	
  //  cout<<"\r"<<"f="<<mCurrentKF.nFrameIdAbs<<flush;
	
  if(mbDraw)
  {
    glDrawPixels(mCurrentKF.aLevels[0].im);
    //if(GV2.GetInt("Tracker.DrawFASTCorners",1, SILENT))
		{
			glColor3f(1,0,1);  glPointSize(1); glBegin(GL_POINTS);
			for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCorners.size(); i++) 
				glVertex(mCurrentKF.aLevels[0].vCorners[i]);
			glEnd();
		}
  }
  
  // Decide what to do - if there is a map, try to track the map ...
  if(mMap.IsGood())
  {
    if(mnLostFrames < 3)  // .. but only if we're not lost!
		{
			if(mbUseSBIInit)
				CalcSBIRotation();
			ApplyMotionModel();       // 
			TrackMap(imFrame);               //  These three lines do the main tracking work.
			UpdateMotionModel();      // 
	  
			AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.
	  
			{ // Provide some feedback for the user:
				mMessageForUser << "Tracking Map, quality ";
				if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
				if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
				if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
				mMessageForUser << " Found:";
				for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
				//	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
				mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF";
				//cout<<"Point size = "<<mMap.vpPoints.size()<<endl;
			}
	  
			// Heuristics to check if a key-frame should be added to the map:
			
			if(mTrackingQuality == GOOD &&
				 mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
				 mnFrame - mnLastKeyFrameDropped > 20 &&
				 mMapMaker.QueueSize() < 3)
				{
					usleep(1000);
					AddNewKeyFrame();
				};
		}
    else  // what if there is a map, but tracking has been lost?
		{
			mMessageForUser << "** Attempting recovery **.";
			if(AttemptRecovery())
			{
	      TrackMap(imFrame);
	      AssessTrackingQuality();
	    }
		}
    if(mbDraw)
			RenderGrid();
  } 
  else // If there is no map, try to make one.
  {
    TrackForInitialMap();
    //LoadInitialMapFromLog();
  }
	
  // GUI interface
  while(!mvQueuedCommands.empty())
	{
		GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
		mvQueuedCommands.erase(mvQueuedCommands.begin());
	}
  
  mPreviousFrameKF = mCurrentKF;	//Set the Previous Frame
};

// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit 
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery()
{
  bool bRelocGood = mRelocaliser.AttemptRecovery(mCurrentKF);
  if(!bRelocGood)
    return false;
  
  SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;
  return true;
}

// Draw the reference grid to give the user an idea of wether tracking is OK or not.
void Tracker::RenderGrid()
{
  // The colour of the ref grid shows if the coarse stage of tracking was used
  // (it's turned off when the camera is sitting still to reduce jitter.)
  if(mbDidCoarse)
    glColor4f(.0, 0.5, .0, 0.6);
  else
    glColor4f(0,0,0,0.6);
  
  // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
  int nHalfCells = 8;
  int nTot = nHalfCells * 2 + 1;
  Image<Vector<2> >  imVertices(ImageRef(nTot,nTot));
  for(int i=0; i<nTot; i++)
    for(int j=0; j<nTot; j++)
      {
	Vector<3> v3;
	v3[0] = (i - nHalfCells) * 0.1;
	v3[1] = (j - nHalfCells) * 0.1;
	v3[2] = 0.0;
	Vector<3> v3Cam = mse3CamFromWorld * v3;
	if(v3Cam[2] < 0.001)
	  v3Cam[2] = 0.001;
	imVertices[i][j] = mCamera.Project(project(v3Cam));
      }
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++)
    {
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(imVertices[i][j]);
      glEnd();
      
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(imVertices[j][i]);
      glEnd();
    };
  
  glLineWidth(1);
  glColor3f(1,0,0);
}

// GUI interface. Stuff commands onto the back of a queue so the tracker handles
// them in its own thread at the end of each frame. Note the charming lack of
// any thread safety (no lock on mvQueuedCommands).
void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}

// This is called in the tracker's own thread.
void Tracker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="Reset")
    {
      Reset();
      return;
    }

  // KeyPress commands are issued by GLWindow
  if(sCommand=="KeyPress")
	{
		if(sParams == "Space")
		{
			mbUserPressedSpacebar = true;
		}
		else if(sParams == "r")
		{
			Reset();
		}
		else if(sParams == "q" || sParams == "Escape")
		{
			GUI.ParseLine("quit");
		}
		else if(sParams == "n")
		{
			AddNewKeyFrame();
		}
		//else if(sParams == "g")
		//{
		//    LoadDensePoints();
		//}
		//else if(sParams == "f")
		//{
		//    SaveDensePoints();
		//}
		return;
	}
	if((sCommand=="PokeTracker"))
	{
		mbUserPressedSpacebar = true;
		return;
	}
	
  
  cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}; 

// Routine for establishing the initial map. This requires two spacebar presses from the user
// to define the first two key-frames. Salient points are tracked between the two keyframes
// using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
// break it.) The salient points are stored in a list of `Trail' data structures.
// What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
void Tracker::TrackForInitialMap()
{
	
	// MiniPatch tracking threshhold.
  static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
  MiniPatch::mnMaxSSD = *gvnMaxSSD;
   // What stage of initial tracking are we at?
  if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED) 
  {
//if(fkf_sel)
if(mbUserPressedSpacebar)
///orig  // First spacebar = this is the 1st kf.
// if(mnFrameAbs==33)//lab_video_16
   // if(mnFrameAbs==33)//lab_video_16_better
    //if(mnFrameAbs==563)//lab_video_16_later
    //if(mnFrameAbs==395)//outdoor
    //if(mnFrameAbs==31)//slope_26_images1
    //if(mnFrameAbs==35)//slope_26_images2  
    //if(mnFrameAbs==46)//slope_29
    //if(mnFrameAbs==159)//slope_29_late
    //if(mnFrameAbs==9)//plane1_images2_early
    //if(mnFrameAbs==184)//plane1_images2_late
    //if(mnFrameAbs==138)//plane1_images2
    //if(mnFrameAbs==112)//plane25
    //if(mnFrameAbs==198)//corridor
    //if(mnFrameAbs==47)//plane31
    //if(mnFrameAbs==113)//plane31
    //if(mnFrameAbs==363)//plane31_box
    //if(mnFrameAbs==748)//plane31_7
    //if(mnFrameAbs==22)//side_9 exploration_36
    //if(mnFrameAbs==16)//low_02, low_11
    //if(mnFrameAbs==305)//low_02
    //if(mnFrameAbs==9)//low_592
    //if(mnFrameAbs==191)//MSC_s1c
    //if(mnFrameAbs==265)//MSC_s1c
    //if(mnFrameAbs==235)//MSC_s1u
    //if(mnFrameAbs==731)//MSC_s2c
    //if(mnFrameAbs==692)//MSC_s2c
    //if(mnFrameAbs==295)//MSC_s2u


		{
//ros::spinOnce();
//altitude_fkf=altitude;
pos_x_fkf=pos_x;
pos_y_fkf=pos_y;

fprintf(stderr,"pos_x_1:,pos_y_1:%f,%f\n",pos_x_fkf,pos_y_fkf);
//fprintf(stderr,"altitude1:%d\nframe:%lf,ultrasound:%lf,dif:%lf",altitude_fkf,frame_stamp,alti_raw.header.stamp.toSec(),frame_stamp-alti_raw.header.stamp.toSec());

		mbUserPressedSpacebar = false;
			TrailTracking_Start();
			mnInitialStage = TRAIL_TRACKING_STARTED;
			cout<<"first frame id = "<<mnFrameAbs<<endl;
		}
    else
			mMessageForUser << "TrackForInitialMap(): mnFrameAbs = " << mnFrameAbs << "  Hit 1st Spacebar.." <<endl;
    return;
  };
 

  if(mnInitialStage == TRAIL_TRACKING_STARTED)
  {
    int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
    if(nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
		{
			Reset();
			return;
		}
      
     //If the user pressed spacebar here, use trails to run stereo and make the intial map..
if(mbUserPressedSpacebar)

//if(skf_sel)
 //orig://   
//if(mnFrameAbs==41)//lab_video_16
    //if(mnFrameAbs==54)//lab_video_16_better
    //if(mnFrameAbs==568)//lab_video_16_later
    //if(mnFrameAbs==400)//outdoor  
    //if(mnFrameAbs==34)//slope_26_image1
    //if(mnFrameAbs==39)//slope_26_image2
    //if(mnFrameAbs==52)//slope_29
    //if(mnFrameAbs==162)//slope_29_late
    //if(mnFrameAbs==16)//plane1_images2_early
    //if(mnFrameAbs==188)//plane1_images2_late
    //if(mnFrameAbs==141)//plane1_images2
    //if(mnFrameAbs==115)//plane25
    //if(mnFrameAbs==204)//corridor
    //if(mnFrameAbs==61)//plane31
    //if(mnFrameAbs==121)//plane31
    //if(mnFrameAbs==388)//plane31_box
    //if(mnFrameAbs==450)//plane31_better_box
    //if(mnFrameAbs==768)//plane31_7
    //if(mnFrameAbs==43)//side_9
    //if(mnFrameAbs==30)//exploration_36
    //if(mnFrameAbs==26)//low_02
    //if(mnFrameAbs==310)//low_02
    //if(mnFrameAbs==28)//low_11
    //if(mnFrameAbs==22)//low_592
    //if(mnFrameAbs==251)//MSC_s1c
    //if(mnFrameAbs==306)//MSC_s1c
    //if(mnFrameAbs==280)//MSC_s1u
    //if(mnFrameAbs==779)//MSC_s2c
    //if(mnFrameAbs==733)//MSC_s2c
    //if(mnFrameAbs==322)//MSC_s2u
		{
//ros::spinOnce();
//altitude_skf=altitude;
pos_x_skf=pos_x;
pos_y_skf=pos_y;

fprintf(stderr,"pos_x_1:,pos_y_1:%f,%f\n",pos_x_skf,pos_y_skf);


//fprintf(stderr,"altitude1:%d\n",altitude_skf);
//fprintf(stderr,"altitude1:%d\nframe:%lf,ultrasound:%lf,dif:%lf",altitude_skf,frame_stamp,alti_raw.header.stamp.toSec(),frame_stamp-alti_raw.header.stamp.toSec());
			cout<<"second frame id = "<<mnFrameAbs<<endl;
			mbUserPressedSpacebar = false;
			vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
			for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
				vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos, i->irCurrentPos));

      mMapMaker.InitFromStereo_altitude(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld,sqrt((pos_x_fkf-pos_x_skf)*(pos_x_fkf-pos_x_skf)+(pos_y_fkf-pos_y_skf)*(pos_y_fkf-pos_y_skf)));
//			mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);  // This will take some time!
			mnInitialStage = TRAIL_TRACKING_COMPLETE;
			//sw.start();
		}
    else
			mMessageForUser << "TrackForInitialMap(): mnFrameAb=" << mnFrameAbs << "  Trails=" << nGoodTrails<<endl;
  }	
	
}

void Tracker::LoadInitialMapFromLog()
{

  // MiniPatch tracking threshhold.
  static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
  MiniPatch::mnMaxSSD = *gvnMaxSSD;
  static int nFrameSecond;

  // What stage of initial tracking are we at?
  if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED) 
  {
    //   if(mbUserPressedSpacebar)  // First spacebar = this is the first keyframe
    //		{
    //		mbUserPressedSpacebar = false;
    TrailTracking_Start();
    mnInitialStage = TRAIL_TRACKING_STARTED;
    string path;
    cin>>path;
    ifstream poseFile(path.c_str());
    //ifstream poseFile("/home/laxit/projects/vslam/codes/5point/lab2_200_SURF/0_4_CamPose.txt");
    if(!poseFile.is_open())
    {
      cout<<"MapPoints file not opened"<<endl;
      return;
    }
    //CamPose cp;
    //poseFile >> cp;
    //mse3CamFromWorld = cp.se3CfromW;
    //nFrameSecond = cp.nFrameIdAbs;
    poseFile>>nFrameSecond;//when not reading the camera pose, just the frame number.
    cout<<"second frame for initialization = "<<nFrameSecond<<endl;
    //cout<<cp;
    poseFile.close();
    //		}
    //  else
    //			mMessageForUser << "TrackForInitialMap(): mnFrameAbs = " << mnFrameAbs << "  Hit 1st Spacebar.." <<endl;
    return;
  }


  if(mnInitialStage == TRAIL_TRACKING_STARTED)
  {
    int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
    if(nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
    {
      Reset();
      return;
    }

    //If the user pressed spacebar here, use trails to run stereo and make the intial map..
    //if(mbUserPressedSpacebar) //original PTAM
    //if(mnFrameAbs==50)//New College
    //if(mnFrameAbs==4) //Versailles
    //if(mnFrameAbs==19) //lab
    if(mnFrameAbs==nFrameSecond) //should be general
    {
      mbUserPressedSpacebar = false;
      vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
      //By commenting the below two lines, you make the tracking till now obsolete. But not to worry,
      //the original is intact in the function InitFromStereo, which let's face it is the origianl function.
      //for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
      //vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos, i->irCurrentPos));
      //Now since the above two are obsolete we shall read it from the loged file.
      string path;
      cin >> path;
      cout<<"path="<<path<<endl;
      ifstream pointsFile(path.c_str());
      //ifstream pointsFile("/home/laxit/projects/vslam/codes/5point/NewCollege_MapPoints.txt");
      //ifstream pointsFile("torr_MapPoints.txt");
      //ifstream pointsFile("/media/d_drive/Versailles_Rond_Original/Gauche/MapPoints.txt");
      //ifstream pointsFile("/media/d_drive/lab_images/MapPoints.txt");
      if(!pointsFile.is_open())
      {
	cout<<"MapPoints file not opened"<<endl;
	return;
      }
      ImageRef irFirst, irSecond;
      int count=0;
      while(!pointsFile.eof())
      {
	pointsFile>>irFirst; pointsFile>>irSecond;
	//cout<<irFirst.x<<", "<<irFirst.y;//<<endl;
	//cout<<", "<<irSecond.x<<", "<<irSecond.y;//<<endl;
	vMatches.push_back(pair<ImageRef, ImageRef>(irFirst, irSecond));
	count++;
      }
      vMatches.pop_back();
      cout<<"coming out from diging out point's"<<endl;
      cout<<"no. of matches = "<<vMatches.size()<<endl;
      mMapMaker.InitFromStereo_altitude(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld,altitude_skf-altitude_fkf);  // This will take some time!
      //mMapMaker.InitFromLog(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);
      cout<<mse3CamFromWorld;

      mnInitialStage = TRAIL_TRACKING_COMPLETE;
    }
    else
      mMessageForUser << "TrackForInitialMap(): mnFrameAb=" << mnFrameAbs << "  Trails=" << nGoodTrails<<endl;
  }	

}

// The current frame is to be the first keyframe!
void Tracker::TrailTracking_Start()
{
  mCurrentKF.MakeKeyFrame_Rest();  // This populates the Candidates list, which is Shi-Tomasi thresholded.
  mFirstKF = mCurrentKF; 
  vector<pair<double,ImageRef> > vCornersAndSTScores;
  for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCandidates.size(); i++)  // Copy candidates into a trivially sortable vector
    {                                                                     // so that we can choose the image corners with max ST score
      Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
      if(!mCurrentKF.aLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize))
				continue;
      vCornersAndSTScores.push_back(pair<double,ImageRef>(-1.0 * c.dSTScore, c.irLevelPos)); // negative so highest score first in sorted list
    };
  sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  // Sort according to Shi-Tomasi score
  //int nToAdd = GV2.GetInt("MaxInitialTrails", 1000, SILENT);
  int nToAdd = GV2.GetInt("MaxInitialTrails", 2000, SILENT);
  cout<<"nToAdd = "<<nToAdd;
  cout<<", vCor.size = "<<vCornersAndSTScores.size()<<endl;
  for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++)
    {
      if(!mCurrentKF.aLevels[0].im.in_image_with_border(vCornersAndSTScores[i].second, MiniPatch::mnHalfPatchSize))
	continue;
      Trail t;
      t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aLevels[0].im);
      t.irInitialPos = vCornersAndSTScores[i].second;
      t.irCurrentPos = t.irInitialPos;
      mlTrails.push_back(t);
      nToAdd--;
    }
}

int Tracker::TrailTracking_Advance()
{
  int nGoodTrails = 0;
  if(mbDraw)
	{
		glPointSize(5);
		glLineWidth(2);
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_LINE_SMOOTH);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
		glBegin(GL_LINES);
	}
  
  MiniPatch BackwardsPatch;
  Level &lCurrentFrame = mCurrentKF.aLevels[0];
  Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];
 // cout<<"mT.s="<<mlTrails.size()<<", ";
  
  for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
  {
	  list<Trail>::iterator next = i; next++;

		Trail &trail = *i;
		ImageRef irStart = trail.irCurrentPos;
		ImageRef irEnd = irStart;
		//bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
		bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 20, lCurrentFrame.vCorners);
		if(bFound)
		{
			// Also find backwards in a married-matches check
			BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
			ImageRef irBackWardsFound = irEnd;
			//bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
			bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 20, lPreviousFrame.vCorners);
			//if((irBackWardsFound - irStart).mag_squared() > 2)
			if((irBackWardsFound - irStart).mag_squared() > 20)
				bFound = false;
			
			trail.irCurrentPos = irEnd;
			nGoodTrails++;
		}
    
		if(mbDraw)
		{
			if(!bFound)
				glColor3f(0,1,1); // Failed trails flash purple before dying.
			else
				glColor3f(1,1,0);
			glVertex(trail.irInitialPos);
			if(bFound) glColor3f(1,0,0);
			glVertex(trail.irCurrentPos);
		}
    
		if(!bFound) // Erase from list of trails if not found this frame.
		{
	  mlTrails.erase(i);
		}
		i = next;
  }
 // cout<<"mT.s="<<mlTrails.size()<<endl;
  if(mbDraw)
    glEnd();

  //mPreviousFrameKF = mCurrentKF;
  return nGoodTrails;
}

// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the image;
// Then it updates camera pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
void Tracker::TrackMap(Image<byte> &imFrame)
{ 
    //cout<<", kf="<<mMapMaker.LatestKeyFrameId()<<flush;
    // Some accounting which will be used for tracking quality assessment:
    for(int i=0; i<LEVELS; i++)
	manMeasAttempted[i] = manMeasFound[i] = 0;

    // The Potentially-Visible-Set (PVS) is split into pyramid levels.
    vector<TrackerData*> avPVS[LEVELS]; 
    for(int i=0; i<LEVELS; i++)
	avPVS[i].reserve(500);

    // For all points in the map..
    for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
	MapPoint &p= *(mMap.vpPoints[i]); 
	// Ensure that this map point has an associated TrackerData struct.
	if(!p.pTData) p.pTData = new TrackerData(&p);   
	TrackerData &TData = *p.pTData;

	// Project according to current view, and if it's not in the image, skip.
	TData.Project(mse3CamFromWorld, mCamera);//equation 2, function in TrackerData.cc 
	if(!TData.bInImage)
	    continue;
#if 0
	if(mCurrentKF.nFrameIdAbs==202)
	{
	    //if( (abs(TData.v2Image[0]-467)<5 && abs(TData.v2Image[1]-253)<5) || (abs(TData.v2Image[0]-435)<5 && abs(TData.v2Image[1]-125)<5) /*|| (abs(TData.v2Image[0]-255)<5 && abs(TData.v2Image[1]-255)<5)*/ )
	    if( (abs(TData.v2Image[0]-420)<5 && abs(TData.v2Image[1]-150)<5) )
	    {
		cout<<"Trueing "<<i<<endl;
		mMap.interestPoint[i]=true;
	    }
	}
#endif

	// Calculate camera projection derivatives of this point.
	TData.GetDerivsUnsafe(mCamera);//equation 7, matrix for warping

	// And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
	TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);///at what level should point be searched.

	if(TData.nSearchLevel == -1)
	    continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

	// Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
	TData.bSearched = false;
	TData.bFound = false;
	avPVS[TData.nSearchLevel].push_back(&TData);
    };

    // Next: A large degree of faffing about and deciding which points are going to be measured!
    // First, randomly shuffle the individual levels of the PVS.
    for(int i=0; i<LEVELS; i++)
	random_shuffle(avPVS[i].begin(), avPVS[i].end());

    // The next two data structs contain the list of points which will next 
    // be searched for in the image, and then used in pose update.
    vector<TrackerData*> vNextToSearch;
    vector<TrackerData*> vIterationSet;

    // Tunable parameters to do with the coarse tracking stage:
    static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
    static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
    static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
    static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
    static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
    static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT);  // Speed above which coarse stage is used.

    unsigned int nCoarseMax = *gvnCoarseMax;
    unsigned int nCoarseRange = *gvnCoarseRange;

    mbDidCoarse = false;

    // Set of heuristics to check if we should do a coarse tracking stage.
    bool bTryCoarse = true;
    if(*gvnCoarseDisabled || 
	    mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
	    nCoarseMax == 0)
	bTryCoarse = false;
    if(mbJustRecoveredSoUseCoarse)
    {
	bTryCoarse = true;
	nCoarseMax *=2;
	nCoarseRange *=2;
	mbJustRecoveredSoUseCoarse = false;
    };

    // If we do want to do a coarse stage, also check that there's enough high-level 
    // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
    // with preference to LEVELS-1.
    if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > *gvnCoarseMin )
    {
	// Now, fill the vNextToSearch struct with an appropriate number of 
	// TrackerDatas corresponding to coarse map points! This depends on how many
	// there are in different pyramid levels compared to CoarseMin and CoarseMax.

	if(avPVS[LEVELS-1].size() <= nCoarseMax) 
	{ // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
	    vNextToSearch = avPVS[LEVELS-1];
	    avPVS[LEVELS-1].clear();
	}
	else
	{ // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
	    for(unsigned int i=0; i<nCoarseMax; i++)
		vNextToSearch.push_back(avPVS[LEVELS-1][i]);
	    avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
	}

	// If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
	if(vNextToSearch.size() < nCoarseMax)
	{
	    unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
	    if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded)
	    {
		vNextToSearch = avPVS[LEVELS-2];
		avPVS[LEVELS-2].clear();
	    }
	    else
	    {
		for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
		    vNextToSearch.push_back(avPVS[LEVELS-2][i]);
		avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
	    }
	}
	// Now go and attempt to find these points in the image!
	unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
	vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
	if(nFound >= *gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
	{
	    mbDidCoarse = true;
	    for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
	    {
		if(iter != 0)
		{ // Re-project the points on all but the first iteration.
		    for(unsigned int i=0; i<vIterationSet.size(); i++)
			if(vIterationSet[i]->bFound)  
			    vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
		}
		for(unsigned int i=0; i<vIterationSet.size(); i++)
		    if(vIterationSet[i]->bFound)
			vIterationSet[i]->CalcJacobian();
		double dOverrideSigma = 0.0;
		if(iter > 5)
		    dOverrideSigma = 1.0;

		// Calculate and apply the pose update...
		Vector<6> v6Update = 
		    CalcPoseUpdate(vIterationSet, dOverrideSigma);
		mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
	    };
	}
    };


    int nFineRange = 10;  // Pixel search range for the fine stage. 
    if(mbDidCoarse)       // Can use a tighter search if the coarse stage was already done.
	nFineRange = 5;

    // What patches shall we use this time? The high-level ones are quite important,
    // so do all of these, with sub-pixel refinement.
    {
	int l = LEVELS - 1;
	for(unsigned int i=0; i<avPVS[l].size(); i++)
	    avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
	SearchForPoints(avPVS[l], nFineRange, 8);
	for(unsigned int i=0; i<avPVS[l].size(); i++)
	    vIterationSet.push_back(avPVS[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
    };

    // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
    vNextToSearch.clear();
    for(int l=LEVELS - 2; l>=0; l--)
	for(unsigned int i=0; i<avPVS[l].size(); i++)
	    vNextToSearch.push_back(avPVS[l][i]);

    //static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
    static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 100000, SILENT); //E
    int nFinePatchesToUse = *gvnMaxPatchesPerFrame - vIterationSet.size();
    if(nFinePatchesToUse < 0)
	nFinePatchesToUse = 0;
    if((int) vNextToSearch.size() > nFinePatchesToUse)
    {
	random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
	vNextToSearch.resize(nFinePatchesToUse); // Chop!
    };

    // If we did a coarse tracking stage: re-project and find derivs of fine points
    if(mbDidCoarse)
	for(unsigned int i=0; i<vNextToSearch.size(); i++)
	    vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);

    // Find fine points in image:
    SearchForPoints(vNextToSearch, nFineRange, 0);
    // And attach them all to the end of the optimisation-set.
    for(unsigned int i=0; i<vNextToSearch.size(); i++)
	vIterationSet.push_back(vNextToSearch[i]);

    // Again, ten gauss-newton pose update iterations.
    Vector<6> v6LastUpdate;
    v6LastUpdate = Zeros;
    for(int iter = 0; iter<10; iter++)
    {
	bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
	// reprojection at every iteration - it really isn't necessary!
	if(iter == 0 || iter == 4 || iter == 9)
	    bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
	else                            // iterations is for M-Estimator convergence rather than 
	    bNonLinearIteration = false;  // linearisation effects.

	if(iter != 0)   // Either way: first iteration doesn't need projection update.
	{
	    if(bNonLinearIteration)
	    {
		for(unsigned int i=0; i<vIterationSet.size(); i++)
		    if(vIterationSet[i]->bFound)
			vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
	    }
	    else
	    {
		for(unsigned int i=0; i<vIterationSet.size(); i++)
		    if(vIterationSet[i]->bFound)
			vIterationSet[i]->LinearUpdate(v6LastUpdate);
	    };
	}

	if(bNonLinearIteration)
	    for(unsigned int i=0; i<vIterationSet.size(); i++)
		if(vIterationSet[i]->bFound)
		    vIterationSet[i]->CalcJacobian();

	// Again, an M-Estimator hack beyond the fifth iteration.
	double dOverrideSigma = 0.0;
	if(iter > 5)
	    dOverrideSigma = 16.0;

	// Calculate and update pose; also store update vector for linear iteration updates.
	Vector<6> v6Update = 
	    CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
	mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
	v6LastUpdate = v6Update;
    };

    if(mbDraw)
    {
	glPointSize(6);
	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_POINTS);
	int foundCount=0;
	for(vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
		it!= vIterationSet.rend(); 
		it++)
	{
	    if(! (*it)->bFound)
		continue;

	    //MSC'13 result stats operations
	    //foundCount++;
	    //(*it)->Point.foundFrameCount++;

	    if((*it)->Point.nSceneLabel==1)
		glColor(gavLevelColors[0]);
	    else
		glColor(gavLevelColors[2]);

	    //glColor(gavLevelColors[(*it)->nSearchLevel]);
#if 0
	    //if(mCurrentKF.nFrameIdAbs==79 || mCurrentKF.nFrameIdAbs==73)
	    if(mCurrentKF.nFrameIdAbs>90)
	    {
		if(abs((*it)->v2Image[0]-255)<5 && abs((*it)->v2Image[1]-255)<5)
		{
		    glColor3f(1,0,1);
		    cout<<"pinkPt = "<<(*it)->v2Image<<", "<<(*it)->Point.v3WorldPos<<endl;
		}
	    }
#endif
	    glVertex((*it)->v2Image);
	}
	//double frameAvg = vIterationSet.size()==0?0:foundCount/(double)vIterationSet.size();
	//int nfafc = mCurrentKF.nFrameIdAbs - mMap.vpKeyFrames[1]->nFrameIdAbs;//number of frames for avg feature count. Subtract with keyframes[1] a we want the average second kf onwards(second space pressed)
	//avgFeatHitRatio = ((avgFeatHitRatio*(nfafc-1))+frameAvg)/((double)nfafc);// (<prev avg>*<prev num of frames> + <current count>)/<num of frames>
	//printf("avg = %lf\n", avgFeatHitRatio);
	//Vector<2> th = makeVector(420, 150);//(330, 210);
	//glColor(gavLevelColors[3]);
	//glVertex(th);
	glEnd();
	glDisable(GL_BLEND);
    }

#if 0
    cv::Mat img = cv::Mat(mirSize.y, mirSize.x, CV_8UC1, imFrame.data()); 
    cv::cvtColor(img, img, CV_GRAY2BGR);
	for(vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
		it!= vIterationSet.rend(); 
		it++)
	{
	    if(! (*it)->bFound)
		continue;

	    cv::Point2f p( (*it)->v2Image[0], (*it)->v2Image[1] );
	    //cout<<"["<<(*it)->v2Image[0]<<", "<<(*it)->v2Image[1]<<"] :: ["<<p.x<<", "<<p.y<<"]"<<endl;

	    if((*it)->Point.nSceneLabel==1)
		cv::circle(img, p, 2, cv::Scalar(0,0,255),-1, 8);
	    else
		cv::circle(img, p, 2, cv::Scalar(0,255,0),-1, 8);

	}
	cv::imshow("image", img);
	cv::waitKey(5);
#endif

    // Update the current keyframe with info on what was found in the frame.
    // Strictly speaking this is unnecessary to do every frame, it'll only be
    // needed if the KF gets added to MapMaker. Do it anyway.
    // Export pose to current keyframe:
    mCurrentKF.se3CfromW = mse3CamFromWorld;

    // Record successful measurements. Use the KeyFrame-Measurement struct for this.
    mCurrentKF.mMeasurements.clear();
    for(vector<TrackerData*>::iterator it = vIterationSet.begin();
	    it!= vIterationSet.end(); 
	    it++)
    {
	if(! (*it)->bFound)
	    continue;
	Measurement m;
	m.v2RootPos = (*it)->v2Found;
	m.nLevel = (*it)->nSearchLevel;
	m.bSubPix = (*it)->bDidSubPix; 
	mCurrentKF.mMeasurements[& ((*it)->Point)] = m;
    }

    // Finally, find the mean scene depth from tracked features
    {
	double dSum = 0;
	double dSumSq = 0;
	int nNum = 0;
	for(vector<TrackerData*>::iterator it = vIterationSet.begin();
		it!= vIterationSet.end(); 
		it++)
	    if((*it)->bFound)
	    {
		double z = (*it)->v3Cam[2];
		dSum+= z;
		dSumSq+= z*z;
		nNum++;
	    };
	if(nNum > 20)
	{
	    mCurrentKF.dSceneDepthMean = dSum/nNum;
	    mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
	}
    }
Vector<3> v3Pos;
    CamPose cp(mCurrentKF.nFrameIdAbs, mCurrentKF.se3CfromW, false);
v3Pos=cp.se3CfromW.inverse().get_translation();

//fprintf(stderr,"%f,%f,%f\n",v3Pos[1],v3Pos[0],v3Pos[2]);
//fprintf(stderr,"z:%f,%f\n",v3Pos[1]+(altitude_fkf/1000.0),altitude/1000.0);
    mMap.vCamPoses.push_back(cp);
    //sw.stop();
    //cout<<"t = "<<sw.getStoppedTime()<<flush;
    //sw.start();

#if 0
    //if(mCurrentKF.nFrameIdAbs > 211 && mCurrentKF.nFrameIdAbs<=217)
    if(mCurrentKF.nFrameIdAbs == 212 )
    {
	CalculateHomographyAndTrianculate(mPreviousFrameKF, mCurrentKF, "../codes/misc/mask.png");
    }
#endif
#if 0
    vector<Vector<3> > maskPt;
    if(mCurrentKF.nFrameIdAbs==100)
    {
	maskPt = mMapMaker.MaskPoints(mCurrentKF, "mask/plane31_100_1.png");
	vector<Vector<3> > junk = mMapMaker.FindMaskPlane(maskPt);
	printf("maskPt.s = %d\n", maskPt.size());
    }
#endif

}

//E
//Calculate the homography of the points tracked by PTAM, Obviously inside a mask.
#if 0
bool Tracker::CalculateHomographyAndTrianculate(KeyFrame &kSrc, KeyFrame &kTarget, const char *cMaskPath)
{
    cout<<"kf="<<kSrc.nFrameIdAbs<<", "<<kTarget.nFrameIdAbs<<endl;
    using namespace cv;
#define WRITE 1
#define DENSE 1

    Mat colorMask, mask;
    colorMask = imread(cMaskPath);
    cvtColor(colorMask, mask, CV_BGR2GRAY);
    if(mask.empty())
    {
	cout<<"Mask was not loaded, hence exiting"<<endl;
	return false;
    }

    //mask = mask/255;
    char fn[100];

    //imshow("Video", mask);
    Mat image, img, targetImage;
    Mat H;
    Mat x(3, 1, CV_64FC1);
    vector<Point2f> points[2];
#if !DENSE
    vector<Point2f> initPoints;
    Mat hx;//the points tracked using homography.
#endif
#if DENSE
    vector<Point2f> densePoints;
#endif

    sprintf(fn, "%s/%d.jpg", cDatasetPath, kSrc.nFrameIdAbs);
    image = imread(fn);
    sprintf(fn, "%s/%d.jpg", cDatasetPath, kTarget.nFrameIdAbs);
    targetImage = imread(fn);
    sprintf(fn, "masked_%d.png", kSrc.nFrameIdAbs);
    imwrite(fn, 0.5*image + 0.5*colorMask);

#if DENSE
    int row=0, col=0, count=0;
    for(row=0; row<mask.rows; row++)
    {
	for(col=0; col<mask.cols; col++)
	{
	    if(mask.at<uchar>(row,col)==0)
	    {
		continue;
	    }
	    count++;
	    densePoints.push_back(Point2f(col,row));
	}
    }
    cout<<"count="<<count<<endl;
#endif

    vector<MapPoint*> vMapPoints = mMap.vpPoints;
    TooN::Vector<2> v2ImPoint[2];
    Mat pt = Mat::zeros(image.size(), CV_8UC1);
    cout<<"mapSize = "<<vMapPoints.size()<<endl;
    for(unsigned int i=0; i<vMapPoints.size(); i++)
    {
	if(kSrc.mMeasurements.count(vMapPoints[i])==0 || kTarget.mMeasurements.count(vMapPoints[i])==0)
	    continue;
	v2ImPoint[0] = kSrc.mMeasurements[vMapPoints[i]].v2RootPos;
	v2ImPoint[1] = kTarget.mMeasurements[vMapPoints[i]].v2RootPos;
	if(mask.at<uchar>(v2ImPoint[0][1], v2ImPoint[0][0])==255 && mask.at<uchar>(v2ImPoint[1][1], v2ImPoint[1][0])==255)
	{
	    points[0].push_back(Point2f(v2ImPoint[0][0], v2ImPoint[0][1]));
	    points[1].push_back(Point2f(v2ImPoint[1][0], v2ImPoint[1][1]));
	    pt.at<uchar>(v2ImPoint[0][1], v2ImPoint[0][0]) = 255;
	}
    }
    cout<<"Sz="<<points[0].size()<<endl;

    sprintf(fn, "pt_%d.png", kSrc.nFrameIdAbs);
    imwrite(fn, pt);

    if(points[0].size() <4)
    {
	return false;
    }
    H = findHomography(points[0], points[1], CV_RANSAC, 5);
    H1n = H*H1n;
#if WRITE 
    Mat oimg = image.mul(colorMask/255);
    sprintf(fn, "oimg_%d.png", kSrc.nFrameIdAbs);
    imwrite(fn, oimg);
    img = Mat::zeros(oimg.size(), oimg.type());

    warpPerspective(oimg, img, H, img.size());
    sprintf(fn, "warped_%d.png", kSrc.nFrameIdAbs);
    imwrite(fn, 0.5*img + 0.5*targetImage);
#endif

    /*for(unsigned int i=0; i<points[0].size(); i++)
    {
	x.at<double>(0,0)=points[0][i].x; x.at<double>(1,0)=points[0][i].y; x.at<double>(2,0)=1;
	printf("%.2lf %.2lf\t", x.at<double>(0,0)/x.at<double>(2,0), x.at<double>(1,0)/x.at<double>(2,0));
	x=H*x;
	printf("%.2lf %.2lf\n", x.at<double>(0,0)/x.at<double>(2,0), x.at<double>(1,0)/x.at<double>(2,0));
    }*/

    TooN::Vector<2> v2Correspondance[2];
    vector<pair<TooN::Vector<2>, TooN::Vector<2> > > vDenseMatches;
    for(unsigned int i=0; i<densePoints.size(); i++)
    {
	x.at<double>(0,0)=densePoints[i].x; x.at<double>(1,0)=densePoints[i].y; x.at<double>(2,0)=1;
	x = H*x; 
	if(x.at<double>(0,0) < 0 || x.at<double>(1,0) < 0 || x.at<double>(0,0) > img.cols || x.at<double>(1,0)>img.rows)
	{
	    continue;
	}
	v2Correspondance[0] = makeVector(densePoints[i].x, densePoints[i].y);
	v2Correspondance[1] = makeVector((int)(x.at<double>(0,0)/x.at<double>(2,0)), (int)(x.at<double>(1,0)/x.at<double>(2,0)));
	vDenseMatches.push_back(pair<TooN::Vector<2>, TooN::Vector<2> >(v2Correspondance[1], v2Correspondance[0]));
    }
    mMapMaker.TriangulateDense(kTarget, kSrc, vDenseMatches);

    return true;
}
#endif

// Find points in the image. Uses the PatchFiner struct stored in TrackerData
int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
	{
		// First, attempt a search at pixel locations which are FAST corners.
		// (PatchFinder::FindPatchCoarse)
		TrackerData &TD = *vTD[i];
		PatchFinder &Finder = TD.Finder;
		Finder.MakeTemplateCoarseCont(TD.Point);
		if(Finder.TemplateBad())
		{
			TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
			continue;
		}
		manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta
		
		bool bFound = Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nRange);
		TD.bSearched = true;
		if(!bFound) 
		{
			TD.bFound = false;
			continue;
		}
		
		TD.bFound = true;
		TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());
		
		nFound++;
		manMeasFound[Finder.GetLevel()]++;
		
		// Found the patch in coarse search - are Sub-pixel iterations wanted too?
		if(nSubPixIts > 0)
		{
			TD.bDidSubPix = true;
			Finder.MakeSubPixTemplate();
			bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF, nSubPixIts);
			if(!bSubPixConverges)
				{ // If subpix doesn't converge, the patch location is probably very dubious!
					TD.bFound = false;
					nFound--;
					manMeasFound[Finder.GetLevel()]--;
					continue;
				}
			TD.v2Found = Finder.GetSubPixPos();
		}
		else
		{
			TD.v2Found = Finder.GetCoarsePosAsVector();
			TD.bDidSubPix = false;
		}
	}
  return nFound;
};

//Calculate a pose update 6-vector from a bunch of image measurements.
//User-selectable M-Estimator.
//Normally this robustly estimates a sigma-squared for all the measurements
//to reduce outlier influence, but this can be overridden if
//dOverrideSigma is positive. Also, bMarkOutliers set to true
//records any instances of a point being marked an outlier measurement
//by the Tukey MEstimator.
Vector<6> Tracker::CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
{
  // Which M-estimator are we using?
  int nEstimator = 0;
  static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
  if(*gvsEstimator == "Tukey")
    nEstimator = 0;
  else if(*gvsEstimator == "Cauchy")
    nEstimator = 1;
  else if(*gvsEstimator == "Huber")
    nEstimator = 2;
  else 
    {
      cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
      nEstimator = 0;
      *gvsEstimator = "Tukey";
    };
  
  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  vector<double> vdErrorSquared;
  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
	continue;
      TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
      vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
    };
  
  // No valid measurements? Return null update.
  if(vdErrorSquared.size() == 0)
    return makeVector( 0,0,0,0,0,0);
  
  // What is the distribution of errors?
  double dSigmaSquared;
  if(dOverrideSigma > 0)
    dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
  else
    {
      if (nEstimator == 0)
      {
	dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
      }
      else if(nEstimator == 1)
	dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
      else 
	dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
    }
  
  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  WLS<6> wls;
  wls.add_prior(100.0); // Stabilising prior
  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
	continue;
      Vector<2> &v2 = TD.v2Error_CovScaled;
      double dErrorSq = v2 * v2;
      double dWeight;
      
      if(nEstimator == 0)
	dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
      else if(nEstimator == 1)
	dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else 
	dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
      
      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      if(dWeight == 0.0)
	{
	  if(bMarkOutliers)
	    TD.Point.nMEstimatorOutlierCount++;
	  continue;
	}
      else
	if(bMarkOutliers)
	  TD.Point.nMEstimatorInlierCount++;
      
      Matrix<2,6> &m26Jac = TD.m26Jacobian;
      wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
    }
  
  wls.compute();
  return wls.get_mu();
}


// Just add the current velocity to the current pose.
// N.b. this doesn't actually use time in any way, i.e. it assumes
// a one-frame-per-second camera. Skipped frames etc
// are not handled properly here.
void Tracker::ApplyMotionModel()
{
  mse3StartPos = mse3CamFromWorld;
  Vector<6> v6Velocity = mv6CameraVelocity;
  if(mbUseSBIInit)
    {
      v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>();
      v6Velocity[0] = 0.0;
      v6Velocity[1] = 0.0;
    }
  mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;//equation 6 in the paper
};


// The motion model is entirely the tracker's, and is kept as a decaying
// constant velocity model.
void Tracker::UpdateMotionModel()
{
  SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
  Vector<6> v6Motion = SE3<>::ln(se3NewFromOld);
  Vector<6> v6OldVel = mv6CameraVelocity;
  
  mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
  mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);
  
  // Also make an estimate of this which has been scaled by the mean scene depth.
  // This is used to decide if we should use a coarse tracking stage.
  // We can tolerate more translational vel when far away from scene!
  Vector<6> v6 = mv6CameraVelocity;
  v6.slice<0,3>() *= 1.0 / mCurrentKF.dSceneDepthMean;
  mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}

// Time to add a new keyframe? The MapMaker handles most of this.
void Tracker::AddNewKeyFrame()
{
	mMessageForUser << " Adding key-frame.";

	// saving rgb image for test purposes -- KINDLY IGNORE

	ostringstream ss;
	ss<<"/home/nazrul/teset/"<<cnt<<".jpg";
	img_save(mCurrentKF.imRGB, ss.str());	
	cnt++;

	// END OF IGNORANCE

  mMapMaker.AddKeyFrame(mCurrentKF);
  mnLastKeyFrameDropped = mnFrame;
  
	mMap.vCamPoses.back().bIsKeyFrame = true;
}

// Some heuristics to decide if tracking is any good, for this frame.
// This influences decisions to add key-frames, and eventually
// causes the tracker to attempt relocalisation.
void Tracker::AssessTrackingQuality()
{
  int nTotalAttempted = 0;
  int nTotalFound = 0;
  int nLargeAttempted = 0;
  int nLargeFound = 0;
  
  for(int i=0; i<LEVELS; i++)
    {
      nTotalAttempted += manMeasAttempted[i];
      nTotalFound += manMeasFound[i];
      if(i>=2) nLargeAttempted += manMeasAttempted[i];
      if(i>=2) nLargeFound += manMeasFound[i];
    }
  
  if(nTotalFound == 0 || nTotalAttempted == 0)
    mTrackingQuality = BAD;
  else
    {
      double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
      double dLargeFracFound;
      if(nLargeAttempted > 10)
	dLargeFracFound = (double) nLargeFound / nLargeAttempted;
      else
	dLargeFracFound = dTotalFracFound;

      static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);
      static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);
      
      
      if(dTotalFracFound > *gvdQualityGood)
	mTrackingQuality = GOOD;
      else if(dLargeFracFound < *gvdQualityLost)
	mTrackingQuality = BAD;
      else
	mTrackingQuality = DODGY;
    }
  
  if(mTrackingQuality == DODGY)
    {
      // Further heuristics to see if it's actually bad, not just dodgy...
      // If the camera pose estimate has run miles away, it's probably bad.
      if(mMapMaker.IsDistanceToNearestKeyFrameExcessive(mCurrentKF))
	mTrackingQuality = BAD;
    }
  
  if(mTrackingQuality==BAD)
    mnLostFrames++;
  else
    mnLostFrames = 0;
}

string Tracker::GetMessageForUser()
{
  return mMessageForUser.str();
}

void Tracker::CalcSBIRotation()
{
  mpSBILastFrame->MakeJacs();
  pair<SE2<>, double> result_pair;
  result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
  SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
  mv6SBIRot = se3Adjust.ln();
}

#ifdef RM
bool Tracker::ReadMap()
{  
    string sMapPath = GV3::get<string>("dir") + "/" + GV3::get<string>("md");
    int kf;

    string sFileName = sMapPath + "/pose.dump";
    ifstream ifs(sFileName.c_str());
    if(!ifs.is_open())
    {
	cout<<"File containing poses is not there!!! :O"<<endl;
	return false;
    }
    while(ifs.good())
    {
	SE3<> se3;
	ifs>>kf;
	ifs>>se3;
	KeyFrame *kfNew = new KeyFrame;
	kfNew->se3CfromW = se3;
	kfNew->nFrameIdAbs = kf;
	mpKeyFrames[kf] = kfNew;
    }
    ifs.close();

    sFileName = sMapPath + "/points.dump";
    ifs.open(sFileName.c_str());
    if(!ifs.is_open())
    {
	cout<<"File Containing points is not there!!! :O"<<endl;
	return false;
    }
    while(ifs.good())
    {
	Vector<3> v3Point;
	ImageRef irRoot;
	ifs>>v3Point;
	ifs>>kf;
	ifs>>irRoot;
	MapPoint *pNew = new MapPoint;
	pNew->v3WorldPos = v3Point;
	pNew->irCenter = irRoot;
	pNew->pTData = new TrackerData(pNew);
	mpMapPoints.insert(pair<int,MapPoint*>(kf, pNew));
    }
    ifs.close();


    cout<<"Read Map"<<endl;
}

void Tracker::UpdateMap(int nFrameId)
{
#define MMIV3 multimap<int,MapPoint* >
 //   cout<<"\r"<<"f="<<nFrameId<<flush;//", kf="<<mMapMaker.LatestKeyFrameId()<<flush;
    if(!mpKeyFrames.count(nFrameId))//count is 0 then return.
	return;

  //  cout<<", kf = "<<nFrameId<<flush;
    mMap.vpKeyFrames.push_back(mpKeyFrames[nFrameId]);

    MMIV3::iterator it;
    pair<MMIV3::iterator, MMIV3::iterator> ret;
    ret = mpMapPoints.equal_range(nFrameId);
    for(it=ret.first; it!=ret.second; ++it)
	mMap.vpPoints.push_back((*it).second);

    mse3CamFromWorld = mpKeyFrames[nFrameId]->se3CfromW;

    if(mbDraw)
    {
	glPointSize(6);
	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_POINTS);
    }

    for(size_t i=0; i<mMap.vpPoints.size(); i++)
    {
	MapPoint &p = *(mMap.vpPoints[i]);
	TrackerData &TData = *p.pTData;
	TData.Project(mse3CamFromWorld, mCamera);
	if(!TData.bInImage)
	    continue;

	Measurement m;
	m.v2RootPos = TData.v2Image;
	mMap.vpKeyFrames[mMap.vpKeyFrames.size() -1]->mMeasurements[&p] = m;
	if(mbDraw)
	{
	    glColor3f(0,1,0);
	    glVertex(TData.v2Image);
	}
    }
    if(mbDraw)
    {
	glEnd();
	glDisable(GL_BLEND);
    }
    //mMapMaker.mDmm.AGP(mMap.vpKeyFrames.size()-1);
}
#endif
//E
#if 0
void Tracker::SaveDensePoints()
{
    string sMapPath = GV3::get<string>("dir") + "/" + GV3::get<string>("md") + "/";
    cout<<"SDP: "<<sMapPath<<endl;
    for(size_t i=0; i<mMap.mvoObjects.size(); i++)
    { 
	vector<Vector<3> > vv3Points = mMap.mvoObjects[i].mvv3Points;
	ostringstream oss;
	oss<<sMapPath<<mMap.mvoObjects[i].mnBodyId<<".dense";
	ofstream ofs(oss.str().c_str());
	for(size_t j=0; j<vv3Points.size(); j++)
	{
	    ofs<<vv3Points[j]<<endl;
	}
    }
    cout<<"SDP out!!"<<endl;
}

//E
void Tracker::LoadDensePoints()
{
    int nObjCount=0;
    string sMapPath = GV3::get<string>("dir") + "/" + GV3::get<string>("md") + "/";
    ostringstream oss;
    oss<<sMapPath<<nObjCount<<".dense";
    bool bFileFound = !access(oss.str().c_str(), F_OK);
    cout<<"LDP:"<<oss.str()<<", "<<bFileFound<<endl;;

    while(bFileFound)
    {
	ifstream ifs(oss.str().c_str());
	vector<Vector<3> > vv3Points;
	Vector<3> v3Point;

	while(ifs.good())
	{
	    ifs>>v3Point;
	    vv3Points.push_back(v3Point);
	}
	ifs.close();
	objects obj;
	obj.mnBodyId = nObjCount;
	obj.mvv3Points = vv3Points;
	mMap.mvoObjects.push_back(obj);
	nObjCount++;
	oss.str("");oss<<sMapPath<<nObjCount<<".dense";
	bFileFound = !access(oss.str().c_str(), F_OK);
    }
    cout<<"LDP Out!!"<<endl;
}
#endif

ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here
