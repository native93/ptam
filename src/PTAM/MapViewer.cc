#include "MapViewer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LevelHelpers.h"
#include <iomanip>

#include <cvd/gl_helpers.h>
#include <gvars3/instances.h>

//#include "ColorUtil.h"

using namespace CVD;
using namespace std;
using namespace GVars3;


MapViewer::MapViewer(Map &map, GLWindow2 &glw):
  mMap(map), mGLWindow(glw)
{
path.header.frame_id="world_ptam";
trajectory=nh.advertise<nav_msgs::Path>("Trajectory",1);
  mse3ViewerFromWorld = 
    SE3<>::exp(makeVector(0,0,2,0,0,0)) * SE3<>::exp(makeVector(0,0,0,0.8 * M_PI,0,0));
a=2.6;
b=0;
c=-0.4;
d=0.8;
e=1.2;
f=9;
mScale = 1.0;
}

void MapViewer::DrawMapDots()
{
  SetupFrustum();
  SetupModelView();
  
  int nForMass = 0;
  /*glPointSize(15);
  glBegin(GL_POINTS);
  Vector<3> origin = makeVector(0,0.5,0);
  glColor(gavLevelColors[0]);
  glVertex(origin);
  glEnd();*/
  glPointSize(4);
  glBegin(GL_POINTS);
  mv3MassCenter = Zeros;
	for(size_t i = 0; i < mMap.densepoints.size(); i++){
	Vector<3> v3Pos;

	v3Pos[0] = mMap.densepoints[i].x;
	v3Pos[1] = mMap.densepoints[i].y;
	v3Pos[2] = mMap.densepoints[i].z;
	glColor(Rgb<uint8_t>(0,255,255));//mMap.densepoints[i].r,mMap.densepoints[i].g,mMap.densepoints[i].b));
	glVertex(v3Pos);
	}
#if 1
  for(size_t i=0; i<mMap.vpPoints.size(); i++)
  {
      Vector<3> v3Pos = mMap.vpPoints[i]->v3WorldPos;

      if(mMap.vpPoints[i]->nSceneLabel)
	  glColor(gavLevelColors[0]);
      else
	  glColor(gavLevelColors[1]);
#if 0
      if(mMap.interestPoint[i])
      {
	  //glEnd();
	  //glPointSize(6);
	  //glBegin(GL_POINTS);
	  glColor3f(0,0,1);
	  //glVertex(v3Pos);
	  //glEnd();
	  //glPointSize(2);
	  //glBegin(GL_POINTS);
	  //continue;
      }
#endif
/*      else
	  continue;*/



      if(v3Pos * v3Pos < 10000)
      {
	  nForMass++;
	  mv3MassCenter += v3Pos;
      }
      //double z = v3Pos[2] / 0.007;
      //double Z_MIN = -1, Z_MAX = 2;
      //double z_norm = (z - Z_MIN) / (Z_MAX - Z_MIN);
      //glColor3fv(color_util_jet(z_norm));

      glVertex(v3Pos);
  }
#endif
#if 0
  for(size_t i=0; i<mMap.vpDensePoints.size(); i++)
  { 
    Vector<3> v3Pos;
    v3Pos = mMap.vpDensePoints[i];

    glColor(mMap.vpPixelValues[i]);
    //glColor(gavLevelColors[2]);

    glVertex(v3Pos);
  }
#endif
#if 0
  for(size_t obj=0; obj< mMap.mvoObjects.size(); obj++)
  {
      //cout<<"\r"<<"obj = "<<mMap.mvoObjects.size()<<endl;
      vector<Vector<3> > vv3Points = mMap.mvoObjects[obj].mvv3Points;
      vector<Vector<3> > vv3Pixels = mMap.mvoObjects[obj].mvv3Pixels;
      for(size_t i=0; i<vv3Points.size(); i++)
      {
	  //glColor(vv3Pixels[i]);
	  glColor3f(1,0.5,0);
	  glVertex(vv3Points[i]);
      }
  }
#endif
#if 0
  for(size_t i=0; i<mMap.vpBAPoints.size(); i++)
  {
    glColor(mMap.vpBAPixelValues[i]);
    //glColor3f(0,0,1);
    glVertex(mMap.vpBAPoints[i]);
  }
#endif
#if 0
  glColor3f(1,0,1);
  glPointSize(6);
  for(size_t i=0; i<mMap.vpPlanePoints.size(); i++)
  {
    Vector<3> v3Pos = mMap.vpPlanePoints[i];
    glVertex(v3Pos);
  }
#endif
  glEnd();
#if 0
  glPointSize(2);
  glBegin(GL_POINTS);
  glColor3f(1,0.5,0);
  for(size_t i=0;i<mMap.vpOFPoints.size(); i++)
  {
      glVertex(mMap.vpOFPoints[i]);
  }
  glEnd();
#endif
  mv3MassCenter = mv3MassCenter / (0.1 + nForMass);
}

//E
#if 0
void MapViewer::DrawDense(int fnum)
{
    static bool load=true;
    if(load)
    {
	char name[200];
	const char *preName = GV3::get<string>("mp").c_str();
	for(int i=0; i<2; i++)
	{
	    sprintf(name, "%s_%d_%d.png_f.txt", preName, fnum, i);
	    printf("%s\n", name);
	    ifstream ifs(name);
	    Vector<3> pt,px;
	    if( ifs.is_open() )
	    {
		while(!ifs.eof())
		{
		    ifs>>pt; ifs>>px;
		    //cout<<pt<<", "<<px<<endl;
		    mMap.vpDensePoints.push_back(pt);
		    mMap.vpPixelValues.push_back(px);
		}
		ifs.close();
		cout<<"dps="<<mMap.vpDensePoints.size()<<endl;
	    }
	}
#if 0
	ifstream ifs("mask/low_11_82_2.png_f.txt");
	Vector<3> pt,px;
	if( ifs.is_open() )
	{
	    while(!ifs.eof())
	    {
		ifs>>pt; ifs>>px;
		//cout<<pt<<", "<<px<<endl;
		mMap.vpDensePoints.push_back(pt);
		mMap.vpPixelValues.push_back(px);
	    }
	    ifs.close();
	    cout<<"dps="<<mMap.vpDensePoints.size()<<endl;
	}
#endif
	load=false;
    }
    SetupFrustum();
    SetupModelView();
    glPointSize(2);
    glBegin(GL_POINTS);
    for(size_t i=0; i<mMap.vpDensePoints.size(); i++)
    {
	//cout<<"i"<<endl;
	Vector<3> v3Pos = mMap.vpDensePoints[i];

	glColor(mMap.vpPixelValues[i]);

	glVertex(v3Pos);
    }
    glEnd();
	//load=false;
}
#endif

void MapViewer::DrawGrid()
{
  SetupFrustum();
  SetupModelView();
  glLineWidth(1);
  
  glBegin(GL_LINES);
  
  // Draw a larger grid around the outside..
  double dGridInterval = 0.1;
  
  double dMin = -100.0 * dGridInterval;
  double dMax =  100.0 * dGridInterval;
  
  for(int x=-10;x<=10;x+=1)
	{
    //if(x==0)
			//glColor3f(1,1,1);
    //else
			glColor3f(0.3,0.3,0.3);
      glVertex3d((double)x * 10 * dGridInterval, dMin, 0.0);
      glVertex3d((double)x * 10 * dGridInterval, dMax, 0.0);
  }
  for(int y=-10;y<=10;y+=1)
  {
    //if(y==0)
			//glColor3f(1,1,1);
    //else
			glColor3f(0.3,0.3,0.3);
    glVertex3d(dMin, (double)y * 10 *  dGridInterval, 0.0);
    glVertex3d(dMax, (double)y * 10 * dGridInterval, 0.0);
  }
  
  glEnd();
	
	
	//Temporary commenting the inner grid
  glBegin(GL_LINES);
  dMin = -10.0 * dGridInterval;
  dMax =  10.0 * dGridInterval;
  
  for(int x=-10;x<=10;x++)
    {
      if(x==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.5,0.5,0.5);
      
      glVertex3d((double)x * dGridInterval, dMin, 0.0);
      glVertex3d((double)x * dGridInterval, dMax, 0.0);
    }
  for(int y=-10;y<=10;y++)
    {
      if(y==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.5,0.5,0.5);
      glVertex3d(dMin, (double)y * dGridInterval, 0.0);
      glVertex3d(dMax, (double)y * dGridInterval, 0.0);
    }
  
  glColor3f(1,0,0);
  glVertex3d(0,0,0);
  glVertex3d(0.2,0,0);
  glColor3f(0,1,0);
  glVertex3d(0,0,0);
  glVertex3d(0,0.2,0);
  glColor3f(1,1,1);
  glVertex3d(0,0,0);
  glVertex3d(0,0,0.2);
  glEnd();
  
} 

void MapViewer::DrawMap(SE3<> se3CamFromWorld, CVD::Image<CVD::byte> &imFrame)
{
  mMessageForUser.str(""); // Wipe the user message clean

  //{
  //  pair<Vector<6>, Vector<6> > pv6 = mGLWindow.GetMousePoseUpdate();
  //  SE3<> se3CamFromMC;
  //  se3CamFromMC.get_translation() = mse3ViewerFromWorld * mv3MassCenter;
  //  mse3ViewerFromWorld = SE3<>::exp(pv6.first) * 
  //    se3CamFromMC * SE3<>::exp(pv6.second) * se3CamFromMC.inverse() * mse3ViewerFromWorld;
  //}

  {
    pair<Vector<6>, Vector<6> > pv6 = mGLWindow.GetMousePoseUpdate();
    SE3<> se3CamFromMC;
    se3CamFromMC.get_translation() = mse3ViewerFromWorld * mv3MassCenter;
    mse3ViewerFromWorld = SE3<>::exp(pv6.first) * 
      se3CamFromMC * SE3<>::exp(pv6.second) * se3CamFromMC.inverse() * mse3ViewerFromWorld;
//mse3ViewerFromWorld= SE3<>(makeVector(a, b, c),makeVector(d, e, f));
    //static SE3<> se3PrevCamFromWorld = se3CamFromWorld;
    //SE3<> se3CamToViewer = mse3ViewerFromWorld * se3PrevCamFromWorld.inverse();    
    //mse3ViewerFromWorld = se3CamToViewer * se3CamFromWorld;

    //Vector<6> v6Velocity = makeVector(0,0.3,0.6,0,0,0);
    //mse3ViewerFromWorld = SE3<>::exp(v6Velocity) * se3CamFromWorld;
    //mse3ViewerFromWorld = SE3<>::exp(makeVector(0,0.0,0.006,0,0,0)) * mse3ViewerFromWorld;   

    //se3PrevCamFromWorld = se3CamFromWorld;
  }

  mGLWindow.SetupViewport();
  glClearColor(0.1,0.1,0.1,0);
  //glClearColor(1.0,1.0,1.0,0);
  glClearDepth(1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColorMask(1,1,1,1);

  glEnable(GL_DEPTH_TEST);
  DrawGrid();
  //Drawing the refrence plane if available.
#define NM 0

#if 0
  //cerr<<"vPlanePoints.size() = "<<mMap.vpPlanePoints.size()<<endl;
#if NM
  if(mMap.vpPlanePoints.size()==2 || mMap.vpPlanePoints.size()==4 || mMap.vpPlanePoints.size()==7)
#else
  if(mMap.vpPlanePoints.size())
#endif
  {
      //glPointSize(10);
      //glBegin(GL_POINTS);
      glLineWidth(3);
      glBegin(GL_LINES);
      glColor3f(1,0,0);
#if NM
      //mMap.vpPlanePoints[1] = mMap.vpPlanePoints[0] + mMap.vpPlanePoints[1];
#endif
      //glVertex(mMap.vpPlanePoints[0]);
      Vector<3> orig = makeVector(0,0,0);
      glVertex(mMap.vpPlanePoints[0]); //orig line
      glVertex(mMap.vpPlanePoints[0] + 0.5*mMap.vpPlanePoints[1]); //orig line

      glVertex(orig);
      glVertex(mMap.vpPlanePoints[0]); //orig line

      if(mMap.vpPlanePoints.size()>=4)
      {
	  glColor3f(0,1,0);
	  glVertex(mMap.vpPlanePoints[2]); //orig line
	  glVertex(mMap.vpPlanePoints[2] + 0.5*mMap.vpPlanePoints[3]); //orig line

	  glVertex(orig);
	  glVertex(mMap.vpPlanePoints[2]); //orig line
      }

      if(mMap.vpPlanePoints.size()==7)
      {
	  glColor3f(1,1,0);
	  glVertex(mMap.vpPlanePoints[4]);
	  glVertex(mMap.vpPlanePoints[5]);

	  glColor3f(0,1,1);
	  glVertex(orig);
	  glVertex(mMap.vpPlanePoints[6]);
      }

#if !NM
      glEnd();
      Vector<4> v4Plane=mMap.mvoObjects[0].mv4Plane;
      glPointSize(6);
      glBegin(GL_POINTS);
      glColor3f(1,0.5,0);
      for(int i=-10; i<=10; i+=1)
      {
	  for(int j=-10; j<=10; j+=1)
	  {
	      int z = (-1*(v4Plane[0]*i+v4Plane[1]*j+v4Plane[3]))/v4Plane[2];
	      glVertex3f(i,j,z);
	  }
      }
#endif
      glEnd();

  }
#endif

#define SAD 1
#if SAD
//Sparse And Dense
  DrawMapDots();
#else
  DrawDense(100);
#endif

  DrawCamera(se3CamFromWorld);
//printf("calling trajectory\n");
  DrawCameraTrajectory(se3CamFromWorld);
  //DrawCameraTrajectory();

  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

}

string MapViewer::GetMessageForUser()
{
  return mMessageForUser.str();
}

void MapViewer::SetupFrustum()
{
  glMatrixMode(GL_PROJECTION);  
  glLoadIdentity();
  double zNear = 0.03;
  glFrustum(-zNear, zNear, 0.75*zNear, -0.75*zNear, zNear, 50);
  glScalef(1*mScale,1*mScale,-1*mScale);
  return;
};

void MapViewer::SetupModelView(SE3<> se3WorldFromCurrent)
{
  glMatrixMode(GL_MODELVIEW);  
  glLoadIdentity();
  glMultMatrix(mse3ViewerFromWorld * se3WorldFromCurrent);
  return;
};

void MapViewer::DrawCamera(SE3<> se3CfromW)
{

    SetupModelView(se3CfromW.inverse());
    SetupFrustum();

    glLineWidth(3);

    float fLengthY = 0.04f ;
    float fLengthX = 1.333f * fLengthY;

    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(fLengthY, 0.0f, 0.0f);
    glColor3f(0,1,0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, fLengthY, 0.0f);
    glColor3f(0,0,1);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, fLengthY);
    glEnd();

    if(1)
    {
	glLineWidth(2);
	glColor3f(0,1,1);
	glBegin(GL_LINES);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(fLengthX, -fLengthY, fLengthX);

	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(fLengthX, fLengthY, fLengthX);

	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(-fLengthX, fLengthY, fLengthX);

	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(-fLengthX, -fLengthY, fLengthX);
	glEnd();

#if 0
	//Draw Camera Screen
	//glEnable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D, mTextureId);	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glColor4f(1.0,1.0,1.0,0.6);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(fLengthX, -fLengthY, fLengthX);

	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(fLengthX, fLengthY, fLengthX);

	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-fLengthX, fLengthY, fLengthX);

	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-fLengthX, -fLengthY, fLengthX);
	glEnd();
	glDisable(GL_TEXTURE_2D);
#endif
    }	

    if(1)
    {	//Draw a small cross(projection of current position in Grid)
	glLineWidth(1);
	glColor3f(0.5,0.5,0.5);
	SetupModelView();
	Vector<2> v2CamPosXY = se3CfromW.inverse().get_translation().slice<0,2>();
	glBegin(GL_LINES);
	glColor3f(1,1,1);
	glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] + 0.04);
	glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] - 0.04);
	glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] - 0.04);
	glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] + 0.04);
	glEnd();
    }

}

#if 0
void MapViewer::DrawCamera(SE3<> se3CfromW, bool bSmall)
{
  
  SetupModelView(se3CfromW.inverse());
  SetupFrustum();
  
  if(bSmall)
    glLineWidth(1);
  else
    glLineWidth(3);
  
  glBegin(GL_LINES);
  glColor3f(1,0,0);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.1f, 0.0f, 0.0f);
  glColor3f(0,1,0);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.1f, 0.0f);
  glColor3f(1,1,1);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.1f);
  glEnd();

  if(!bSmall)
  {
		
	}
	
	
  if(!bSmall)
  {	//Draw a small cross(projection of current position in Grid)
	  glLineWidth(1);
	  glColor3f(0.5,0.5,0.5);
	  SetupModelView();
	  Vector<2> v2CamPosXY = se3CfromW.inverse().get_translation().slice<0,2>();
	  glBegin(GL_LINES);
	  glColor3f(1,1,1);
	  glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] + 0.04);
	  glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] - 0.04);
  	  glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] - 0.04);
	  glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] + 0.04);
	  glEnd();
  }
  
}
#endif

void MapViewer::DrawCameraTrajectory(SE3<> se3CamFromWorld)
{

path.poses.clear();
//printf("inside trajectory\n");
geometry_msgs::PoseStamped pose;



	vector<Vector<3> > camTraj;
	Vector<3> v3Pos;
	for(size_t i=0; i<mMap.vpKeyFrames.size(); i++)
	{
	    v3Pos = mMap.vpKeyFrames[i]->se3CfromW.inverse().get_translation();

pose.pose.position.x=v3Pos[0];
pose.pose.position.y=v3Pos[1];
pose.pose.position.z=v3Pos[2];
//fprintf(stderr,"position z %f\n",pose.pose.position.z);
pose.pose.orientation.x=0;
pose.pose.orientation.y=0;
pose.pose.orientation.z=0;
pose.pose.orientation.w=1;
path.poses.push_back(pose);

	    camTraj.push_back(v3Pos);
	}




	v3Pos = se3CamFromWorld.inverse().get_translation();
	camTraj.push_back(v3Pos);
//printf("path &d %\n",path.poses.size());	
trajectory.publish(path);	
	
	
	SetupFrustum();
  SetupModelView();
  
  glColor3f(0.3,0.7,0.5);
  glPointSize(6);
  glBegin(GL_POINTS);	
	for(size_t i=0; i<camTraj.size(); i++)
	{
    glVertex(camTraj[i]);
	}
  glColor3f(0,0,0);
  if(mMap.baPose.size()==2)
  {
	glVertex(mMap.baPose[0].se3CfromW.inverse().get_translation());
	glVertex(mMap.baPose[1].se3CfromW.inverse().get_translation());
  }
	glEnd();
	
	glColor3f(0.3,0.7,0.5);
	glBegin(GL_LINES);
	for(size_t i=0; i< (camTraj.size()-1); i++)
	{
    glVertex3d(camTraj[i][0], camTraj[i][1], camTraj[i][2]);
		glVertex3d(camTraj[i+1][0], camTraj[i+1][1], camTraj[i+1][2]);
	}
	glEnd();
	
  
}


void MapViewer::DrawCameraTrajectory()
{
	SetupFrustum();
  SetupModelView();
  
  glBegin(GL_POINTS);
  glColor3f(0.3,0.7,0.5);
	glPointSize(3);
  for(unsigned int i=0; i<mMap.vCamPoses.size(); i++)
	{
		Vector<3> v3Pos = mMap.vCamPoses[i].se3CfromW.inverse().get_translation();
		glVertex(v3Pos);		
  }
  glEnd();
  
}

void MapViewer::LoadTexture(CVD::Image<CVD::byte> &imFrame)
{
	glDeleteTextures(1, &mTextureId);
  glGenTextures(1, &mTextureId); //Make room for our texture
	glBindTexture(GL_TEXTURE_2D, mTextureId); //Tell OpenGL which texture to edit
	
	glTexImage2D(imFrame);		//Map the image to the texture
}
