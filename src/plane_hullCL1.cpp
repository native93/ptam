#include <iostream>
#include <math.h>       /* sqrt */

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/project_inliers.h>
#include <algorithm>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>


#include <gvars3/instances.h>
using namespace GVars3;

#include "PTAM/ATANCamera.h"
#include "PTAM/MapPoint.h"
#include "PTAM/TrackerData.h"
#include "TooN/TooN.h"
using namespace TooN;
#include "TooN/se3.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

std::vector<std::vector<Vector<2> > > vanishp;
std::vector<std::vector<Vector<3> > > bound_line;
std::vector<Vector<3> > lineT;
Vector<2> line_point;

std::vector<std::vector<Vector<2> > > boundP2d;
int line = 0;
std::vector<Vector<2> > tempP;
cv::Point po1,po2,start;


Vector<3> line_2p(Vector<2> a,Vector<2> b)
{
Vector<3> line;
float c = (a[0] - b[0]);
float d = (a[1] - b[1]);
line[0] = d;
line[1] = -1 * c;
line[2] = a[0] * b[1] - a[1] * b[0];
return line;
}


Vector<2> line_intersection(Vector<3> l1, Vector<3> l2)
{
Vector<2> p;
float d = l1[1] * l2[0] - l1[0] * l2[1];
p[0] = (l1[2] * l2[1] - l2[2] * l1[1])/d;
p[1] = (l1[0] * l2[2] - l1[2] * l2[0])/d;
//std::cerr<<p[0]<<"      intersection            "<<p[1]<<"\n";;
return p;
}

//////////////////////////////////////////////////////////////////
//    point normal form

std::vector<float> point_normal_plane(Vector<3> planenormal, pcl::PointXYZ point)
{
std::vector<float> plane;
plane.push_back(planenormal[0]); plane.push_back(planenormal[1]); plane.push_back(planenormal[2]);
plane.push_back(-1*(point.x * planenormal[0] +point.y * planenormal[1] + point.z * planenormal[2]));
return plane;
}

std::vector<float> point_normal_plane(Vector<3> planenormal, Vector<3> point)
{
std::vector<float> plane;
plane.push_back(planenormal[0]); plane.push_back(planenormal[1]); plane.push_back(planenormal[2]);
plane.push_back(-1*(point[0] * planenormal[0] +point[1] * planenormal[1] + point[2] * planenormal[2]));
return plane;
}

////////////////////////////////////////////////////////////////////

float plane_line_intersection(std::vector<float> coef, Vector<3> point3d1, Vector<3> point3d2)
{
return ( coef[0]*point3d1[0] + coef[1]*point3d1[1] + coef[2]*point3d1[2] + coef[3])/(coef[0]*(point3d1[0] - point3d2[0]) +coef[1]*(point3d1[1] - point3d2[1]) + coef[2]*(point3d1[2] - point3d2[2]));
}

Vector<3> cross_product_3d(Vector<3> A, Vector<3> B)
{
Vector<3> result;
result[0] = (A[1]*B[2] - A[2]*B[1]);
result[1] = (-1*(A[0]*B[2] - A[2]*B[0]));
result[2] = (A[0]*B[1]-A[1]*B[0]);
return result;
}

std::vector<float> plane_3point(Vector<3> p1, Vector<3> p2, Vector<3> p3)
{
std::vector<float> plane;
Vector<3> normal = cross_product_3d( p2 - p1, p3 - p1);
plane = point_normal_plane(normal, p1);
return plane;
}

float plane_pts_distance(std::vector<float> plane, pcl::PointXYZ p)
{
return (plane[0]*p.x + plane[1]*p.y + plane[2]*p.z + plane[3])/(sqrt(pow(plane[0],2) + pow(plane[1],2) + pow(plane[2],2)));
}

void draw_line( cv::Mat img ){
cv::line( img,po1,po2, cv::Scalar(0,0,255) );
}


// Implement mouse calliick
void my_mouse_callback( int event, int x, int y, int flags, void* param ){
	cv::Mat* image = (cv::Mat*) param;
	Vector<2> local_line_point;

	switch( event ){
		case cv::EVENT_MOUSEMOVE:
			po1 = cv::Point(x,y);
			break;

		case cv::EVENT_LBUTTONDOWN:
			if(line == 0)                                                 // first line
			{
				line_point[0]=x; line_point[1]=y;
				po2 = cv::Point(x,y);
				start = po2;
			}
			po1 = cv::Point(x,y);
			local_line_point[0]=x; local_line_point[1]=y;
			draw_line(*image);
			if ( line > 0)
				lineT.push_back(line_2p(local_line_point,line_point));
			po2 = po1;                      
			line_point = local_line_point;                            // save old val 
			tempP.push_back(local_line_point);
			//std::cerr<<tempP[line].x<<"         "<<tempP[line].y<<"           "<<line<<"\n";
			line = line + 1;                                            // next line
			break;

		case cv::EVENT_RBUTTONDOWN:
			boundP2d.push_back(tempP);
			tempP.clear();
			po1 = start;
			local_line_point[0] = start.x; local_line_point[1] = start.y;
			draw_line(*image);
			lineT.push_back(line_2p(local_line_point, line_point));
			bound_line.push_back(lineT);
			lineT.clear();
			line = 0;
			break;
	}
}



int main (int argc, char** argv)
{

///////////////////////////////////////////////////////////////////////////
//                 display image and take input boundaries

std::stringstream ss;

ss<<*(argv+3)<<"/"<<*(argv+1)<<".jpg";
std::cerr<<ss.str();
cv::Mat img = cv::imread(ss.str().c_str());
ss.str("");
if ( img.empty() ) 
{ 
std::cout << "Error loading the image" << endl;
return -1; 
}

cv::namedWindow("My Window", 1);
cv::setMouseCallback("My Window", my_mouse_callback, (void*) &img);

while( 1 ){
		cv::Mat temp = img.clone();
		if(line>0)
		draw_line( temp );
		cv::imshow( "My Window", temp );

		if( cv::waitKey( 15 )==27 ) 
			break;
	}


/////////////////////////////////////////////////////////////////////////////
//                 get camera pose

SE3<> cfw;

        ss << *(argv+3)<<"/"<<*(argv+1)<<".info";
        std::ifstream ifs2;
        ifs2.open(ss.str().c_str());
	ss.str("");
	ifs2>>cfw;
	ifs2.close();
ss.str("");
Vector<3> campose;
campose = cfw.inverse().get_translation();

//////////////////////////////////////////////////////////////////////////////
//                 vanishing point and lines   and plane normal

if(argc>4)
{
std::vector<Vector<3> > vanishline;
std::vector<Vector<3> > planenormal;

for(unsigned int i =0 ; i<boundP2d.size();i=i+1)
{
std::vector<Vector<2> > vanishpT;
Vector<3> normal;

vanishpT.push_back(line_intersection(bound_line[i][0], bound_line[i][2]));                            //       vanishing points 
vanishpT.push_back(line_intersection(bound_line[i][1], bound_line[i][3]));
vanishp.push_back(vanishpT);

vanishline.push_back(line_2p(vanishp[i][0], vanishp[i][1]));                                   //     vanishing line  

normal = vanishline[i];
double focal = (-1*vanishp[i][0][0]*vanishp[i][1][0]-1*vanishp[i][0][1]*vanishp[i][1][1]);        // focal length ???? -x1x2-y1y2
normal[2] = normal[2]/focal;

planenormal.push_back(normal);                                                               // plane normal
planenormal[i] = cfw.inverse()*planenormal[i];
}
}

////////////////////////////////////////////////////////////////////////////////////
//                   plane boundary projection


std::vector<std::vector<Vector<3> > > boundP3d;

GUI.LoadFile("/home/irl/workspace/ptam/kinect_config.cfg");
GUI.StartParserThread();                                                                      // Start parsing of the console input
atexit(GUI.StopParserThread);

Vector<NUMTRACKERCAMPARAMETERS> vTest;
vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);

ATANCamera *Cam = new ATANCamera("Camera");

for(unsigned int i=0 ;i<boundP2d.size();i++)
{
std::vector<Vector<3> > v3boundT;
for(unsigned int j=0; j<boundP2d[i].size();j++)
	v3boundT.push_back(cfw.inverse() *  unproject(Cam->UnProject(boundP2d[i][j])));
boundP3d.push_back(v3boundT);
}

//////////////////////////////////////////////////////////////////////////////
//          PCL declarations

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudC(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;

pcl::PointCloud<pcl::PointXYZ> cloud_segs[boundP3d.size()];

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
viewer->setBackgroundColor (0,0,0);
viewer->addCoordinateSystem (1.0);
viewer->initCameraParameters ();


pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


  // Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_cptr;  // viewer reqires this


/////////////////////////////////////////////////////////////////////////////////////
//                      read points

if(argc>0)
reader.read (*(argv+2), *cloudC);
else
reader.read ("test_pcd.pcd", *cloudC);

///////////////////////////////////////////////////////////////////////////
//                 points filters


////////           using VP points from PTAM
# if(argc >5)

ss <<*(argv+3)<< "/"<<*(argv+1)<<"vp.info";
std::cerr<<ss.str();
ifstream ifs3;
ifs3.open(ss.str().c_str());
ss.str("");
if(ifs3.is_open())
{
std::cerr<<"opened";
}
//  read  PV points
for(;;)
{
int no;
float cx,cy;

ifs3>>no;
ifs3>>cx;
ifs3>>cy;
if(ifs3.eof())
	break;
for(unsigned int i = 0 ; i < boundP2d.size(); i++)
{
std::vector<cv::Point> boundary4cv;
for(unsigned int j = 0 ;j < boundP2d[i].size(); j++)
	boundary4cv.push_back(cv::Point(boundP2d[i][j][0],boundP2d[i][j][1]));
if(cv::pointPolygonTest(boundary4cv,cv::Point(cx,cy),false)>-1)
	cloud_segs[i].push_back(cloudC->points[no]);
}
}
ifs3.close();

//////////////////////   using frustum culling  
#else
pcl::PCDWriter writer;
for(unsigned int i = 0; i < boundP2d.size(); i++)
{

pcl::PointCloud<pcl::PointXYZ>::Ptr PCL1(new pcl::PointCloud<pcl::PointXYZ>),PCL2( new pcl::PointCloud<pcl::PointXYZ>);
PCL1 = cloudC->makeShared();

std::vector<float> plane;

for( unsigned int k = 0; k<4;k++)
{
plane = plane_3point(campose,boundP3d[i][k%4],boundP3d[i][(k+1)%4]);

pcl::PointXYZ p;
p.x = boundP3d[i][(k+2)%4][0];p.y = boundP3d[i][(k+2)%4][1];p.z = boundP3d[i][(k+2)%4][2]; 
float test_dist =  plane_pts_distance(plane ,p);
bool test = std::signbit(test_dist);

cloud_cptr = PCL1;
for(unsigned int j= 0; j < PCL1->size(); j++)
{
float dist = plane_pts_distance(plane , PCL1->points[j]);
if(test)
{
	if(dist <  0)
		PCL2->push_back(PCL1->points[j]);
}
else
{
	if(dist > 0)
		PCL2->push_back(PCL1->points[j]);
}
}
PCL1->swap(*PCL2);
PCL2->clear();
}
cloud_segs[i] = *PCL1;
}
# endif

writer.write<pcl::PointXYZ> ("/home/irl/workspace/ptam/dt.pcd",cloud_segs[0],false);
///////////////////////////////////////////////////////////////////
/*
pcl::PointCloud<pcl::PointXYZ> tempcloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudi (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ> Tcloud;
pcl::PassThrough<pcl::PointXYZ> pass;

std::string nm="out.pcd";

 
pcl::PCDWriter writer;


std::cerr<<cloudC->size();
pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers1 (new pcl::PointIndices);
  // Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg1;


pcl::ProjectInliers<pcl::PointXYZ> proj;
 


// hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh


*/
/*

 pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_v (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*cloud_filtered_v);

*/


// hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh

/*

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_cptr = cloudC;



 viewer->addPointCloud<pcl::PointXYZ> (cloud_cptr, "sample cloud");
   viewer->addCoordinateSystem (1.0);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->initCameraParameters ();
viewer->spinOnce(10000);








pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloudC);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1); // 2cm
  ec.setMinClusterSize(6);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloudC);
  ec.extract (cluster_indices);

std::cerr<<"\natleast till here\n";


for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {


 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud->points.push_back (cloudC->points[*pit]); // *
    cloud->width = cloud->points.size ();
    cloud->height = 1;



viewer->removePointCloud("sample cloud");
cloud_cptr = cloud;
viewer->addPointCloud<pcl::PointXYZ> (cloud_cptr, "sample cloud");


viewer->spinOnce (100);



double r,g,b;
 pcl::visualization::getRandomColors(r,g,b);


std::cerr<<"\n"<<cloud->size()<<"\n";



for(;;)
{
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.08);

  seg.setInputCloud (cloud->makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () < 7)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    break;
return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

cloudi->resize(inliers->indices.size());

//std::sort (inliers->indices.begin(), inliers->indices.end()); 

//tempcloud.resize(cloud.size() - inliers->indices.size()) ;




std::string a= "0";


  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);




//pcl::PointXYZ point=pcl::PointXYZ(0,0,0);
//point = cloud_plane->points[*it->indices.begin()];


  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
 chull.setInputCloud (cloud_projected);
//  chull.setAlpha (3);
  chull.reconstruct (*cloud_hull);


if(cloud_hull->size()>3)
{
std::cerr<<"drawn";
pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudh_cptr = cloud_hull;
viewer->addPolygon<pcl::PointXYZ>(cloudh_cptr,r,g,b,nm+a);
viewer->setRepresentationToSurfaceForAllActors();
viewer->spinOnce(1000);
}
else
{
std::cerr<<"nope \n";
}
std::cerr<<"point "<<cloud_hull->size()<<"\n";






                // Extract fInliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract ;
                extract.setInputCloud (cloud);
                extract.setIndices (inliers);

 extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloudi);



               extract.setNegative (true); // Removes part_of_cloud from full cloud  and
                extract.filter (*cloud);


nm = "1" + nm;
cloudi->width=1;
cloudi->height=cloudi->size();

viewer->spinOnce (100);



Tcloud=Tcloud+*cloud_projected;

//  std::cout << "Press ENTER to continue...";
//  std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
}

}
*////////////////////////////////////////////////////////


cloud_cptr = cloudC;

viewer->addPointCloud<pcl::PointXYZ> (cloud_cptr, "sample cloud");
pcl::PointXYZ p,p1;

p1.x=campose[0]*1;p1.y=campose[1]*1;p1.z=campose[2]*1;

viewer->addLine (p, p1, 255, 255, 255, "5");


/////////////////////////////////////////////////////////////
//               RANSAC

std::vector<std::vector<float> > coef;
std::vector<pcl::PointXYZ> point_on_plane;

for(unsigned int i = 0; i<boundP3d.size(); i++)
{
  // Optional
std::vector<float> onecoef;
seg.setOptimizeCoefficients (true);
  // Mandatory
seg.setModelType (pcl::SACMODEL_PLANE);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setDistanceThreshold (0.04);

seg.setInputCloud (cloud_segs[i].makeShared ());
seg.segment (*inliers, *coefficients);

if (inliers->indices.size () < 7)
{
PCL_ERROR ("Could not estimate a planar model for the given dataset.");
break;
return (-1);
}

std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
onecoef.push_back(coefficients->values[0]);
onecoef.push_back(coefficients->values[1]);
onecoef.push_back(coefficients->values[2]);
onecoef.push_back(coefficients->values[3]);
coef.push_back(onecoef);
point_on_plane.push_back(cloud_segs[i].points[inliers->indices[0]]);
}

/////////////////////////////////////////////////////////////////////////

std::cerr<<"''''''''''''''''''''''''''''''''''planes found''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''\n";


for(unsigned int i =0;i<boundP3d.size();i++)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr clouda (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb (new pcl::PointCloud<pcl::PointXYZ>);

ss.str("line");
ss << i;

//viewer->addLine (p, p1, 255, 0, 0, ss.str());
#if argc >4
std::vector<float> plane = point_normal_plane(planenormal[i],point_on_plane[i]);
#endif
for (unsigned int j =0;j<boundP3d[i].size();j++)
{
// plane ray intersection     ------------- RANSAC

boundP3d[i][j]=(boundP3d[i][j]-campose)*plane_line_intersection(coef[i], campose, boundP3d[i][j]) + campose;// P = P1 + u(P2 - P1)

p.x=boundP3d[i][j][0];p.y=boundP3d[i][j][1];p.z=boundP3d[i][j][2];

ss<<j;
viewer->addLine (p, p1, 0, 255, 0, ss.str());
clouda->push_back(p);

// plane ray intersection       --------------   using vanishing line

# if(argc >4)

boundP3d[i][j]=(boundP3d[i][j]-campose) * plane_line_intersection(plane, campose, boundP3d[i][j]) + campose;// P = P1 + u(P2 - P1)

p.x=boundP3d[i][j][0];p.y=boundP3d[i][j][1];p.z=boundP3d[i][j][2];

ss<<j;
//viewer->addLine (p, p1, 0, 255, 0, ss.str());
cloudb->push_back(p);
# endif
}

ss<<i;
cloud_cptr = clouda;
viewer->addPolygon<pcl::PointXYZ>(cloud_cptr,255,0,0,ss.str());
viewer->setRepresentationToSurfaceForAllActors();

if(argc>4)
{
ss<<i;
cloud_cptr = cloudb;
viewer->addPolygon<pcl::PointXYZ>(cloud_cptr,0,0,255,ss.str());
viewer->setRepresentationToSurfaceForAllActors();
}
//viewer->addLine(pcl::PointXYZ(0,0,0),pcl::PointXYZ(planenormal[i][0],planenormal[i][1],planenormal[i][2]), 255,255,255,ss.str());
}

while (!viewer->wasStopped ())
    viewer->spinOnce (100);

//cloudi->width=1;
//cloudi->height=cloudi->size();
// writer.write ("out2_pcd.pcd", Tcloud, false);
//  writer.write ("out_pcd.pcd", *cloud_hull, false);

  return (0);

}
