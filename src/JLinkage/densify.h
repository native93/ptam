#include <iostream>
#include <math.h>       /* sqrt */

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

#include "../PTAM/ATANCamera.h"
#include "../PTAM/Map.h"

#include "TooN/TooN.h"
#include "TooN/se3.h"

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>

using namespace std;
using namespace GVars3;
using namespace TooN;

class densify{

	ATANCamera &Cam;
	Map &dmap;
	float cc_lembda ;               // image match threshold
	double patch_size;             // size of 3d patch
	int patch_resolution;          // no of points inside a patch
	int imSize[2];
	int no_of_frames;		   // no of images to check in 
	Vector<3> U,V;

	pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud;

	Vector<3> cross_product(Vector<3> A, Vector<3> B);
	float magnitude(Vector<3> P);
	vector<cv::Point2f> project_to_image(SE3<> camfwrld, vector<Vector<3> > patch, ATANCamera Cam, bool &patchInIm);
	std::vector<float> get_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	vector<Vector<3> > get_patch( Vector<2> cen, pcl::PointXYZ p);
	void project_to_plane(vector<float> plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<pcl::PointIndices> &cluster_indices );
	cv::Mat warp_patch(cv::Mat *im, cv::Mat homography);
	float get_warp_factor();
	bool cross_corelation(cv::Mat A,cv::Mat B);
	void get_next_pos(int &fail);
	float distance_3D(Vector<3> A, Vector<3> B);
	void add_3Dpoints(vector<Vector<3> > patch, cv::Mat image);
	void update_cloud(vector<Vector<3> > patch);
	void draw_patch(vector<Vector<3> > patch, int i = 1);
	vector<Vector<2> > get_grid(pcl::PointXYZ mean, pcl::PointCloud<pcl::PointXYZ>::Ptr point, vector<float> plane);
	vector <Vector<2> > get_NN(Vector<2> p);

public:
	densify(Map &imap, ATANCamera &iCam, float ilembda, double ipatch_size, int ipatch_resolution, int iimSize1, int imSize2, int ino_of_frames);
	pcl::PointCloud<pcl::PointXYZ>::Ptr make_dense(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int keyframeno);
	pcl::PointCloud<pcl::PointXYZRGB> colcloudT;
};
