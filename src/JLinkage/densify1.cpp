#include "densify.h"
#include "../PTAM/KeyFrame.h"

densify::densify(Map *imap, ATANCamera *Cam, float ilembda, float ipatch_size, int ipatch_resolution, int iimSize1, int iimSize2, int ino_of_frames){
	dmap = imap;
	cc_lembda = ilembda;               // image match threshold
	double patch_size = ipatch_size;             // size of 3d patch
	patch_size = ipatch_size;
	patch_resolution = ipatch_resolution;          // no of points inside a patch
	imSize[0] = iimSize1;
	imSize[1] = iimSize2;
	no_of_frames = ino_of_frames;
}

Vector<3> densify::cross_product(Vector<3> A, Vector<3> B){
	Vector<3> result;
	result[0] = (A[1]*B[2] - A[2]*B[1]);
	result[1] = (-1*(A[0]*B[2] - A[2]*B[0]));
	result[2] = (A[0]*B[1]-A[1]*B[0]);
	return result;
}

float densify::magnitude(Vector<3> P){
	double mag = sqrt(pow(P[0],2) + pow(P[1],2) + pow(P[2],2));
	return mag;
}

vector<cv::Point2f> densify::project_to_image(SE3<> camfwrld, vector<Vector<3> > patch, ATANCamera *Cam, bool &patchInIm){
	vector<cv::Point2f> impatch;
	for(unsigned int i = 0; i < patch.size(); i++){
		Vector<3> camcoord = camfwrld * patch[i];
		if (camcoord[2] < 0.001){
			patchInIm = 0;
			break;
		}
		Vector<2> imPoint = Cam->Project(project(camcoord));
		if (imPoint[0] < 0 || imPoint[0] > imSize[0] || imPoint[1] < 0 || imPoint[1] > imSize[1]){
			patchInIm = 0;
			cerr<<"patch not in image\n";
			break;
		}
		else{
			impatch.push_back(cv::Point2f(imPoint[0], imPoint[1]));
			patchInIm = 1;
		}
	}
	
	return impatch;
}

std::vector<float> densify::get_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  	
	std::vector<float> plane;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  // Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients (true);
 	 // Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);

	seg.setInputCloud (cloud);
  	seg.segment (*inliers, *coefficients);
        pcl::ExtractIndices<pcl::PointXYZ> extract ;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
	extract.setNegative(false);
	extract.filter(*cloud);

	plane.push_back(coefficients->values[0]);
	plane.push_back(coefficients->values[1]);
	plane.push_back(coefficients->values[2]);
	plane.push_back(coefficients->values[3]);
	return plane;
}

vector<Vector<3> > densify::get_patch( Vector<2> cen, pcl::PointXYZ p){
	vector<Vector<3> > patch;
	Vector<3> mean;
	mean[0] = p.x; mean[1] = p.y; mean[2] = p.z;
	patch.push_back(mean + (cen[0] + patch_size/2) * U + (cen[1] + patch_size/2) * V);
	patch.push_back(mean + (cen[0] + patch_size/2) * U + (cen[1] - patch_size/2) * V);
	patch.push_back(mean + (cen[0] - patch_size/2) * U + (cen[1] - patch_size/2) * V);
	patch.push_back(mean + (cen[0] - patch_size/2) * U + (cen[1] + patch_size/2) * V);
	return patch;
}
 
void densify::project_to_plane(vector<float> plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	coefficients->values.push_back(plane[0]);coefficients->values.push_back(plane[1]);coefficients->values.push_back(plane[2]);coefficients->values.push_back(plane[3]);
 	cerr<<"knew it";
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud);
}

cv::Mat densify::warp_patch(cv::Mat *im, cv::Mat homography){
	cv::Mat patch;
	cv::Rect rectangle(0, 0, patch_resolution, patch_resolution);
	cv::warpPerspective(*im, patch, homography, cv::Size(imSize[0], imSize[1]));
	return patch(rectangle);
}

float densify::get_warp_factor(){
	return 0;
}

bool densify::cross_corelation(cv::Mat A,cv::Mat B){
	double sum_A_B = 0, sum_A_2 = 0, sum_B_2 = 0;
	cv::Scalar A_avg = cv::mean(A);
	cv::Scalar B_avg = cv::mean(B);
	for( int m = 0; m < patch_resolution; ++m ){
		for( int n = 0; n < patch_resolution; ++n ){
			sum_A_B = sum_A_B + (A.at<uchar>(m, n) - A_avg.val[0]) * (B.at<uchar>(m, n) - B_avg.val[0]);  
	       		sum_A_2 = sum_A_2 + pow((A.at<uchar>(m, n) - A_avg.val[0]), 2);
       			sum_B_2 = sum_B_2 + pow((B.at<uchar>(m, n) - B_avg.val[0]), 2);
		}
	}
	double corr = sum_A_B/sqrt(sum_A_2*sum_B_2);
	if(fabs(corr) > cc_lembda)
		return 1;
	else
		return 0;
}

void densify::update_cloud(vector<Vector<3> > patch){
	Vector<3> U1 = patch[1] - patch[0], V1 = patch[3] - patch[0], P;
	U1 = U1/magnitude(U1);
	V1 = V1/magnitude(V1);
	for(int i =0; i < patch_resolution; i=i+5)
		for(int j = 0; j < patch_resolution; j=j+5){
			P = (U1*j*patch_size + V1*i*patch_size)/patch_resolution + patch[0];     // need to be corrected ---- logically 
			pcl::PointXYZ PI;
			PI.x = P[0]; PI.y = P[1]; PI.z =P[2];
			outcloud->push_back(PI);
		}
}

void densify::add_3Dpoints(vector<Vector<3> > patch, cv::Mat image){
	Vector<3> U1 = patch[1] - patch[0], V1 = patch[3] - patch[0], P;
	U1 = U1/magnitude(U1);
	V1 = V1/magnitude(V1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr newcloud;
	for(int i =0; i < patch_resolution; i=i+10)
		for(int j = 0; j < patch_resolution; j=j+10){
			P = (U1*j*patch_size + V1*i*patch_size)/patch_resolution + patch[0];     // need to be corrected ---- logically 
			pcl::PointXYZRGB PI;
			PI.x = P[0]; PI.y = P[1]; PI.z =P[2]; PI.r = image.at<uchar>(i, j); PI.b = image.at<uchar>(i, j); PI.g = image.at<uchar>(i, j);
			newcloud->push_back(PI);
		}
	
}

void densify::draw_patch(vector<Vector<3> > patch, int i = 1){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);

	cloud_hull->push_back(pcl::PointXYZ(patch[0][0],patch[0][1],patch[0][2]));
	cloud_hull->push_back(pcl::PointXYZ(patch[1][0],patch[1][1],patch[1][2]));
	cloud_hull->push_back(pcl::PointXYZ(patch[2][0],patch[2][1],patch[2][2]));
	cloud_hull->push_back(pcl::PointXYZ(patch[3][0],patch[3][1],patch[3][2]));

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudh_cptr = cloud_hull;
//	name = name +"a";
	if(i == 1)
		//viewer->addPolygon<pcl::PointXYZ>(cloudh_cptr,0,255,0,name);
	if(i == 2){
		//viewer->addPolygon<pcl::PointXYZ>(cloudh_cptr,255,0,0,name);
		//viewer->spinOnce(3);
		//viewer->removeShape(name);
	}
}


vector<Vector<2> > densify::get_grid(pcl::PointXYZ mean, pcl::PointCloud<pcl::PointXYZ>::Ptr point, vector<float> plane){
	Vector<3> xAxis,normal, q;
	normal[0] = plane[0]; normal[1] = plane[1]; normal[2] = plane[2];
	xAxis[0] = 1; xAxis[1] = 0; xAxis[2] = 0;
	normal = normal/magnitude(normal);
	if (magnitude(normal - xAxis) == 0){
		q[0] = 0; q[1] = 1; q[2] = 0;
	}
	else{
		q[0] = 1; q[1] = 0; q[2] = 0;
	}	
	U = cross_product(normal, q);
	U = U/magnitude(U);
	V = cross_product(normal, U);
	V = V/magnitude(V);

	vector<Vector<3> > patch;
	vector<Vector<2> > cloud2d;
	double minx = 0,maxx = 0,miny = 0,maxy = 0;	
	for(int i = 0; i < point->size(); i++){
		Vector<2> a;
		Vector<3> l1,l2;
		l1[0] = U[0]; l1[1] = V[0]; l1[2] = (-1 * point->points[i].x) + point->points[0].x;
		l2[0] = U[1]; l2[1] = V[1]; l2[2] = (-1* point->points[i].y) + point->points[0].y; 
		Vector<3> out = cross_product(l1, l2); 
		a[0] = out[0]/out[2];
		a[1] = out[1]/out[2];

		cloud2d.push_back(a);
		if(i ==0){
			minx = a[0]; maxx = a[0];
			miny = a[1]; maxy = a[1];
		}
		else{
			if(a[0] > maxx)
				maxx = a[0];
			else if( a[0] < minx)
				minx = a[0];
			if(a[1] > maxy)
				maxy = a[1];
			else if( a[1] < miny)
				miny = a[1];
		}
	}
	vector<Vector<2> > grid;
	
	for(int j = 0;;j++){
		if(miny + j*patch_size > maxy)
			break;
		for(int k = 0;;k++){
			if(minx + k*patch_size > maxx)
				break;
			for (int i = 0; i < cloud2d.size(); i++){
				//if((cloud2d[i][0] > minx + k*patch_size - patch_size/2) && (cloud2d[i][0] <  minx + k*patch_size + patch_size/2 ) && (cloud2d[i][1] > miny + j*patch_size - patch_size/2) && (cloud2d[i][1] <  miny + j*patch_size + patch_size/2) ){
				Vector<2> temp;
				temp[0] = (minx + k*patch_size);
				temp[1] = (miny + j*patch_size);
				grid.push_back(temp);
				break;
			//	}
			}
		}
	}
	
	return grid;
}

vector <Vector<2> > densify::get_NN(Vector<2> p){
	vector<Vector<2> > NN;
	Vector<2> nn;
	nn[0] = p[0] + patch_size; nn[1] = p[1] + patch_size;
	NN.push_back(nn);
	nn[0] = p[0] + patch_size; nn[1] = p[1] ;
	NN.push_back(nn);
	nn[0] = p[0] + patch_size; nn[1] = p[1] - patch_size;
	NN.push_back(nn);
	nn[0] = p[0]; nn[1] = p[1] - patch_size;
	NN.push_back(nn);
	nn[0] = p[0] - patch_size; nn[1] = p[1] - patch_size;
	NN.push_back(nn);
	nn[0] = p[0] - patch_size; nn[1] = p[1];
	NN.push_back(nn);
	nn[0] = p[0] - patch_size; nn[1] = p[1] + patch_size;
	NN.push_back(nn);
	nn[0] = p[0] ; nn[1] = p[1] + patch_size;
	NN.push_back(nn);
	return NN;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr densify::make_dense(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int keyframeno){
	
	outcloud = cloud;
	vector<float> plane = get_plane(cloud);
	project_to_plane(plane, cloud);
	vector<Vector<2> > grid = get_grid(cloud->points[0], cloud, plane);

	cv::Point2f arr[4] =  {cv::Point2f(0, 0), cv::Point2f(patch_resolution, 0), cv::Point2f(patch_resolution, patch_resolution), cv::Point2f(0, patch_resolution)};
	vector<cv::Point2f> squarePatch (arr, arr + sizeof(arr) / sizeof(arr[0]) );

	int check = 0, fail = 0;
	for(int l = 0; l < grid.size(); l++){
		if(check == 0){
			if (0)
				break;
		}
		else
		check++;
		vector<Vector<3> > patch = get_patch(grid[l], cloud->points[0]);       // ()
		//draw_patch(patch );
		//viewer->spinOnce(100);
		bool patchInIm = 1;
		vector<cv::Point2f> imPatch = project_to_image(dmap->vpKeyFrames[keyframeno]->se3CfromW, patch, Cam, patchInIm);
		if (!patchInIm){
			fail++;
			continue;
		}
		cv::Mat homography = cv::getPerspectiveTransform(imPatch, squarePatch);
		cv::Mat warpedImPatch = warp_patch(&(cv::Mat(imSize[1], imSize[0], CV_8UC3, dmap->vpKeyFrames[keyframeno]->aLevels[0].im.data())), homography);
//	cv::imshow("1", warpedImPatch);
		int onPlaneBeleif = 0;
		/////////////////////////////////// image consistency /////////
		for(int i = 1; i < no_of_frames; i++){
			vector<cv::Point2f> currImPatch = project_to_image(dmap->vpKeyFrames[keyframeno-i]->se3CfromW, patch, Cam, patchInIm);
			if (!patchInIm)
				continue;
			homography = cv::findHomography(currImPatch, squarePatch);
			cv::Mat currWarpedImPatch = warp_patch(&(cv::Mat(imSize[1], imSize[0], CV_8UC3, dmap->vpKeyFrames[keyframeno-i]->aLevels[0].im.data())), homography);                   // get warp_factor
//	cv::imshow("2", currWarpedImPatch);
//	cv::waitKey(1);
			bool matchResult = cross_corelation(currWarpedImPatch, warpedImPatch);
			if (!matchResult && (onPlaneBeleif > no_of_frames*0.4))
				onPlaneBeleif--;
			else if (matchResult)
				onPlaneBeleif++;
			else
				break;
		}
	//	std::cerr<<"  plne   ->"<<onPlaneBeleif<<std::endl;	
	///////////////////////////////////////////////////////////////
		
		if (onPlaneBeleif >= no_of_frames*0.4){
			//draw_patch(patch,1);
			//draw_patch(patch,2);
			//add_3Dpoints(patch, warpedImPatch);
			update_cloud(patch);
			fail = 0;
			vector<Vector<2> > NN = get_NN (grid[l]);
			for(int j = 0; j<NN.size(); j++){
				int add = 0;
				for(int i = 0; i< grid.size(); i++){
					if((grid[i][0] - NN[j][0] <patch_size)&&(grid[i][1] - NN[j][1] <patch_size)){
						add++;
						break;
					}
				}
				if(add == 0){
					grid.push_back(NN[j]);
				}
			}
		}
		else{
			fail++;
		}
	cerr<<grid.size()<<"      "<<l<<"\n";
	}

		//fill_small_holes()
		//find NN and update list
	return outcloud;
}
