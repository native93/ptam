// This header declares the CamPose class.
// This is not essential: A simple class to export
// the camera trajectory. CamPoses are stored in
// Map to represent the camera trajectory. Each 
// CamPose contains a SE3 representing its worldPose.
//

#ifndef __CAMPOSE_H
#define __CAMPOSE_H

#include <fstream>
#include <TooN/TooN.h>
#include <TooN/se3.h>

using namespace TooN;

class CamPose
{
	public:
  inline CamPose()
	{
		nFrameIdAbs = -1;
		bIsKeyFrame = false;
	};
	
	inline CamPose(int frameId, SE3<> camPose, bool isKF)
	{
		nFrameIdAbs = frameId;
		se3CfromW = camPose;
		bIsKeyFrame = isKF;
	};
		
	int nFrameIdAbs;		// Stores the mnFrameAbs of Tracker
  SE3<> se3CfromW;    // The coordinate frame of this CamPose as a Camera-From-World transformation
  bool bIsKeyFrame;   // Is the Pose corresponding to a keyframe?
	
	
};

inline std::ostream& operator<<(std::ostream& os, const CamPose& rhs)
{
	os << rhs.nFrameIdAbs << std::endl;
	os << rhs.se3CfromW;
	os << rhs.bIsKeyFrame << std::endl;
	return os;
}


inline std::istream& operator>>(std::istream& is, CamPose& rhs)
{
	is >> rhs.nFrameIdAbs;
	is >> rhs.se3CfromW;
	is >> rhs.bIsKeyFrame;
	return is;
}

#endif
