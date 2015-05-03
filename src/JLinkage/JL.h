#include "RandomSampler.h"
#include "JLinkage.h"

#include <ctime>
#include <cvd/thread.h>
#include "densify.h"

using namespace TooN;

struct dataPoint{
	std::vector<float> *val;
	MapPoint *mp;
	int indX;
	sPtLnk *clP;
	int processed;
};

class JL : protected CVD::Thread{

	public:

		JL(Map *iMap, ATANCamera *iCam, int inSample, int isamplingType, float isamplingCoef, float iinlierThreshold, int iKdTreeRAnge, int minpts, double iKdTreeCloseProb = 1.0, double iKdTreeFarProb = 1.0 );
		~JL();
virtual void run(); 
	private:
		                      			//thread    
		         
		/// COMMON
		int padded;
		Map *jMap;
		ATANCamera *Cam;
		int minPts;

		std::vector<dataPoint*> mDataPoints; // Data Points
		void add_points();
		void update_point(dataPoint*);
		void load_points();
		void delete_point(dataPoint *dP, int i);
		/// SAMPLE		
		
		int samplingType;				//sampling type
		int nSample;     			        //   no of samples
		int mMSS;                                   //   PLANE
		float samplingCoef; 				//sampling variables

		double KdTreeCloseProb;				// kdtree search
		double KdTreeFarProb;				// kdtree search
		
		std::vector<std::vector<float> *> *sample(); 
		
		densify denser;

		RandomSampler mRandomSampler;
		
		//// CLUSTER				
		
		int KdTreeRange;				//clustering variable
		int planepointsthresh;				//min points in cluster
		float inlierThreshold;				//threshold for inliers

		JLinkage mJLinkage;

		/// HELPER FUNCTIONS		
		
		
};
