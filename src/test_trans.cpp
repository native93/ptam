#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btQuaternion.h>
#include<TooN/TooN.h>
#include<TooN/so3.h>
#include <tf/transform_broadcaster.h>
#include<TooN/se3.h>

using namespace TooN;
int main()
{
//btQuaternion test(btScalar(-0.7583),btScalar(0.0),btScalar(0.0));
//double yaw,pitch,roll;
//mat.getEulerZYX(yaw,pitch,roll);

//printf("%lf\n",yaw);
//btMatrix3x3 rot=btMatrix3x3(btQuaternion(btScalar(0.0),btScalar(0.0),btScalar(1.0),btScalar(0.0)));
static tf::TransformBroadcaster br_new;
tf::Transform transform_new;


btMatrix3x3 mat(0.0,0.0,1.0,-1.0,0.0,0.0,0.0,-1.0,0.0);
btVector
}

