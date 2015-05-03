#include<ptam/waypoints.h>
#include<ptam/RRT.h>
#include<ptam/move_robot_server.h>


#include<ros/ros.h>
#include"engine.h"
int main(int argc,char **argv)
{
Engine *ep;
	if (!(ep = engOpen(""))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
///		return EXIT_FAILURE;
	}

ros::init(argc,argv,"waypoints");

move_robot move(*ep);
RRT rrt(*ep);
waypoints way(*ep);
way.run();
//rrt.run();
//move.run();











return (1);
}



