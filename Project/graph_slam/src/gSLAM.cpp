#include "ros/ros.h"
#include <Eigen/Dense>
class gSLAM{

	public:
        Eigen::Matrix2d test(3,3);

				void testmethod(){	
					ROS_ERROR("testerdetest %f", test.coef(1,1);
				}

				void run() {
				 	ros::Rate rate(1);
				 	while(ros::ok()) {
					 	ros::spinOnce();
					  rate.sleep();
				 	}
			 	}
};

int main(int argc, char **argv){ 
	ros::init(argc, argv, "graphSlam");  // Initiate new ROS node named "fastSlam"
	ros::NodeHandle n;
	gSLAM gslam;
	//fslam::FastSLAMAlgorithm fslam(n);      // Create new fSLAM object
	gslam.testmethod();
	gslam.run();
}
