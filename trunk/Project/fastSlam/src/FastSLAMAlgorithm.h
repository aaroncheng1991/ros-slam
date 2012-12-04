#ifndef FASTSLAMALGORITHM_H
#define FASTSLAMALGORITHM_H
#include <vector>
#include <map>
#include <Eigen/Dense>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <cmath>
#include <cstdlib>          // Needed for rand()
#include <ctime>            // Needed to seed random number generator with a time value
#include <fstream>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/GetPlan.h>
#include "std_msgs/Int32.h"
#include <ostream>
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "../../visualization/include/visualizeParticles.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

namespace fslam {

	class FastSLAMAlgorithm {

		public:

			// Static definition
                //Amount of particles in total
			static const unsigned int PARTICLECOUNT;
                //Amount of particles to keep from previous iteration
            static const unsigned int KEEP_PARTICLES;
			// Con- / De- structors

            FastSLAMAlgorithm(ros::NodeHandle& nh);

            // Methods
            Eigen::MatrixXd measurementPrediction(Eigen::Vector2d mean, Eigen::Vector3d x, const sensor_msgs::LaserScan::ConstPtr& scan, Feature &feature);
            Eigen::MatrixXd jacobian(Eigen::Vector3d x,Eigen::Vector2d mean);
            Eigen::MatrixXd measurementCovariance(Eigen::MatrixXd g, Eigen::Vector2d cov);
            Eigen::Vector2d initMean(const sensor_msgs::LaserScan::ConstPtr& scan, Eigen::Vector3d x);
            double featureWeight(Eigen::MatrixXd q, Eigen::VectorXd z, Eigen::MatrixXd prediction);
            std::vector<Particle> fastSLAM(Eigen::Matrix2d zt, Eigen::Vector3d motion, std::vector<Particle> Y);
			void sensorModel(const sensor_msgs::LaserScan::ConstPtr& scan);
			void motionModel(const nav_msgs::Odometry msg);
			void resample();
            void run();

		protected:

			// Variables

			std::vector<fslam::Particle> particles;
			nav_msgs::Odometry lastOdom;
			ros::Subscriber laserSub;		// Subscriber to the simulated robot's laser scan topic (sensor model)
			ros::Subscriber cmdvelSub;		// subscribing to the odometry (Action model)
			float movementnoise;
            boost::mt19937 rng;
            Eigen::MatrixXd Rt;
            visualization::Visualization visualization;
	};


}

#endif
