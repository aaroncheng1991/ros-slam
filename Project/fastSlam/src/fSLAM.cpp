#include <actionlib/client/simple_action_client.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <cmath>
#include <cstdlib>          // Needed for rand()
#include <ctime>            // Needed to seed random number generator with a time value
#include <fstream>
#include "geometry_msgs/Twist.h"
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
#include "FastSLAMAlgorithm.h"
#include <Eigen/Dense>

class fSLAM {

public:

    fSLAM(ros::NodeHandle& nh) {

        //Eigen::MatrixXd m(10,10);
        //initially setting all particles to the map's origin. Since we build our own map we known the initial location on the map we're building around him. The particles will start the spread out
        //in the motion model. We will have no landmarks yet.
        for(unsigned int i=0;i<PARTICLECOUNT;++i) {
            fslam::_pose position;
            position.x = 0;
            position.y = 0;

            fslam::Particle p;
            p.robotPos = position;
            particles.push_back(p);
        }

        //Subscribing to the laser topic for the sensor model
        laserSub = nh.subscribe("scan", 1, &fSLAM::commandCallback, this);
        cmdvelSub = nh.subscribe("cmd_vel",1 , &fSLAM::motionModel, this);
    }

    void motionModel(const geometry_msgs::TwistConstPtr& cmd_vel) {
        cmd_vel.get();
    }

    void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    }

protected:
    ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic (sensor model)
    ros::Subscriber cmdvelSub; // subscribing to the odometry (Action model)
    ros::Publisher mapPub;
    std::vector<fslam::Particle> particles;

    static const unsigned int PARTICLECOUNT=30;
};
