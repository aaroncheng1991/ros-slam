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
#include "FastSLAMAlgorithm.h"
#include <Eigen/Dense>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include "../../visualization/include/visualizeParticles.h"


class fSLAM {

public:

    fSLAM(ros::NodeHandle& nh) : visualization(visualization::Visualization(nh)) {
         movementnoise=0.0001;
        //initially setting all particles to the map's origin. Since we build our own map we known the initial location on the map we're building around him. The particles will start the spread out
        //in the motion model. We will have no landmarks yet.
        for(unsigned int i=0;i<PARTICLECOUNT;++i) {
            fslam::_pose position;
            position.position.x = 0;
            position.position.y = 0;

            fslam::Particle p;
            p.robotPos = position;
            particles.push_back(p);
        }

        //Subscribing to the laser topic for the sensor model
        laserSub = nh.subscribe("scan", 1, &fSLAM::sensorModel, this);
        cmdvelSub = nh.subscribe("/odom",1 , &fSLAM::motionModel, this);

    }

    void motionModel(const nav_msgs::Odometry msg) {
        ROS_INFO_STREAM("updating");
        //First we calculate the difference between new and old ODOM to get the new pose
        float dx=msg.pose.pose.position.x-lastOdom.pose.pose.position.x;
        float dy=msg.pose.pose.position.y-lastOdom.pose.pose.position.y;

        geometry_msgs::Quaternion dorientation=msg.pose.pose.orientation;
        dorientation.y -= lastOdom.pose.pose.orientation.y;
        dorientation.x -= lastOdom.pose.pose.orientation.x;
        dorientation.z -= lastOdom.pose.pose.orientation.z;
        dorientation.w -= lastOdom.pose.pose.orientation.w;

        boost::normal_distribution<> xPosDistr(dx,movementnoise);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varX(rng, xPosDistr);

        boost::normal_distribution<> yPosDistr(dy,movementnoise);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varY(rng, yPosDistr);

        boost::normal_distribution<> xDistr(dorientation.x,movementnoise);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varOX(rng, xDistr);
        boost::normal_distribution<> yDistr(dorientation.y,movementnoise);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varOY(rng, yDistr);
        boost::normal_distribution<> wDistr(dorientation.w,movementnoise);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varOW(rng, wDistr);
        boost::normal_distribution<> zDistr(dorientation.z,movementnoise);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varOZ(rng, zDistr);



        for(unsigned int i=0;i<PARTICLECOUNT;++i) {
            particles[i].robotPos.position.x += varX();
            particles[i].robotPos.position.y += varY();
            particles[i].robotPos.orientation.y+=varOY();
            particles[i].robotPos.orientation.x+=varOX();
            particles[i].robotPos.orientation.z+=varOZ();
            particles[i].robotPos.orientation.w+=varOW();
        }
        lastOdom = msg;
        visualization.visualizeParticles(particles);
    }

    void sensorModel(const sensor_msgs::LaserScan::ConstPtr& scan) {
        resample();
        //Finally, resample
    }

    void resample() {
        for (unsigned int i = 0; i < PARTICLECOUNT; ++i) {

        }
    }

    void run() {
        ros::Rate rate(1);
        while(ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

protected:
    ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic (sensor model)
    ros::Subscriber cmdvelSub; // subscribing to the odometry (Action model)

    nav_msgs::Odometry lastOdom;
    boost::mt19937 rng;
    std::vector<fslam::Particle> particles;
    float movementnoise;
    static const unsigned int PARTICLECOUNT=30;
    visualization::Visualization visualization;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "fastSlam");
    ros::NodeHandle n;
    fSLAM fastSlam(n);
    fastSlam.run();
}
