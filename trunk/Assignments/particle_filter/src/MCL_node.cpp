

//-------------------------------------------------------------------------//
// INCLUDES

#include </usr/include/eigen3/Eigen/Dense>
using namespace Eigen;

// roscpp
#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf/transform_datatypes.h"

#include "MCLocaliser.hpp"
#include "ParticleFilter.hpp"

//-------------------------------------------------------------------------//
// GLOBAL VARIABLES

// The interface to the particle filter localiser
MCLocaliser*  mcl; 
    
ros::Publisher posePublisher;
ros::Publisher PCPublisher;
ros::Publisher tfPublisher;
tf::TransformBroadcaster* tfBroadcaster;
tf::TransformListener* tfListener;
  
bool mapReceived = false;
sensor_msgs::LaserScan latestScan; 


//-------------------------------------------------------------------------//
// FUNCTIONS

void scanCallback(const sensor_msgs::LaserScan& msg)
{
  latestScan = msg;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  mcl->setInitialPose( msg );
}

void mapCallback( const nav_msgs::OccupancyGrid& msg )
{
  mcl->setMap( msg );
  mapReceived = true;
}

/**
 * This is called whenever a new transform is received, and triggers
 * updating of the particle filter.
 */
void tfCallback( const tf::tfMessage& msg )
{
  if (mapReceived)
  {
    mcl->update( latestScan, msg, ros::Time::now() );
    
    posePublisher.publish( mcl->getPoseStamped() );
    PCPublisher.publish( mcl->getParticleCloud () );

  }
}


void odomCallback( const nav_msgs::Odometry& msg )
{
  if (mapReceived)
  {
    ROS_DEBUG( "got odometry (position %f %f, velocity %f %f) from time %f",
               msg.pose.pose.position.x,
               msg.pose.pose.position.y,
               msg.twist.twist.linear.x,
               msg.twist.twist.linear.y,
               msg.header.stamp.toSec() 
              );
    
    //mcl->update( latestScan, msg, ros::Time::now() );

    
    // posePublisher.publish( mcl->getPoseStamped() );
    // PCPublisher.publish( mcl->getParticleCloud () );
  }
}


//-------------------------------------------------------------------------//
// MAIN

int main( int argc, char** argv )
{
  // Do ROS initialisation. Let ROS know of this node.
  ros::init(argc, argv, "probrob_mcl");
  ros::NodeHandle ros;

  // Initialise the particle filter
  // Replace the following line with a call to your own MCL class 
  mcl = new MyLocaliser();
  // Create publishers to push info to ROS
  posePublisher =
    ros.advertise<geometry_msgs::PoseStamped>("mcl_pose", 100);
  PCPublisher = 
    ros.advertise<geometry_msgs::PoseArray>("particle_cloud", 100);
  tfPublisher =
    ros.advertise<tf::tfMessage>("tf", 100);
  tfBroadcaster = new tf::TransformBroadcaster;
  geometry_msgs::TransformStamped map_odom;
  map_odom.child_frame_id = "/odom";
  map_odom.header.frame_id = "/map";
  map_odom.header.stamp = ros::Time::now();
  tfBroadcaster->sendTransform( map_odom );        
  
  // Create subscribers to listen to messages from ROS
  ros::Subscriber laser_sub =
    ros.subscribe("base_scan", 100, scanCallback);
  ros::Subscriber initial_pose_sub =
    ros.subscribe("initialpose", 100, initialPoseCallback);
  ros::Subscriber map_sub =
    ros.subscribe("map", 1, mapCallback);

  // The tf_sub below listens to the odometry transform that is
  // published by the simulator, and can be used for an odometry
  // motion model. However, tf doesn't contain information about
  // velocity, so for a velocity motion model, you would need to
  // subscribe to "/odom" messages instead, as in the (commented)
  // odom_sub.
  ros::Subscriber tf_sub =
    ros.subscribe("tf", 10, tfCallback);
  ros::Subscriber odom_sub =
    ros.subscribe("odom", 10, odomCallback); 

  // Wait until map received
  ROS_INFO( "Wating for map." );
  while (not mapReceived)
  {
    ros::spinOnce();
    usleep( 0.1 * 1e6 );
  }
  ROS_INFO( "Received map." );

  // Go!
  ros::spin();
  
  return 0;
}



