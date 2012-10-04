
#include "MCLocaliser.hpp" 

#include <tf/transform_broadcaster.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace tf;

MCLocaliser::MCLocaliser( int particleCount )
{
  this->particleCloud.poses.resize( particleCount );

  this->estimatedPose.pose.pose.position.x = 0;
  this->estimatedPose.pose.pose.position.y = 0;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  this->estimatedPose.pose.pose.orientation = odom_quat;

  this->prevX = 0;
  this->prevY = 0;
  this->prevT = 0;
}

void MCLocaliser::setMap( const nav_msgs::OccupancyGrid& map )
{
  this->map = map;

  // New map, so we need to re-initialise
  initialisePF( estimatedPose );
}


void MCLocaliser::update( const LaserScan& scan
                          , const tfMessage& transform
                          //, const tf::StampedTransform& transform
                          , const ros::Time& currentTime
                          )
{
  // Compute odometry increment
  double deltaX = 0;
  double deltaY = 0;
  double deltaT = 0; 
  for (unsigned int i = 0; i < transform.transforms.size(); ++i)
  {
    geometry_msgs::TransformStamped o = transform.transforms[i];

    if (o.header.frame_id == "/odom")
    {
      // OK, found transform from odom origin to the robot base
      ROS_DEBUG( "update pose with msg from time %f", o.header.stamp.toSec() );
      
      deltaX = o.transform.translation.x - this->prevX;
      deltaY = o.transform.translation.y - this->prevY;
      deltaT = tf::getYaw( o.transform.rotation ) - this->prevT;

      this->prevX = o.transform.translation.x;
      this->prevY = o.transform.translation.y;
      this->prevT = tf::getYaw( o.transform.rotation ); 
        
      // Skip rest of the tf graph
      break;
    }
    
  }


  // Call methods defined in inheriting class, to apply motion and
  // sensor models.
  this->applyMotionModel( deltaX, deltaY, deltaT );
  this->applySensorModel( scan );
  this->updateParticleCloud(scan, map, particleCloud);

  // Update the most likely pose (the output of the algorithm)
  this->updatePose();
  
  // Insert timestamp and tf frames (making sure that the poses in the
  // particle cloud and the estimated pose relate to the fixed
  // coordinate frame "/map").
  this->particleCloud.header.stamp = currentTime;
  this->particleCloud.header.frame_id = "/map";
  this->estimatedPose.header.stamp = currentTime;
  this->estimatedPose.header.frame_id = "/map";
}


geometry_msgs::PoseWithCovarianceStamped MCLocaliser::updatePoseStamped()
{
  PoseWithCovarianceStamped p;
  p.pose = this->updatePose(  );
  return p;
}


void MCLocaliser::setInitialPose
( const geometry_msgs::PoseWithCovarianceStamped& pose )
{  
  this->estimatedPose =  pose ;
  this->initialisePF( pose );
}



geometry_msgs::PoseStamped MCLocaliser::getPoseStamped()
{
  geometry_msgs::PoseStamped p;
  p.header = this->estimatedPose.header;
  p.pose = this->estimatedPose.pose.pose;
  // TODO subtract /odom
  return p;    
}
