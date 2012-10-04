#include "ParticleFilter.hpp"

// Used for simulateRangeScan
#include "occupancy_grid_utils/ray_tracer.h"
#include <iostream>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>



 /**  
  * Just place all particles on a line along x=y. This should be
  * replaced with something more sensible, like drawing particles
  * from a Gaussian distribution with large variance centered on
  * the supplied initial pose, or just placing them in a regular
  * grid across the map.
  */
  void MyLocaliser::initialisePF( const geometry_msgs::PoseWithCovarianceStamped& initialpose )
  {
	  boost::mt19937 rng; 
	  boost::normal_distribution<> xDistr(initialpose.pose.pose.position.x,2.0);
	  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varX(rng, xDistr);

	  boost::normal_distribution<> yDistr(initialpose.pose.pose.position.y,2.0);
	  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varY(rng, yDistr);

	  boost::normal_distribution<> oDistr(tf::getYaw(initialpose.pose.pose.orientation),2.0);
	  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varO(rng, oDistr);

    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      particleCloud.poses[i].position.x = varX();
      particleCloud.poses[i].position.y = varY();
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(varO());
      particleCloud.poses[i].orientation = odom_quat;
    }
  }
  


  /**
   * Your implementation of this should sample from a random
   * distribution instead of blindly adding the odometry increment. It
   * should also update the angle of the particles.
   */
  void MyLocaliser::applyMotionModel( double deltaX, double deltaY, double deltaT )
  {
    if (deltaX > 0 or deltaY > 0)
      ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      particleCloud.poses[i].position.x += deltaX;
      particleCloud.poses[i].position.y += deltaY;      
    }
  }


  /**
   * After the motion model moves the particles around, approximately
   * according to the odometry readings, the sensor model is used to
   * weight each particle according to how likely it is to get the
   * sensor reading that we have from that pose.
   */
  void MyLocaliser::applySensorModel( const sensor_msgs::LaserScan& scan )
  {
    /* This method is the beginning of an implementation of a beam
     * sensor model */  
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      geometry_msgs::Pose sensor_pose;      
      sensor_pose =  particleCloud.poses[i];
      /* If the laser and centre of the robot weren't at the same
       * position, we would first apply the tf from /base_footprint
       * to /base_laser here. */
      sensor_msgs::LaserScan::Ptr simulatedScan;
      try{
        simulatedScan
          = occupancy_grid_utils::simulateRangeScan
          ( this->map, sensor_pose, scan, true );
      }
      catch (occupancy_grid_utils::PointOutOfBoundsException)
      {
        continue;
      }

      /* Now we have the actual scan, and a simulated version ---
       * i.e., how a scan would look if the robot were at the pose
       * that particle i says it is in. So now we should evaluate how
       * likely this pose is; i.e., the actual sensor model. */
      
      //   if (i == 0)
      //   {
      //     for (unsigned int k = 0; k < simulatedScan->ranges.size(); ++k)
      //       ROS_INFO_STREAM(simulatedScan->ranges[k]);
      //     std::cerr << "\n\n";
      //   }
    }
  }

  
  /**
   * This is where resampling should go, after applying the motion and
   * sensor models.
   */
  geometry_msgs::PoseArray MyLocaliser::updateParticleCloud (const sensor_msgs::LaserScan& scan, const nav_msgs::OccupancyGrid& map 		,const geometry_msgs::PoseArray& particleCloud )
  {
    return this->particleCloud;
  }



  /**
   * Update and return the most likely pose. 
   */
  geometry_msgs::PoseWithCovariance MyLocaliser::updatePose()
  {
    this->estimatedPose.pose.pose = particleCloud.poses[1];
    //TODO: also update the covariance
    return this->estimatedPose.pose;
  }

  



