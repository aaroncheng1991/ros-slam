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
      double xR = initialpose.pose.pose.position.x,
              yR = initialpose.pose.pose.position.y,
             tR = tf::getYaw(initialpose.pose.pose.orientation);

	  boost::mt19937 rng; 
      boost::normal_distribution<> xDistr(0,1.0);
	  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varX(rng, xDistr);

      boost::normal_distribution<> yDistr(0,1.0);
	  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varY(rng, yDistr);

      boost::normal_distribution<> oDistr(0,1.0);
	  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varO(rng, oDistr);

    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      particleCloud.poses[i].position.x = xR + varX();
      particleCloud.poses[i].position.y = yR +  varY();
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( tR + varO());
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
      boost::mt19937 rng;
      boost::normal_distribution<> xDistr(0,(deltaX/10)*(deltaX/10));
      boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varX(rng, xDistr);

      boost::normal_distribution<> yDistr(0,(deltaY/10)*(deltaY/10));
      boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varY(rng, yDistr);

      boost::normal_distribution<> oDistr(0,(deltaT/10)*(deltaT/10));
      boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varO(rng, oDistr);


    if (deltaX > 0 or deltaY > 0 or deltaT > 0){
      ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );

    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
        particleCloud.poses[i].position.x += deltaX + varX();
        particleCloud.poses[i].position.y += deltaY + varY();

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(tf::getYaw(particleCloud.poses[i].orientation)+varO()+deltaT);
        particleCloud.poses[i].orientation = odom_quat;
    }
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

      double eR = 0, maxR = 0;
      for(unsigned int k = 0; k < scan.ranges.size(); ++k){
          eR += scan.ranges[k];

          if(maxR < scan.ranges[k])
              maxR = scan.ranges[k];
      }
      eR /= scan.ranges.size();
      eR /= maxR;

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

      double eS = 0, maxS=0;
      for(unsigned int k = 0; k < simulatedScan->ranges.size(); ++k){
          eS += simulatedScan->ranges[k];

          if(maxS < simulatedScan->ranges[k])
              maxS = simulatedScan->ranges[k];
      }
      eS /= simulatedScan->ranges.size();
      eS /= maxS;

      /* Now we have the actual scan, and a simulated version ---
       * i.e., how a scan would look if the robot were at the pose
       * that particle i says it is in. So now we should evaluate how
       * likely this pose is; i.e., the actual sensor model. */

      double dist = 0;

      for(unsigned int k = 0; k < simulatedScan->ranges.size(); ++k){
          double simCoVar = simulatedScan->ranges[k]/maxS,
                  realCoVar = scan.ranges[k]/maxR;

          //ROS_ERROR("A: %f::%f",simCoVar, realCoVar);

          simCoVar -= eS;
          realCoVar -= eR;

          //ROS_ERROR("B: %f::%f",simCoVar, realCoVar);


          dist+= fabs(simCoVar * realCoVar);
      }

      distances[key][i] = dist;

      //ROS_ERROR("B: %f", distances[key][i]);

         //if (i == 0)
         //{
         //  for (unsigned int k = 0; k < simulatedScan->ranges.size(); ++k)
         //    ROS_INFO_STREAM(simulatedScan->ranges[k]);
         //  std::cerr << "\n\n";
         //}
    }

    key = (key+1)%WEIGHTS_LENGTH;
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
      int particleCount = this->particleCloud.poses.size();
     double lowestDistance = DBL_MAX;
    int bestGuess=0;
    for(unsigned int i=0;i<particleCount;++i){

        if(distances[key][i] < 0.1)
            continue;

        double dist = 0;
        for(int k = 0; k < WEIGHTS_LENGTH ; ++k){
           // ROS_ERROR("A - %d=%d :: %f", i, k, distances[(key + k) % WEIGHTS_LENGTH][i]);
            dist += distances[(key + k) % WEIGHTS_LENGTH][i] * weights[k];
        }

        if(dist < lowestDistance){
            bestGuess=i;
            lowestDistance = dist;
            //ROS_ERROR("NEW BEST: %f - %d", dist, i);
        }
    }

    this->estimatedPose.pose.pose = particleCloud.poses[bestGuess];
    //TODO: also update the covariance
    return this->estimatedPose.pose;
  }

  



