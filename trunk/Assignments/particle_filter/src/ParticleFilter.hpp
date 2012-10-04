#ifndef MMN_PARTICLEFILTER_HPP_1403
#define MMN_PARTICLEFILTER_HPP_1403

#include "MCLocaliser.hpp"

/**
 * This class implements the pure virtual methods of PFLocaliser. It
 * is an "empty" class, in that the methods don't actually update
 * anything. The assignment consists in implementing a class just like
 * this one, but with actual content in the methods.
 */

class MyLocaliser: public MCLocaliser
{
public:

 /**  
  * Just place all particles on a line along x=y. This should be
  * replaced with something more sensible, like drawing particles
  * from a Gaussian distribution with large variance centered on
  * the supplied initial pose, or just placing them in a regular
  * grid across the map.
  */
  virtual void initialisePF( const geometry_msgs::PoseWithCovarianceStamped& initialpose );
  
protected:

  /**
   * Your implementation of this should sample from a random
   * distribution instead of blindly adding the odometry increment. It
   * should also update the angle of the particles.
   */
  virtual void applyMotionModel( double deltaX, double deltaY, double deltaT );

  /**
   * After the motion model moves the particles around, approximately
   * according to the odometry readings, the sensor model is used to
   * weight each particle according to how likely it is to get the
   * sensor reading that we have from that pose.
   */
  virtual void applySensorModel( const sensor_msgs::LaserScan& scan );
  
  /**
   * This is where resampling should go, after applying the motion and
   * sensor models.
   */
  virtual geometry_msgs::PoseArray updateParticleCloud
  ( const sensor_msgs::LaserScan& scan,
    const nav_msgs::OccupancyGrid& map,
    const geometry_msgs::PoseArray& particleCloud );

  /**
   * Update and return the most likely pose. 
   */
  virtual geometry_msgs::PoseWithCovariance updatePose();

  
};

#endif
