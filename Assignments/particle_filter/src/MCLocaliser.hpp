#ifndef MMN_PFLOCALISER_HPP_4918
#define MMN_PFLOCALISER_HPP_4918


#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf/transform_datatypes.h"

/**
 * Abstract base class for a particle-filter localiser. You should
 * implement a class that inherits from this one for your localisation
 * module.
 */
class MCLocaliser
{
public:
  /** Default constructor */
  MCLocaliser( int particleCount = 100 );

  /** Destructor */
  virtual ~MCLocaliser(){}
  
  void setInitialPose( const geometry_msgs::PoseWithCovarianceStamped& pose );

  geometry_msgs::PoseArray getParticleCloud()
  { return this->particleCloud; }

  tf::tfMessage getTransform()
  { return this->tf; }

  geometry_msgs::PoseStamped getPoseStamped();

  void setTransform( const tf::tfMessage& tf )
  { this->tf = tf; }

  void setMap( const nav_msgs::OccupancyGrid& map );

  /**
   * Called whenever there is a new LaserScan message.  This calls
   * update methods (implemented by your subclass) to do actual
   * particle filtering, given the map and the LaserScan, and then
   * updates Transform tf appropriately.
   * @param scan Laser reading
   * @param odom Current odometry reading
   * @param currentTime ROS /clock time if using simulation (e.g. in
   * stage), or system time.
   */
  void update( const sensor_msgs::LaserScan& scan
               , const tf::tfMessage& odom
               , const ros::Time& currentTime
               );


  
protected:
  geometry_msgs::PoseWithCovarianceStamped estimatedPose;
  nav_msgs::OccupancyGrid map;
  geometry_msgs::PoseArray particleCloud;
  tf::tfMessage tf;

  double prevX, prevY; // Previous odometry (position from origin)
  double prevT; // Previous odometry (heading angle from origin)

  /**
   * Called whenever an initialpose message is received (to change the starting
   * location of the robot), or a new map is received.
   * @param initialpose the initial pose estimate
   * @return PoseArray object containing ArrayList of Poses,
   */
  virtual 
  void
  initialisePF
  ( const geometry_msgs::PoseWithCovarianceStamped& initialpose)
  = 0;

  /**
   * This should take in a laser scan, map, and current particle cloud.
   * After updating the particle cloud according to the particle filter and
   * the comparison between the observations and the model, it should then
   * return the particle cloud as a PoseArray.
   * @param scan LaserScan message
   * @param map OccupancyGrid describing the world in which the robot is located
   * @param particlecloud PoseArray containing ArrayList of Poses
   * describing current particle cloud 
   * @return Updated PoseArray of particles
   */
  virtual geometry_msgs::PoseArray updateParticleCloud
  ( const sensor_msgs::LaserScan& scan,
    const nav_msgs::OccupancyGrid& map,
    const geometry_msgs::PoseArray& particleCloud )
  = 0;    

  /**
   * Given a particle cloud, this should calculate and return an
   * updated robot pose estimate.
   * @return Pose describing robot's estimated
   * position and orientation.
   */  
  virtual geometry_msgs::PoseWithCovariance updatePose()
  = 0;

  geometry_msgs::PoseWithCovarianceStamped updatePoseStamped();


  tf::tfMessage updateTf
  ( const geometry_msgs::PoseWithCovarianceStamped& pose,
    const tf::tfMessage& tf,
    const ros::Time& currentTime
    );

  void updateOdom( double x, double y );

  /**    
   * Given an odometry increment for X, Y position and angle, apply an
   * odometry motion model to all particles.   
   * \param deltaX Difference in X position
   * \param deltaY Difference in Y position
   * \param deltaT Difference in heading angle 
   */
  virtual void applyMotionModel( double deltaX, double deltaY, double deltaT )
  = 0;

  /**    
   * Given an odometry increment for X, Y position and angle, apply an
   * odometry motion model to all particles.   
   * \param deltaX Difference in X position
   * \param deltaY Difference in Y position
   * \param deltaT Difference in heading angle 
   */
  virtual void applySensorModel( const sensor_msgs::LaserScan& scan )
  = 0;
  
};

#endif
