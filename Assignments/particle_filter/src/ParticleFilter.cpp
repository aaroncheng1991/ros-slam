#include "ParticleFilter.hpp"

// Used for simulateRangeScan
#include "occupancy_grid_utils/ray_tracer.h"
#include <iostream>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <math.h>


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

    boost::normal_distribution<> xDistr(0,motOffset);
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varX(rng, xDistr);

    boost::normal_distribution<> yDistr(0,motOffset);
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
void MyLocaliser::applyMotionModel( double dRX, double dRY, double dRT )
{
    // Y U NO GIVE ME WORKING CODE SHRIEK?
    // First: dRX, dRY are vectors in free space => I could cheat and just get the robots orientation from this
    // Second: dRT jumps when it goes from OoB at a higher level [-PI, PI] and gives me a 2*PI - real DRT, -2*PI + real DRT relatively.
    //          My apologies for the following sanitization policy which will prohibit the code from working on DRT outside of bounds  [-PI, PI].

    //if (dRX != 0 or dRY != 0 or dRT != 0)
        //ROS_ERROR("Before sanitization: %f",dRT);

    dRT = dRT < -M_PI ? dRT + (2*M_PI) : dRT;  // If smaller than -PI, real dRT is positive
    dRT = dRT > M_PI ? dRT - (2*M_PI) : dRT;  // If larger than PI, real dRT is negative

    //if (dRX != 0 or dRY != 0 or dRT != 0)
        //ROS_ERROR("After sanitization: %f::%f -- %f = %f = %f",dRX, dRY, atan2(dRY,dRX), sin(atan2(dRY,dRX)), cos(atan2(dRY,dRX)));
        //ROS_ERROR("A: %f",dRT);

    // Not used as local variables for prototyping; will refactor if performance requires it.

    if (dRX != 0 or dRY != 0 or dRT != 0){
        ++skippedScans;

        // As it is more realistic to model noise on the distance travelled and the delta orientation
        double orientation = dRT, distance = sqrt((dRX*dRX) + (dRY*dRY));

        // We want to add noise based on the percentage of the two parameters;
        double vP = distance * motNoise,
                vT = dRT * rotNoise;

        // Assume a noise normal distribution with sigma of motor noise / rotation noise *
        boost::normal_distribution<> pDistr(0,vP*vP);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varP(rng, pDistr);

        boost::normal_distribution<> oDistr(0,vT*vT);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varT(rng, oDistr);

        for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
        {
            double dP = distance + varP();
            double dT = orientation + varT();


            double theta = tf::getYaw(particleCloud.poses[i].orientation) + dT; // Quaternion creation handles rotation sanitization

            particleCloud.poses[i].position.x += dP * cos(theta);
            particleCloud.poses[i].position.y += dP * sin(theta);

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
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
    // Check if we need to skip the scan
    if(skippedScans < SKIP_SCANS)
        return;

    skippedScans = 0;
    ++scanCounter;
    ROS_ERROR("SCAN # %d", scanCounter);

    ++resampleCounter;      // Resample when we have a full observation wheel
    if(resampleCounter > WEIGHTS_LENGTH){
        resampleCounter = 0;
        resample();
    }

    // Construct normalized mean
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
            validScans[key][i] = false;
            continue;
        }

        validScans[key][i] = true;

        double eS = 0, maxS=0;
        for(unsigned int k = 0; k < simulatedScan->ranges.size(); ++k){
            eS += simulatedScan->ranges[k];

            if(maxS < simulatedScan->ranges[k])
                maxS = simulatedScan->ranges[k];
        }
        eS /= simulatedScan->ranges.size(); // calculate the mean
        eS /= maxS;                         // normalize the mean

        /* Now we have the actual scan, and a simulated version ---
       * i.e., how a scan would look if the robot were at the pose
       * that particle i says it is in. So now we should evaluate how
       * likely this pose is; i.e., the actual sensor model. */

        double dist = 0;
        double sumDistSim = 0, sumDistReal = 0;

        for(unsigned int k = 0; k < simulatedScan->ranges.size(); ++k){
            double normSimObs = simulatedScan->ranges[k]/maxS,    // normalize the random variable X
                    normRealObs = scan.ranges[k]/maxR;            // normalize the random variable Y

            double distSim = normSimObs - eS,       // distance (x - e(X))
                    distReal = normRealObs - eR;    // distance (y - e(Y))

            sumDistSim += distSim * distSim;
            sumDistReal += distReal * distReal;

            dist+= distSim * distReal;
        }

        double sigmaXY = sqrt(sumDistSim) * sqrt(sumDistReal);
        double Pearson_product_moment_correlation_coefficient = fabs( dist / sigmaXY );

        prob[key][i] = 1.0f - Pearson_product_moment_correlation_coefficient;

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

        if(!validScans[key][i])
            continue;

        bool validSequence = true;

        double dist = 0;
        for(int k = 0; k < WEIGHTS_LENGTH ; ++k){
            int prevK = (key - k + WEIGHTS_LENGTH) % WEIGHTS_LENGTH;

            if(validScans[prevK][i]){
                // ROS_ERROR("A - %d=%d :: %f", i, k, distances[(key + k) % WEIGHTS_LENGTH][i]);
                dist += prob[prevK][i] * weights[k];
            } else {
                validSequence = false;
            }
        }

        if(dist < lowestDistance && validSequence){
            bestGuess=i;
            lowestDistance = dist;
            //ROS_ERROR("NEW BEST: %f - %d", dist, i);
        }
    }

    this->estimatedPose.pose.pose = particleCloud.poses[bestGuess];

    return this->estimatedPose.pose;
}





