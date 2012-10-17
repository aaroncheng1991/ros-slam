#ifndef MMN_PARTICLEFILTER_HPP_1403
#define MMN_PARTICLEFILTER_HPP_1403

#include "MCLocaliser.hpp"
#include <boost/random.hpp>

/**
 * This class implements the pure virtual methods of PFLocaliser. It
 * is an "empty" class, in that the methods don't actually update
 * anything. The assignment consists in implementing a class just like
 * this one, but with actual content in the methods.
 */

class MyLocaliser: public MCLocaliser
{
public:

    static const int WEIGHTS_LENGTH = 5, SKIP_SCANS = 3, PARTICLE_RESAMPLING_KEEP=25;

    MyLocaliser( int particleCount = 100 ) : MCLocaliser(particleCount), key(0), skippedScans(0), scanCounter(0), resampleCounter(0), motOffset(2.0) {

        prob = new double*[WEIGHTS_LENGTH];
        validScans = new bool*[WEIGHTS_LENGTH];

        for( int i = 0 ; i < WEIGHTS_LENGTH ; ++i ){
            weights[i] = 1 - (i * (1.0f / WEIGHTS_LENGTH));
            prob[i] = new double[particleCount];
            validScans[i] = new bool[particleCount];
        }

        motNoise = 0.2;
        rotNoise = 0.2;
    }
    virtual ~MyLocaliser(){
        for( int i = 0 ; i < WEIGHTS_LENGTH ; ++i ){
            delete prob[i];
            delete validScans[i];
        }
        delete validScans;
        delete prob;
    }

    int key, skippedScans, scanCounter, resampleCounter;
    bool** validScans;
    double** prob;
    double weights[WEIGHTS_LENGTH];
    double motNoise, rotNoise, motOffset;      // Noise in percentages for Motion and Rotation
    boost::mt19937 rng;

    struct ValuePose{

        ValuePose(geometry_msgs::Pose ps, double v) : pose(ps), val(v){}

        geometry_msgs::Pose pose;
        double val;

        bool operator<(const ValuePose & rhs) const { return this->val < rhs.val;}

    };

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

    void resample(){
       int particleCount = this->particleCloud.poses.size();
       std::vector<ValuePose> keep;

       // Create intermediary datastructure used to sort (Probably could have used a functor object... :-/)
        for(int i=0;i<particleCount;++i){
            if(validScans[key][i]){

                bool validSequence = true;

                double dist = 0;
                for(int k = 0; k < WEIGHTS_LENGTH ; ++k){
                    int prevK = (key - k + WEIGHTS_LENGTH) % WEIGHTS_LENGTH;
                    if(validScans[prevK][i]){
                        dist += prob[prevK][i] * weights[k];
                    } else {
                        validSequence = false;
                    }
                }

                if(validSequence){
                    keep.push_back(ValuePose(particleCloud.poses[i], dist));
                }
            }

            // Reset sampling wheel
            for(int k = 0; k < WEIGHTS_LENGTH ; ++k){
                prob[k][i] = 1;
            }
        }

        // Sort on pose probability
        std::sort(keep.begin(), keep.end());

        // Gather X best particles
        for(int i = 0 ; i < PARTICLE_RESAMPLING_KEEP ; ++i){
            particleCloud.poses[i] = keep[i].pose;
        }

        // The rest of the particles (PARTICLES - X) are reset with a diminishing noise around the X best poses
        int remaining = particleCount - PARTICLE_RESAMPLING_KEEP;

        motOffset *= 0.6;   // Converge

        boost::normal_distribution<> posDistr(0,motOffset);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varPos(rng, posDistr);

        boost::normal_distribution<> rotDistr(0,1.0);
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > varRot(rng, rotDistr);

        while(--remaining >= 0){
            geometry_msgs::Pose pose = keep[remaining % PARTICLE_RESAMPLING_KEEP].pose;

            int i = PARTICLE_RESAMPLING_KEEP + remaining;

            particleCloud.poses[i].position.x = pose.position.x + varPos();
            particleCloud.poses[i].position.y = pose.position.y + varPos();
            particleCloud.poses[i].orientation = tf::createQuaternionMsgFromYaw(  tf::getYaw(pose.orientation) + varRot());
        }

        ROS_ERROR("Resampling done; best found:  %f", keep[0].val);
    }

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
