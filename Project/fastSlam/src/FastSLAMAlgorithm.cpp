#include "FastSLAMAlgorithm.h"

namespace fslam {
	
	// Static Declarations
    const unsigned int FastSLAMAlgorithm::PARTICLECOUNT = 15;
    const unsigned int FastSLAMAlgorithm::KEEP_PARTICLES = 10;
	// Con- / De- structor Declarations

    FastSLAMAlgorithm::FastSLAMAlgorithm(ros::NodeHandle& nh) : visualization(visualization::Visualization(nh)){

		ROS_ERROR("entered constructor");
		movementnoise=0.0001;

		//initially setting all particles to the map's origin. Since we build our own map we known the initial location on the map we're building around him. The particles will start the spread out
		//in the motion model. We will have no landmarks yet.
        for(unsigned int i=0;i<PARTICLECOUNT;++i) {
            fslam::_pose position;
			position.position.x = 0;
			position.position.y = 0;

			fslam::Particle p;
			p.weight=1.0/PARTICLECOUNT;
			p.robotPos = position;
			particles.push_back(p);
        }

		//Subscribing to the laser topic for the sensor model
        laserSub = nh.subscribe("/base_scan", 1, &FastSLAMAlgorithm::sensorModel, this);
        cmdvelSub = nh.subscribe("/odom",1 , &FastSLAMAlgorithm::motionModel, this);

	}


	// Method Declarations

	std::vector<Particle> FastSLAMAlgorithm::fastSLAM(Eigen::Matrix2d zt, _pose motion, std::vector<Particle> Y){
		//    for(unsigned int k = 0; k < Y.size(); ++k) {
		//        _pose xOld = Y[k].robotPos;
		//        std::vector<Feature> features = Y[k].features;
		//        _pose xNew = sampleNewPose(xOld, m);
		//        for(unsigned int j = 0 ; j < features.size(); ++j) {
		//            Feature feature = features[j];
		//            Eigen::Matrix2d z = measurementPrediction(feature.mean, xNew);
		//            Eigen::Matrix2d H = jacobian(feature.mean, xNew);
		//            // measurement covariance
		//            Eigen::Matrix2d Qt;
		//            Eigen::Matrix2d Q = H * feature.covariance * H.transpose() + Qt; // TODO: WHERE does Qt come from???
		//            // strange calculation: wj = sqrt(2*pi*Q) * ??exp(-0.5*(zt-zj).transpose()*Q*(zt-z))

		//        }
		//    }
		// TODO implement code from line 11 page 461

        std::vector<Particle> ret;

        return ret;
	}

	void FastSLAMAlgorithm::sensorModel(const sensor_msgs::LaserScan::ConstPtr& scan){
        //TODO: Implement complicated stuff for weighing features 'n shit.

		//Finally, resample
        this->resample();
	}

	void FastSLAMAlgorithm::motionModel(const nav_msgs::Odometry msg){
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
        visualization.visualizeParticles(particles);
        lastOdom = msg;
	}

    void FastSLAMAlgorithm::resample(){
        //Sorting weights from Big to small.
        std::sort(particles.begin(),particles.end());
        //We remove an amount of particles at the end. In this case 15 - the 10 we keep.
        unsigned int toResample = PARTICLECOUNT-KEEP_PARTICLES;
        for (unsigned int i = 0; i < toResample; ++i) {
            particles.pop_back();
        }

        //We now add the same amount back to the list.
        //For now based on the best particle might decide to add some gaussian noise to these particles.
        //Maybe change initial weight and make it copy random particles based on weight?
        for (unsigned int i = 0; i < toResample; ++i) {
            fslam::Particle p;
            p.robotPos = particles[0].robotPos;
            p.weight=1.0/PARTICLECOUNT;
            particles.push_back(p);
        }

	}

    void FastSLAMAlgorithm::run() {
        ros::Rate rate(1);
        while(ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

//_pose sampleNewPose(_pose xOld, _pose m) {
    // TODO
    // calculate p(xNew | xOld, m)
//}

//Eigen::Matrix2d measurementPrediction(Eigen::Matrix2d mean, _pose x) {
    // TODO
    // calculate h(mean, x)
    //     =     h(mean, sampledPose)
//}

//Eigen::Matrix2d jacobian(Eigen::Matrix2d mean, _pose x) {
    // TODO
    // calculate jacobian h'(mean, newSampledPoseX)
//}

// zt is measurement pose, m is the motion and Y the fast slam particles
//std::vector<Particle> FastSLAMAlgorithm::fastSLAM(Eigen::Matrix2d zt, _pose m, std::vector<Particle> Y) {
//    for(unsigned int k = 0; k < Y.size(); ++k) {
//        _pose xOld = Y[k].robotPos;
//        std::vector<Feature> features = Y[k].features;
//        _pose xNew = sampleNewPose(xOld, m);
//        for(unsigned int j = 0 ; j < features.size(); ++j) {
//            Feature feature = features[j];
//            Eigen::Matrix2d z = measurementPrediction(feature.mean, xNew);
//            Eigen::Matrix2d H = jacobian(feature.mean, xNew);
//            // measurement covariance
//            Eigen::Matrix2d Qt;
//            Eigen::Matrix2d Q = H * feature.covariance * H.transpose() + Qt; // TODO: WHERE does Qt come from???
//            // strange calculation: wj = sqrt(2*pi*Q) * ??exp(-0.5*(zt-zj).transpose()*Q*(zt-z))

//        }
//    }
    // TODO implement code from line 11 page 461
//}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fastSlam");	// Initiate new ROS node named "fastSlam"
    ros::NodeHandle n;
    fslam::FastSLAMAlgorithm fslam(n);			// Create new fSLAM object
    fslam.run();
}

