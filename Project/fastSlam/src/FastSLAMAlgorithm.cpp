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
            Eigen::Vector3d position;
            position[0] = 0;
            position[1] = 0;

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

    Eigen::MatrixXd FastSLAMAlgorithm::measurementPrediction(Eigen::Vector2d mean, Eigen::Vector3d x) {
        Eigen::MatrixXd z;
        //TODO
         //calculate h(mean, x)=h(mean, sampledPose)
        //IN the paper they call it g instead of h, who the fuck knows what they mean
        return z;
    }
    Eigen::MatrixXd FastSLAMAlgorithm::jacobian(Eigen::Vector3d x,Eigen::Vector2d mean){
        Eigen::MatrixXd g;
        //TODO
         //calc jacobians lol

        return g;
    }
    Eigen::MatrixXd FastSLAMAlgorithm::measurementCovariance(Eigen::MatrixXd g, Eigen::Vector2d cov){
        //Calculate measurement covariance (G transposed x covariance x G + Rt)
        //Rt is the noise, maybe start without it?
        Eigen::MatrixXd q;
        return q;
    }
    double FastSLAMAlgorithm::featureWeight(Eigen::MatrixXd q, const sensor_msgs::LaserScan::ConstPtr& scan, Eigen::MatrixXd prediction){
        //Calculate the likelyhood of this feature's correspondance (I think with each detected part of the laserscan)
        double weight;
        return weight;
    }
    Eigen::Vector2d FastSLAMAlgorithm::initMean(const sensor_msgs::LaserScan::ConstPtr& scan, Eigen::Vector3d x) {
        //g^-1(zt, xt)
        Eigen::Vector2d mean;
        return mean;
    }

    std::vector<Particle> FastSLAMAlgorithm::fastSLAM(Eigen::Matrix2d zt, Eigen::Vector3d motion, std::vector<Particle> Y){
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

        //For all particles
        for (unsigned int i = 0; i < PARTICLECOUNT; ++i) {
            //For all features in this particle
            for (unsigned int j = 0; j < particles[i].features.size(); ++j) {
                //Measurement prediction step h(mean, pose) (p319?)
                Eigen::MatrixXd z = measurementPrediction(particles[i].features[j].mean, particles[i].robotPos);
                //Calculate jacobian with pose and mean
                Eigen::MatrixXd gMatrix = jacobian(particles[i].robotPos,particles[i].features[j].mean);
                Eigen::MatrixXd qMatrix = measurementCovariance(particles[i].robotPos,particles[i].features[j].mean);
                double w = featureWeight(qMatrix, scan, z);
                this->particles[i].features[j].weight = w;
            }
            int oldFeatureCount = particles[i].features.size();
            //Part I don't really understand. Weighing of new landmark? landmark <-> feature. only one per iteration of the algo?
            //Somewhere here, features should be added to the feature list for this particle.
            //skipping to the next loop

            for (unsigned int j = 0; j < particles[i].features.size(); ++j) {
                //If new feature
                if(j > oldFeatureCount-1){
                    particles[i].features[j].mean = initMean(scan, particles[i].robotPos);
                    //particles[i].features[j].covariance =
                    //  inverse(jacobian(particles[i].robotPos,particles[i].features[j].mean))*noise*jacobian(particles[i].robotPos,particles[i].features[j].mean);
                    //Iterator is already initiated when creating a new feature
                }
                //else if(Observed feature){
                    //Not sure yet how to detect if it was observed, it is done in the part I skipped earlier I think
                    //First, calculate Kalman Gain
                    //Eigen::MatrixXd K = particles[i].features[j].covariance*jacobian*inverse(measurementCovariance())
                    //Updating the mean
                    //particles[i].features[j].mean = particles[i].features[j].mean * transposed(K(zt-zn))
                    //Updated Covariance
                    //particles[i].features[j].covariance= ((Identity matrix)-K*transposed(jacobian)) * particles[i].features[j].covariance
                    //particles[i].features[j].iterator++;
                //}
                //else if(not observed but should have been)
                    //particles[i].features[j].iterated -= 1;
                    //if(particles[i].features[j].iterated < 0) {
                    //  particles[i].features.removeAt(j); removing dubious feature, this code won't work btw, just pseudocode.
                    //}
                //}
            }

        }
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
            geometry_msgs::Quaternion prevQ = tf::createQuaternionMsgFromYaw(particles[i].robotPos[2]);
            prevQ.y+=varOY();
            prevQ.x+=varOX();
            prevQ.z+=varOZ();
            prevQ.w+=varOW();
            particles[i].robotPos[0] += varX();
            particles[i].robotPos[1] += varY();
            particles[i].robotPos[2] = tf::getYaw(prevQ);

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

