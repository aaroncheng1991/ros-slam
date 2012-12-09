#include "FastSLAMAlgorithm.h"
#define STANDARD_WEIGHT 0.42
namespace fslam {

int findMaxFeature(std::vector<Feature> features) {
    // TODO, maybe better search function
    int maxIndex = -1;
    for(int i = 0 ; i < features.size() ; ++i) {
        if(maxIndex == -1 || features[i].weight > features[maxIndex].weight) {
            maxIndex = i;
        }
    }
    return maxIndex;
}

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

Eigen::MatrixXd FastSLAMAlgorithm::measurementPrediction(Eigen::Vector2d mean, Eigen::Vector3d x, const sensor_msgs::LaserScan::ConstPtr& scan, Feature &feature) {
    Eigen::VectorXd z;
    double dy = mean[1]-x[1];
    double dx = mean[0]-x[0];
    double alpha = atan2(abs(dy), abs(dx));
    double distance = sqrt(dx*dx+dy*dy);
    double angle = x[2] - alpha;
    if(alpha > x[2]) angle = alpha - x[2];
    feature.inRange = false;
    for(int i = scan->angle_min ; i < scan->angle_max ; i += scan->angle_increment) {
        if(distance >= (scan->range_min) && distance <= (scan->range_max) && angle >= (scan->angle_min) && angle <= (scan->angle_max) && i <= angle && (i + (scan->angle_increment) > angle)) {
            z[i-(scan->angle_min)] = distance;
            feature.inRange = true;
        } else {
            z[i-(scan->angle_min)] = 0;
        }
    }
    //TODO
    //calculate h(mean, x)=h(mean, sampledPose)
    //IN the paper they call it g instead of h, who the fuck knows what they mean

    return z;
}
Eigen::MatrixXd FastSLAMAlgorithm::jacobian(Eigen::Vector3d x,Eigen::Vector2d mean){
    Eigen::VectorXd s = mean - x;
    double de = sqrt(s.norm());
    double phi = atan2(s[1], s[0]);
    Eigen::Matrix3d v;
    v[0][0] = de * cos(phi);
    v[0][1] = -de * sin(phi);
    v[0][2] = v[1][2] = v[2][0] = v[2][1] = 0;
    v[1][0] = de * sin(phi);
    v[1][1] = de * cos(phi);
    v[2][2] = 1;
    return v;
}
Eigen::MatrixXd FastSLAMAlgorithm::measurementCovariance(Eigen::MatrixXd g, Eigen::Vector2d cov){
    //Calculate measurement covariance (G transposed x covariance x G + Rt)
    //Rt is the noise, maybe start without it? We should ask the swarmlab friends
    Eigen::MatrixXd q = g.transpose()*cov*g + Rt;
    return q;
}
double FastSLAMAlgorithm::featureWeight(Eigen::MatrixXd q, Eigen::VectorXd z, Eigen::MatrixXd zprime){
    //Calculate the likelyhood of this feature's correspondance (I think with each detected part of the laserscan)

    double powerexp = -0.5*(z-zprime).transpose() * q.inverse()*(z-zprime);
    double weight = pow((2*M_PI*q).determinant(), 0.5)*exp(powerexp);
    return weight;
}
Eigen::Vector2d FastSLAMAlgorithm::initMean(const sensor_msgs::LaserScan::ConstPtr& scan, Eigen::Vector3d x) {
    //g^-1(zt, xt)
    Eigen::Vector2d mean;
    return mean;
}

void FastSLAMAlgorithm::sensorModel(const sensor_msgs::LaserScan::ConstPtr& scan){
    //TODO: Implement complicated stuff for weighing features 'n shit.
    Eigen::VectorXd zt;
    for(unsigned int i = 0 ; i < scan->ranges.size() ; ++i) {
        zt[i] = scan->ranges[i];
    }
    //For all particles
    for (unsigned int i = 0; i < PARTICLECOUNT; ++i) {
        //For all features in this particle
        for (unsigned int j = 0; j < particles[i].features.size(); ++j) {
            //Measurement prediction step h(mean, pose) (p319?) check p14 of the paper
            Eigen::VectorXd z = measurementPrediction(particles[i].features[j].mean, particles[i].robotPos, scan, particles[i].features[j]);
            particles[i].features[j].measurementPrediction = z;
            //Calculate jacobian with pose and mean
            Eigen::MatrixXd gMatrix = jacobian(particles[i].robotPos,particles[i].features[j].mean);
            particles[i].features[j].jacobian = gMatrix;
            Eigen::MatrixXd qMatrix = measurementCovariance(gMatrix,particles[i].features[j].covariance);
            particles[i].features[j].measurementCovariance = qMatrix;
            double w = featureWeight(qMatrix, zt, z);
            this->particles[i].features[j].weight = w;
        }
        int oldFeatureCount = particles[i].features.size();
        Feature newFeature;
        newFeature.weight = STANDARD_WEIGHT;
        particles[i].features.push_back(newFeature);
        int c = findMaxFeature(particles[i].features);
        int maxWeight = particles[i].features[c].weight;
        int newFeatureCount = std::max(oldFeatureCount, c);

        for (unsigned int j = 0; j < newFeatureCount; ++j) {
            //If new feature
            if(j == oldFeatureCount+1){
                particles[i].features[j].mean = initMean(scan, particles[i].robotPos);
                particles[i].features[j].covariance = jacobian(particles[i].robotPos,particles[i].features[j].mean).inverse()*Rt*jacobian(particles[i].robotPos,particles[i].features[j].mean);
                particles[i].features[j].iterated = 1;
            }
            else if(j == c && c < oldFeatureCount){
                //First, calculate Kalman Gain
                // TODO optimize multiple jacobian calculation, and measurementCovariance
                Eigen::MatrixXd jac = particles[i].features[j].jacobian;
                Eigen::MatrixXd mCov = particles[i].features[j].measurementCovariance;
                Eigen::VectorXd mPred = particles[i].features[j].measurementPrediction;
                Eigen::MatrixXd K = particles[i].features[j].covariance * jac.transpose() * mCov.inverse();
                //Updating the mean
                particles[i].features[j].mean = particles[i].features[j].mean + K * (zt-mPred).transpose();
                //Updated Covariance
                Eigen::MatrixXd KH = K * jac;
                Eigen::MatrixXd I = Eigen::MatrixXd::Identity(KH.rows(), KH.cols());
                particles[i].features[j].covariance= (I-KH) * particles[i].features[j].covariance;
                particles[i].features[j].iterated++;
            }
            else if(!particles[i].features[j].inRange) {
                particles[i].features[j].iterated -= 1;
                if(particles[i].features[j].iterated < 0) {
                    particles[i].features.erase(particles[i].features.begin() + j);
                }
            }
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

    //Normalizing weights
    double sum = 0;
    for (unsigned int i = 0; i < PARTICLECOUNT; ++i) {
        sum+=particles[i].weight;
    }
    for (unsigned int var = 0; var < PARTICLECOUNT; ++var) {
        particles[var].weight = particles[var].weight / sum;
    }
    //Sorting weights from Big to small.
    std::sort(particles.begin(),particles.end());
    //We remove an amount of particles at the end. In this case 15 - the 10 we keep.

    std::vector<Particle> keep;
    std::vector<Particle*> randomParticles;
    for (int i = 0; i < particles.size(); ++i) {
        for (int j = 0; j < int (particles[i].weight*100); ++j) {
            randomParticles.push_back(&particles[i]);
        }
    }
    std::random_shuffle(randomParticles.begin(), randomParticles.end());
    for (int var = 0; var < KEEP_PARTICLES; ++var) {
        keep.push_back(*randomParticles[var]);
    }
    std::sort(keep.begin(),keep.end());

    //We now add the same amount back to the list.
    //For now based on the best particle might decide to add some gaussian noise to these particles.
    //Maybe change initial weight and make it copy random particles based on weight?
    unsigned int toResample = PARTICLECOUNT - KEEP_PARTICLES;
    for (unsigned int i = 0; i < toResample; ++i) {
        fslam::Particle p;
        p.robotPos = keep[0].robotPos;
        p.weight=1.0/PARTICLECOUNT;
        keep.push_back(p);
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

