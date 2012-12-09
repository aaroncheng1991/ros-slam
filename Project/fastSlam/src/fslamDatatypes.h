#ifndef FSLAMDATATYPES_H
#define FSLAMDATATYPES_H
#include <Eigen/Dense>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include <vector>
namespace fslam {

    // Static Declarations
    struct Particle;

    struct Feature {
        //Here we place the mean and standard deviation
        Eigen::Vector3d mean;
        Eigen::Vector3d covariance;
        //Iterations used for likelyhood of existance.
        int iterated;
        //Not sure if we need weight or if it's just calculated through i.
        double weight;
        Eigen::VectorXd measurementPrediction;
        Eigen::MatrixXd jacobian;
        Eigen::MatrixXd measurementCovariance;
        bool inRange;

        Feature() { iterated = 0; inRange = false;}
    };

    struct Particle {
        double weight;
        std::vector<fslam::Particle> history;

        Eigen::Vector3d robotPos;
        std::vector<Feature> features;

        bool operator<(const Particle & rhs) const { return rhs.weight < this->weight;}
    };
}

#endif
