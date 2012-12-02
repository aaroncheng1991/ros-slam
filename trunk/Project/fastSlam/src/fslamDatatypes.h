#ifndef FSLAMDATATYPES_H
#define FSLAMDATATYPES_H
#include <Eigen/Dense>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include <vector>
namespace fslam {

    // Static Declarations
    typedef geometry_msgs::Pose _pose;
    struct Particle;

    struct Feature {
        //Here we place the mean and standard deviation
        Eigen::Vector2d mean;
        Eigen::Vector2d covariance;
        //Iterations used for likelyhood of existance.
        int iterated;
        //Not sure if we need weight or if it's just calculated through i.
        double weight;
        Feature(Eigen::Vector2d mu, Eigen::Vector2d sigma){
            mean = mu;
            covariance = sigma;
            iterated = 1;
        }
    };

    struct Particle {
        double weight;
        std::vector<fslam::Particle> history;
        _pose robotPos;
        std::vector<Feature> features;

        bool operator<(const Particle & rhs) const { return rhs.weight < this->weight;}
    };
}

#endif
