#ifndef FASTSLAMALGORITHM_H
#define FASTSLAMALGORITHM_H
#include <vector>
#include <map>
#include <Eigen/Dense>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <Eigen/Dense>

namespace fslam {

    typedef  geometry_msgs::Pose _pose;

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
    };

class FastSLAMAlgorithm {
public:
    std::vector<Particle> fastSLAM(Eigen::Matrix zt, _pose motion, std::vector<Particle> Y);

};
}
#endif
