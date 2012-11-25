#ifndef FASTSLAMALGORITHM_H
#define FASTSLAMALGORITHM_H
#include <vector>
#include <map>
#include <Eigen/src/Core/Matrix.h>
#include "nav_msgs/OccupancyGrid.h"
namespace fslam {

typedef  nav_msgs::OccupancyGrid::_info_type::_origin_type::_position_type _pose;

struct Feature {
    Eigen::Matrix mean;
    Eigen::Matrix covariance;
    //Iterations used for likelyhood of existance.
    int iterated;
    //Not sure if we need weight or if it's just calculated through i.
    double weight;
    _pose location;
    Feature(Eigen::Vector mu, Eigen::Matrix sigma, _pose loc) {
        mean = mu;
        covariance = sigma;
        location = loc;
    }


};

struct Particle {
    _pose robotPos;
    std::vector<Feature> features;
};

class FastSLAMAlgorithm {
public:
    std::vector<Particle> fastSLAM(Eigen::Matrix zt, _pose motion, std::vector<Particle> Y);

};
}
#endif
