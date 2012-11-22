#include <queue>
#include "nav_msgs/OccupancyGrid.h"


namespace fslam {

    typedef  nav_msgs::OccupancyGrid::_info_type::_origin_type::_position_type _pose;



    struct Particle;

    struct Feature {
        double mean;
        double covariance;
        //Iterations used for likelyhood of existance.
        int iterated;
        //Not sure if we need weight or if it's just calculated through i.
        double weight;
        _pose location;
        Feature(double mu, double sigma, _pose loc) {
            mean = mu;
            covariance = sigma;
            location = loc;
        }


    };

    struct Particle {
        std::vector<fslam::Particle> history;
        _pose robotPos;
        std::vector<Feature> features;
    };



}
