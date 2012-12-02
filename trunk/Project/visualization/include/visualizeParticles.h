#ifndef VISUALIZEPARTICLES_H
#define VISUALIZEPARTICLES_H
#include <vector>
#include "../../fastSlam/src/fslamDatatypes.h"
#include <ros/ros.h>
namespace visualization {
class Visualization {
private:
    ros::Publisher marker_pub;
public:
    Visualization(ros::NodeHandle &n);
    void visualizeParticles(std::vector<fslam::Particle> particles);
};

}
#endif
