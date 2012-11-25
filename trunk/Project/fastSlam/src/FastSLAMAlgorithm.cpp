#include "FastSLAMAlgorithm.h"
#include <cmath>
namespace fslam {

_pose sampleNewPose(_pose xOld, _pose m) {
    // TODO
    // calculate p(xNew | xOld, m)
}

_pose measurementPrediction(Eigen::Vector mean, _pose x) {
    // TODO
    // calculate h(mean, x)
    //     =     h(mean, sampledPose)
}

Matrix jacobian(Eigen::Vector mean, _pose x) {
    // TODO
    // calculate jacobian h'(mean, newSampledPoseX)
}

// zt is measurement pose, m is the motion and Y the fast slam particles
std::vector<Particle> FastSLAMAlgorithm::fastSLAM(Eigen::Matrix zt, _pose m, std::vector<Particle> Y) {
    for(unsigned int k = 0; k < Y.size(); ++k) {
        _pose xOld = Y[k].robotPos;
        std::vector<Feature> features = Y[k].features;
        _pose xNew = sampleNewPose(xOld, m);
        for(unsigned int j = 0 ; j < features.size(); ++j) {
            Feature feature = features[j];
            Eigen::Matrix z = measurementPrediction(feature.mean, xNew);
            Eigen::Matrix H = jacobian(feature.mean, xNew);
            // measurement covariance
            Eigen::Matrix Q = H * feature.covariance * H.transpose() + Qt; // TODO: WHERE does Qt come from???
            // strange calculation: wj = sqrt(2*pi*Q) * ??exp(-0.5*(zt-zj).transpose()*Q*(zt-z))

        }
    }
    // TODO implement code from line 11 page 461
}

}
