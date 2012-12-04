#pragma once

#include "gSLAM.h"
#include "ros/ros.h"

using namespace gslam;
using namespace std;

Vector* gSLAM::graphslam(Matrix* control, Matrix* measurements){

    Vector* correspondence = createCorrespondence(control, measurements);

    Matrix* mu = initialize(control);

    pair<Matrix*, Vector*> omega_xi         = linearize(control, measurements, correspondence, mu);
    pair<Matrix*, Vector*> reducedOmega_Xi  = reduce(omega_xi.first, omega_xi.second);
    pair<Vector*, Matrix*> mu_sigma         = solve(reducedOmega_Xi.first, reducedOmega_Xi.second, omega_xi.first, omega_xi.second);

    boolean pairFound = false;
    do {
        pairFound = false;

        for(int a = 0 ; a < features ; a++){
            for(int b = a ; b < features ; b++){
                // Check all possible pairs of map features

                if ( correspondence(a) != b ){  // TODO check this; is this the non-correspondence test in the algo?
                    double pi_a_b = correspondenceTest(omega_xi.first, omega_xi.second, mu_sigma.second, a, b);

                    if( pi_a_b > gSLAM::CHI){

                        for(int i = 0 ; i < features ; i++) // The features are the same; we remove feature B
                            if(correspondence(i) == b) correspondence(a) = a;

                        // TODO: Maybe we can do the recalculation in the external side of the loop? (This way optimization madness lies)

                        // Update the matrix with the reduced / found features
                        omega_xi         = linearize(control, measurements, correspondence, mu);
                        reducedOmega_Xi  = reduce(omega_xi.first, omega_xi.second);
                        mu_sigma         = solve(reducedOmega_Xi.first, reducedOmega_Xi.second, omega_xi.first, omega_xi.second);

                    }
                } else {
                    double pi_a_b = correspondenceTest(omega_xi.first, omega_xi.second, mu_sigma.second, a, b);

                    if(pi_a_b < gSLAM::CHI)
                        pairFound = true;
                }
            }
        }

    } while(pairFound);

    return mu_sigma.first;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "graphSlam");  // Initiate new ROS node named "fastSlam"
    ros::NodeHandle n;
    gSLAM gslam;
    //fslam::FastSLAMAlgorithm fslam(n);      // Create new fSLAM object
    gslam.testmethod();
    gslam.run();
}
