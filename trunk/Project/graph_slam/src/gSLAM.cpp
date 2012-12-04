#pragma once

#include "gSLAM.h"
#include "ros/ros.h"

using namespace gslam;
using namespace std;

double gSLAM::CHI = 0.2;        // TODO: Set this through experimentation


/*****************************************
 *  CON- / De- structors Declarations
 *****************************************/

gSLAM::gSLAM(){}
gSLAM::~gSLAM(){}

/*****************************************
 *  Method Declarations
 *****************************************/

Vector* gSLAM::graphslam(Matrix* control, Matrix* measurements){

    Vector*  correspondence = createCorrespondence(control, measurements);
    Matrix* mu = initialize(control);

    pair<Matrix*, Vector*> omega_xi         = linearize(control, measurements, correspondence, mu);
    pair<Matrix*, Vector*> reducedOmega_Xi  = reduce(omega_xi.first, omega_xi.second);
    pair<Vector*, Matrix*> mu_sigma         = solve(reducedOmega_Xi.first, reducedOmega_Xi.second, omega_xi.first, omega_xi.second);

    int features = correspondence->size();

    bool pairEliminated = false;
    do {
        pairEliminated = false;

        for(int a = 0 ; a < features ; a++){
            for(int b = a ; b < features ; b++){
                // Check all possible pairs of map features

                if ( (*correspondence)(a) != b ){ // If A != B then check if correspondence is the same
                    double pi_a_b = correspondenceTest(omega_xi.first, omega_xi.second, mu_sigma.second, a, b);

                    if( pi_a_b > gSLAM::CHI){

                        for(int i = 0 ; i < features ; i++) // The features are the same; we remove feature B
                            if((*correspondence)(i) == b) (*correspondence)(a) = a;

                        // TODO: Maybe we can do the recalculation in the external side of the loop? (This way optimization madness lies)

                        // Update the matrix with the reduced / found features
                        omega_xi         = linearize(control, measurements, correspondence, mu);
                        reducedOmega_Xi  = reduce(omega_xi.first, omega_xi.second);
                        mu_sigma         = solve(reducedOmega_Xi.first, reducedOmega_Xi.second, omega_xi.first, omega_xi.second);

                        pairEliminated = true;

                    }
                }
            }
        }

    } while(pairEliminated);

    return mu_sigma.first;
}

/*****************************************
 *  GRAPH SLAM HIGH LEVEL METHODS
 *****************************************/

Matrix* /* allPreviousMu */ gSLAM::initialize(Matrix* control){
    return NULL;
}

pair<Matrix* /* omega */, Vector* /* xi */> gSLAM::linearize(Matrix* input, Matrix* measurements, Vector* correspondence, Matrix* estimatedPoses){
    return pair<Matrix*, Vector*>();
}

pair<Matrix* /* reducedOmega */, Vector* /* reducedXi */ > gSLAM::reduce(Matrix* omega, Vector* xi){
    return pair<Matrix*, Vector*>();
}

pair<Vector* /* mu */, Matrix* /* sigma */> gSLAM::solve(Matrix* reducedOmega, Vector* reducedXi, Matrix* omega, Vector* xi){
    return pair<Vector*, Matrix*>();
}

double /* Pi_a_b */ gSLAM::correspondenceTest(Matrix* omega, Vector* xi, Matrix* sigma, int featureIdxA, int featureIdxB){
    return NULL;
}


/*****************************************
 *  GRAPH SLAM LOW LEVEL UTILITY METHODS
 *****************************************/

Vector* /* correspondence */ gSLAM::createCorrespondence(Matrix* input, Matrix* measurements){
    return NULL;
}

