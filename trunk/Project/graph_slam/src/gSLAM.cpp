#pragma once

#include "gSLAM.h"
#include "ros/ros.h"

// #define EIGEN_NO_DEBUG

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

    MatrixI* correspondenceRef = createCorrespondence(control, measurements);
    Matrix* mu = initialize(*control);

    pair<Matrix*, Vector*> omega_xi         = linearize(control, measurements, correspondenceRef, mu);
    pair<Matrix*, Vector*> reducedOmega_Xi  = reduce(omega_xi.first, omega_xi.second);
    pair<Vector*, Matrix*> mu_sigma         = solve(reducedOmega_Xi.first, reducedOmega_Xi.second, omega_xi.first, omega_xi.second);

    MatrixI& correspondence = *correspondenceRef;
    int features = correspondence.size();

    bool pairEliminated = false;
    do {
        pairEliminated = false;

        for(int a = 0 ; a < features ; a++){
            for(int b = a ; b < features ; b++){
                // Check all possible pairs of map features

                if ( correspondence(a) != b ){ // If A != B then check if correspondence is the same
                    double pi_a_b = correspondenceTest(omega_xi.first, omega_xi.second, mu_sigma.second, a, b);

                    if( pi_a_b > gSLAM::CHI){

                        for(int i = 0 ; i < features ; i++) // The features are the same; we remove feature B
                            if(correspondence(i) == b) correspondence(a) = a;

                        // TODO: Maybe we can do the recalculation in the external side of the loop? (This way optimization madness lies)

                        // Update the matrix with the reduced / found features

                        omega_xi         = linearize(control, measurements, correspondenceRef, mu);
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

Matrix* /* allPreviousMu */ gSLAM::initialize(Matrix& input){
    int rows = input.rows();
    Matrix* muRef = new Matrix(rows+1, 3);
    Matrix& mu = *muRef;

    // Setup initial pose
    mu.row(0).setZero();

    // Initialize the new poses with control data
    for(int i = 0 ; i < rows ; ++i)
        mu.row(i+1) = mu.row(i) + input.row(i);

    return muRef;
}

pair<Matrix* /* omega */, Vector* /* xi */> gSLAM::linearize(Matrix* input, Matrix* measurements, MatrixI* correspondence, Matrix* estimatedPoses){
    pair<Matrix*, Vector*> output;




    return output;
}

pair<Matrix* /* reducedOmega */, Vector* /* reducedXi */ > gSLAM::reduce(Matrix* omega, Vector* xi){
    return pair<Matrix*, Vector*>();
}

pair<Vector* /* mu */, Matrix* /* sigma */> gSLAM::solve(Matrix* reducedOmega, Vector* reducedXi, Matrix* omega, Vector* xi){
    return pair<Vector*, Matrix*>();
}

double /* Pi_a_b */ gSLAM::correspondenceTest(Matrix* omega, Vector* xi, Matrix* sigma, int featureIdxA, int featureIdxB){
    return 0.0;
}


/*****************************************
 *  GRAPH SLAM LOW LEVEL UTILITY METHODS
 *****************************************/

MatrixI* /* correspondence */ gSLAM::createCorrespondence(Matrix* input, Matrix* measurements){

    int cols = input->rows(), rows = measurements->rows();
    MatrixI* correspondence = new MatrixI(rows, cols);
    MatrixI& cor = *correspondence;

    int idx = 0;

    for(int i=0 ; i < rows; ++i)
        for(int j = 0; j < cols; ++j)
            cor(i,j) = idx++;

    return correspondence;
}

