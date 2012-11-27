#include "controller/fd/IFrontierDetection.h"
namespace fd {
    // Distance based on 2D (X,Y) plane, disregarding _pose.z
    double dist(_pose& lhs, _pose& rhs){
        double dX = lhs.x - rhs.x,
                dY = lhs.y - rhs.y;
        return sqrt((dX*dX) + (dY*dY));
    }

    double dist(_pose& lhs, double x, double y){
        double dX = lhs.x - x,
                dY = lhs.y - y;
        return sqrt((dX*dX) + (dY*dY));
    }



    // Location of robot as parameter
    std::vector<ValuePose> sortFrontiers(sorttype t, double rX, double rY, std::vector<_pose> &frontiers){
        std::vector<ValuePose> list;
        int s = frontiers.size();

        if(s == 0){
            return list;
        }

        switch(t){
            case DIST_ROBOT : {
                for(int i = 0; i < s; i++){
                    _pose lhs = frontiers[i];
                    double dist = fd::dist(lhs, rX,rY);

                    list.push_back(ValuePose(lhs, dist));
                }
            }; break;
            case DENSEST_CLUSTER : {
                    for(int i = 0; i < s; i++){
                        _pose lhs = frontiers[i];
                        double dist = 0;

                        for(int j = 0; j < s; j++){
                            if(i != j){
                                double distLoc = fd::dist(lhs, frontiers[j]);
                                dist += distLoc < 20 ? -1 : 0;
                            }
                        }

                        list.push_back(ValuePose(lhs, dist));
                    }

                }; break;
            default : break;
        }

        std::sort(list.begin(), list.end());

        return list;
    }
}
