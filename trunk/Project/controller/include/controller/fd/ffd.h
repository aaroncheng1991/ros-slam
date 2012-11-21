#ifndef FFD_H
#define FFD_H

#include <vector>
#include <set>
#include "IFrontierDetection.h"
using namespace fd;

namespace ffd {
    class FastFrontierDetection : public IFrontierDetection< std::vector<_pose> > {
    private:
        std::vector<_pose> lr;
        std::vector<std::vector<_pose> > frontiersDB;
        _pose findInFrontierDB(_pose pose);
        int overlaps(std::vector<_pose> f);
    public:
        FastFrontierDetection();
        std::vector<_pose> frontierDetection(_pose pose);
        void update(std::vector<_pose> laserreadings);
    };
}

#endif
