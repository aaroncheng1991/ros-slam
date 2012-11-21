#ifndef WAVEFRONTIERDETECTOR_H
#define WAVEFRONTIERDETECTOR_H

#include <queue>
#include "IFrontierDetection.h"
using namespace fd;

namespace wfd {
    struct nothing {};

    enum wfdstate {
        MAP_OPEN_LIST,
        MAP_CLOSE_LIST,
        FRONTIER_OPEN_LIST,
        FRONTIER_CLOSE_LIST,
        NONE
    };

    class WaveFrontierDetector : public IFrontierDetection<nothing>
    {
    private:
        wfdstate **states;
        std::vector<_pose> frontiers;

        void save(std::vector<_pose> toSave);
        wfdstate getState(_pose pose);
        void setState(_pose pose, wfdstate state);

    public:
        void update(nothing n);
        ~WaveFrontierDetector();

//        /**
//         * qm is used for detecting frontier points from a given map
//         * qf is used for extracting a frontier from a given frontier cell
//         * pose is the actual pose of the robot
//         **/
        virtual std::vector<_pose> frontierDetection(_pose pose);
    };
}
#endif // WAVEFRONTIERDETECTOR_H
