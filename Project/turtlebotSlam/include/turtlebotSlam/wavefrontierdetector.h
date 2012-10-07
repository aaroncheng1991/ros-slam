#ifndef WAVEFRONTIERDETECTOR_H
#define WAVEFRONTIERDETECTOR_H

#include <queue>
#include "nav_msgs/MapMetaData.h"
typedef nav_msgs::MapMetaData::_origin_type _pose;

enum wfdstate {
    MAP_OPEN_LIST,
    MAP_CLOSE_LIST,
    FRONTIER_OPEN_LIST,
    FRONTIER_CLOSE_LIST
};

struct wfdpose {
    _pose pose;
    wfdstate state;
};

class WaveFrontierDetector
{
private:
    bool isFrontier(wfdpose pose);
    std::vector<wfdpose> adj(wfdpose pose);
    void save(std::vector<wfdpose> toSave);
    bool hasOpenSpaceNeighbor(wfdpose pose);
public:
    WaveFrontierDetector();

    /**
     * qm is used for detecting frontier points from a given map
     * qf is used for extracting a frontier from a given frontier cell
     * pose is the actual pose of the robot
     **/
    void wfd(std::queue<wfdpose> qm, std::queue<wfdpose> qf, wfdpose pose);
};

#endif // WAVEFRONTIERDETECTOR_H
