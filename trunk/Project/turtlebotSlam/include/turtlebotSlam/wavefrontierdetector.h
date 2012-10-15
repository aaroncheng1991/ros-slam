#ifndef WAVEFRONTIERDETECTOR_H
#define WAVEFRONTIERDETECTOR_H

#include <queue>
#include "nav_msgs/OccupancyGrid.h"
namespace wfd {
    typedef  nav_msgs::OccupancyGrid::_info_type::_origin_type::_position_type _pose;

    enum wfdstate {
        MAP_OPEN_LIST,
        MAP_CLOSE_LIST,
        FRONTIER_OPEN_LIST,
        FRONTIER_CLOSE_LIST,
        NONE
    };

    class WaveFrontierDetector
    {
    private:
        nav_msgs::OccupancyGrid::ConstPtr map;
        wfdstate **states;
        std::vector<_pose> frontiers;

        bool isFrontier(_pose pose);
        std::vector<_pose> adj(_pose pose);
        void save(std::vector<_pose> toSave);
        bool hasOpenSpaceNeighbor(_pose pose);
        bool isUnexplored(int x, int y);
        bool allowed(int x, int y);
        wfdstate getState(_pose pose);
        void setState(_pose pose, wfdstate state);
    public:
        WaveFrontierDetector(const nav_msgs::OccupancyGrid::ConstPtr& map);

        /**
         * qm is used for detecting frontier points from a given map
         * qf is used for extracting a frontier from a given frontier cell
         * pose is the actual pose of the robot
         **/
        std::vector<_pose> wfd(_pose pose);
    };
}
#endif // WAVEFRONTIERDETECTOR_H
