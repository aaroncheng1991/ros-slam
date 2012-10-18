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

    enum sorttype {
        DIST_ROBOT,
        DENSEST_CLUSTER
    };

    struct ValuePose {

        ValuePose() : pos(), val(1000000000) {}
        ValuePose(_pose p, double v) : pos(p), val(v) {}

        _pose pos;
        double val;

        bool operator<(const ValuePose& rhs) const { return this->val < rhs.val; }
    };

    struct same_pose
    {
        same_pose(const ValuePose& x): p(x) {}

        ValuePose p;

        bool operator()(const ValuePose& pp) const { return p.pos.x == pp.pos.x && p.pos.y == pp.pos.y; }
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
        ~WaveFrontierDetector();

        /**
         * qm is used for detecting frontier points from a given map
         * qf is used for extracting a frontier from a given frontier cell
         * pose is the actual pose of the robot
         **/
        std::vector<_pose> wfd(_pose pose);
        std::vector<ValuePose> sortFrontiers(sorttype t, double rX, double rY);

        // Distance based on 2D (X,Y) plane, disregarding _pose.z
        static double dist(_pose& lhs, _pose& rhs){
            double dX = lhs.x - rhs.x,
                    dY = lhs.y - rhs.y;
            return sqrt((dX*dX) + (dY*dY));
        }

        static double dist(_pose& lhs, double x, double y){
            double dX = lhs.x - x,
                    dY = lhs.y - y;
            return sqrt((dX*dX) + (dY*dY));
        }
    };
}
#endif // WAVEFRONTIERDETECTOR_H
