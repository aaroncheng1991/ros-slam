#ifndef IFRONTIERDETECTION_H
#define IFRONTIERDETECTION_H
#include "nav_msgs/OccupancyGrid.h"
namespace fd {
    typedef  nav_msgs::OccupancyGrid::_info_type::_origin_type::_position_type _pose;
    
    class IFrontierDetection {
        public:
            virtual std::vector<_pose> frontierDetection(_pose pose) = 0;
    };
}

#endif // FRONTIERDETECTION_H
