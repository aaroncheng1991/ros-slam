#ifndef IFRONTIERDETECTION_H
#define IFRONTIERDETECTION_H
#include "nav_msgs/OccupancyGrid.h"
namespace fd {
    typedef  nav_msgs::OccupancyGrid::_info_type::_origin_type::_position_type _pose;

    struct PoseComparer {
        bool operator() (_pose const &x, _pose const &y) {
            return compare(x, y);
        }

        bool compare(_pose x, _pose y) {
            if(x.y < y.y) return true;
            if(x.x < y.x) return true;
            return false;
        }
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



    enum sorttype {
        DIST_ROBOT,
        DENSEST_CLUSTER
    };


    double dist(_pose& lhs, _pose& rhs);

    double dist(_pose& lhs, double x, double y);

    std::vector<ValuePose> sortFrontiers(sorttype t, double rX, double rY, std::vector<_pose> &frontiers);

    const int operations[8][2] = {{1,0}, {-1,0}, {0,1}, {0,-1},{1,1},{-1,1},{1,-1},{-1,-1}};

    template <class T>
    class IFrontierDetection {
    protected:
        nav_msgs::OccupancyGrid::ConstPtr map;
    public:
        virtual std::vector<_pose> frontierDetection(_pose pose) = 0;

        void update(T newdata);

        void updateMap(const nav_msgs::OccupancyGrid::ConstPtr& map) {
            this->map = map;
        }


        bool allowed(int x, int y) {
            int width = this->map->info.width;
            int height = this->map->info.height;
            return x >= 0 && x < width && y >= 0 && y < height;
        }

        bool isUnexplored(int x, int y) {
            int width = this->map->info.width;
            return (this->map->data[y*width+x]) == -1;
        }

        bool isFrontier(_pose pose) {
            // a point is a frontier, when one neighbour of that point has unexplored regions, but there have to be also explored neighbours
            bool unexplored = false;

            for(unsigned int i = 0 ; i < 8 ; ++i) {
                int nx = pose.x + (operations[i][0]);
                int ny = pose.y + (operations[i][1]);
                if(allowed(nx, ny)) {
                    if(isUnexplored(nx, ny)) {
                        unexplored = true;
                    }
                }
            }
            int width = this->map->info.width;
            return unexplored &&  this->map->data[pose.y*width+pose.x] == 0;
        }

        std::vector<_pose> adj(_pose pose) {
            std::vector<_pose> poses;
            for(unsigned int i = 0 ; i < 8 ; ++i) {
                int nx = pose.x + (operations[i][0]);
                int ny = pose.y + (operations[i][1]);
                _pose npose;
                npose.x = nx;
                npose.y = ny;
                if(allowed(nx, ny)) poses.push_back(npose);
            }
            return poses;
        }

        bool hasOpenSpaceNeighbor(_pose pose) {
            // one neighbour with no obstacles
            int width = this->map->info.width;
            std::vector<_pose> adja = adj(pose);
            for(unsigned int i = 0 ; i < adja.size() ; ++i) {
                if(this->map->data[adja[i].y*width+adja[i].x]==0) return true;
            }
            return false;
        }

        std::vector<_pose> activeArea(_pose pose) {
            std::vector<_pose> ret;
            for(int x = pose.x - 20 ; x < pose.x + 20 ; ++x) {
                for(int y = pose.y - 20 ; y < pose.y + 20 ; ++y) {
                    if(allowed(x,y)) {
                        _pose npose;
                        npose.x = x;
                        npose.y = y;
                        ret.push_back(npose);
                    }
                }
            }
            return ret;
        }

    };
}

#endif // FRONTIERDETECTION_H
