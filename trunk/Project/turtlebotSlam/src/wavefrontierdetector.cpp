#include "turtlebotSlam/wavefrontierdetector.h"
#include <vector>
#include "ros/console.h"
namespace wfd {
    const int operations[4][2] = {{1,0}, {-1,0}, {0,1}, {0,-1}};

    WaveFrontierDetector::WaveFrontierDetector(const nav_msgs::OccupancyGrid::ConstPtr& map)
    {
        this->map = map;
        int height = map->info.height;
        int width = map->info.width;
        this->states = new wfdstate*[height];
        for(unsigned int i = 0 ; i < map->info.height ; ++i) {
            this->states[i] = new wfdstate[width];
            std::fill_n(this->states[i], map->info.width, NONE);
        }
    }

    WaveFrontierDetector::~WaveFrontierDetector() {
        for(unsigned int i = 0 ; i < map->info.height ; ++i) {
            delete this->states[i];
        }
        delete states;
    }

    wfdstate WaveFrontierDetector::getState(_pose pose) {
        return (this->states[int(pose.y)][int(pose.x)]);
    }

    void WaveFrontierDetector::setState(_pose pose, wfdstate state) {
        (this->states[int(pose.y)][int(pose.x)]) = state;
    }

    bool WaveFrontierDetector::allowed(int x, int y) {
        int width = this->map->info.width;
        int height = this->map->info.height;
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    bool WaveFrontierDetector::isUnexplored(int x, int y) {
        int width = this->map->info.height;
        return (this->map->data[y*width+x]) == -1;
    }

    bool WaveFrontierDetector::isFrontier(_pose pose) {
        // a point is a frontier, when one neighbour of that point has unexplored regions, but there have to be also explored neighbours
        bool unexplored = false;
        bool explored = false;
        for(unsigned int i = 0 ; i < 4 ; ++i) {
            int nx = pose.x + (operations[i][0]);
            int ny = pose.y + (operations[i][1]);
            if(allowed(nx, ny)) {
                if(isUnexplored(nx, ny)) {
                    unexplored = true;
                } else {
                    explored = true;
                }
            }
        }
        return unexplored && explored;
    }

    std::vector<_pose> WaveFrontierDetector::adj(_pose pose) {
        std::vector<_pose> poses;
        for(unsigned int i = 0 ; i < 4 ; ++i) {
            int nx = pose.x + (operations[i][0]);
            int ny = pose.y + (operations[i][1]);
            _pose npose;
            npose.x = nx;
            npose.y = ny;
            if(allowed(nx, ny)) poses.push_back(npose);
        }
        return poses;
    }

    void WaveFrontierDetector::save(std::vector<_pose> toSave) {
        for(unsigned int i = 0 ; i < toSave.size() ; ++i) {
            this->frontiers.push_back(toSave[i]);
        }
    }

    bool WaveFrontierDetector::hasOpenSpaceNeighbor(_pose pose) {
        // one neighbour with no obstacles
        int width = this->map->info.width;
        float x = pose.x;
        float y = pose.y;
        for(unsigned int i = 0 ; i < 4 ; ++i) {
            if(allowed(x, y) && this->map->data[y*width+x] < 5) return true;
        }
        return false;
    }

    std::vector<_pose> WaveFrontierDetector::wfd(_pose pose) {
        std::queue<_pose> qm;
        qm.push(pose);
        setState(pose, MAP_OPEN_LIST);

        while(!qm.empty()) {
            _pose p = qm.front();
            qm.pop();
            if(getState(p) == MAP_CLOSE_LIST) {
                continue;
            } else if(isFrontier(p)) {
                std::queue<_pose> qf;
                std::vector<_pose> newFrontier;
                qf.push(p);
                setState(p, FRONTIER_OPEN_LIST);

                while(!qf.empty()) {
                    _pose q = qf.front();
                    qf.pop();
                    if(getState(q) == MAP_CLOSE_LIST || getState(q) == FRONTIER_CLOSE_LIST) {
                        ROS_ERROR("continue");
                        continue;
                    }
                    if(isFrontier(q)) {
                        newFrontier.push_back(q);
                        std::vector<_pose> adjlist = adj(q);
                        for(unsigned int i = 0 ; i < adjlist.size() ; ++i) {
                            _pose w = adjlist[i];
                            if(getState(w) != FRONTIER_OPEN_LIST && getState(w) != FRONTIER_CLOSE_LIST && getState(w) != MAP_CLOSE_LIST) {
                                qf.push(w);
                                setState(w, FRONTIER_OPEN_LIST);
                            }
                        }
                    }
                    setState(q, FRONTIER_CLOSE_LIST);
                }
                save(newFrontier);
                for(unsigned int i = 0 ; i < newFrontier.size() ; ++i) {
                    setState(newFrontier[i], MAP_CLOSE_LIST);
                }
            }
            std::vector<_pose> adjlist = adj(p);
            for(unsigned int i = 0 ; i < adjlist.size() ; ++i) {
                _pose v = adjlist[i];
                if(getState(v) != MAP_OPEN_LIST && getState(v) != MAP_CLOSE_LIST && hasOpenSpaceNeighbor(v)) {
                    qm.push(v);
                    setState(v, MAP_OPEN_LIST);
                }
            }
            setState(p, MAP_CLOSE_LIST);
        }
        return this->frontiers;
    }
}
