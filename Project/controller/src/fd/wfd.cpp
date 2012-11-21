#include "controller/fd/wfd.h"
#include "ros/console.h"
#include <vector>
namespace wfd {

    void WaveFrontierDetector::update(nothing n)
    {
        ROS_ERROR("START UPDATE VOID");
        int height = this->map->info.height;
        int width = this->map->info.width;
        this->states = new wfdstate*[height];
        for(unsigned int i = 0 ; i < this->map->info.height ; ++i) {
            this->states[i] = new wfdstate[width];
            std::fill_n(this->states[i], this->map->info.width, NONE);
        }
        ROS_ERROR("END UPDATE VOID");
    }

    WaveFrontierDetector::~WaveFrontierDetector() {
        for(unsigned int i = 0 ; i < this->map->info.height ; ++i) {
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


    void WaveFrontierDetector::save(std::vector<_pose> toSave) {
        for(unsigned int i = 0 ; i < toSave.size() ; ++i) {
            this->frontiers.push_back(toSave[i]);
        }
    }

    std::vector<_pose> WaveFrontierDetector::frontierDetection(_pose pose) {
        ROS_ERROR("START frontierDetection");
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
