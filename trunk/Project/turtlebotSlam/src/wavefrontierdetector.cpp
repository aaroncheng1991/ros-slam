#include "turtlebotSlam/wavefrontierdetector.h"
#include <vector>

WaveFrontierDetector::WaveFrontierDetector()
{
}

bool WaveFrontierDetector::isFrontier(wfdpose pose) {
    // TODO
    return true;
}

std::vector<wfdpose> WaveFrontierDetector::adj(wfdpose pose) {
    // TODO
    return std::vector<wfdpose>();
}

void WaveFrontierDetector::save(std::vector<wfdpose> toSave) {
    // TODO
}

bool WaveFrontierDetector::hasOpenSpaceNeighbor(wfdpose pose) {
    // TODO
    return true;
}

void WaveFrontierDetector::wfd(std::queue<wfdpose> qm, std::queue<wfdpose> qf, wfdpose pose) {
    qm = std::queue<wfdpose>();
    qm.push(pose);
    pose.state = MAP_OPEN_LIST;

    while(!qm.empty()) {
        wfdpose p = qm.front();
        if(p.state == MAP_CLOSE_LIST) {
            continue;
        } else if(isFrontier(p)) {
            qf = std::queue<wfdpose>();
            std::vector<wfdpose> newFrontier;
            qf.push(p);
            p.state = FRONTIER_OPEN_LIST;

            while(!qf.empty()) {
                wfdpose q = qf.front();
                if(q.state == MAP_CLOSE_LIST || q.state == FRONTIER_CLOSE_LIST) {
                    continue;
                }
                if(isFrontier(q)) {
                    newFrontier.push_back(q);
                    std::vector<wfdpose> adjlist = adj(q);
                    for(unsigned int i = 0 ; i < adjlist.size() ; ++i) {
                        wfdpose w = adjlist[i];
                        if(w.state != FRONTIER_OPEN_LIST && w.state != FRONTIER_CLOSE_LIST && w.state != MAP_CLOSE_LIST) {
                            qf.push(w);
                            w.state = FRONTIER_OPEN_LIST;
                        }
                    }
                }
                q.state = FRONTIER_CLOSE_LIST;
                save(newFrontier);
                for(int i = 0 ; i < newFrontier.size() ; ++i) {
                    newFrontier[i].state = MAP_CLOSE_LIST;
                }
            }
            std::vector<wfdpose> adjlist = adj(p);
            for(int i = 0 ; i < adjlist.size() ; ++i) {
                wfdpose v = adjlist[i];
                if(v.state != MAP_OPEN_LIST && v.state != MAP_CLOSE_LIST && hasOpenSpaceNeighbor(v)) {
                    qm.push(v);
                    v.state = MAP_OPEN_LIST;
                }
            }
            p.state = MAP_CLOSE_LIST;
        }

    }

}
