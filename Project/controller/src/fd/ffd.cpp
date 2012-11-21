#include "controller/fd/ffd.h"
#include <vector>
#include <set>
#include <algorithm>
#include "ros/console.h"

namespace ffd {

    class angle_sort
    {
        _pose m_origin;
        _pose m_dreference;

        // z-coordinate of cross-product, aka determinant
        static double xp(_pose a, _pose b) { return a.x * b.y - a.y * b.x; }
    public:
        angle_sort(const _pose origin, const _pose reference) {
            this->m_origin = origin;
            _pose dreference;
            dreference.x = reference.x - origin.x;
            dreference.y = reference.y - origin.y;
            this->m_dreference = dreference;
        }
        bool operator()(const _pose a, const _pose b) const
        {
            _pose da, db;
            da.x = a.x - m_origin.x;
            da.y = a.y - m_origin.y;
            db.x = b.x - m_origin.x;
            db.y = b.y - m_origin.y;
            const double detb = xp(m_dreference, db);

            // nothing is less than zero degrees
            if (detb == 0 && db.x * m_dreference.x + db.y * m_dreference.y >= 0) return false;

            const double deta = xp(m_dreference, da);

            // zero degrees is less than anything else
            if (deta == 0 && da.x * m_dreference.x + da.y * m_dreference.y >= 0) return true;

            if (deta * detb >= 0) {
                // both on same side of reference, compare to each other
                return xp(da, db) > 0;
            }

            // vectors "less than" zero degrees are actually large, near 2 pi
            return deta > 0;
        }
    };

    std::vector<_pose> sortPolar(std::vector<_pose> lr, _pose pose) {
        // angles should be sorted
        return lr;
    }

    std::set<_pose, PoseComparer> getLine(_pose p0, _pose p1) {
        // http://www.gocad.org/~caumon/Teach/CG/bresenham.pdf
        std::set<_pose, PoseComparer> ret;
        int dx = p1.x - p0.x;
        int dy = p1.y - p0.y;
        int p = 2 * dy - dx;
        int xk = p0.x;
        int yk = p0.y;
        while(xk <= p1.x) {
            _pose add;
            add.x = xk;
            add.y = yk;
            ret.insert(add);
            xk = xk + 1;
            if(p <= 0) p += 2 * yk;
            else {
                dy = yk + 1;
                p += 2 * dy - 2 * dx;
            }
        }
        return ret;
    }

    void FastFrontierDetection::update(std::vector<_pose> laserreadings) {
        this->lr = laserreadings;
    }

    _pose FastFrontierDetection::findInFrontierDB(_pose pose) {
        for(unsigned int y = 0 ; y < frontiersDB.size() ; ++y) {
            for(unsigned int x = 0 ; x < frontiersDB[y].size() ; ++x) {
                if(pose.y == frontiersDB[y][x].y && pose.x == frontiersDB[y][x].x) {
                    _pose pos;
                    pos.x = x;
                    pos.y = y;
                }
            }
        }
        _pose pos;
        pos.y = -1;
        return pos;
    }

    std::vector<_pose> FastFrontierDetection::frontierDetection(_pose pose) {
        if(lr.empty()) {
            return std::vector<_pose>();
        }
        std::vector<_pose> sorted = sortPolar(lr, pose);
        _pose prev = sorted.front();
        sorted.erase(sorted.begin());
        std::set<_pose, fd::PoseComparer> contour;
        for(std::vector<_pose>::iterator it = sorted.begin() ; it != sorted.end() ; ++it) {
            _pose curr = *it;
            std::set<_pose, PoseComparer> line = getLine(prev, curr);
            contour.insert(line.begin(), line.end());
        }


        ROS_ERROR("before new frontiers");
        std::vector<std::vector<_pose> > newFrontiers;
        prev = *contour.begin();
        contour.erase(contour.begin());
        if(isFrontier(prev)) {
            std::vector<_pose> toAdd;
            newFrontiers.push_back(toAdd);
        }
        ROS_ERROR("after isFrontier special");
        for(std::vector<_pose>::iterator it = sorted.begin() ; it != sorted.end() ; ++it) {
            _pose curr = *it;
            if(!isFrontier(curr)) prev = curr;
            else if(findInFrontierDB(curr).y != -1) prev = curr;
            else if(isFrontier(curr) && isFrontier(prev)) {
                if(!newFrontiers.empty()) {
                    (*newFrontiers.rbegin()).push_back(curr);
                } else {
                    std::vector<_pose> toAdd;
                    toAdd.push_back(curr);
                    newFrontiers.push_back(toAdd);
                }
                prev = curr;
            }
            else {
                std::vector<_pose> toAdd;
                toAdd.push_back(curr);
                newFrontiers.push_back(toAdd);
                prev = curr;
            }
        }
        ROS_ERROR("after iterating sorted");
        std::vector<_pose> activeArea = this->activeArea(pose);
        for(std::vector<_pose>::iterator it = activeArea.begin() ; it != activeArea.end() ; ++it) {
            _pose p = *it;
            if(isFrontier(p)) {
                _pose pos = findInFrontierDB(p);
                std::vector<_pose> f = frontiersDB[pos.y];
                std::vector<_pose> f1(pos.x);
                std::vector<_pose> f2(frontiersDB[pos.y].size()-pos.x);
                for(unsigned int i = 0 ; i < f.size() ; ++i) {
                    if(i <= pos.x) f1.push_back(f[i]);
                    else f2.push_back(f[i]);
                }
                frontiersDB.erase(frontiersDB.begin()+pos.y-1);
                frontiersDB.push_back(f1);
                if(!f2.empty()) frontiersDB.push_back(f2);

            }
        }
        for(std::vector< std::vector<_pose> >::iterator it = newFrontiers.begin() ; it != newFrontiers.end() ; ++it) {
            std::vector<_pose> f = *it;
            int pos = overlaps(f);
            if(pos != -1) {
                std::vector<_pose> mergedFrontier = std::vector<_pose>(f.begin(), f.end());
                std::vector<_pose> existFrontier = frontiersDB[pos];
                for(std::vector<_pose>::iterator fit = existFrontier.begin() ; fit != existFrontier.end() ; ++fit) {
                    int pos = -1;
                    for(unsigned int i = 0 ; i < mergedFrontier.size() ; ++i) {
                        if(mergedFrontier[i].y == (*fit).y && mergedFrontier[i].x == (*fit).x) {
                            pos = i;
                            break;
                        }
                    }
                    if(pos == -1) mergedFrontier.push_back(*fit);
                }
                frontiersDB.erase(frontiersDB.begin()+pos-1);
                frontiersDB.push_back(mergedFrontier);
            }
        }
        std::vector<_pose> ret = *frontiersDB.begin();
        frontiersDB.erase(frontiersDB.begin());
        return ret;
    }

    int FastFrontierDetection::overlaps(std::vector<_pose> f) {
        for(std::vector<_pose>::iterator it1 = f.begin() ; it1 != f.end() ; ++it1) {
            _pose p1 = *it1;
            for(unsigned int i = 0 ; i < frontiersDB.size() ; ++i) {
                for(std::vector<_pose>::iterator it2 = frontiersDB[i].begin() ; it2 != frontiersDB[i].end() ; ++it2) {
                    _pose p2 = *it2;
                    if(p1.x == p2.x && p1.y == p2.y) return i;
                }
            }
        }
        return -1;
    }

    FastFrontierDetection::FastFrontierDetection() {}
}
