#ifndef __DBSCAN_H__
#define __DBSCAN_H__

#include <vector>
#include <queue>
#include <set>
#include <memory>
#include <functional>
#include "Kdtree.h"

/**
 * If big epsilon value and dense points, the kdtree becomes a bottleneck of performance (Han 5/18/2021)
 *
 */

typedef unsigned int uint;

class DBSCAN final {
    enum ERROR_TYPE { SUCCESS = 0, FAILED, COUNT };

  public:
    DBSCAN() {}
    ~DBSCAN() {}
    int Run(std::vector<vec3f>* V, const float eps, const uint min);
    std::vector<std::vector<uint>> getClusters() { return this->clusters; };

  private:
    std::vector<uint> regionQuery(const uint pid) const;
    void addToCluster(const uint pid, const uint cid);
    void expandCluster(const uint cid, const std::vector<uint>& neighbors);
    void addToBorderSet(const uint pid) { this->borderset.insert(pid); }
    void addToBorderSet(const std::vector<uint>& pids) {
        for (uint pid : pids)
            this->borderset.insert(pid);
    }
    bool isInBorderSet(const uint pid) const { return this->borderset.end() != this->borderset.find(pid); }
    void buildKdtree(const std::vector<vec3f>* V);
    void destroyKdtree();

  private:
    std::vector<bool> visited;
    std::vector<bool> assigned;
    std::set<uint> borderset;
    uint datalen;
    uint minpts;
    float epsilon;
    kdtree* tree;
    std::vector<vec3f>* data;
    std::vector<std::vector<uint>> clusters;
    std::vector<uint> noise;
};

#endif  //__DBSCAN_H__