/*
This file is part of ``kdtree'', a library for working with kd-trees.
Copyright (C) 2007-2009 John Tsiombikas <nuclear@siggraph.org>
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
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