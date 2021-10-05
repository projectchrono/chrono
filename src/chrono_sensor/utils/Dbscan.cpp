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
#include "Dbscan.h"

int DBSCAN::Run(std::vector<vec3f>* V, const float eps, const uint min) {
    // Validate
    if (V->size() < 1)
        return ERROR_TYPE::FAILED;
    if (min < 1)
        return ERROR_TYPE::FAILED;

    // initialization
    this->datalen = (uint)V->size();
    this->visited = std::vector<bool>(this->datalen, false);
    this->assigned = std::vector<bool>(this->datalen, false);
    this->clusters.clear();
    this->noise.clear();
    this->minpts = min;
    this->data = V;
    this->epsilon = eps;

    this->buildKdtree(this->data);

    for (uint pid = 0; pid < this->datalen; ++pid) {
        this->borderset.clear();
        if (!this->visited[pid]) {
            this->visited[pid] = true;

            const std::vector<uint> neightbors = this->regionQuery(pid);
            if (neightbors.size() < this->minpts) {
                continue;
            } else {
                uint cid = (uint)this->clusters.size();
                this->clusters.push_back(std::vector<uint>());
                this->addToBorderSet(pid);
                this->addToCluster(pid, cid);
                this->expandCluster(cid, neightbors);
            }
        }
    }

    for (uint pid = 0; pid < this->datalen; ++pid) {
        if (!this->assigned[pid]) {
            this->noise.push_back(pid);
        }
    }

    this->destroyKdtree();

    return ERROR_TYPE::SUCCESS;
}

void DBSCAN::destroyKdtree() {
    kd_free(this->tree);
}

void DBSCAN::buildKdtree(const std::vector<vec3f>* V) {
    this->tree = kd_create();
    std::shared_ptr<float[]> v(new float[3]);
    for (uint r = 0; r < this->datalen; ++r) {
        for (uint c = 0; c < 3; ++c) {
            v[c] = (float)(*V)[r][c];
        }
        kd_insert(this->tree, v.get(), (void*)&(*V)[r]);
    }
}

std::vector<uint> DBSCAN::regionQuery(const uint pid) const {
    std::vector<uint> neighbors;

    std::shared_ptr<float[]> v(new float[3]);
    for (uint c = 0; c < 3; ++c) {
        v[c] = (float)((*this->data)[pid][c]);
    }

    kdres* presults = kd_nearest_range(this->tree, v.get(), this->epsilon);
    while (!kd_res_end(presults)) {
        vec3f* pch = (vec3f*)kd_res_item(presults, v.get());
        uint pnpid = (uint)(pch - &(*this->data)[0]);
        if (pid != pnpid)
            neighbors.push_back(pnpid);
        kd_res_next(presults);
    }
    kd_res_free(presults);

    return neighbors;
}

void DBSCAN::expandCluster(const uint cid, const std::vector<uint>& neighbors) {
    std::queue<uint> border;
    for (uint pid : neighbors)
        border.push(pid);
    this->addToBorderSet(neighbors);

    while (border.size() > 0) {
        const uint pid = border.front();
        border.pop();

        if (!this->visited[pid]) {
            this->visited[pid] = true;
            const std::vector<uint> pidneighbors = this->regionQuery(pid);

            if (pidneighbors.size() >= this->minpts) {
                this->addToCluster(pid, cid);
                for (uint pidnid : pidneighbors) {
                    if (!this->isInBorderSet(pidnid)) {
                        border.push(pidnid);
                        this->addToBorderSet(pidnid);
                    }
                }
            }
        }
    }
}

void DBSCAN::addToCluster(const uint pid, const uint cid) {
    this->clusters[cid].push_back(pid);
    this->assigned[pid] = true;
}
