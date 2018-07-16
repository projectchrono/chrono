// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include "chrono_distributed/ChDistributedDataManager.h"

using namespace chrono;

ChDistributedDataManager::ChDistributedDataManager(ChSystemDistributed* my_sys) {
    this->my_sys = my_sys;
    data_manager = my_sys->data_manager;
    first_empty = 0;
    initial_add = true;
}

ChDistributedDataManager::~ChDistributedDataManager() {}

int ChDistributedDataManager::GetLocalIndex(unsigned int gid) {
    auto search = gid_to_localid.find(gid);
    if (search != gid_to_localid.end()) {
        return search->second;
    }
    return -1;
}

// TODO replace
void ChDistributedDataManager::DefragmentFreeList() {
    struct LocalShapeNode* curr = local_free_shapes;
    if (curr == NULL)
        return;

    while (curr->next != NULL) {
        if (curr->free && curr->next->free) {
            struct LocalShapeNode* new_next = curr->next->next;
            curr->size += curr->next->size;
            delete curr->next;
            curr->next = new_next;
        }
    }
}