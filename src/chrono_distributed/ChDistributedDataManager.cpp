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
}

ChDistributedDataManager::~ChDistributedDataManager() {}

// TODO make much better search
int ChDistributedDataManager::GetLocalIndex(unsigned int gid) {
    auto search = gid_to_localid.find(gid);

    /*  int localid = -1;
      if (search != gid_to_localid.end()) {
          localid = search->second;
      }

          for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
              if (global_id[i] == gid && comm_status[i] != distributed::EMPTY) {
                  if (localid == i) {
                      GetLog() << "Found correct local id with map\n";
                  } else {
                      GetLog() << "INCORRECT LOCAL ID FROM MAP\n";
                  }
                  return i;
              }
          }
    */
    if (search != gid_to_localid.end()) {
        return search->second;
    }
    return -1;
}
