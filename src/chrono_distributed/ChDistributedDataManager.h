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


#ifndef CHRONO_DISTRIBUTED_CHDISTRIBUTEDDATAMANAGER_H_
#define CHRONO_DISTRIBUTED_CHDISTRIBUTEDDATAMANAGER_H_

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/other_types.h"

#include "chrono_parallel/ChDataManager.h"

#include <vector>

namespace chrono {

class ChSystemDistr;

class CH_DISTR_API ChDistributedDataManager {
public:
	ChDistributedDataManager(ChSystemDistr *my_sys);
	virtual ~ChDistributedDataManager();

	std::vector<unsigned int> global_id;
	std::vector<distributed::COMM_STATUS> comm_status;

	std::vector<int> ghosts;

	std::vector<int> shape_start;
	std::vector<int> num_shapes;

	ChParallelDataManager *data_manager;
	ChSystemDistr *my_sys;

	int num_sharedup;
	int num_shareddown;
	int num_ghostup;
	int num_ghostdown;

	/// Returns the local index of a body, given its global id
	int GetLocalIndex(unsigned int gid);
};
} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_CHDISTRIBUTEDDATAMANAGER_H_ */
