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

#pragma once

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"
#include "chrono_distributed/other_types.h"

#include "chrono_parallel/math/other_types.h"
#include "chrono_parallel/ChDataManager.h"

#include <vector>

namespace chrono {

class ChSystemDistributed;

class CH_DISTR_API ChDistributedDataManager {
public:
	ChDistributedDataManager(ChSystemDistributed *my_sys);
	virtual ~ChDistributedDataManager();

	std::vector<unsigned int> global_id; ///< Global id of each body
	std::vector<distributed::COMM_STATUS> comm_status; ///< Communication status of each body


	// TODO
	std::vector<int> num_shapes;	///< The number of collision shapes that each body has
	std::vector<vec3> shape_indices;	///< Maps body index to shape index in data_manager->shape_data
										///< Requires that each body have 3 or fewer contact shapes
	std::vector<bool> free_shapes;	///< Indicates if a shape index is empty TODO

	ChParallelDataManager *data_manager;
	ChSystemDistributed *my_sys;

	std::vector<int> ghosts; // TODO

	int num_sharedup;
	int num_shareddown;
	int num_ghostup;
	int num_ghostdown;

	/// Returns the local index of a body, given its global id
	int GetLocalIndex(unsigned int gid);
};
} /* namespace chrono */
