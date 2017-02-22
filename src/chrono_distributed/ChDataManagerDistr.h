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

#ifndef CHRONO_DISTRIBUTED_COLLISION_CHDATAMANAGERDISTR_H_
#define CHRONO_DISTRIBUTED_COLLISION_CHDATAMANAGERDISTR_H_

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

#include "chrono_parallel/ChDataManager.h"

#include <memory>

namespace chrono {

class ChSystemDistr;

class CH_DISTR_API ChDataManagerDistr : public ChParallelDataManager {
public:
	ChDataManagerDistr(ChSystemDistr *my_sys);
	virtual ~ChDataManagerDistr();

	std::vector<unsigned int> global_id;

protected:
	ChSystemDistr *my_sys;
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_COLLISION_CHDATAMANAGERDISTR_H_ */
