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

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"

#include "chrono_distributed/ChDistributedDataManager.h"

namespace chrono {
namespace collision {

class ChCollisionSystemDistributed : public ChCollisionSystemParallel {
public:
	ChCollisionSystemDistributed(ChParallelDataManager *dm, ChDistributedDataManager *ddm);
	virtual ~ChCollisionSystemDistributed();

	virtual void Add(ChCollisionModel *model) override;
	virtual void Remove(ChCollisionModel *model) override;
protected:
	ChDistributedDataManager *ddm;
};

} /* namespace collision */
} /* namespace chrono */
