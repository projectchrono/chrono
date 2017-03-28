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

#include "chrono_parallel/collision/ChCollisionModelParallel.h"

namespace chrono {

namespace collision {

class CH_DISTR_API ChCollisionModelDistributed : public ChCollisionModelParallel {
public:
	ChCollisionModelDistributed();
	virtual ~ChCollisionModelDistributed();

	virtual bool AddBox(double hx,
	                        double hy,
	                        double hz,
	                        const ChVector<>& pos = ChVector<>(),
	                        const ChMatrix33<>& rot = ChMatrix33<>(1)) override; // TODO add other AddShape methods that could be global


	virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const override;

protected:
	 ChVector<double> aabb_max;
	 ChVector<double> aabb_min;
	 bool aabb_valid;
};

}
}
