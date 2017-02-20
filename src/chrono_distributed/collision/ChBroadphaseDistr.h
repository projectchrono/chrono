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

#ifndef CHRONO_DISTRIBUTED_COLLISION_CHBROADPHASEDISTR_H_
#define CHRONO_DISTRIBUTED_COLLISION_CHBROADPHASEDISTR_H_

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/collision/ChDataManagerDistr.h"

#include <memory>

namespace chrono {

// Forward Declaration
class ChDataManagerDistr;

class ChBroadphaseDistr {
public:
	ChBroadphaseDistr(std::shared_ptr<ChSystemDistr> my_sys);
	virtual ~ChBroadphaseDistr();

	virtual void DetectPossibleCollisions() = 0;

protected:
	std::shared_ptr<ChSystemDistr> my_sys;
	std::shared_ptr<ChDataManagerDistr> data;

// TODO: Interface

};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_COLLISION_CHBROADPHASEDISTR_H_ */
