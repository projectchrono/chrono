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

#ifndef CHRONO_DISTRIBUTED_COLLISION_CHNARROWPHASEDISTRBASIC_H_
#define CHRONO_DISTRIBUTED_COLLISION_CHNARROWPHASEDISTRBASIC_H_

#include "chrono_distributed/collision/ChNarrowphaseDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

class ChNarrowphaseDistrBasic : public ChNarrowphaseDistr {
public:
	ChNarrowphaseDistrBasic(ChSystemDistr *my_sys);
	virtual ~ChNarrowphaseDistrBasic();
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_COLLISION_CHNARROWPHASEDISTRBASIC_H_ */
