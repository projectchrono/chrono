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

#ifndef CHRONO_DISTRIBUTED_COLLISION_CHNARROWPHASEDISTR_H_
#define CHRONO_DISTRIBUTED_COLLISION_CHNARROWPHASEDISTR_H_

#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

class ChNarrowphaseDistr {
public:
	ChNarrowphaseDistr(ChSystemDistr *my_sys);
	virtual ~ChNarrowphaseDistr();
	// TODO: Outline interface

protected:
	ChSystemDistr *my_sys;
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_COLLISION_CHNARROWPHASEDISTR_H_ */
