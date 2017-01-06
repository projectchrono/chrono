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

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"

namespace chrono {

class ChDataManagerDistr {
public:
	ChDataManagerDistr(ChSystemDistr *my_sys);
	virtual ~ChDataManagerDistr();

protected:
	ChSystemDistr *my_sys;

	// TODO: DS
	ChBodyDistr **local_bodylist;
	ChBodyDistr **shared_bodylist;
	ChBodyDistr **ghost_bodylist;

	//TODO: Outline interface
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_COLLISION_CHDATAMANAGERDISTR_H_ */
