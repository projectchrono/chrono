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

#ifndef CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTRLONG_H_
#define CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTRLONG_H_

#include <forward_list>

#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

class ChDomainDistrLong : public ChDomainDistr {
public:
	ChDomainDistrLong(ChSystemDistr *my_sys) : ChDomainDistr(my_sys) {}
	virtual ~ChDomainDistrLong() {}
	
	void SplitDomain();
	bool HasLeft(ChBodyDistr *body);
	typename std::forward_list<int>::iterator GetNeighItr();

};
}
#endif
