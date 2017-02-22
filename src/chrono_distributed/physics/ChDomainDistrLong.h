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
#include <memory>

#include "chrono/physics/ChBody.h"

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

class CH_DISTR_API ChDomainDistrLong : public ChDomainDistr {
public:
	ChDomainDistrLong(std::shared_ptr<ChSystemDistr> my_sys) : ChDomainDistr(my_sys) {}
	virtual ~ChDomainDistrLong() {}
	
	virtual void SplitDomain() override;
	virtual bool InSub(std::shared_ptr<ChBody> body) override;
	virtual bool InGhost(std::shared_ptr<ChBody> body) override;
	virtual std::forward_list<int>::iterator GetNeighItr() override;

    bool HasLeft(std::shared_ptr<ChBody> body);

	void PrintDomain();
};

}

#endif
