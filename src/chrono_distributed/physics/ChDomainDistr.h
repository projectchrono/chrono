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

#ifndef CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTR_H_
#define CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTR_H_

#include <forward_list>
#include <memory>

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono/physics/ChBody.h"

#include "chrono/core/ChVector.h"

namespace chrono {

class ChSystemDistr;
class ChBodyDistr;

class ChDomainDistr {

public:
	ChDomainDistr(std::shared_ptr<ChSystemDistr> sys);
	virtual ~ChDomainDistr();
	void SetSimDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi);
	
	virtual void SplitDomain() = 0;
	virtual bool InSub(std::shared_ptr<ChBody> body) = 0;
	virtual bool InGhost(std::shared_ptr<ChBody>) = 0;
	virtual typename std::forward_list<int>::iterator GetNeighItr() = 0;

	ChVector<double> GetBoxLo() {return boxlo;}
	ChVector<double> GetBoxHi() {return boxhi;}

	ChVector<double> GetSubLo() {return sublo;}
	ChVector<double> GetSubHi() {return subhi;}

	int GetLongAxis() {return long_axis;}

protected:
	std::shared_ptr<ChSystemDistr> my_sys;

	int long_axis;

	ChVector<double> boxlo; // Lower coordinates of the global domain
	ChVector<double> boxhi; // Upper coordinates of the global domain

	ChVector<double> sublo; // Lower coordinates of this subdomain, 0=x,1=y,2=z
	ChVector<double> subhi; // Upper coordinates of this subdomain

	std::forward_list<int> neigh_ranks; // List of the MPI ranks whose subdomains border the local subdomain
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTR_H_ */
