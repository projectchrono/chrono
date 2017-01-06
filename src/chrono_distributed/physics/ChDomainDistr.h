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

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"

namespace chrono {

class ChSystemDistr;
class ChBodyDistr;

class ChDomainDistr {

public:
	ChDomainDistr(ChSystemDistr * sys);
	virtual ~ChDomainDistr();
	void SetSimDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi);
	
	virtual void SplitDomain() = 0;
	virtual bool HasLeft(ChBodyDistr *body) = 0;
	virtual typename std::forward_list<int>::iterator GetNeighItr() = 0;

protected:
	ChSystemDistr * my_sys;
	double boxlo[3]; // Lower coordinates of the global domain
	double boxhi[3]; // Upper coordinates of the global domain

	double sublo[3]; // Lower coordinates of this subdomain, 0=x,1=y,2=z
	double subhi[3]; // Upper coordinates of this subdomain

	int long_axis;

	std::forward_list<int> neigh_ranks; // List of the MPI ranks whose subdomains border the local subdomain
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTR_H_ */
