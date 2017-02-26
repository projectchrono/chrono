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

#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/ChDataManagerDistr.h"
#include "chrono_distributed/ChApiDistributed.h"

namespace chrono {

class ChSystemDistr;

class CH_DISTR_API ChDomainDistr {

public:
	ChDomainDistr(ChSystemDistr *sys);
	virtual ~ChDomainDistr();
	void SetSimDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi);
	
	/// Calculates the borders of this subdomain based on the rank
	virtual void SplitDomain();

	/// Returns true if the specified body is strictly inside this subdomain
	virtual int InSub(int index);
	virtual int InSub(std::shared_ptr<ChBody> body);

	/// Returns: 0 if the body is not in the ghost layer
	/// 1 if the body is in the next rank up
	/// 2 if the body is in the next rank down
	virtual int InGhost(int index);
	virtual int InGhost(std::shared_ptr<ChBody> body);

	virtual int InShared(int index);
	virtual int InShared(std::shared_ptr<ChBody> body);

	virtual int GetCommStatus(int index);
	virtual int GetCommStatus(std::shared_ptr<ChBody> body);

	virtual void PrintDomain();

	ChVector<double> GetBoxLo() {return boxlo;}
	ChVector<double> GetBoxHi() {return boxhi;}

	ChVector<double> GetSubLo() {return sublo;}
	ChVector<double> GetSubHi() {return subhi;}

	int GetLongAxis() {return long_axis;}

protected:
	ChSystemDistr *my_sys;

	int long_axis;

	ChVector<double> boxlo; // Lower coordinates of the global domain
	ChVector<double> boxhi; // Upper coordinates of the global domain

	ChVector<double> sublo; // Lower coordinates of this subdomain, 0=x,1=y,2=z
	ChVector<double> subhi; // Upper coordinates of this subdomain
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTR_H_ */
