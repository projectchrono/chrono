/*
 * ChDomainDistr.h
 *
 *  Created on: Dec 28, 2016
 *      Author: nic
 */

#ifndef CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTR_H_
#define CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTR_H_

#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

class ChDomainDistr {
public:
	ChDomainDistr(ChSystemDistr *sys);
	virtual ~ChDomainDistr();
	void SplitDomain();
	void SetDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi);

protected:
	ChSystemDistr * my_sys;
	double boxlo[3]; // Lower coordinates of the global domain
	double boxhi[3]; // Upper coordinates of the global domain

	double sublo[3]; // Lower coordinates of this subdomain, 0=x,1=y,2=z
	double subhi[3]; // Upper coordinates of this subdomain

	int long_axis;
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHDOMAINDISTR_H_ */
