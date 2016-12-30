/*
 * ChBodyDistr.h
 *
 *  Created on: Dec 29, 2016
 *      Author: nic
 */

#ifndef CHRONO_DISTRIBUTED_PHYSICS_CHBODYDISTR_H_
#define CHRONO_DISTRIBUTED_PHYSICS_CHBODYDISTR_H_

#include "chrono/physics/ChBody.h"

namespace chrono {

class ChBodyDistr : public ChBody {
public:
	ChBodyDistr();
	virtual ~ChBodyDistr();
	void SetGlobalId(int id) { if (id >= 0) global_id = id; }
	int GetGloablId() {return global_id;}

protected:
	int global_id;
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHBODYDISTR_H_ */
