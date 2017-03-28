/*
 * ChCollisionSystemDistributed.h
 *
 *  Created on: Mar 28, 2017
 *      Author: nic
 */

#pragma once

#include "chrono_parallel/ChDataManager.h"

#include "chrono_distributed/ChDistributedDataManager.h"

namespace chrono {
namespace collision {

class ChCollisionSystemDistributed: public ChCollisionSystemParallel {
public:
	ChCollisionSystemDistributed(ChParallelDataManager *dm, ChDistributedDataManager *ddm);
	virtual ~ChCollisionSystemDistributed();

	virtual void Add(ChCollisionModel *model) override;
protected:
	ChDistributedDataManager *ddm;
};

} /* namespace collision */
} /* namespace chrono */
