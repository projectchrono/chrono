/*
 * ChCollisionSystemDistributed.cpp
 *
 *  Created on: Mar 28, 2017
 *      Author: nic
 */

#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"
#include "chrono_distributed/ChDistributedDataManager.h"

#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {
namespace collision {

ChCollisionSystemDistributed::ChCollisionSystemDistributed(ChParallelDataManager *dm, ChDistributedDataManager *ddm) :
	ChCollisionSystemParallel(dm)
{
	this->ddm = ddm;
}

ChCollisionSystemDistributed::~ChCollisionSystemDistributed() {}

} /* namespace collision */
} /* namespace chrono */
