/*
 * ChDistributedDataManager.h
 *
 *  Created on: Mar 3, 2017
 *      Author: nic
 */

#ifndef CHRONO_DISTRIBUTED_CHDISTRIBUTEDDATAMANAGER_H_
#define CHRONO_DISTRIBUTED_CHDISTRIBUTEDDATAMANAGER_H_

#include "chrono_distributed/ChApiDistributed.h"

#include <vector>

namespace chrono {

class CH_DISTR_API ChDistributedDataManager {
public:
	ChDistributedDataManager();
	virtual ~ChDistributedDataManager();
	std::vector<unsigned int> global_id;
	std::vector<int> comm_status;
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_CHDISTRIBUTEDDATAMANAGER_H_ */
