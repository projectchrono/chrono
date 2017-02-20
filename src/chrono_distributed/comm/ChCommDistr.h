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

#ifndef CHRONO_DISTRIBUTED_PHYSICS_CHCOMMDISTR_H_
#define CHRONO_DISTRIBUTED_PHYSICS_CHCOMMDISTR_H_

#include "chrono_distributed/physics/ChSystemDistr.h"

#include "chrono/physics/ChBody.h"

#include "chrono_parallel/ChDataManager.h"


#include <memory>

namespace chrono {


class ChSystemDistr;

class ChCommDistr {
public:
	ChCommDistr(std::shared_ptr<ChSystemDistr> my_sys);
	virtual ~ChCommDistr();

	// Locate each body that has left the subdomain,
	// remove it from the appropriate body list,
	// and pack it for communication.
	// Updates the statuses of each body (local/shared/ghost)
	void Exchange();

	// Called when a sphere leaves this rank's subdomain.
	// The body should have been removed from the local system's
	// list before sending.
	// Packages the body into buf.
	// Returns the number of elements which the body took in the buffer
	int PackExchange(double *buf, int index);

	// Unpacks a sphere body from the buffer into body.
	// Note: body is meant to be a ptr into the data structure
	// where the body should be unpacks.
	void UnpackExchange(double *buf, std::shared_ptr<ChBody> body);

	int PackUpdate(double *buf, int index);
	void UnpackUpdate(double *buf, std::shared_ptr<ChBody> body);

	ChParallelDataManager* data_manager;

protected:
	std::shared_ptr<ChSystemDistr> my_sys;
	double *send_buf;
	int num_send;

	double *recv_buf;
	int num_recv;

	int doubles_per_body;

private:
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHCOMMDISTR_H_ */
