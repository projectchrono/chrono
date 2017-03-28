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

#pragma once

#include <memory>

#include "chrono/physics/ChBody.h"

#include "chrono_parallel/ChDataManager.h"

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"
#include "chrono_distributed/ChDistributedDataManager.h"

namespace chrono {

/// A type corresponding to an exchange message
struct exchange
{
	// distributed::EXCHANGE;

	unsigned int gid;	///< Global Id

	double pos[3];	///< Position

	double rot[4];	///< Rotation

	double vel[3];	///< Velocity

	double omega[3];	///< Angular Velocity

	double mass;	///< Mass

	// Material DEM
	double mu; ///< Static Friction
	double adhesionMultDMT; ///< Adhesion

	// TODO:

	double dem_data[4]; // At largest 4
};

/// A type corresponding to an  update message
struct update
{
	int update_type;

	unsigned int gid;	///< Global Id

	double pos[3];	///< Position

	double rot[4];	///< Rotation

	double vel[3];	///< Velocity

	double omega[3];	///< Angular Velocity
};


class ChDistributedDataManager;
class ChSystemDistributed;


/// This class holds functions for processing the system's bodies to determine
/// when a body needs to be sent to another rank for either an update or for
/// creation of a ghost. The class also decides how to update the comm_status of
/// each body based on its position and its comm_status.
/// --------------------------------------------------------------------------------
/// Actions:
///
/// A body with an OWNED comm_status will be packed for exchange to create a ghost body on another rank when it
/// passes into one of this rank's shared regions, at which point the body is given a SHARED comm_status on this rank.
///
/// A body with a SHARED comm_status will become OWNED when it moves into the owned region on this rank.
/// A body with a SHARED comm_status will be removed when it moves into one of this rank's unowned regions.
///
/// A body with a GHOST comm_status will become OWNED when it moves into the owned region of this rank.
/// A body with a GHOST comm_status will be removed when it moves into the one of this rank's unowned regions.
class CH_DISTR_API ChCommDistributed {
public:
	ChCommDistributed(ChSystemDistributed *my_sys);
	virtual ~ChCommDistributed();

	/// Scans the system's data structures for bodies that:
	///	- need to be sent to another rank to create ghosts
	///  - need to be sent to another rank to update ghosts
	///	- need to update their comm_status
	/// Sends updates via mpi to the appropriate rank
	/// Processes incoming updates from other ranks
	void Exchange();

	/// Packages the body data into buf.
	/// Returns the number of elements which the body took in the buffer
	int PackExchange(double *buf, int index);

	/// Unpacks a sphere body from the buffer into body object.
	int UnpackExchange(double *buf, std::shared_ptr<ChBody> body);

	/// Packs a body to be sent to update its ghost on another rank
	int PackUpdate(double *buf, int index, int update_type);
	
	/// Unpacks an incoming body to update a ghost
	int UnpackUpdate(double *buf, std::shared_ptr<ChBody> body);

	int PackUpdateTake(double *buf, int index);

	/// Checks for consistency in comm status between ranks
	void CheckExchange();

	ChParallelDataManager* data_manager;
	ChDistributedDataManager *ddm;

protected:
	ChSystemDistributed *my_sys;
	double *sendup_buf;
	double *senddown_buf;
	int num_sendup;
	int num_senddown;

private:
	void ProcessBuf(int num_recv, double* buf, int updown);
};

} /* namespace chrono */
