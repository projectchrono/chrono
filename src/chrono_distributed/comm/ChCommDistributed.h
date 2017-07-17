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

class ChDistributedDataManager;
class ChSystemDistributed;

#ifdef TestStruct
typedef struct BodyExchange {
    uint gid;
    bool collide;
    double pos[3];
    double rot[4];
    double vel[6];
    double mass;
    double inertiaXX[3];
    double inertiaXY[3];
    float mu;
    float adhesionMultDMT;
    float ym_kn;
    float pr_kt;
    float restit_gn;
    float gt;
} BodyExchange;

typedef struct BodyUpdate {
    uint gid;
    int update_type;
    double pos[3];
    double rot[4];
    double vel[6];
} BodyUpdate;

typedef struct Shape {
    uint gid;
    int type;
    double A[3];
    double R[3];
    double data[3];
} Shape;
#endif

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
    ChCommDistributed(ChSystemDistributed* my_sys);
    virtual ~ChCommDistributed();

    /// Scans the system's data structures for bodies that:
    ///	- need to be sent to another rank to create ghosts
    ///  - need to be sent to another rank to update ghosts
    ///	- need to update their comm_status
    /// Sends updates via mpi to the appropriate rank
    /// Processes incoming updates from other ranks
    void Exchange();

#ifdef TestStruct
    /// Packages the body data into buf.
    /// Returns the number of elements which the body took in the buffer
    void PackExchange(BodyExchange* buf, int index);

    /// Unpacks a sphere body from the buffer into body object.
    void UnpackExchange(BodyExchange* buf, std::shared_ptr<ChBody> body);

    /// Packs a body to be sent to update its ghost on another rank
    void PackUpdate(BodyUpdate* buf, int index, int update_type);

    /// Unpacks an incoming body to update a ghost
    void UnpackUpdate(BodyUpdate* buf, std::shared_ptr<ChBody> body);

    void PackUpdateTake(uint* buf, int index);

    int PackShapes(Shape* buf, int index);

    void ProcessExchanges(int num_recv, BodyExchange* buf, int updown);
    void ProcessUpdates(int num_recv, BodyUpdate* buf);
    void ProcessTakes(int num_recv, uint* buf);
    void ProcessShapes(int num_recv, Shape* buf);

#else
    int PackExchange(double* buf, int index);

    int UnpackExchange(double* buf, std::shared_ptr<ChBody> body);

    int PackUpdate(double* buf, int index, int update_type);

    int UnpackUpdate(double* buf, std::shared_ptr<ChBody> body);

    int PackUpdateTake(double* buf, int index);

    /// Checks for consistency in comm status between ranks
    void CheckExchange();
#endif

    ChParallelDataManager* data_manager;
    ChDistributedDataManager* ddm;

  protected:
    ChSystemDistributed* my_sys;
#ifndef TestStruct
    double* sendup_buf;
    double* senddown_buf;
    int num_sendup;
    int num_senddown;
#else
    // MPI Data Types
    MPI_Datatype BodyExchangeType;
    MPI_Datatype BodyUpdateType;
    MPI_Datatype ShapeType;
#endif
  private:
#ifndef TestStruct
    void Pack();
    void ProcessBuffer(int num_recv, double* buf, int updown);
#endif
};
} /* namespace chrono */
