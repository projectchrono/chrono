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
#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

namespace chrono {

class ChDistributedDataManager;
class ChSystemDistributed;

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
    float cohesion;
    float ym_kn;
    float pr_kt;
    float restit_gn;
    float gt;
    int identifier;
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
    short coll_fam[2];
    double A[3];  // A
    double R[4];
    double data[6];  // B C and shape-specific data
} Shape;

/// This class holds functions for processing the system's bodies to determine
/// when a body needs to be sent to another rank for either an update or for
/// creation of a ghost. The class also decides how to update the comm_status of
/// each body based on its position and its comm_status.
///
/// Actions:
///
/// A body with an OWNED comm_status will be packed for exchange to create a ghost body on another rank when it
/// passes into one of this rank's shared regions, at which point the body is given a SHARED comm_status on this rank.
///
/// A body with a SHARED comm_status will become OWNED when it moves into the owned region on this rank.
/// A body with a SHARED comm_status will become GHOST when it crosses subhi or sublo and the other rank will change its
/// comm_status for the body to SHARED.
///
/// A body with a GHOST comm_status will become OWNED when it moves into the owned region of this rank.
/// A body with a GHOST comm_status will be removed when it moves into the one of this rank's unowned regions.
class CH_DISTR_API ChCommDistributed {
  public:
    ChCommDistributed(ChSystemDistributed* my_sys);
    virtual ~ChCommDistributed();

    /// Scans the system's data structures for bodies that:
    ///	- need to be sent to another rank to create ghosts
    /// - need to be sent to another rank to update ghosts
    ///	- need to update their comm_status
    /// Sends updates via mpi to the appropriate rank
    /// Processes incoming updates from other ranks
    void Exchange();

  protected:
    ChSystemDistributed* my_sys;

    // MPI Data Types
    MPI_Datatype BodyExchangeType;
    MPI_Datatype BodyUpdateType;
    MPI_Datatype ShapeType;

    ChParallelDataManager* data_manager;
    ChDistributedDataManager* ddm;

  private:
    /// Helper function for processing incoming exchange messages.
    void ProcessExchanges(int num_recv, BodyExchange* buf, int updown);

    /// Helper function for processing incoming update messages.
    void ProcessUpdates(int num_recv, BodyUpdate* buf);

    /// Helper function for processing incoming take messages.
    void ProcessTakes(int num_recv, uint* buf);

    /// Helper function for processing incoming shape messages.
    void ProcessShapes(int num_recv, Shape* buf);

    /// Packages the body data into buf.
    /// Returns the number of elements which the body took in the buffer
    void PackExchange(BodyExchange* buf, int index);

    /// Unpacks a sphere body from the buffer into body object.
    void UnpackExchange(BodyExchange* buf, std::shared_ptr<ChBody> body);

    /// Packs a body to be sent to update its ghost on another rank
    void PackUpdate(BodyUpdate* buf, int index, int update_type);

    /// Unpacks an incoming body to update a ghost
    void UnpackUpdate(BodyUpdate* buf, std::shared_ptr<ChBody> body);

    /// Packs the gid of the body at index index into buf
    void PackUpdateTake(uint* buf, int index);

    /// Packs all shapes for the body at index into buf and returns
    /// the number of shapes that it has packed.
    int PackShapes(std::vector<Shape>* buf, int index);
};
} /* namespace chrono */
