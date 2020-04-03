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

/// @addtogroup distributed_comm
/// @{

/// Structure of data for sending a new body to a rank
typedef struct BodyExchange {
    uint gid;
    bool collide;
    double pos[3];
    double rot[4];
    double vel[6];
    double mass;
    double inertiaXX[3];
    double inertiaXY[3];
    int identifier;
} BodyExchange;

/// @} distributed_comm

/// @addtogroup distributed_comm
/// @{

/// Structure of data for sending an update of an existing body to a rank
typedef struct BodyUpdate {
    uint gid;
    int update_type;
    double pos[3];
    double rot[4];
    double vel[6];
} BodyUpdate;

/// @} distributed_comm

/// @addtogroup distributed_comm
/// @{

/// Structure of data for sending a collision shape to a rank. The encapsulated contact material information depends on
/// whether or not the system uses material properties to infer contact properties.
typedef struct Shape {
    uint gid;           ///< global shape ID
    int type;           ///< shape type
    short coll_fam[2];  ///< collision family and mask
    double A[3];        ///< position
    double R[4];        ///< orientation
    double data[6];     ///< shape-specific geometric data
    float mu;           ///< coefficient of friction
    float cohesion;     ///< adhesion (constant OR DMT parameter)
    float ym_kn;        ///< Young's modulus OR normal stiffness
    float pr_kt;        ///< Poisson ratio OR tangential stiffness
    float restit_gn;    ///< coefficient of restitution OR normal damping
    float gt;           ///< tangential damping
} Shape;

/// @} distributed_comm

/// @addtogroup distributed_comm
/// @{

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

    /// MPI Data Types for sending 1) new body 2) body update 3) new collision shape
    MPI_Datatype BodyExchangeType;
    MPI_Datatype BodyUpdateType;
    MPI_Datatype ShapeType;

    /// Pointer to underlying chrono::parallel data
    ChParallelDataManager* data_manager;

    /// Set of data for scaffolding on top of chrono::parallel
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
/// @} distributed_comm

} /* namespace chrono */
