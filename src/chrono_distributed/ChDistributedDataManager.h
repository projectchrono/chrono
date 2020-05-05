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

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/ChTypesDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono_parallel/ChDataManager.h"

#include <vector>

namespace chrono {

class ChSystemDistributed;

// TODO replace this

// typedef struct ShapeNode {
//     ShapeNode(int bsi, int s, bool f) : body_shapes_index(bsi), size(s), free(f) {}
//     int body_shapes_index;
//     int size;
//     bool free;
// } ShapeNode;

/// @addtogroup distributed_module
/// @{

/// Linked-list node for tracking free shapes
struct LocalShapeNode {
    struct LocalShapeNode* next;  ///< Next node in the free list
    int body_shapes_index;        ///< Index into ChDistributedDataManager::body_shapes
    int size;                     ///< Number of
    bool free;                    ///< True if this index is free
};

/// A class for storing data for maintaining a consistent view of a distributed
/// simulation consisting of multiple wrapped instances of ChSystemParallelSMC.
class CH_DISTR_API ChDistributedDataManager {
  public:
    ChDistributedDataManager(ChSystemDistributed* my_sys);
    virtual ~ChDistributedDataManager();

    bool initial_add;  ///< Indicates that the system is undergoing its inital setup. Allows for some shortcuts
                       ///< by assuming contiguous valid data.

    /* Basic distributed values */
    std::vector<unsigned int> global_id;                ///< Global id of each body. Maps local index to global index.
    std::vector<distributed::COMM_STATUS> comm_status;  ///< Communication status of each body.
    std::vector<distributed::COMM_STATUS> curr_status;  ///< Used as a reference only by ChCommDistributed.

    std::unordered_map<uint, int> gid_to_localid;  ///< Maps gloabl id to local id on this rank

    /* Pointers to the rest of the system's data */
    ChParallelDataManager* data_manager;  ///< Pointer to the main Chrono::Parallel Data Manager
    ChSystemDistributed* my_sys;          ///< Pointer to the main dynamical system

    /* Collision system tracking */
    std::vector<int>
        body_shape_start;  ///< Start index in this->body_shapes of the shapes associated with this local BODY index
    std::vector<int> body_shape_count;  ///< Number of shapes associated with this local BODY index
    std::vector<int> body_shapes;       ///< Indices of shape in DATA_MANAGER->shape_data struct for a given SHAPE

    /*
     * NOTE: The values in this->body_shapes for a given BODY begin at body_shape_start[local_id] and there are
     * body_shape_count[local_id] consecutive values starting there for the body at index local_id.
     */

    /*
     * NOTE: DON'T need to track open spots in this->body_shape_start/count because those correspond with a BODY
     * local_id and therefore can be checked for validity by checking the body comm_status
     */

    /*
     * NOTE: When receiving a body and adding its collision shapes, need to
     * 1) Find ONE block in ddm->body_shapes large enough for ALL of the body's
     * 	shape indices into data_manager->shape_data
     *
     * 2) Find individual indices in data_manager->shape_data to index to from body_shapes
     */

    // std::list<ShapeNode> ddm_free_shapes;

    // TODO replace
    struct LocalShapeNode* local_free_shapes;  ///< Linked-list of BLOCKS in ddm->body_shapes, indicating free/alloc'd

    std::vector<bool> dm_free_shapes;  ///< Indicates that the space in the data_manager->shape_data is available

    int first_empty;  ///< Index of the first unused body in the bodylist (internal)

    int first_cosim, last_cosim;  // Global ID range [first, last] of co-simulation triangles

    /// Returns the local index of a body, given its global id.
    /// Returns -1 if the body is not found on this rank.
    int GetLocalIndex(unsigned int gid);

    /// Internal function for cleaning free list that tracks space for shapes.
    /// Should not be called by the user.
    void DefragmentFreeList();
};
/// @} distributed_module

} /* namespace chrono */
