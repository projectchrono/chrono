// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Shared data structure for the custom multicore collision system.
// =============================================================================

#pragma once

#include <memory>

#include "chrono/physics/ChContactContainer.h"

#include "chrono/multicore_math/ChMulticoreMath.h"

namespace chrono {
namespace collision {

/// @addtogroup collision_mc
/// @{

/// Structure with all measures associated with the collision detection system.
class collision_measures {
  public:
    collision_measures() {
        min_bounding_point = real3(0);
        max_bounding_point = real3(0);
        global_origin = real3(0);
        bin_size = real3(0);
        number_of_contacts_possible = 0;
        number_of_bins_active = 0;
        number_of_bin_intersections = 0;

        rigid_min_bounding_point = real3(0);
        rigid_max_bounding_point = real3(0);

        ff_min_bounding_point = real3(0);
        ff_max_bounding_point = real3(0);
        ff_bins_per_axis = vec3(0);

        mpm_min_bounding_point = real3(0);
        mpm_max_bounding_point = real3(0);
        mpm_bins_per_axis = vec3(0);
    }

    real3 min_bounding_point;          ///< The minimal global bounding point
    real3 max_bounding_point;          ///< The maximum global bounding point
    real3 global_origin;               ///< The global zero point
    real3 bin_size;                    ///< Vector holding bin sizes for each dimension
    real3 inv_bin_size;                ///< Vector holding inverse bin sizes for each dimension
    uint number_of_bins_active;        ///< Number of active bins (containing 1+ AABBs)
    uint number_of_bin_intersections;  ///< Number of AABB bin intersections
    uint number_of_contacts_possible;  ///< Number of contacts possible from broadphase

    real3 rigid_min_bounding_point;
    real3 rigid_max_bounding_point;

    // Fluid Collision info
    vec3 ff_bins_per_axis;
    real3 ff_min_bounding_point;
    real3 ff_max_bounding_point;

    // MPM Collision info
    real3 mpm_min_bounding_point;
    real3 mpm_max_bounding_point;
    vec3 mpm_bins_per_axis;
};

/// Structure of arrays containing contact shape information.
struct shape_container {
    std::vector<short2> fam_rigid;  ///< family information
    std::vector<uint> id_rigid;     ///< ID of associated body
    std::vector<int> typ_rigid;     ///< shape type
    std::vector<int> local_rigid;   ///< local shape index in collision model of associated body
    std::vector<int> start_rigid;   ///< start index in the appropriate container of dimensions
    std::vector<int> length_rigid;  ///< usually 1, except for convex

    std::vector<quaternion> ObR_rigid;  ///< Shape rotation
    std::vector<real3> ObA_rigid;       ///< Position of shape

    std::vector<real> sphere_rigid;      ///< radius for sphere shapes
    std::vector<real3> box_like_rigid;   ///< dimensions for box-like shapes
    std::vector<real3> triangle_rigid;   ///< vertices of all triangle shapes (3 per shape)
    std::vector<real2> capsule_rigid;    ///< radius and half-length for capsule shapes
    std::vector<real4> rbox_like_rigid;  ///< dimensions and radius for rbox-like shapes
    std::vector<real3> convex_rigid;     ///<

    std::vector<real3> triangle_global;
    std::vector<real3> obj_data_A_global;
    std::vector<quaternion> obj_data_R_global;
};

/// Structure of arrays containing state data.
struct state_container {
    // Counters
    uint num_rigid_bodies;  ///< The number of rigid bodies in a system
    uint num_fluid_bodies;  ///< The number of fluid bodies in the system

    // Object data
    std::vector<real3>* pos_rigid;
    std::vector<quaternion>* rot_rigid;
    std::vector<char>* active_rigid;
    std::vector<char>* collide_rigid;

    // Information for 3dof nodes
    std::vector<real3>* pos_3dof;
    std::vector<real3>* sorted_pos_3dof;
};

/// Global data for the custom Chrono multicore collision system.
class ChApi ChCollisionData {
  public:
    ChCollisionData(bool owns_data)
        : owns_state_data(owns_data),
          collision_envelope(0),
          num_rigid_contacts(0),
          num_rigid_fluid_contacts(0),
          num_fluid_contacts(0),
          num_rigid_shapes(0) {
        if (owns_data) {
            state_data.pos_rigid = new std::vector<real3>;
            state_data.rot_rigid = new std::vector<quaternion>;
            state_data.active_rigid = new std::vector<char>;
            state_data.collide_rigid = new std::vector<char>;

            state_data.pos_3dof = new std::vector<real3>;
            state_data.sorted_pos_3dof = new std::vector<real3>;
        }
    }

    ~ChCollisionData() {
        if (owns_state_data) {
            delete state_data.pos_rigid;
            delete state_data.rot_rigid;
            delete state_data.active_rigid;
            delete state_data.collide_rigid;

            delete state_data.pos_3dof;
            delete state_data.sorted_pos_3dof;
        }
    }

    bool owns_state_data;  ///< if false, state data set from outside

    state_container state_data;  ///< State data arrays
    shape_container shape_data;  ///< Shape information data arrays

    collision_measures measures;  ///< Container for various statistics for collision detection

    real collision_envelope;  ///< Collision envelope for rigid shapes

    real p_collision_envelope;  ///< Collision envelope for 3-dof particles
    real p_kernel_radius;       ///< 3-dof particle radius
    short2 p_collision_family;  ///< collision family for 3-dof particles

    // Collision detection output data
    // -------------------------------

    std::vector<real3> aabb_min;  ///< List of bounding boxes minimum point
    std::vector<real3> aabb_max;  ///< List of bounding boxes maximum point

    std::vector<long long> pair_shapeIDs;     ///< Shape IDs for each shape pair (encoded in a single long long)
    std::vector<long long> contact_shapeIDs;  ///< Shape IDs for each contact (encoded in a single long long)

    // Rigid-rigid geometric collision data 
    std::vector<real3> norm_rigid_rigid;
    std::vector<real3> cpta_rigid_rigid;
    std::vector<real3> cptb_rigid_rigid;
    std::vector<real> dpth_rigid_rigid;
    std::vector<real> erad_rigid_rigid;
    std::vector<vec2> bids_rigid_rigid;

    // Rigid-particle geometric collision data
    std::vector<real3> norm_rigid_fluid;
    std::vector<real3> cpta_rigid_fluid;
    std::vector<real> dpth_rigid_fluid;
    std::vector<int> neighbor_rigid_fluid;
    std::vector<int> c_counts_rigid_fluid;

    // 3dof particle neighbor information
    std::vector<int> neighbor_3dof_3dof;
    std::vector<int> c_counts_3dof_3dof;

    // Sorting map for 3dof particles
    std::vector<int> particle_indices_3dof;
    std::vector<int> reverse_mapping_3dof;

    // Broadphase Data
    std::vector<uint> bin_intersections;
    std::vector<uint> bin_number;
    std::vector<uint> bin_number_out;
    std::vector<uint> bin_aabb_number;
    std::vector<uint> bin_start_index;
    std::vector<uint> bin_num_contact;

    // Indexing variables
    // ------------------

    uint num_rigid_shapes;          ///< The number of collision models in a system
    uint num_rigid_contacts;        ///< The number of contacts between rigid bodies in a system
    uint num_rigid_fluid_contacts;  ///< The number of contacts between rigid and fluid objects
    uint num_fluid_contacts;        ///< The number of contacts between fluid objects
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
