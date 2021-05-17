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

#include "chrono/collision/chrono/ChCollisionDefines.h"

#include "chrono/multicore_math/real.h"
#include "chrono/multicore_math/real2.h"
#include "chrono/multicore_math/real3.h"
#include "chrono/multicore_math/real4.h"
#include "chrono/multicore_math/other_types.h"

namespace chrono {
namespace collision {

/// @addtogroup collision_mc
/// @{

/// This structure contains all measures associated with the collision detection phase
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
    custom_vector<short2> fam_rigid;  ///< family information
    custom_vector<uint> id_rigid;     ///< ID of associated body
    custom_vector<int> typ_rigid;     ///< shape type
    custom_vector<int> local_rigid;   ///< local shape index in collision model of associated body
    custom_vector<int> start_rigid;   ///< start index in the appropriate container of dimensions
    custom_vector<int> length_rigid;  ///< usually 1, except for convex

    custom_vector<quaternion> ObR_rigid;  ///< Shape rotation
    custom_vector<real3> ObA_rigid;       ///< Position of shape

    custom_vector<real> sphere_rigid;      ///< radius for sphere shapes
    custom_vector<real3> box_like_rigid;   ///< dimensions for box-like shapes
    custom_vector<real3> triangle_rigid;   ///< vertices of all triangle shapes (3 per shape)
    custom_vector<real2> capsule_rigid;    ///< radius and half-length for capsule shapes
    custom_vector<real4> rbox_like_rigid;  ///< dimensions and radius for rbox-like shapes
    custom_vector<real3> convex_rigid;     ///<

    custom_vector<real3> triangle_global;
    custom_vector<real3> obj_data_A_global;
    custom_vector<quaternion> obj_data_R_global;
};

/// Structure of arrays containing collision detection results.
struct host_container {
    custom_vector<real3> aabb_min;  ///< List of bounding boxes minimum point
    custom_vector<real3> aabb_max;  ///< List of bounding boxes maximum point

    custom_vector<long long> pair_shapeIDs;     ///< Shape IDs for each shape pair (encoded in a single long long)
    custom_vector<long long> contact_shapeIDs;  ///< Shape IDs for each contact (encoded in a single long long)

    // Contact data
    custom_vector<real3> norm_rigid_rigid;
    custom_vector<real3> cpta_rigid_rigid;
    custom_vector<real3> cptb_rigid_rigid;
    custom_vector<real> dpth_rigid_rigid;
    custom_vector<real> erad_rigid_rigid;
    custom_vector<vec2> bids_rigid_rigid;

    custom_vector<real3> norm_rigid_fluid;
    custom_vector<real3> cpta_rigid_fluid;
    custom_vector<real> dpth_rigid_fluid;
    custom_vector<int> neighbor_rigid_fluid;
    custom_vector<int> c_counts_rigid_fluid;

    // each particle has a finite number of neighbors preallocated
    custom_vector<int> neighbor_3dof_3dof;
    custom_vector<int> c_counts_3dof_3dof;
    custom_vector<int> particle_indices_3dof;
    custom_vector<int> reverse_mapping_3dof;

    // Broadphase Data
    custom_vector<uint> bin_intersections;
    custom_vector<uint> bin_number;
    custom_vector<uint> bin_number_out;
    custom_vector<uint> bin_aabb_number;
    custom_vector<uint> bin_start_index;
    custom_vector<uint> bin_num_contact;
};

/// Structure of arrays containing state data.
struct state_container {
    //// TODO: provide mechanism (shared_ptr?) to allow using external arrays when using this collision library from
    ////       within Chrono::Multicore

    // Counters
    uint num_rigid_bodies;  ///< The number of rigid bodies in a system
    uint num_fluid_bodies;  ///< The number of fluid bodies in the system

    // Object data
    custom_vector<real3> pos_rigid;
    custom_vector<quaternion> rot_rigid;
    custom_vector<char> active_rigid;
    custom_vector<char> collide_rigid;

    // Information for 3dof nodes
    custom_vector<real3> pos_3dof;
    custom_vector<real3> sorted_pos_3dof;
    custom_vector<real3> vel_3dof;
    custom_vector<real3> sorted_vel_3dof;
};

/// Structure with information on 3DOF fluid nodes
struct node_container {
    //// TODO: provide mechanism (shared_ptr?) to allow using external arrays when using this collision library from
    ////       within Chrono::Multicore

    real kernel_radius;
    real collision_envelope;
    short2 family;
};

/// Global data for the custom Chrono collision system.
class ChApi ChCollisionData {
  public:
    ChCollisionData()
        : num_rigid_contacts(0),
          num_rigid_fluid_contacts(0),
          num_fluid_contacts(0),
          num_rigid_shapes(0),
          add_contact_callback(nullptr) {}

    ~ChCollisionData() {}

    //// TODO: rename "host_container" and "host_data" to "collision_container" and "collision_data"

    state_container state_data;  ///< State data arrays
    host_container host_data;    ///< Collision data arrays
    shape_container shape_data;  ///< Shape information data arrays
    node_container node_data;    ///< 3DOF fluid nodes data

    collision_measures measures;  ///< Container for various statistics for collision detection

    // Indexing variables
    uint num_rigid_shapes;          ///< The number of collision models in a system
    uint num_rigid_contacts;        ///< The number of contacts between rigid bodies in a system
    uint num_rigid_fluid_contacts;  ///< The number of contacts between rigid and fluid objects
    uint num_fluid_contacts;        ///< The number of contacts between fluid objects

    /// User-provided callback for overriding composite material properties.
    std::shared_ptr<ChContactContainer::AddContactCallback> add_contact_callback;
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
