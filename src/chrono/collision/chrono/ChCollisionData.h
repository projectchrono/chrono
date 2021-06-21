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

/// Readibility type definition.
typedef int shape_type;

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
          //
          collision_envelope(0),
          //
          num_rigid_contacts(0),
          num_rigid_fluid_contacts(0),
          num_fluid_contacts(0),
          num_rigid_shapes(0),
          //
          bins_per_axis(vec3(10, 10, 10)),
          fixed_bins(true),
          grid_density(5),
          //
          min_bounding_point(real3(0)),
          max_bounding_point(real3(0)),
          global_origin(real3(0)),
          bin_size(real3(0)),
          num_bins(0),
          num_active_bins(0),
          num_bin_aabb_intersections(0),
          num_possible_collisions(0),
          //
          rigid_min_bounding_point(real3(0)),
          rigid_max_bounding_point(real3(0)),
          //
          ff_min_bounding_point(real3(0)),
          ff_max_bounding_point(real3(0)),
          ff_bins_per_axis(vec3(0)) {
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

    ////collision_measures measures;  ///< Container for various statistics for collision detection

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
    vec3 bins_per_axis;  ///< (input)number of slices along each axis of the collision detection grid
    bool fixed_bins;     ///< (input) keep number of bins fixed
    real grid_density;   ///< (input) collision grid density

    real3 min_bounding_point;         ///< LBR (left-bottom-rear) corner of union of all AABBs
    real3 max_bounding_point;         ///< RTF (right-top-front) corner of union of all AABBs
    real3 global_origin;              ///< The grid zero point (same as LBR)
    real3 bin_size;                   ///< Bin sizes in each direction
    real3 inv_bin_size;               ///< Bin size reciprocals in each direction
    uint num_bins;                    ///< Total number of bins
    uint num_active_bins;             ///< Number of bins intersecting at least one shape AABB
    uint num_bin_aabb_intersections;  ///< Number of shape AABB - bin intersections
    uint num_possible_collisions;     ///< Number of candidate collisions from broadphase

    real3 rigid_min_bounding_point;  ///< LBR (left-bottom-rear) corner of union of rigid AABBs
    real3 rigid_max_bounding_point;  ///< RTF (right-top-front) corner of union of rigid AABBs

    vec3 ff_bins_per_axis;        ///< grid resolution for fluid particles
    real3 ff_min_bounding_point;  ///< LBR (left-bottom-rear) corner of union of fluid AABBs
    real3 ff_max_bounding_point;  ///< RTF (right-top-front) corner of union of fluid AABBs

    std::vector<uint> bin_intersections;  ///< [num_rigid_shapes+1] number of bin intersections for each shape AABB
    std::vector<uint> bin_number;         ///< [num_bin_aabb_intersections] bin index for bin-shape AABB intersections
    std::vector<uint> bin_number_out;     ///< [num_active_bins] bin index of active bins (no duplicates)
    std::vector<uint> bin_aabb_number;    ///< [num_bin_aabb_intersections] shape ID for bin-shape AABB intersections
    std::vector<uint> bin_start_index;    ///< [num_active_bins+1]
    std::vector<uint> bin_num_contact;    ///< [num_active_bins+1]

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
