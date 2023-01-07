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

/// Structure of arrays containing rigid collision shape information.
struct shape_container {
    // All arrays of num_shapes length and indexed by the shape ID.

    std::vector<short2> fam_rigid;  ///< family information
    std::vector<uint> id_rigid;     ///< ID of associated body
    std::vector<int> typ_rigid;     ///< shape type
    std::vector<int> local_rigid;   ///< local shape index in collision model of associated body
    std::vector<int> start_rigid;   ///< start index in the appropriate container of dimensions
    std::vector<int> length_rigid;  ///< usually 1, except for convex

    std::vector<quaternion> ObR_rigid;  ///< shape rotations
    std::vector<real3> ObA_rigid;       ///< shape positions

    std::vector<real3> obj_data_A_global;
    std::vector<quaternion> obj_data_R_global;

    std::vector<real> sphere_rigid;      ///< radius for sphere shapes
    std::vector<real3> box_like_rigid;   ///< dimensions for box-like shapes
    std::vector<real3> triangle_rigid;   ///< vertices of all triangle shapes (3 per shape)
    std::vector<real2> capsule_rigid;    ///< radius and half-length for capsule shapes
    std::vector<real4> rbox_like_rigid;  ///< dimensions and radius for rbox-like shapes
    std::vector<real3> convex_rigid;     ///< points for convex hull shapes

    std::vector<real3> triangle_global;  ///< triangle vertices in global frame
};

/// Structure of arrays containing state data.
struct state_container {
    state_container()
        : num_rigid_bodies(0),
          num_fluid_bodies(0),
          pos_rigid(nullptr),
          rot_rigid(nullptr),
          active_rigid(nullptr),
          collide_rigid(nullptr),
          pos_3dof(nullptr),
          sorted_pos_3dof(nullptr) {}

    // Counters
    uint num_rigid_bodies;  ///< number of rigid bodies in a system
    uint num_fluid_bodies;  ///< number of fluid bodies in the system

    // Object data
    std::vector<real3>* pos_rigid;       ///< [num_rigid_bodies] rigid body positions
    std::vector<quaternion>* rot_rigid;  ///< [num_rigid_bodies] rigid body rotations
    std::vector<char>* active_rigid;     ///< [num_rigid_bodies] flags indicating rigid bodies that active
    std::vector<char>* collide_rigid;    ///< [num_rigid_bodies] flags indicating bodies that participate in collision

    // Information for 3dof nodes
    std::vector<real3>* pos_3dof;         ///< [num_fluid_bodies] 3-dof particle positions
    std::vector<real3>* sorted_pos_3dof;  ///< [num_fluid_bodies] (output) 3-dof particle positions sorted by bin index
};

/// Global data for the custom Chrono multicore collision system.
class ChApi ChCollisionData {
  public:
    ChCollisionData(bool owns_data)
        : owns_state_data(owns_data),
          //
          collision_envelope(0),
          //
          p_collision_envelope(0),
          p_kernel_radius(real(0.04)),
          p_collision_family(short2(1, 0x7FFF)),
          //
          bins_per_axis(vec3(10, 10, 10)),
          //
          bin_size(real3(0)),
          min_bounding_point(real3(0)),
          max_bounding_point(real3(0)),
          global_origin(real3(0)),
          num_bins(0),
          num_bin_aabb_intersections(0),
          num_active_bins(0),
          num_possible_collisions(0),
          //
          rigid_min_bounding_point(real3(0)),
          rigid_max_bounding_point(real3(0)),
          //
          ff_min_bounding_point(real3(0)),
          ff_max_bounding_point(real3(0)),
          ff_bins_per_axis(vec3(0)),
          //
          num_rigid_shapes(0),
          num_rigid_contacts(0),
          num_rigid_fluid_contacts(0),
          num_fluid_contacts(0) {
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

    state_container state_data;  ///< state data arrays
    shape_container shape_data;  ///< shape information data arrays

    real collision_envelope;  ///< collision envelope for rigid shapes

    real p_collision_envelope;  ///< collision envelope for 3-dof particles
    real p_kernel_radius;       ///< 3-dof particle radius
    short2 p_collision_family;  ///< collision family and family mask for 3-dof particles

    // Collision detection output data
    // -------------------------------

    std::vector<real3> aabb_min;  ///< list of bounding boxes minimum point
    std::vector<real3> aabb_max;  ///< list of bounding boxes maximum point

    std::vector<long long> pair_shapeIDs;     ///< shape IDs for each shape pair (encoded in a single long long)
    std::vector<long long> contact_shapeIDs;  ///< shape IDs for each contact (encoded in a single long long)

    // Rigid-rigid geometric collision data
    std::vector<real3> norm_rigid_rigid;  ///< [num_rigid_contacts] normal for each rigid-rigid contact
    std::vector<real3> cpta_rigid_rigid;  ///< [num_rigid_contacts] point on first shape for each rigid-rigid contact
    std::vector<real3> cptb_rigid_rigid;  ///< [num_rigid_contacts] point on second shape for each rigid-rigid contact
    std::vector<real> dpth_rigid_rigid;   ///< [num_rigid_contacts] penetration depth for each rigid-rigid contact
    std::vector<real> erad_rigid_rigid;  ///< [num_rigid_contacts] effective contact radius for each rigid-rigid contact
    std::vector<vec2> bids_rigid_rigid;  ///< [num_rigid_contacts] body IDs for each rigid-rigid contact pair

    // Rigid-particle geometric collision data
    std::vector<real3> norm_rigid_fluid;    ///< [num_rigid_fluid_contacts]
    std::vector<real3> cpta_rigid_fluid;    ///< [num_rigid_fluid_contacts]
    std::vector<real> dpth_rigid_fluid;     ///< [num_rigid_fluid_contacts]
    std::vector<int> neighbor_rigid_fluid;  ///< [num_rigid_fluid_contacts]
    std::vector<int> c_counts_rigid_fluid;  ///< [num_fluid_bodies]

    // 3dof particle neighbor information
    std::vector<int> neighbor_3dof_3dof;  ///< [num_fluid_contacts]
    std::vector<int> c_counts_3dof_3dof;  ///< [num_fluid_contacts]

    // Sorting map for 3dof particles
    std::vector<int> particle_indices_3dof;
    std::vector<int> reverse_mapping_3dof;

    // Broadphase Data
    vec3 bins_per_axis;               ///< number of slices along each axis of the collision detection grid
    real3 bin_size;                   ///< bin sizes in each direction
    real3 inv_bin_size;               ///< bin size reciprocals in each direction
    real3 min_bounding_point;         ///< LBR (left-bottom-rear) corner of union of all AABBs
    real3 max_bounding_point;         ///< RTF (right-top-front) corner of union of all AABBs
    real3 global_origin;              ///< grid zero point (same as LBR)
    uint num_bins;                    ///< total number of bins
    uint num_bin_aabb_intersections;  ///< number of bin - shape AABB intersections
    uint num_active_bins;             ///< number of bins intersecting at least one shape AABB
    uint num_possible_collisions;     ///< number of candidate collisions from broadphase

    real3 rigid_min_bounding_point;  ///< LBR (left-bottom-rear) corner of union of rigid AABBs
    real3 rigid_max_bounding_point;  ///< RTF (right-top-front) corner of union of rigid AABBs

    real3 ff_min_bounding_point;  ///< LBR (left-bottom-rear) corner of union of fluid AABBs
    real3 ff_max_bounding_point;  ///< RTF (right-top-front) corner of union of fluid AABBs
    vec3 ff_bins_per_axis;        ///< grid resolution for fluid particles

    std::vector<uint> bin_intersections;    ///< [num_rigid_shapes+1] number of bin intersections for each shape AABB
    std::vector<uint> bin_number;           ///< [num_bin_aabb_intersections] bin index for bin-shape AABB intersections
    std::vector<uint> bin_aabb_number;      ///< [num_bin_aabb_intersections] shape ID for bin-shape AABB intersections
    std::vector<uint> bin_active;           ///< [num_active_bins] bin index of active bins (no duplicates)
    std::vector<uint> bin_start_index;      ///< [num_active_bins+1]
    std::vector<uint> bin_start_index_ext;  ///< [num_bins+1]
    std::vector<uint> bin_num_contact;      ///< [num_active_bins+1]

    // Indexing variables
    // ------------------

    uint num_rigid_shapes;          ///< number of collision models in a system
    uint num_rigid_contacts;        ///< number of contacts between rigid bodies in a system
    uint num_rigid_fluid_contacts;  ///< number of contacts between rigid and fluid objects
    uint num_fluid_contacts;        ///< number of contacts between fluid objects
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
