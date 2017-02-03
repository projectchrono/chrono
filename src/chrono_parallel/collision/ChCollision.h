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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Class definitions for the AABB generator, Broadpahse
// and Narrowphase
// =============================================================================

#pragma once

#include "chrono/collision/ChCCollisionModel.h"

#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {
namespace collision {

class ConvexShape;

class CH_PARALLEL_API ChCAABBGenerator {
  public:
    ChCAABBGenerator();
    void GenerateAABB();
    ChParallelDataManager* data_manager;
};

class CH_PARALLEL_API ChCBroadphase {
  public:
    ChCBroadphase();
    void DispatchRigid();
    void OneLevelBroadphase();
    void DetermineBoundingBox();
    void OffsetAABB();
    void ComputeTopLevelResolution();
    void RigidBoundingBox();
    void FluidBoundingBox();
    void TetBoundingBox();
    ChParallelDataManager* data_manager;

  private:
};

class CH_PARALLEL_API ChCNarrowphaseDispatch {
  public:
    ChCNarrowphaseDispatch() {}
    ~ChCNarrowphaseDispatch() {}
    // clear contact data structures
    void ClearContacts();
    // Perform collision detection
    void ProcessRigids();

    void PreprocessCount();
    // Transform the shape data to the global reference frame
    // Perform this as a preprocessing step to improve performance
    // Performance is improved because the amount of data loaded is still the same
    // but it does not have to be transformed per contact pair, now it is
    // transformed once per shape
    void PreprocessLocalToParent();

    // For each contact pair decide what to do.
    void DispatchRigid();
    void DispatchRigidFluid();
    void DispatchRigidTet();
    void DispatchFluid();

    void SphereSphereContact(const int num_fluid_bodies,
                             const int body_offset,
                             const real radius,
                             const real collision_envelope,
                             const real3& min_bounding_point,
                             const real3& max_bounding_point,
                             const custom_vector<real3>& pos_fluid,
                             const custom_vector<real3>& vel_fluid,
                             custom_vector<real3>& sorted_pos_fluid,
                             custom_vector<real3>& sorted_vel_fluid,
                             DynamicVector<real>& v,
                             custom_vector<int>& neighbor_fluid_fluid,
                             custom_vector<int>& contact_counts,
                             custom_vector<int>& particle_indices,
                             custom_vector<int>& reverse_mapping,
                             vec3& bins_per_axis,
                             uint& num_fluid_contacts);

    void RigidSphereContact(const real sphere_radius,
                            const int num_spheres,
                            const custom_vector<real3>& pos_sphere,
                            const short2& family,
                            custom_vector<real3>& norm_rigid_sphere,
                            custom_vector<real3>& cpta_rigid_sphere,
                            custom_vector<real>& dpth_rigid_sphere,
                            custom_vector<int>& neighbor_rigid_sphere,
                            custom_vector<int>& contact_counts,
                            uint& num_contacts);

    void RigidTetContact(custom_vector<real3>& norm_rigid_tet,
                         custom_vector<real3>& cpta_rigid_tet,
                         custom_vector<real3>& cptb_rigid_tet,
                         custom_vector<real>& dpth_rigid_tet,
                         custom_vector<int>& neighbor_rigid_tet,
                         custom_vector<real4>& face_rigid_tet,
                         custom_vector<int>& contact_counts,
                         uint& num_contacts);

    void MarkerTetContact(const real sphere_radius,
                          const int num_spheres,
                          const custom_vector<real3>& pos_sphere,
                          const short2& family_sphere,
                          custom_vector<real3>& norm_marker_tet,
                          custom_vector<real3>& cptb_marker_tet,
                          custom_vector<real>& dpth_marker_tet,
                          custom_vector<int>& neighbor_marker_tet,
                          custom_vector<real4>& face_marker_tet,
                          custom_vector<int>& contact_counts,
                          uint& num_contacts);

    void DispatchMPR();
    void DispatchR();
    void DispatchHybridMPR();
    void Dispatch_Init(uint index, uint& icoll, uint& ID_A, uint& ID_B, ConvexShape* shapeA, ConvexShape* shapeB);
    void Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC);
    ChParallelDataManager* data_manager;

  private:
    custom_vector<char> contact_rigid_active;
    custom_vector<char> contact_rigid_fluid_active;
    custom_vector<char> contact_fluid_active;
    custom_vector<uint> contact_index;
    uint num_potential_rigid_contacts;
    uint num_potential_fluid_contacts;
    uint num_potential_rigid_fluid_contacts;

    real collision_envelope;
    NarrowPhaseType narrowphase_algorithm;
    SystemType system_type;

    custom_vector<uint> f_bin_intersections;
    custom_vector<uint> f_bin_number;
    custom_vector<uint> f_bin_number_out;
    custom_vector<uint> f_bin_fluid_number;
    custom_vector<uint> f_bin_start_index;
    custom_vector<uint> is_rigid_bin_active;
    uint f_number_of_bins_active;
    custom_vector<int> ff_bin_ids;
    custom_vector<int> ff_bin_starts;
    custom_vector<int> ff_bin_ends;

    custom_vector<uint> t_bin_intersections;
    custom_vector<uint> t_bin_number;
    custom_vector<uint> t_bin_number_out;
    custom_vector<uint> t_bin_fluid_number;
    custom_vector<uint> t_bin_start_index;
};
}
}
