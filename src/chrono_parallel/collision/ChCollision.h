// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
// This class generates an AABB for every collision shape
// =============================================================================

#pragma once

#include "collision/ChCCollisionModel.h"

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
    void DetectPossibleCollisions();
    void OneLevelBroadphase();
    void DetermineBoundingBox();
    void OffsetAABB();
    void ComputeTopLevelResolution();
    void DispatchTets();
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
    void DispatchRigidNode();
    void DispatchRigidMPM();
    void DispatchFluid();

    void RigidSphereContact(const real sphere_radius,
                            const int num_spheres,
                            const custom_vector<real3>& pos_sphere,
                            custom_vector<real3>& norm_rigid_sphere,
                            custom_vector<real3>& cpta_rigid_sphere,
                            custom_vector<real>& dpth_rigid_sphere,
                            custom_vector<int>& neighbor_rigid_sphere,
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
    NARROWPHASETYPE narrowphase_algorithm;
    SYSTEMTYPE system_type;

    custom_vector<uint> f_bin_intersections;
    custom_vector<uint> f_bin_number;
    custom_vector<uint> f_bin_number_out;
    custom_vector<uint> f_bin_fluid_number;
    custom_vector<uint> f_bin_start_index;
    custom_vector<uint> is_rigid_bin_active;
    custom_vector<int> ff_bin_ids;
    custom_vector<int> ff_bin_starts;
    custom_vector<int> ff_bin_ends;

    custom_vector<uint> n_bin_intersections;
    custom_vector<uint> n_bin_number;
    custom_vector<uint> n_bin_number_out;
    custom_vector<uint> n_bin_node_number;
    custom_vector<uint> n_bin_start_index;
};
}
}
