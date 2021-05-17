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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: Class definitions for the NarrowphaseDispatch
//
// =============================================================================

#pragma once

#include "chrono/collision/ChCollisionModel.h"

#include "chrono/collision/chrono/ChCollisionData.h"

namespace chrono {
namespace collision {

class ConvexShape;

/// @addtogroup collision_mc
/// @{

/// Class for performing narrow-phase collision detection.
class ChApi ChNarrowphase {
  public:
    ChNarrowphase() {}
    ~ChNarrowphase() {}

    /// Clear contact data structures.
    void ClearContacts();

    /// Perform collision detection.
    void ProcessRigids();

    /// Calculate total number of potential contacts.
    int PreprocessCount();

    /// Transform the shape data to the global reference frame.
    /// Perform this as a preprocessing step to improve performance.
    /// Performance is improved because the amount of data loaded is still the same
    /// but it does not have to be transformed per contact pair, now it is
    /// transformed once per shape.
    void PreprocessLocalToParent();

    // For each contact pair decide what to do.
    void DispatchRigid();
    void DispatchRigidFluid();
    void DispatchFluid();

    void SphereSphereContact();
    void RigidSphereContact();

    void DispatchMPR();
    void DispatchR();
    void DispatchHybridMPR();
    void Dispatch_Init(uint index, uint& icoll, uint& ID_A, uint& ID_B, ConvexShape* shapeA, ConvexShape* shapeB);
    void Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC);

    std::shared_ptr<ChCollisionData> data_manager;

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

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
