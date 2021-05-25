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
    /// Narrowphase algorithm
    enum class Algorithm {
        MPR,    ///< Minkovski Portal Refinement for convex-convex collision
        PRIMS,  ///< Analytical collision algorithms for primitive shapes
        HYBRID  ///< Analytical collision algorithms with fallback on MPR
    };

    ChNarrowphase();
    ~ChNarrowphase() {}

    /// Clear contact data structures.
    void ClearContacts();

    /// Perform collision detection.
    void ProcessRigids(const vec3& bins_per_axis);

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
    void DispatchRigidFluid(const vec3& bins_per_axis);
    void DispatchFluid();

    void DispatchMPR();
    void DispatchPRIMS();
    void DispatchHybridMPR();
    void Dispatch_Init(uint index, uint& icoll, uint& ID_A, uint& ID_B, ConvexShape* shapeA, ConvexShape* shapeB);
    void Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC);

  private:
    std::shared_ptr<ChCollisionData> data_manager;

    std::vector<char> contact_rigid_active;
    std::vector<char> contact_rigid_fluid_active;
    std::vector<char> contact_fluid_active;
    std::vector<uint> contact_index;

    uint num_potential_rigid_contacts;        //// TODO: obsolete these!
    uint num_potential_fluid_contacts;
    uint num_potential_rigid_fluid_contacts;

    Algorithm algorithm;

    std::vector<uint> f_bin_intersections;
    std::vector<uint> f_bin_number;
    std::vector<uint> f_bin_number_out;
    std::vector<uint> f_bin_fluid_number;
    std::vector<uint> f_bin_start_index;
    std::vector<uint> is_rigid_bin_active;
    uint f_number_of_bins_active;          //// TODO: obsolete this?
    std::vector<int> ff_bin_ids;
    std::vector<int> ff_bin_starts;
    std::vector<int> ff_bin_ends;

    std::vector<uint> t_bin_intersections;
    std::vector<uint> t_bin_number;
    std::vector<uint> t_bin_number_out;
    std::vector<uint> t_bin_fluid_number;
    std::vector<uint> t_bin_start_index;

    friend class ChCollisionSystemChrono;
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
