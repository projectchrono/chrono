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
// Dispatcher for the narrowphase collision detection phase.
//
// =============================================================================

#pragma once

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/multicore/ChCollisionData.h"
#include "chrono/collision/multicore/ChConvexShape.h"

namespace chrono {

/// @addtogroup collision_mc
/// @{

/// Class for performing narrowphase collision detection. The default is a hybrid approach where pairs of known
/// primitive shapes are treated analytically with fallback to a Minkovski Portal Refinement algorithm for any other
/// pair of shapes.
///
/// Currently supported analytical pair-wise interactions:
/// <pre>
///          |  sphere   box   rbox   capsule   cylinder   rcyl   trimesh
/// ---------+----------------------------------------------------------
/// sphere   |    Y       Y      Y       Y         Y        Y        Y
/// box      |            Y      N       Y         N        N        Y
/// rbox     |                   N       N         N        N        N
/// capsule  |                           Y         N        N        N
/// cylinder |                                     N        N        N
/// rcyl     |                                              N        N
/// trimesh  |                                                       N
/// </pre>
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

    /// Perform narrowphase collision detection.
    /// This function generates rigid-rigid, fluid-fluid, and rigid-fluid collisions, as applicable.
    /// Collision detection results are loaded in the shared data object (see ChCollisionData).
    void Process();

    /// Minkovski Portal Refinement convex-convex collision detection (adapted from Xeno Collide).
    /// Each candidate pair can result in 0 or 1 contacts. For each contact, the function calculates and returns:
    ///   - pointA:   contact point on first shape (in global frame)
    ///   - pointB:   contact point on second shape (in global frame)
    ///   - depth:    penetration distance (negative if overlap exists)
    ///   - normal:   contact normal, from ct_pt2 to ct_pt1 (in global frame)
    static bool MPRCollision(const ConvexBase* ShapeA,
                             const ConvexBase* ShapeB,
                             real envelope,
                             real3& normal,
                             real3& pointA,
                             real3& pointB,
                             real& depth);

    /// Dispatcher for analytic collision functions between a pair of candidate shapes.
    /// Each candidate pair of shapes can result in 0, 1, or more contacts.  For each actual contact, the function
    /// calculates various geometrical quantities and load them in the output arguments (starting from the given
    /// addresses)
    ///   - ct_pt1:      contact point on first shape (in global frame)
    ///   - ct_pt2:      contact point on second shape (in global frame)
    ///   - ct_depth:    penetration distance (negative if overlap exists)
    ///   - ct_norm:     contact normal, from ct_pt2 to ct_pt1 (in global frame)
    ///   - ct_eff_rad:  effective contact radius
    ///
    /// Note that we also report collisions for which the distance between the two shapes is at most 'separation'
    /// (typically twice the collision envelope). In these cases, the corresponding ct_depth is a positive value. This
    /// function returns true if it was able to determine the collision state for the given pair of shapes and false if
    /// the shape types are not supported.
    static bool PRIMSCollision(const ConvexBase* shapeA,  ///< first candidate shape
                               const ConvexBase* shapeB,  ///< second candidate shape
                               real separation,           ///< maximum separation
                               real3* ct_norm,            ///< [output] contact normal (per contact pair)
                               real3* ct_pt1,             ///< [output] point on shape1 (per contact pair)
                               real3* ct_pt2,             ///< [output] point on shape2 (per contact pair)
                               real* ct_depth,            ///< [output] penetration depth (per contact pair)
                               real* ct_eff_rad,          ///< [output] effective contact radius (per contact pair)
                               int& nC                    ///< [output] number of contacts found
    );

    /// Set the fictitious radius of curvature used for collision with a corner or an edge.
    static void SetDefaultEdgeRadius(real radius);

    /// Return the fictitious radius of curvature used for collisions with a corner or an edge.
    static real GetDefaultEdgeRadius();

    static const int max_neighbors = 64;
    static const int max_rigid_neighbors = 32;

  private:
    /// Calculate total number of potential contacts.
    int PreprocessCount();

    /// Transform the shape data to the global reference frame.
    /// Perform this as a preprocessing step to improve performance. Performance is improved because the amount of data
    /// loaded is still the same but it does not have to be transformed per contact pair, now it is transformed once per
    /// shape.
    void PreprocessLocalToParent();

    /// Perform collision detection fluid-fluid.
    void ProcessFluid();

    /// Perform collision detection involving rigid shapes (rigid-rigid and rigid-fluid).
    void ProcessRigids();
    void ProcessRigidRigid();
    void ProcessRigidFluid();

    void DispatchMPR();
    void DispatchPRIMS();
    void DispatchHybridMPR();
    void Dispatch_Init(uint index, uint& icoll, uint& ID_A, uint& ID_B, ConvexShape* shapeA, ConvexShape* shapeB);
    void Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC);

    std::shared_ptr<ChCollisionData> cd_data;

    std::vector<char> contact_rigid_active;
    std::vector<char> contact_rigid_fluid_active;
    std::vector<char> contact_fluid_active;
    std::vector<uint> contact_index;

    uint num_potential_rigid_contacts;
    uint num_potential_fluid_contacts;
    uint num_potential_rigid_fluid_contacts;

    Algorithm algorithm;

    std::vector<uint> f_bin_intersections;
    std::vector<uint> f_bin_number;
    std::vector<uint> f_bin_number_out;  //// TODO: rename to f_bin_active
    std::vector<uint> f_bin_fluid_number;
    std::vector<uint> f_bin_start_index;
    std::vector<uint> is_rigid_bin_active;
    std::vector<int> ff_bin_ids;
    std::vector<int> ff_bin_starts;
    std::vector<int> ff_bin_ends;

    std::vector<uint> t_bin_intersections;
    std::vector<uint> t_bin_number;
    std::vector<uint> t_bin_number_out;
    std::vector<uint> t_bin_fluid_number;
    std::vector<uint> t_bin_start_index;

    friend class ChCollisionSystemMulticore;
    friend class ChCollisionSystemChronoMulticore;
};

/// @} collision_mc

}  // end namespace chrono
