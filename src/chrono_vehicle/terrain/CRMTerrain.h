// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Continuum representation (SPH-based) deformable terrain model.
//
// Reference frame is ISO (X forward, Y left, Z up).
// All units SI.
//
// =============================================================================

#ifndef CRM_TERRAIN_H
#define CRM_TERRAIN_H

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChWorldFrame.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Continuum representation (CRM) deformable terrain model using SPH.
class CH_VEHICLE_API CRMTerrain : public ChTerrain, public fsi::sph::ChFsiProblemCartesian {
  public:
    /// Create a CRM terrain object.
    CRMTerrain(ChSystem& sys, double spacing);

    /// Set dimensions of the active domain AABB.
    /// This value activates only those SPH particles that are within an AABB of the specified size from an object
    /// interacting with the "fluid" phase.
    void SetActiveDomain(const ChVector3d& box_dim);

    /// Set the delay time for the active domain.
    void SetActiveDomainDelay(double delay);

    /// Register callback to control dynamics of an associated vehicle.
    /// If using this function, the calling program should not explicitly call ChVehicle::Advance. This function will be
    /// called, in the appropriate sequence, when advancing the dynamics of the coupled FSI problem through
    /// CRMTerrain::Advance.
    void RegisterVehicle(ChVehicle* vehicle);

    /// Construct a rectangular terrain with moving patch capabilities.
    /// The terrain patch is rectangular (of the specified dimensions,), and positioned so that the "lowest" point of
    /// the terrain box is at the global origin. The moving boundary is always assumed to be in the positive x
    /// direction. The algorithm monitors the distance from a sentinel body (see SetMovingPatchSentinel) to the current
    /// front boundary. When the sentinel nears the front boundary, the rear and front boundary BCE markers are shifted
    /// by the specified shift distance and SPH particles relocated from rear to front.
    void ConstructMovingPatch(const ChVector3d& box_size,    ///< box dimensions
                              std::shared_ptr<ChBody> body,  ///< tracked body
                              double buffer_distance,        ///< look-ahead distance
                              double shift_distance          ///< chunk size of relocated particles
    );

    /// Return true after a call to Synchronize during which the moving patch was relocated.
    bool PatchMoved() const { return m_moved; }

    virtual void Synchronize(double time) override;
    virtual void Advance(double step) override;
    virtual double GetHeight(const ChVector3d& loc) const override { return 0.0; }
    virtual chrono::ChVector3d GetNormal(const ChVector3d& loc) const override { return ChWorldFrame::Vertical(); }
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return 0.0f; }

  private:
    void UpdateAABBs();

    std::shared_ptr<ChBody> m_sentinel;  ///< tracked sentinel body
    bool m_moving_patch;                 ///< moving patch feature enabled?
    bool m_moved;                        ///< was the patch moved?
    double m_buffer;                     ///< minimum distance to front boundary

    int m_Ishift;  ///< number of grid cells in X direction of relocated volume
    int m_Irear;   ///< current X grid coordinate of rear-most SPH particles
    int m_Ifront;  ///< current X grid coordinate of front-most SPH particles

    ChAABB m_rearAABB;       ///< AABB containing the particles to be moved
    ChAABB m_frontAABB;      ///< AABB destination for moved particles
    ChIntAABB m_IfrontAABB;  ///< grid AABB destination for moved particles
};

/// @} vehicle_terrain

}  // namespace vehicle
}  // namespace chrono

#endif
