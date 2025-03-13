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

#ifndef CRM_SPH_TERRAIN_H
#define CRM_SPH_TERRAIN_H

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
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

    /// Set half-dimensions of the active domain.
    /// This value activates only those SPH particles that are within an AABB of the specified size from an object
    /// interacting with the "fluid" phase.
    void SetActiveDomain(const ChVector3d& half_dim);

    /// Set the delay time for the active domain.
    void SetActiveDomainDelay(double delay);

    virtual void Synchronize(double time) override {}
    virtual void Advance(double step) override;
    virtual double GetHeight(const ChVector3d& loc) const override { return 0.0; }
    virtual chrono::ChVector3d GetNormal(const ChVector3d& loc) const override { return ChWorldFrame::Vertical(); }
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return 0.0f; }
};

/// @} vehicle_terrain

}  // namespace vehicle
}  // namespace chrono

#endif
