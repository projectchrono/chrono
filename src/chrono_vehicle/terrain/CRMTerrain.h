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
class CH_VEHICLE_API CRMTerrain : public ChTerrain, public fsi::ChFsiProblemCartesian {
  public:
    /// Create a CRM terrain object.
    CRMTerrain(ChSystem& sys, double spacing);

    //// TODO - anything needed here?
    virtual void Synchronize(double time) override {}
    virtual double GetHeight(const ChVector3d& loc) const override { return 0.0; }
    virtual chrono::ChVector3d GetNormal(const ChVector3d& loc) const override { return ChWorldFrame::Vertical(); }
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return 0.0f; }
};

/// @} vehicle_terrain

}  // namespace vehicle
}  // namespace chrono

#endif
