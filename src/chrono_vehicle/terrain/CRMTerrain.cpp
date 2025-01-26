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

#include "chrono_vehicle/terrain/CRMTerrain.h"

namespace chrono {
namespace vehicle {

CRMTerrain::CRMTerrain(ChSystem& sys, double spacing) : ChFsiProblemCartesian(sys, spacing) {}

void CRMTerrain::SetActiveDomain(const ChVector3d& half_dim) {
    GetFluidSystemSPH().SetActiveDomain(half_dim);
}

void CRMTerrain::Advance(double step) {
    DoStepDynamics(step);
}

}  // end namespace vehicle
}  // end namespace chrono
