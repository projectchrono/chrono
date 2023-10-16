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
// Authors: Alessandro Tasora, Jayne Henry
// =============================================================================
//
// ARTcar simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/artcar/ARTcar_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace artcar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double ARTcar_BrakeSimple::m_maxtorque = 1.0; //TODO

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ARTcar_BrakeSimple::ARTcar_BrakeSimple(const std::string& name) : chrono::vehicle::ChBrakeSimple(name) {}

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono
