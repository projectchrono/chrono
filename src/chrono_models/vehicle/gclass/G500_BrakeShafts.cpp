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
// Authors: Radu Serban
// =============================================================================
//
// G500 shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/gclass/G500_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace gclass {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double G500_BrakeShaftsFront::m_maxtorque = 1600;
const double G500_BrakeShaftsFront::m_shaft_inertia = 0.4;

const double G500_BrakeShaftsRear::m_maxtorque = 1500;
const double G500_BrakeShaftsRear::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
G500_BrakeShaftsFront::G500_BrakeShaftsFront(const std::string& name) : ChBrakeShafts(name) {}

G500_BrakeShaftsRear::G500_BrakeShaftsRear(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
