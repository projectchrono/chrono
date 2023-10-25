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
// Authors: Alessandro Tasora
// =============================================================================
//
// G500 simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/gclass/G500_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace gclass {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double G500_BrakeSimpleFront::m_maxtorque = 1600;

const double G500_BrakeSimpleRear::m_maxtorque = 1500;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
G500_BrakeSimpleFront::G500_BrakeSimpleFront(const std::string& name) : ChBrakeSimple(name) {}

G500_BrakeSimpleRear::G500_BrakeSimpleRear(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
