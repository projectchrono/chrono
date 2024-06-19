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
// Generic simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/generic/brake/Generic_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace generic {

const double Generic_BrakeSimple::m_maxtorque = 4000;

Generic_BrakeSimple::Generic_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
