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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Krone ProfiLiner SP5 simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_trailer_Brake.h"

namespace chrono {
namespace vehicle {
namespace kraz {

const double Kraz_trailer_Brake::m_maxtorque = 13000;

Kraz_trailer_Brake::Kraz_trailer_Brake(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
