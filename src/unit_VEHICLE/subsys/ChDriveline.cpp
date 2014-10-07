// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle driveline.
//
// =============================================================================

#include "subsys/ChDriveline.h"
#include "subsys/ChVehicle.h"

namespace chrono {


ChDriveline::ChDriveline(ChVehicle*   car,
                         DriveType    type)
: m_car(car),
  m_type(type)
{
  car->m_driveline = this;
}


}  // end namespace chrono
