// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Base class for all Vehicle Brains - in particular forces the user to include
// a ChDriver that will determine the throttle/braking/steering inputs
//
// =============================================================================

#include "chrono_synchrono/brain/SynVehicleBrain.h"

namespace chrono {
namespace synchrono {

void SynVehicleBrain::Advance(double step) {
    m_driver->Advance(step);
}

void SynVehicleBrain::Synchronize(double time) {
    m_driver->Synchronize(time);
}

}  // namespace synchrono
}  // namespace chrono
