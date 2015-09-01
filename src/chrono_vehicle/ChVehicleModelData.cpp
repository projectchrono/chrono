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
// Authors: Radu Serban
// =============================================================================
//
// Global function for accessing the Chrono::Vehicle model data.
//
// =============================================================================

#include "physics/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h" 

namespace chrono {
namespace vehicle {

// Obtain the complete path to the specified filename, given relative to the
// ChronoVehicle model data directory (thread safe)
std::string GetDataFile(const std::string& filename) {
    return GetChronoDataPath() + "vehicle/" + filename;
}

}  // end namespace vehicle
} // end namespace chrono

