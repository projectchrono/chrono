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
// Global functions for accessing the Chrono::Vehicle model data.
//
// =============================================================================

#include "chrono/core/ChDataPath.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

namespace chrono {
namespace vehicle {

static std::string chrono_vehicle_data_path("../data/vehicle/");

// Set the path to the Chrono::Vehicle data directory (ATTENTION: not thread safe).
void SetVehicleDataPath(const std::string& path) {
    chrono_vehicle_data_path = path;
}

// Obtain the current path to the Chrono::Vehicle data directory (thread safe).
const std::string& GetVehicleDataPath() {
    return chrono_vehicle_data_path;
}

// Get the complete path to the specified filename (thread safe).
// The filename is assumed to be given relative to the Chrono::Vehicle model
// data directory.
std::string GetVehicleDataFile(const std::string& filename) {
    return chrono_vehicle_data_path + filename;
}

}  // end namespace vehicle
}  // end namespace chrono
