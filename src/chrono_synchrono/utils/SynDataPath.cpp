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
// Global functions for accessing SynChrono data files. Modeled off of
// chrono_vehicle/ChVehicleDataPath.h
//
// =============================================================================

#include "chrono_synchrono/utils/SynDataPath.h"

#include "chrono/core/ChDataPath.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

namespace chrono {
namespace synchrono {

void SetChronoDataPaths(const std::string& chrono_main, const std::string& chrono_vehicle) {
    SetChronoDataPath(chrono_main);
    vehicle::SetVehicleDataPath(chrono_vehicle);
}

// -----------------------------------------------------------------------------
// Functions for manipulating the SynChrono data directory
// -----------------------------------------------------------------------------

static std::string synchrono_data_path("../data/synchrono/");

// Set the path to the SynChrono data directory (ATTENTION: not thread safe)
void SetSynchronoDataPath(const std::string& path) {
    synchrono_data_path = path;
}

// Obtain the current path to the SynChrono data directory (thread safe)
const std::string& GetSynchronoDataPath() {
    return synchrono_data_path;
}

// Obtain the complete path to the specified filename, given relative to the SynChrono data directory (thread safe)
std::string GetSynchronoDataFile(const std::string& filename) {
    return synchrono_data_path + filename;
}

}  // namespace synchrono
}  // namespace chrono
