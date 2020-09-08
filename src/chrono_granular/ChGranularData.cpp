// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================
// Global functions for accessing the Chrono::Granular data path.
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono_granular/ChGranularData.h"

namespace chrono {
namespace granular {

std::string chrono_granular_data_path("../data/granular/");

// Set the path to the Chrono::Granular data directory (ATTENTION: not thread safe).
void SetDataPath(const std::string& path) {
    chrono_granular_data_path = path;
}

// Obtain the current path to the Chrono::Vehicle data directory (thread safe).
const std::string& GetDataPath() {
    return chrono_granular_data_path;
}

// Get the complete path to the specified filename (thread safe).
// The filename is assumed to be given relative to the Chrono::Granular
// data directory.
std::string GetDataFile(const std::string& filename) {
    return chrono_granular_data_path + filename;
}

}  // end namespace granular
}  // end namespace chrono