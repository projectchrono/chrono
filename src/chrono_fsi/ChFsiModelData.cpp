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
// Authors: Arman Pazouki
// =============================================================================
//
// Global functions for accessing the Chrono::Fsi model data.
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"
#include "chrono_fsi/ChFsiModelData.h"

namespace chrono {
//namespace fsi {

static std::string chrono_fsi_data_path("../data/fsi/");

// Set the path to the Chrono::Fsi data directory (ATTENTION: not thread safe).
void SetFsiDataPath(const std::string& path) {
    chrono_fsi_data_path = path;
}

// Obtain the current path to the Chrono::Fsi data directory (thread safe).
const std::string& GetFsiDataPath() {
    return chrono_fsi_data_path;
}

// Get the complete path to the specified filename (thread safe).
// The filename is assumed to be given relative to the Chrono::Fsi model
// data directory.
std::string GetFsiDataFile(const std::string& filename) {
    return chrono_fsi_data_path + filename;
}

//}  // end namespace fsi
}  // end namespace chrono
