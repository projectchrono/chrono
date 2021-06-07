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
// Global functions for accessing the Chrono::Gpu data path.
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono_gpu/ChGpuData.h"

namespace chrono {
namespace gpu {

std::string chrono_gpu_data_path("../data/gpu/");

// Set the path to the Chrono::Gpu data directory (ATTENTION: not thread safe).
void SetDataPath(const std::string& path) {
    chrono_gpu_data_path = path;
}

// Obtain the current path to the Chrono::Vehicle data directory (thread safe).
const std::string& GetDataPath() {
    return chrono_gpu_data_path;
}

// Get the complete path to the specified filename (thread safe).
// The filename is assumed to be given relative to the Chrono::Gpu
// data directory.
std::string GetDataFile(const std::string& filename) {
    return chrono_gpu_data_path + filename;
}

}  // end namespace gpu
}  // end namespace chrono