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

#include "chrono/core/ChDataPath.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Functions for manipulating the Chrono data directory
// -----------------------------------------------------------------------------

static std::string chrono_data_path("../data/");

// Set the path to the Chrono data directory (ATTENTION: not thread safe)
void SetChronoDataPath(const std::string& path) {
    chrono_data_path = path;
}

// Obtain the current path to the Chrono data directory (thread safe)
const std::string& GetChronoDataPath() {
    return chrono_data_path;
}

// Obtain the complete path to the specified filename, given relative to the
// Chrono data directory (thread safe)
std::string GetChronoDataFile(const std::string& filename) {
    return chrono_data_path + filename;
}

// -----------------------------------------------------------------------------
// Functions for manipulating the Chrono data directory
// -----------------------------------------------------------------------------

static std::string chrono_out_path("DEMO_OUTPUT/");

// Set the path to the Chrono output directory (ATTENTION: not thread safe)
void SetChronoOutputPath(const std::string& path) {
    chrono_out_path = path;
}

const std::string& GetChronoOutputPath() {
    // If the directory does not yet exists, create it.
    auto out_path = filesystem::path(chrono_out_path);
    if (!out_path.exists())
        filesystem::create_directory(out_path);

    return chrono_out_path;
}

}  // end namespace chrono
