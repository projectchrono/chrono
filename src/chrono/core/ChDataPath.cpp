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

#include <filesystem>

#include "chrono/core/ChDataPath.h"

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
// Functions for manipulating the Chrono demo output directory
// -----------------------------------------------------------------------------

static std::string chrono_out_path("DEMO_OUTPUT/");

void SetChronoOutputPath(const std::string& path) {
    chrono_out_path = path;
}

const std::string& GetChronoOutputPath() {
    CreateOutputDirectory(chrono_out_path);
    return chrono_out_path;
}

// -----------------------------------------------------------------------------
// Functions for manipulating the Chrono test output directory
// -----------------------------------------------------------------------------

static std::string chrono_test_out_path("TEST_OUTPUT/");

void SetChronoTestOutputPath(const std::string& path) {
    chrono_test_out_path = path;
}

const std::string& GetChronoTestOutputPath() {
    CreateOutputDirectory(chrono_test_out_path);
    return chrono_test_out_path;
}

// -----------------------------------------------------------------------------
// Functions for creating directories
// -----------------------------------------------------------------------------

bool CreateOutputDirectory(const std::filesystem::path& p) {
    // Do not check for existence first: another process (e.g., another participant in a co-simulation) may create the
    // directory between that check and the creation attempt, in which case create_directories reports that it did not
    // create anything. Instead, attempt the creation (including any missing parent directories), ignore its outcome,
    // and report success if the directory exists afterwards, regardless of who created it.
    std::error_code ec;
    std::filesystem::create_directories(p, ec);
    return std::filesystem::is_directory(p, ec);
}

bool CreateOutputDirectory(const std::string& p) {
    return CreateOutputDirectory(std::filesystem::path(p));
}


}  // end namespace chrono
