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

#ifndef CH_DATA_PATH_H
#define CH_DATA_PATH_H

#include <string>
#include "chrono/core/ChApiCE.h"

namespace chrono {

/// Set the path to the Chrono data directory (ATTENTION: not thread safe).
ChApi void SetChronoDataPath(const std::string& path);

/// Obtain the current path to the Chrono data directory (thread safe).
ChApi const std::string& GetChronoDataPath();

/// Get the full path to the specified filename, given relative to the Chrono data directory (thread safe).
ChApi std::string GetChronoDataFile(const std::string& filename);

/// Set the path to the Chrono output directory (ATTENTION: not thread safe).
ChApi void SetChronoOutputPath(const std::string& path);

/// Obtain the path to the output directory for Chrono demos.
ChApi const std::string& GetChronoOutputPath();

}  // end namespace chrono

#endif
