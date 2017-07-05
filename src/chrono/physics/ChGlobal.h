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

#ifndef CHGLOBAL_H
#define CHGLOBAL_H

#include <string>
#include "chrono/core/ChApiCE.h"

namespace chrono {

/// Set the start value for the sequence of IDs (ATTENTION: not thread safe)
/// Subsequent calls to GetUniqueIntID() will return val+1, val+2, etc.
/// The default initial value is 100000.
ChApi void SetFirstIntID(int val);

/// Obtain a unique identifier (thread-safe)
ChApi int GetUniqueIntID();

/// Set the path to the Chrono data directory (ATTENTION: not thread safe)
ChApi void SetChronoDataPath(const std::string& path);

/// Obtain the current path to the Chrono data directory (thread safe)
ChApi const std::string& GetChronoDataPath();

/// Obtain the complete path to the specified filename, given relative to the
/// Chrono data directory (thread safe)
ChApi std::string GetChronoDataFile(const std::string& filename);

/// Obtain the path to the output directory for Chrono demos.
ChApi const std::string& GetChronoOutputPath();

}  // end namespace chrono

#endif
