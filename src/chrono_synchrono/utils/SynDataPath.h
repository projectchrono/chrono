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

#ifndef SYN_DATA_PATH_H
#define SYN_DATA_PATH_H

#include <string>

#include "chrono_synchrono/SynApi.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_utils
/// @{

/// Set the path to the SynChrono data directory (ATTENTION: not thread safe).
SYN_API void SetSynchronoDataPath(const std::string& path);

/// Obtain the current path to the SynChrono data directory (thread safe).
SYN_API const std::string& GetSynchronoDataPath();

/// Obtain the complete path to the specified filename, given relative to the SynChrono data directory (thread safe).
SYN_API std::string GetSynchronoDataFile(const std::string& filename);

/// @} synchrono_utils

}  // namespace synchrono
}  // namespace chrono

#endif
