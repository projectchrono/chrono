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
// chrono_vehicle/ChVehicleModelData.h
//
// =============================================================================

#ifndef SYN_DATA_LOADER_H
#define SYN_DATA_LOADER_H

#include <string>

#include "chrono_synchrono/SynApi.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_utils
/// @{

/// @brief Set the path to the SynChrono data directory (ATTENTION: not thread safe)
SYN_API void SetDataPath(const std::string& path);

/// @brief Obtain the current path to the SynChrono data directory (thread safe)
SYN_API const std::string& GetDataPath();

/// @brief Obtain the complete path to the specified filename, given relative to the SynChrono data directory (thread
/// safe)
SYN_API std::string GetDataFile(const std::string& filename);

/// @} synchrono_utils

}  // namespace synchrono
}  // namespace chrono

#endif
