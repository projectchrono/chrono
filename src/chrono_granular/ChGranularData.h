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

#pragma once

#include <string>

#include "chrono_granular/api/ChApiGranular.h"

namespace chrono {
namespace granular {

/// @addtogroup granular
/// @{

/// Set the path to the Chrono::Granular data directory (ATTENTION: not thread safe).
CH_GRANULAR_API void SetDataPath(const std::string& path);

/// Get the current path to the Chrono::Granular data directory (thread safe).
CH_GRANULAR_API const std::string& GetDataPath();

/// Get the complete path to the specified filename (thread safe).
/// The filename is assumed to be given relative to the Chrono::Granular
/// data directory.
CH_GRANULAR_API std::string GetDataFile(const std::string& filename);

/// @} granular

}  // end namespace granular
}  // end namespace chrono