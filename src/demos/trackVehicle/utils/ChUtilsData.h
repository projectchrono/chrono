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
// Authors: Radu Serban
// =============================================================================
//
// Global functions for accessing the ChronoT model data.
//
// =============================================================================

#ifndef CH_UTILSGLOBAL_H
#define CH_UTILSGLOBAL_H


#include <string>
#include "ChApiUtils.h"


namespace chrono {
namespace utils{


/// Set the path to the ChronoT model data directory (ATTENTION: not thread safe)
CH_UTILS_API void SetModelDataPath(const std::string& path);

/// Obtain the current path to the ChronoT model data directory (thread safe)
CH_UTILS_API const std::string& GetModelDataPath();

/// Obtain the complete path to the specified filename, given relative to the
/// ChronoT model data directory (thread safe)
CH_UTILS_API std::string GetModelDataFile(const std::string& filename);


} // end namespace utils
} // end namespace chrono


#endif
