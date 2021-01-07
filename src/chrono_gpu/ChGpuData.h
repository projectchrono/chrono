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

#pragma once

#include <string>

#include "chrono_gpu/ChApiGpu.h"

namespace chrono {
namespace gpu {

/// @addtogroup gpu_module
/// @{

/// Set the path to the Chrono::Gpu data directory (ATTENTION: not thread safe).
CH_GPU_API void SetDataPath(const std::string& path);

/// Get the current path to the Chrono::Gpu data directory (thread safe).
CH_GPU_API const std::string& GetDataPath();

/// Get the complete path to the specified filename (thread safe).
/// The filename is assumed to be given relative to the Chrono::Gpu
/// data directory.
CH_GPU_API std::string GetDataFile(const std::string& filename);

/// @} gpu_module

}  // end namespace gpu
}  // end namespace chrono