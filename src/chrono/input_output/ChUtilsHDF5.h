// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility functions for parsing HDF5 files.
//
// =============================================================================

#ifndef CH_UTILS_HDF5_H
#define CH_UTILS_HDF5_H

#include <string>

#include <H5Cpp.h>

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

ChApi double ReadDouble(H5::H5File& file, const std::string& data_name);

ChApi ChVectorDynamic<> ReadVector(H5::H5File& file, const std::string& data_name);

ChApi ChMatrixDynamic<> ReadMatrix(H5::H5File& file, const std::string& data_name);

/// @} chrono_io

}  // namespace chrono

#endif
