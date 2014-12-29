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
// Global functions for accessing the model data.
//
// =============================================================================

#include "ChUtilsData.h" 


namespace chrono {
namespace utils {


// -----------------------------------------------------------------------------
// Functions for manipulating the ChronoT data directory
// -----------------------------------------------------------------------------

static std::string model_data_path("../data/");

// Set the path to the ChronoT model data directory (ATTENTION: not thread safe)
void SetModelDataPath(const std::string& path)
{
  model_data_path = path;
}

// Obtain the current path to the ChronoT model data directory (thread safe)
const std::string& GetModelDataPath()
{
  return model_data_path;
}

// Obtain the complete path to the specified filename, given relative to the
// ChronoT model data directory (thread safe)
std::string GetModelDataFile(const std::string& filename)
{
  return model_data_path + filename;
}


} // end namespace utils
} // end namespace chrono

