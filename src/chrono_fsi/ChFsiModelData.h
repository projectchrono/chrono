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
// Authors: Arman Pazouki
// =============================================================================
//
// Global functions for accessing the Chrono::Fsi model data.
//
// =============================================================================

#ifndef CH_FSI_MODELDATA_H
#define CH_FSI_MODELDATA_H

#include <string>



namespace chrono {
//namespace fsi {

/// Set the path to the Chrono::Fsi data directory (ATTENTION: not thread safe).
void SetFsiDataPath(const std::string& path);

/// Get the current path to the Chrono::Fsi data directory (thread safe).
const std::string& GetFsiDataPath();

/// Get the complete path to the specified filename (thread safe).
/// The filename is assumed to be given relative to the Chrono::Fsi model
/// data directory.
std::string GetFsiDataFile(const std::string& filename);

//}  // end namespace fsi
}  // end namespace chrono

#endif
