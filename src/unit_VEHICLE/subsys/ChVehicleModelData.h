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
// Global functions for accessing the ChronoVehicle model data.
//
// =============================================================================

#ifndef CH_VEHICLE_MODELDATA_H
#define CH_VEHICLE_MODELDATA_H


#include <string>

#include "subsys/ChApiSubsys.h"


namespace chrono {
namespace vehicle {


/// Set the path to the ChronoVehicle model data directory (ATTENTION: not thread safe)
CH_SUBSYS_API void SetDataPath(const std::string& path);

/// Obtain the current path to the ChronoVehicle model data directory (thread safe)
CH_SUBSYS_API const std::string& GetDataPath();

/// Obtain the complete path to the specified filename, given relative to the
/// ChronoVehicle model data directory (thread safe)
CH_SUBSYS_API std::string GetDataFile(const std::string& filename);


} // end namespace vehicle
} // end namespace chrono


#endif
