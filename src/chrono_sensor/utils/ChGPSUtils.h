
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
// Authors: Asher Elmquist
// =============================================================================
//
// GPS utils
//
// =============================================================================

#ifndef CHGPSUTILS_H
#define CHGPSUTILS_H

#include "chrono/core/ChVector3.h"
#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// definition of Earth's radius to be used throughout the GPS sensor computations
#define EARTH_RADIUS 6371000.0

/// Utility function for calculating Cartesian coordinates from GPS coordinates given the simulation's reference point
/// @param coords The Cartisian coordinates to be modified. This vector is modified and used as output
/// @param ref The simulation's reference location
CH_SENSOR_API void Cartesian2GPS(ChVector3d& coords, ChVector3d& ref);

/// Utility function for calculating GPS coordinates from Cartesian coordinates given the simulation's reference point
/// @param coords The GPS coordinates to be modified. This vector is modified and used as output
/// @param ref The simulation's reference location
CH_SENSOR_API void GPS2Cartesian(ChVector3d& coords, ChVector3d& ref);

/// Utility function for calculating East-North-Up (ENU) coordinates from Cartesian coordinates given the simulation's reference point
/// @param coords The ENU coordinates to be modified. This ChVector is modified and used as output
/// @param ref The simulation's reference location
CH_SENSOR_API void Cartesian2ENU(ChVector3d& coords, ChVector3d& ref);

/// Utility function for calculating Cartesian coordinates from ENU coordinates given the simulation's reference point
/// @param coords The Cartesian coordinates to be modified. This ChVector is modified and used as output
/// @param ref The simulation's reference location
CH_SENSOR_API void ENU2Cartesian(ChVector3d& coords, ChVector3d& ref);

/// Utility function for calculating GPS coordinates from ENU coordinates given the simulation's reference point
/// @param coords The GPS coordinates to be modified. This ChVector is modified and used as output
/// @param ref The simulation's reference location
CH_SENSOR_API void ENU2GPS(ChVector3d& coords, ChVector3d& ref);

/// Utility function for calculating ENU coordinates from GPS coordinates given the simulation's reference point
/// @param coords The ENU coordinates to be modified. This ChVector is modified and used as output
/// @param ref The simulation's reference location
CH_SENSOR_API void GPS2ENU(ChVector3d& coords, ChVector3d& ref);

}  // namespace sensor
}  // namespace chrono
#endif
