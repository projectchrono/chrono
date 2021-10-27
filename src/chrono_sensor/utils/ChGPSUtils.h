
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

#include "chrono/core/ChVector.h"
#include "chrono/core/ChMathematics.h"
#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// definition of Earth's radius to be used throughout the GPS sensor computations
#define EARTH_RADIUS 6371000.0

/// Utility function for calculating Cartesian coordinates from GPS coordinates given the simulation's reference point
/// @param coords The Cartisian coordinates to be modified. This ChVector is modified and used as output
/// @param ref The simulation's reference location
CH_SENSOR_API void Cartesian2GPS(ChVector<double>& coords, ChVector<double>& ref);

/// Utility function for calculating GPS coordinates from Cartesian coordinates given the simulation's reference point
/// @param coords The GPS coordinates to be modified. This ChVector is modified and used as output
/// @param ref The simulation's reference location
CH_SENSOR_API void GPS2Cartesian(ChVector<double>& coords, ChVector<double>& ref);

}  // namespace sensor
}  // namespace chrono
#endif
