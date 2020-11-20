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
// Container class for an GPS sensor
//
// =============================================================================

#ifndef CHGPSSENSOR_H
#define CHGPSSENSOR_H

// #include "ChApiSensor.h"

#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/filters/ChFilterGPSUpdate.h"

// class ChGPSNoiseModel;

namespace chrono {
namespace sensor {

/// definition of Earth's radius to be used throughout the GPS sensor computations
#define EARTH_RADIUS 6371000.0

/// @addtogroup sensor_sensors
/// @{

/// GPS class. This class uses the reference location of a simulation and spherically maps the cartesian space
/// simulation onto a sphere, calculating the latitude, longitude, altitude relative to the Earth's surface.
/// The reference location defines what GPS coordinates are associated with the origin of the simulation. The mapping is
/// performed such that the +Z-axis points up, +X-axis points East, and the +Y-axis points North.
class CH_SENSOR_API ChGPSSensor : public ChSensor {
  public:
    /// Class constructor
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param lag Lag time between end of data collection and when data becomes available to the user.
    /// @param collection_window Collection time over which the sensor should collect data from the simulation.
    /// @param gps_reference Reference location in GPS coordinates (longitude, latitude, altitude) of simulation origin
    /// @param noise_model The noise model that should be used for augmenting the GPS data.
    ChGPSSensor(std::shared_ptr<chrono::ChBody> parent,
                float updateRate,
                chrono::ChFrame<double> offsetPose,
                ChVector<double> gps_reference,
                std::shared_ptr<ChGPSNoiseModel> noise_model);

    /// Class destructor
    ~ChGPSSensor();

    /// Variable for communicating the sensor's keyframes from the ChSystem into the data generation filter
    std::vector<std::tuple<float, ChVector<double>>> gps_key_frames;
};

/// Utility function for calculating Cartesian coordinates from GPS coordinates given the simulation's reference point
/// @param coords The Cartisian coordinates to be modified. This ChVector is modified and used as output
/// @param ref The simulation's reference location
static void Cartesian2GPS(ChVector<double>& coords, ChVector<double>& ref) {
    // convert from cartesian to gps coordinates assuming a sphere
    double lat = (coords.y() / EARTH_RADIUS) * 180.0 / chrono::CH_C_PI + ref.y();
    double lon = (coords.x() / (EARTH_RADIUS * cos(lat * chrono::CH_C_PI / 180.0))) * 180.0 / chrono::CH_C_PI + ref.x();
    double alt = coords.z() + ref.z();

    // sanitize the new gps coordinates
    if (lat < -90.0) {
        // NOT A GOOD APPROXIMATION NEAR THE POLES ANYWAY
    } else if (lat > 90) {
        // NOT A GOOD APPROXIMATION NEAR THE POLES ANYWAY
    }

    if (lon < -180.0) {
        lon = lon + 360.0;
    } else if (lon > 180.0) {
        lon = lon - 360.0;
    }

    coords = chrono::ChVector<>({lon, lat, alt});
}

/// Utility function for calculating GPS coordinates from Cartesian coordinates given the simulation's reference point
/// @param coords The GPS coordinates to be modified. This ChVector is modified and used as output
/// @param ref The simulation's reference location
static void GPS2Cartesian(ChVector<double>& coords, ChVector<double>& ref) {
    double lon = coords.x();
    double lat = coords.y();
    double alt = coords.z();

    double x = ((lon - ref.x()) * chrono::CH_C_PI / 180.0) * (EARTH_RADIUS * cos(lat * chrono::CH_C_PI / 180.0));
    double y = ((lat - ref.y()) * chrono::CH_C_PI / 180.0) * EARTH_RADIUS;
    double z = alt - ref.z();

    coords = chrono::ChVector<>({x, y, z});
}

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
