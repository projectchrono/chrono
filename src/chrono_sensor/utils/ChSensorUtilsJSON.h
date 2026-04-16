// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Aaron Young
// =============================================================================
//
// Utility functions for parsing JSON files.
//
// =============================================================================

#ifndef CH_SENSOR_JSON_UTILS_H
#define CH_SENSOR_JSON_UTILS_H

#include "chrono/input_output/ChUtilsJSON.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/sensors/ChCameraSensor.h"
    #include "chrono_sensor/sensors/ChLidarSensor.h"
    #include "chrono_sensor/sensors/ChRadarSensor.h"
#endif
#include "chrono_sensor/filters/ChFilter.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_utils
/// @{

// -----------------------------------------------------------------------------

/// Load and return a sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @param offsetPose The position and rotation of the Sensor
/// @return A shared pointer to a ChSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChSensor> ReadSensorJSON(const std::string& filename, std::shared_ptr<ChBody> parent, ChFrame<double> offsetPose);

#ifdef CHRONO_HAS_OPTIX

/// Load and return a camera sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the camera sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @param offsetPose The position and rotation of the Camera Sensor
/// @return A shared pointer to a ChCameraSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChCameraSensor> ReadCameraSensorJSON(const std::string& filename, std::shared_ptr<ChBody> parent, ChFrame<double> offsetPose);

/// Load and return a lidar sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the lidar sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @param offsetPose The position and rotation of the Lidar Sensor
/// @return A shared pointer to a ChLidarSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChLidarSensor> ReadLidarSensorJSON(const std::string& filename, std::shared_ptr<ChBody> parent, ChFrame<double> offsetPose);

/// Load and return a radar sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the radar sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @param offsetPose The position and rotation of the radar Sensor
/// @return A shared pointer to a ChRadarSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChRadarSensor> ReadRadarSensorJSON(const std::string& filename, std::shared_ptr<ChBody> parent, ChFrame<double> offsetPose);

#endif

/// Load and return a gps sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the GPS sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @param offsetPose The position and rotation of the GPS Sensor
/// @return A shared pointer to a ChGPSSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChGPSSensor> ReadGPSSensorJSON(const std::string& filename, std::shared_ptr<ChBody> parent, ChFrame<double> offsetPose);

/// Load and return a accelerometer sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the accelerometer sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @param offsetPose The position and rotation of the accelerometer Sensor
/// @return A shared pointer to a ChAccelerometerSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChAccelerometerSensor> ReadAccelerometerSensorJSON(const std::string& filename, std::shared_ptr<ChBody> parent, ChFrame<double> offsetPose);

/// Load and return a gyroscope sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the gyroscope sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @param offsetPose The position and rotation of the gyroscope Sensor
/// @return A shared pointer to a ChGyroscopeSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChGyroscopeSensor> ReadGyroscopeSensorJSON(const std::string& filename, std::shared_ptr<ChBody> parent, ChFrame<double> offsetPose);

/// Load and return a magnetometer sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the magnetometer sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @param offsetPose The position and rotation of the magnetometer Sensor
/// @return A shared pointer to a ChMagnetometerSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChMagnetometerSensor> ReadMagnetometerSensorJSON(const std::string& filename, std::shared_ptr<ChBody> parent, ChFrame<double> offsetPose);

/// Load and return a sensor filter list from the specified JSON file.
/// @param filename The name/path to the JSON file defining the filters for a sensor
/// @param sensor The sensor to which the filters will be added
CH_SENSOR_API void ReadFilterListJSON(const std::string& filename, std::shared_ptr<ChSensor> sensor);

// -----------------------------------------------------------------------------

/// Load and return a sensor filter from the specified JSON value
/// @param value The JSON value to be parsed
/// @return A ChFilter parsed from the JSON value
CH_SENSOR_API std::shared_ptr<ChFilter> CreateFilterJSON(const rapidjson::Value& value);

/// Load and return a noise model from the specified JSON value
/// @param value The JSON value to be parsed
/// @return A ChNoiseModel parsed from the JSON value
CH_SENSOR_API std::shared_ptr<ChNoiseModel> CreateNoiseJSON(const rapidjson::Value& value);

/// Load and return a std::string from the specified JSON value
/// Will check if member exists and returns if it does, def if not
/// @param value The JSON value to be parsed
/// @param member A member from the file to be read
/// @param def A default value to use if not defined in JSON file
/// @return A string parsed from the JSON value or default if none exists
CH_SENSOR_API std::string GetStringMemberWithDefault(const rapidjson::Value& value, const char* member, const char* def = "");

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
