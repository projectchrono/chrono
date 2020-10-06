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

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
//
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChIMUSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
//
#include "chrono_sensor/filters/ChFilter.h"
//
#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_utils
/// @{

// -----------------------------------------------------------------------------

/// Load and return a RapidJSON document from the specified file.
/// A Null document is returned if the file cannot be opened.
/// @param filename The path to the file to be parsed
/// @return A rapid JSON document corresponding to the file name
CH_SENSOR_API rapidjson::Document ReadFileJSON(const std::string& filename);

// -----------------------------------------------------------------------------

/// Load and return a ChVector from the specified JSON array
/// @param a The value to be read
/// @return A ChVector containing the values in ChVector format
CH_SENSOR_API ChVector<> ReadVectorJSON(const rapidjson::Value& a);

/// Load and return a ChQuaternion from the specified JSON array
/// @param a The value to be read
/// @return A ChQuatertion generated from the JSON value
CH_SENSOR_API ChQuaternion<> ReadQuaternionJSON(const rapidjson::Value& a);

///  Load and return a ChFrame from the specified JSON array
/// @param a The value to be read
/// @return A ChFrame generated from the JSON value
CH_SENSOR_API ChFrame<> ReadFrameJSON(const rapidjson::Value& a);

// -----------------------------------------------------------------------------

/// Load and return a sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @return A shared pointer to a ChSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChSensor> ReadSensorJSON(const std::string& filename,
                                                       std::shared_ptr<chrono::ChBody> parent,
                                                       chrono::ChFrame<double> offsetPose);

/// Load and return a camera sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the camera sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @return A shared pointer to a ChCameraSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChCameraSensor> ReadCameraSensorJSON(const std::string& filename,
                                                                   std::shared_ptr<chrono::ChBody> parent,
                                                                   chrono::ChFrame<double> offsetPose);

/// Load and return a gps sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the GPS sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @return A shared pointer to a ChGPSSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChGPSSensor> ReadGPSSensorJSON(const std::string& filename,
                                                             std::shared_ptr<chrono::ChBody> parent,
                                                             chrono::ChFrame<double> offsetPose);

/// Load and return a imu sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the IMU sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @return A shared pointer to a ChIMUSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChIMUSensor> ReadIMUSensorJSON(const std::string& filename,
                                                             std::shared_ptr<chrono::ChBody> parent,
                                                             chrono::ChFrame<double> offsetPose);

/// Load and return a lidar sensor from the specified JSON file.
/// @param filename The name/path to the JSON file defining the lidar sensor parameters
/// @param parent The ChBody to which the sensor should be attached
/// @return A shared pointer to a ChLidarSensor constructed from the JSON file
CH_SENSOR_API std::shared_ptr<ChLidarSensor> ReadLidarSensorJSON(const std::string& filename,
                                                                 std::shared_ptr<chrono::ChBody> parent,
                                                                 chrono::ChFrame<double> offsetPose);

/// Load and return a sensor filter list from the specified JSON file.
/// @param filename The name/path to the JSON file defining the filters for a sensor
/// @param sensor The sensor to which the filters will be added
CH_SENSOR_API void ReadFilterListJSON(const std::string& filename, std::shared_ptr<ChSensor> sensor);

// -----------------------------------------------------------------------------

/// Load and return a sensor filter from the specified JSON value
/// @param value The JSON value to be parsed
/// @return A ChFilter parsed from the JSON value
CH_SENSOR_API std::shared_ptr<ChFilter> CreateFilterJSON(const rapidjson::Value& value);

/// Load and return a imu noise model from the specified JSON value
/// @param value The JSON value to be parsed
/// @return A ChIMUNoiseModel parsed from the JSON value
CH_SENSOR_API std::shared_ptr<ChIMUNoiseModel> CreateIMUNoiseJSON(const rapidjson::Value& value);

/// Load and return a gps noise model from the specified JSON value
/// @param value The JSON value to be parsed
/// @return A ChGPSNoiseModel parsed from the JSON value
CH_SENSOR_API std::shared_ptr<ChGPSNoiseModel> CreateGPSNoiseJSON(const rapidjson::Value& value);

/// Load and return a std::string from the specified JSON value
/// Will check if member exists and returns if it does, def if not\
/// @param value The JSON value to be parsed
/// @param member A member from the file to be read
/// @param def A default value to use if not definted in JSON file
/// @return A string parsed from the JSON value or default if none exists
CH_SENSOR_API std::string GetStringMemberWithDefault(const rapidjson::Value& value,
                                                     const char* member,
                                                     const char* def = "");

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
