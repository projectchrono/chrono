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
// Authors: Radu Serban
// =============================================================================
//
// Utility functions for parsing JSON files.
//
// =============================================================================

#ifndef CH_JSON_UTILS_H
#define CH_JSON_UTILS_H

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
//
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChAntirollBar.h"
#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"
#include "chrono_vehicle/wheeled_vehicle/ChDriveline.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
//
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackDriveline.h"
//
#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

/// Load and return a ChVector from the specified JSON array
CH_VEHICLE_API ChVector<> ReadVectorJSON(const rapidjson::Value& a);

///  Load and return a ChQuaternion from the specified JSON array
CH_VEHICLE_API ChQuaternion<> ReadQuaternionJSON(const rapidjson::Value& a);

///  Load and return a ChColor from the specified JSON array
CH_VEHICLE_API ChColor ReadColorJSON(const rapidjson::Value& a);

// -----------------------------------------------------------------------------

/// Load and return a chassis subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChChassis> ReadChassisJSON(const std::string& filename);

///  Load and return a suspension subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChSuspension> ReadSuspensionJSON(const std::string& filename);

///  Load and return a steering subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChSteering> ReadSteeringJSON(const std::string& filename);

///  Load and return a driveline subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChDriveline> ReadDrivelineJSON(const std::string& filename);

///  Load and return a steering subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChAntirollBar> ReadAntirollbarJSON(const std::string& filename);

///  Load and return a steering subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChWheel> ReadWheelJSON(const std::string& filename);

///  Load and return a steering subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChBrake> ReadBrakeJSON(const std::string& filename);

/// Load and return a tire from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTire> ReadTireJSON(const std::string& filename);

// -----------------------------------------------------------------------------

/// Load and return a track assembly from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTrackAssembly> ReadTrackAssemblySON(const std::string& filename);

/// Load and return a track driveline from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTrackDriveline> ReadTrackDrivelineJSON(const std::string& filename);

/// Load and return a track brake from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTrackBrake> ReadTrackBrakeJSON(const std::string& filename);

/// Load and return an idler from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChIdler> ReadIdlerJSON(const std::string& filename);

/// Load and return a road-wheel assembly (track suspension) from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChRoadWheelAssembly> ReadRoadWheelAssemblyJSON(const std::string& filename, bool has_shock);

/// Load and return a roller from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChRoller> ReadRollerJSON(const std::string& filename);

/// Load and return a road-wheel from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChRoadWheel> ReadRoadWheelJSON(const std::string& filename);

}  // end namespace vehicle
}  // end namespace chrono

#endif
