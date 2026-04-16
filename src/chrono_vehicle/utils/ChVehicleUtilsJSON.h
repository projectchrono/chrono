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

#include <vector>

#include "chrono/input_output/ChUtilsJSON.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChEngine.h"
#include "chrono_vehicle/ChTransmission.h"

#include "chrono_vehicle/wheeled_vehicle/ChAntirollBar.h"
#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"
#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/ChSubchassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChDrivelineTV.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

/// Load and return a chassis subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChChassis> ReadChassisJSON(const std::string& filename);

/// Load and return a rear chassis subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChChassisRear> ReadChassisRearJSON(const std::string& filename);

/// Load and return a chassis connector subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChChassisConnector> ReadChassisConnectorJSON(const std::string& filename);

/// Load and return an engine subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChEngine> ReadEngineJSON(const std::string& filename);

/// Load and return a transmission subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTransmission> ReadTransmissionJSON(const std::string& filename);

// -----------------------------------------------------------------------------

///  Load and return a suspension subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChSuspension> ReadSuspensionJSON(const std::string& filename);

///  Load and return a steering subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChSteering> ReadSteeringJSON(const std::string& filename);

///  Load and return a driveline subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChDrivelineWV> ReadDrivelineWVJSON(const std::string& filename);

///  Load and return a steering subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChAntirollBar> ReadAntirollbarJSON(const std::string& filename);

///  Load and return a steering subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChWheel> ReadWheelJSON(const std::string& filename);

/// Load and return a subchassis subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChSubchassis> ReadSubchassisJSON(const std::string& filename);

///  Load and return a steering subsystem from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChBrake> ReadBrakeJSON(const std::string& filename);

/// Load and return a tire from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTire> ReadTireJSON(const std::string& filename);

// -----------------------------------------------------------------------------

/// Load and return a track assembly from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTrackAssembly> ReadTrackAssemblyJSON(const std::string& filename);

/// Load and return a track driveline from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChDrivelineTV> ReadDrivelineTVJSON(const std::string& filename);

/// Load and return a track brake from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTrackBrake> ReadTrackBrakeJSON(const std::string& filename);

/// Load and return an idler from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChIdler> ReadIdlerJSON(const std::string& filename);

/// Load and return a track suspension from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTrackSuspension> ReadTrackSuspensionJSON(const std::string& filename,
                                                                          bool has_shock,
                                                                          bool lock_arm);

/// Load and return a road-wheel from the specified JSON file.
CH_VEHICLE_API std::shared_ptr<ChTrackWheel> ReadTrackWheelJSON(const std::string& filename);

}  // end namespace vehicle
}  // end namespace chrono

#endif
