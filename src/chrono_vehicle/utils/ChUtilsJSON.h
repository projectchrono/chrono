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

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"

#include "chrono/functions/ChFunctionInterp.h"

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

/// Load and return a RapidJSON document from the specified file.
/// A Null document is returned if the file cannot be opened.
CH_VEHICLE_API void ReadFileJSON(const std::string& filename, rapidjson::Document& d);

// -----------------------------------------------------------------------------

/// Load and return a ChVector3d from the specified JSON array.
CH_VEHICLE_API ChVector3d ReadVectorJSON(const rapidjson::Value& a);

///  Load and return a ChQuaternion from the specified JSON array.
CH_VEHICLE_API ChQuaternion<> ReadQuaternionJSON(const rapidjson::Value& a);

/// Load and return a coordinate system from the specific JSON value.
CH_VEHICLE_API ChCoordsys<> ReadCoordinateSystemJSON(const rapidjson::Value& a);

///  Load and return a ChColor from the specified JSON array.
CH_VEHICLE_API ChColor ReadColorJSON(const rapidjson::Value& a);

// -----------------------------------------------------------------------------

CH_VEHICLE_API ChContactMaterialData ReadMaterialInfoJSON(const rapidjson::Value& mat);

CH_VEHICLE_API std::shared_ptr<ChVehicleBushingData> ReadBushingDataJSON(const rapidjson::Value& bd);

/// Load and return a vehicle joint type from the specific JSON value.
CH_VEHICLE_API ChVehicleJoint::Type ReadVehicleJointTypeJSON(const rapidjson::Value& a);

// -----------------------------------------------------------------------------

/// Load and return a vehicle geometry structure from the specified JSON value.
/// Collision geometry and contact material information is set in the return ChBodyGeometry object if the given JSON
/// object has a member "Contact". Visualization geometry is loaded if the JSON object has a member "Visualization".
CH_VEHICLE_API utils::ChBodyGeometry ReadVehicleGeometryJSON(const rapidjson::Value& d);

/// Load and return a TSDA geometry structure from the specified JSON value.
CH_VEHICLE_API utils::ChTSDAGeometry ReadTSDAGeometryJSON(const rapidjson::Value& d);

// -----------------------------------------------------------------------------

CH_VEHICLE_API std::shared_ptr<ChLinkTSDA::ForceFunctor> ReadTSDAFunctorJSON(const rapidjson::Value& td,
                                                                             double& free_length);
CH_VEHICLE_API std::shared_ptr<ChLinkRSDA::TorqueFunctor> ReadRSDAFunctorJSON(const rapidjson::Value& td,
                                                                              double& free_angle);

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

// -----------------------------------------------------------------------------

/// Utility class for reading and setting an (x,y) map.
class CH_VEHICLE_API ChMapData {
  public:
    /// Construct a ChMapData with an empty map.
    ChMapData() : m_n(0) {}

    /// Read data from the specified JSON object.
    void Read(const rapidjson::Value& a);

    /// Set the map data to the specified recorder function.
    /// The map data is scaled by the specified factors.
    void Set(ChFunctionInterp& map, double x_factor = 1, double y_factor = 1) const;

    /// Set the map data to the specified vector of pairs.
    /// The map data is scaled by the specified factors.
    void Set(std::vector<std::pair<double, double>>& vec, double x_factor = 1, double y_factor = 1) const;

  private:
    unsigned int m_n;
    std::vector<double> m_x;
    std::vector<double> m_y;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
