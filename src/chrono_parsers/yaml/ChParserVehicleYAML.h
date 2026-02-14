// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_PARSER_VEHICLE_YAML_H
#define CH_PARSER_VEHICLE_YAML_H

#include <string>

#include "chrono_parsers/yaml/ChParserMbsYAML.h"

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Parser for YAML specification file for Chrono::Vehicle models.
/// Supports both wheeled and tracked vehicles (defined through JSON specification files).
class ChApiParsers ChParserVehicleYAML : public ChParserMbsYAML {
  public:
    enum class VehicleType { WHEELED, TRACKED };

    ChParserVehicleYAML(const std::string& yaml_filename, bool verbose = false);
    ~ChParserVehicleYAML();

    /// Return true if a YAML model file has been loaded.
    bool HasModelData() const { return m_model_loaded; }

    // --------------

    /// Load the specified MBS simulation input YAML file.
    virtual void LoadFile(const std::string& yaml_filename) override;

    /// Load the vehicle model from the specified YAML node.
    virtual void LoadModelData(const YAML::Node& yaml) override;

    /// Populate the given system with the cached Chrono::Vehicle model.
    void CreateVehicle(ChSystem& sys);

    // -------------

    /// Get vehicle type (from specified vehicleJSON file).
    VehicleType GetVehicleType() const { return m_vehicle_type; }

    /// Get vehicle type as a string.
    std::string GetVehicleTypeAsString() const;

    /// Return the underlying vehicle model.
    std::shared_ptr<vehicle::ChVehicle> GetVehicle() const { return m_vehicle; }

    /// Return the underlying rigid terrain (may be null).
    std::shared_ptr<vehicle::RigidTerrain> GetTerrain() const { return m_terrain; }

    const ChVector3d& GetChassisPoint() const { return m_chassis_point; }
    double GetChaseDistance() const { return m_chase_distance; }
    double GetChaseHeight() const { return m_chase_height; }

  private:
    VehicleType ReadVehicleType(const std::string& vehicle_json);

    VehicleType m_vehicle_type;

    std::string m_vehicle_json;
    std::string m_tire_json;
    std::string m_engine_json;
    std::string m_transmission_json;
    std::string m_terrain_json;

    VisualizationType m_vis_chassis;
    VisualizationType m_vis_trailer;
    VisualizationType m_vis_subchassis;
    VisualizationType m_vis_suspension;
    VisualizationType m_vis_steering;
    VisualizationType m_vis_wheel;
    VisualizationType m_vis_tire;

    std::shared_ptr<vehicle::ChVehicle> m_vehicle;
    std::shared_ptr<vehicle::RigidTerrain> m_terrain;

    ChVector3d m_init_position;
    double m_init_yaw;

    ChVector3d m_chassis_point;
    double m_chase_distance;
    double m_chase_height;
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
