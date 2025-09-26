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
#include <vector>

#include "chrono_parsers/ChApiParsers.h"
#include "chrono_parsers/yaml/ChParserMbsYAML.h"

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Utility class to parse a YAML specification file for a vehicle model.
class ChApiParsers ChParserVehicleYAML {
  public:
    enum class VehicleType { WHEELED, TRACKED };

    /// Create a YAML parser and load the model from the specified input YAML file.
    ChParserVehicleYAML(const std::string& yaml_model_filename,
                        const std::string& yaml_sim_filename,
                        bool verbose = false);
    ~ChParserVehicleYAML();

    /// Set verbose temrinal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Return true if a YAML simulation file has been loaded.
    bool HasSimulationData() const { return m_parserMBS->HasSimulationData(); }

    /// Return true if a YAML model file has been loaded.
    bool HasModelData() const { return m_model_loaded; }

    // -------------

    double GetTimestep() const { return m_parserMBS->GetTimestep(); }
    double GetEndtime() const { return m_parserMBS->GetEndtime(); }
    bool EnforceRealtime() const { return m_parserMBS->EnforceRealtime(); }
    bool Render() const { return m_parserMBS->Render(); }
    double GetRenderFPS() const { return m_parserMBS->GetRenderFPS(); }
    CameraVerticalDir GetCameraVerticalDir() const { return m_parserMBS->GetCameraVerticalDir(); }
    const ChVector3d& GetCameraLocation() const { return m_parserMBS->GetCameraLocation(); }
    const ChVector3d& GetCameraTarget() const { return m_parserMBS->GetCameraTarget(); }
    bool EnableShadows() const { return m_parserMBS->EnableShadows(); }
    bool Output() const { return m_parserMBS->Output(); }
    ChOutput::Type GetOutputType() const { return m_parserMBS->GetOutputType(); }
    double GetOutputFPS() const { return m_parserMBS->GetOutputFPS(); }
    void SetOutputDir(const std::string& out_dir) { m_parserMBS->SetOutputDir(out_dir); }

    /// Create and return a Chrono system configured from cached simulation parameters.
    std::shared_ptr<ChSystem> CreateSystem() { return m_parserMBS->CreateSystem(); }

    // -------------

    /// Load the specified vehicle model input YAML file.
    void LoadModelFile(const std::string& yaml_filename);

    /// Return the name of the YAML vehicle model.
    const std::string& GetName() const { return m_name; }

    /// Get vehicle type (from specified vehicleJSON file).
    VehicleType GetVehicleType() const { return m_vehicle_type; }

    /// Get vehicle type as a string.
    std::string GetVehicleTypeAsString() const;

    /// Populate the given system with the cached Chrono::Vehicle model.
    void CreateVehicle(ChSystem& sys);

    /// Return the underlying vehicle model.
    std::shared_ptr<vehicle::ChVehicle> GetVehicle() const { return m_vehicle; }

    /// Return the underlying rigid terrain (may be null).
    std::shared_ptr<vehicle::RigidTerrain> GetTerrain() const { return m_terrain; }

    const ChVector3d& GetChassisPoint() const { return m_chassis_point; }
    double GetChaseDistance() const { return m_chase_distance; }
    double GetChaseHeight() const { return m_chase_height; }

  private:
    VehicleType ReadVehicleType(const std::string& vehicle_json);

    bool m_verbose;

    std::string m_name;
    bool m_model_loaded;
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

    std::shared_ptr<ChParserMbsYAML> m_parserMBS;
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
