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

#ifndef CH_PARSER_FSI_YAML_H
#define CH_PARSER_FSI_YAML_H

#include <vector>

#include "chrono_parsers/yaml/ChParserMbsYAML.h"
#include "chrono_parsers/yaml/ChParserCfdYAML.h"

#include "chrono_fsi/ChFsiSystem.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Parser for YAML specification file for a coupled FSI problem.
class ChApiParsers ChParserFsiYAML : public ChParserYAML {
  public:
    /// Create a YAML parser and load the model from the specified input YAML file.
    ChParserFsiYAML(const std::string& yaml_filename, bool verbose = false);
    ~ChParserFsiYAML();

    /// Load the specified input YAML file.
    void LoadFile(const std::string& yaml_filename);

    void LoadFsiData(const YAML::Node& yaml);

    /// Load the simulation and visualization saettings from the specified YAML node.
    void LoadSimData(const YAML::Node& yaml);

    /// Create and return a ChFsiSystem combining a Chrono MBS system and a fluid solver.
    void CreateFsiSystem();

    /// Return the multibody YAML parser.
    ChParserMbsYAML& GetMbsParser() const { return *m_parserMBS; }

    /// Return the fluid YAML parser.
    ChParserCfdYAML& GetCfdParser() const { return *m_parserCFD; }

    /// Return the FSI system.
    std::shared_ptr<fsi::ChFsiSystem> GetFsiSystem() const { return m_sysFSI; }

    /// Return fluid system type.
    ChParserCfdYAML::FluidSystemType GetFluidSystemType() const { return m_parserCFD->GetType(); }

    /// Return the fluid system.
    std::shared_ptr<fsi::ChFsiFluidSystem> GetFluidSystem() const { return m_sysCFD; }

    /// Return the MBS system.
    std::shared_ptr<ChSystem> GetMultibodySystem() const { return m_sysMBS; }

    /// Get meta-step (communication time step).
    double GetTimestep() const { return m_sim.step; }

    /// Get simulation end time.
    /// A value of -1 indicates infinite end time.
    double GetEndtime() const { return m_sim.end_time; }

    /// Set root output directory (default: ".").
    virtual void SetOutputDir(const std::string& out_dir) override;

    /// Return true if generating output.
    virtual bool Output() const override;

    /// Indicate whether to enable run-time visualization.
    bool Render() const { return m_vis.render; }

    /// Return frequency (frames-per-second) for run-time visualization rendering.
    double GetRenderFPS() const { return m_vis.render_fps; }

    CameraVerticalDir GetCameraVerticalDir() const { return m_vis.camera_vertical; }
    const ChVector3d& GetCameraLocation() const { return m_vis.camera_location; }
    const ChVector3d& GetCameraTarget() const { return m_vis.camera_target; }
    bool EnableShadows() const { return m_vis.enable_shadows; }

  private:
    /// FSI rigid body definition.
    struct FsiBody {
        std::string name;                                 ///< body name
        std::vector<std::shared_ptr<ChBodyAuxRef>> body;  ///< underlying Chrono bodies (one per instance)
        std::shared_ptr<utils::ChBodyGeometry> geometry;  ///< FSI geometry
    };

    /// Co-simulation settings.
    struct SimParams {
        SimParams();
        void PrintInfo();

        double step;
        double end_time;
        ChVector3d gravity;
    };

    /// Run-time visualization settings.
    struct VisParams {
        VisParams();
        void PrintInfo();

        bool render;
        double render_fps;
        CameraVerticalDir camera_vertical;
        ChVector3d camera_location;
        ChVector3d camera_target;
        bool enable_shadows;
    };

    std::shared_ptr<utils::ChBodyGeometry> ReadCollisionGeometry(const YAML::Node& a);

    std::shared_ptr<ChParserMbsYAML> m_parserMBS;
    std::shared_ptr<ChParserCfdYAML> m_parserCFD;

    ChParserCfdYAML::FluidSystemType m_sysCFD_type;

    //// TODO: do I need to cache these?
    std::shared_ptr<fsi::ChFsiSystem> m_sysFSI;
    std::shared_ptr<fsi::ChFsiFluidSystem> m_sysCFD;
    std::shared_ptr<ChSystem> m_sysMBS;

    std::vector<FsiBody> m_fsi_bodies;

    SimParams m_sim; ///< co-simulation settings
        VisParams m_vis;  ///< visualization settings
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
