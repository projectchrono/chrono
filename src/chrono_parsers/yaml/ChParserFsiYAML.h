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
    double GetTimestep() const { return m_step; }

    /// Get simulation end time.
    /// A value of -1 indicates infinite end time.
    double GetEndtime() const { return m_end_time; }

    /// Indicate whether to enable run-time visualization.
    bool Render() const { return m_render; }

    /// Return frequency (frames-per-second) for run-time visualization rendering.
    double GetRenderFPS() const { return m_render_fps; }

    /// Indicate whether to enable simulation output.
    virtual bool Output() const override { return m_output; }

    /// Return frequency (frames-per-second) for simulation output.
    virtual double GetOutputFPS() const override { return m_output_fps; }

  private:
    struct FsiBody {
        std::string name;                                 ///< body name
        std::vector<std::shared_ptr<ChBodyAuxRef>> body;  ///< underlying Chrono bodies (one per instance)
        std::shared_ptr<utils::ChBodyGeometry> geometry;  ///< FSI geometry
    };

    std::shared_ptr<utils::ChBodyGeometry> ReadCollisionGeometry(const YAML::Node& a);

    std::string m_file_modelMBS;
    std::string m_file_simMBS;
    std::string m_file_modelCFD;
    std::string m_file_simCFD;

    std::shared_ptr<ChParserMbsYAML> m_parserMBS;
    std::shared_ptr<ChParserCfdYAML> m_parserCFD;

    ChParserCfdYAML::FluidSystemType m_sysCFD_type;

    std::shared_ptr<fsi::ChFsiSystem> m_sysFSI;
    std::shared_ptr<fsi::ChFsiFluidSystem> m_sysCFD;
    std::shared_ptr<ChSystem> m_sysMBS;

    std::vector<FsiBody> m_fsi_bodies;

    double m_step;
    double m_end_time;

    bool m_render;
    double m_render_fps;

    bool m_output;
    double m_output_fps;
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
