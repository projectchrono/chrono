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

#ifndef CH_FSI_PARSER_YAML_H
#define CH_FSI_PARSER_YAML_H

#include <string>
#include <vector>

#include "chrono_parsers/ChApiParsers.h"
#include "chrono_parsers/yaml/ChParserMbsYAML.h"
#include "chrono_parsers/yaml/ChParserCfdYAML.h"
#include "chrono_fsi/ChFsiSystem.h"

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Utility class to parse a YAML specification file for a coupled FSI problem.
class ChApiParsers ChParserFsiYAML {
  public:
    /// Create a YAML parser and load the model from the specified input YAML file.
    ChParserFsiYAML(const std::string& yaml_filename, bool verbose = false);
    ~ChParserFsiYAML();

    /// Set verbose temrinal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Load the specified input YAML file.
    void LoadFile(const std::string& yaml_filename);

    /// Create and return a ChFsiSystem combining a Chrono MBS system and a fluid solver.
    void CreateFsiSystem();

    /// Return the name of the FSI model.
    const std::string& GetName() const { return m_name; }

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
    bool Output() const { return m_output; }

    /// Return frequency (frames-per-second) for simulation output.
    double GetOutputFPS() const { return m_output_fps; }

  private:
    bool m_verbose;

    std::string m_name;

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

    std::vector<std::string> m_fsi_bodies;

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
