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

#ifndef CH_PARSER_YAML_H
#define CH_PARSER_YAML_H

#include "chrono_parsers/ChApiParsers.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/input_output/ChUtilsYAML.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Base class for all YAML parsers.
class ChApiParsers ChParserYAML {
  public:
    /// Type of a Chrono YAML specification file.
    enum class YamlFileType { MBS, SPH, TDPF, FSI, VEHICLE, UNKNOWN };

    ChParserYAML();
    virtual ~ChParserYAML() {}

    /// Set verbose terminal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set root output directory (default: ".").
    /// The specified directory must exist.
    virtual void SetOutputDir(const std::string& out_dir);

    /// Return the name of the YAML model.
    const std::string& GetName() const { return m_name; }

    /// Return true if generating output.
    virtual bool Output() const;

    /// Return all output settings.
    const ChOutput::Settings& GetOutputSettings() const { return m_output_settings; }

    /// Return the output type.
    ChOutput::Format GetOutputFormat() const { return m_output_settings.format; }

    /// Return the output mode.
    ChOutput::Mode GetOutputMode() const { return m_output_settings.mode; }

    /// Return the output frequency.
    virtual double GetOutputFPS() const { return m_output_settings.fps; }

    /// Return true if visualization is enabled.
    virtual bool Render() const { return m_vis_settings.render; }

    /// Return all visualization settings.
    const ChVisualSystem::Settings& GetVisualizationSettings() const { return m_vis_settings; }

    double GetRenderFPS() const { return m_vis_settings.render_fps; }
    CameraVerticalDir GetCameraVerticalDir() const { return m_vis_settings.camera_vertical; }
    const ChVector3d& GetCameraLocation() const { return m_vis_settings.camera_location; }
    const ChVector3d& GetCameraTarget() const { return m_vis_settings.camera_target; }
    bool EnableShadows() const { return m_vis_settings.enable_shadows; }

    /// Write simulation output results at the current time.
    /// This base class implementation creates and initializes the output database. Derived classes must
    virtual void WriteOutput(int frame, double time);

    /// Peek in specified YAML file and read the fluid system type.
    /// Throws a runtime error if the type is unknown.
    static YamlFileType ReadYamlFileType(const std::string& yaml_filename);

  protected:
    /// Read the YAML file type.
    static YamlFileType ReadYamlFileType(const YAML::Node& a);

    std::string m_name;  ///< name of the YAML model
    bool m_verbose;      ///< verbose terminal output (default: false)
    bool m_use_degrees;  ///< all angles given in degrees (default: true)

    ChVisualSystem::Settings m_vis_settings;  ///< visualization settings

    ChOutput::Settings m_output_settings;   ///< output settings
    std::string m_output_dir;               ///< root output directory
    std::shared_ptr<ChOutput> m_output_db;  ///< output database

    ChYamlFileHandler m_file_handler;  ///< handler for data file paths
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
