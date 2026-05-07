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

#include "chrono/physics/ChSystem.h"
#include "chrono/input_output/ChUtilsYAML.h"

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

    /// Return the output type.
    ChOutput::Type GetOutputType() const { return m_output.type; }

    /// Return the output mode.
    ChOutput::Mode GetOutputMode() const { return m_output.mode; }

    /// Return the output frequency.
    virtual double GetOutputFPS() const { return m_output.fps; }

    /// Save simulation output results at the current time.
    /// This base class implementation creates and initializes the output database. Derived classes must  
    virtual void SaveOutput(int frame);

    /// Peek in specified YAML file and read the fluid system type.
    /// Throws a runtime error if the type is unknown.
    static YamlFileType ReadYamlFileType(const std::string& yaml_filename);

  protected:
    /// Output parameters.
    struct OutputParameters {
        OutputParameters();
        void PrintInfo();

        ChOutput::Type type;
        ChOutput::Mode mode;
        double fps;
    };

    /// Read output settings from specified YAML node.
    void ReadOutputParams(const YAML::Node& a);

    /// Read the YAML file type.
    static YamlFileType ReadYamlFileType(const YAML::Node& a);

    std::string m_name;    ///< name of the YAML model
    bool m_verbose;        ///< verbose terminal output (default: false)
    bool m_use_degrees;    ///< all angles given in degrees (default: true)

    std::string m_output_dir;               ///< root output directory
    std::shared_ptr<ChOutput> m_output_db;  ///< output database
    OutputParameters m_output;              ///< output parameters

    ChYamlFileHandler m_file_handler;  ///< handler for data file paths
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
