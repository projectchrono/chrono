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

#include <string>

#include "chrono_parsers/ChApiParsers.h"

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/input_output/ChOutput.h"

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"
#include "chrono_thirdparty/filesystem/path.h"

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

    /// Return the output frequency.
    virtual double GetOutputFPS() const { return m_output.fps; }

    /// Save simulation output results at the current time.
    /// This base class implementation creates and initializes the output database. Derived classes must  
    virtual void SaveOutput(int frame);

    /// Peek in specified YAML file and read the fluid system type.
    /// Throws a runtime error if the type is unknown.
    static YamlFileType ReadYamlFileType(const std::string& yaml_filename);

  protected:
    enum class DataPathType { ABS, REL };

    /// Output parameters.
    struct OutputParameters {
        OutputParameters();
        void PrintInfo();

        ChOutput::Type type;
        ChOutput::Mode mode;
        double fps;
    };

    static void CheckVersion(const YAML::Node& a);

    static std::string ToUpper(std::string in);

    /// Read output settings from specified YAML node.
    void ReadOutputParams(const YAML::Node& a);

    /// Return the path to the specified data file.
    std::string GetDatafilePath(const std::string& filename);

    /// Read the YAML file type.
    static YamlFileType ReadYamlFileType(const YAML::Node& a);

    /// Read the data path type (absolute or relative).
    static DataPathType ReadDataPathType(const YAML::Node& a);

    /// Load and return a ChVector3d from the specified node.
    static ChVector3d ReadVector(const YAML::Node& a);

    /// Load and return a ChQuaternion from the specified node.
    static ChQuaterniond ReadQuaternion(const YAML::Node& a);

    /// Load a Cardan angle sequence from the specified node and return as a quaternion.
    /// The sequence is assumed to be extrinsic rotations X-Y-Z.
    static ChQuaterniond ReadCardanAngles(const YAML::Node& a, bool use_degrees);

    /// Return a quaternion loaded from the specified node.
    /// Data is assumed to provide a quaternion or a Cardan extrinsic X-Y-Z angle set.
    static ChQuaterniond ReadRotation(const YAML::Node& a, bool use_degrees);

    /// Load and return a coordinate system from the specified node.
    static ChCoordsysd ReadCoordinateSystem(const YAML::Node& a, bool use_degrees);

    /// Load and return a ChFunction object from the specified node.
    static std::shared_ptr<ChFunction> ReadFunction(const YAML::Node& a, bool use_degrees);

    /// Load and return a ChColor from the specified node.
    static ChColor ReadColor(const YAML::Node& a);

    static ChOutput::Type ReadOutputType(const YAML::Node& a);
    static ChOutput::Mode ReadOutputMode(const YAML::Node& a);

    /// Print YAML node type.
    static void PrintNodeType(const YAML::Node& node);

    std::string m_name;    ///< name of the YAML model
    bool m_verbose;        ///< verbose terminal output (default: false)
    bool m_use_degrees;    ///< all angles given in degrees (default: true)

    std::string m_output_dir;               ///< root oputput directory
    std::shared_ptr<ChOutput> m_output_db;  ///< output database

    DataPathType m_data_path;
    std::string m_rel_path;
    std::string m_script_directory;

    OutputParameters m_output;
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
