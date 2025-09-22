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

#ifndef CH_CFD_PARSER_YAML_H
#define CH_CFD_PARSER_YAML_H

#include <string>

#include "chrono_parsers/ChApiParsers.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Base class for YAML parsers for fluid systems.
class ChApiParsers ChParserCfdYAML {
  public:
    enum class FluidSystemType { SPH, BEM };

    ChParserCfdYAML(bool verbose = false);
    virtual ~ChParserCfdYAML() {}

    /// Set verbose terminal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set root output directory (default: "").
    void SetOutputDir(const std::string& out_dir) { m_output_dir = out_dir; }

    /// Return the fluid system type.
    FluidSystemType GetType() const { return m_type; }

#ifdef CHRONO_VSG
    /// Return a VSG run-visualization plugin.
    virtual std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> GetVisualizationPlugin() const { return nullptr; }
#endif

    /// Return true if generating output.
    virtual bool Output() const = 0;

    /// Save simulation output results at the current time.
    virtual void SaveOutput(int frame) {}

    // Utility functions

    /// Check version in YAML file against the Chrono version.
    static void CheckVersion(const YAML::Node& a);

    /// Convert string to upper case.
    static std::string ToUpper(std::string in);

    /// Peek in specified YAML file and read the fluid system type.
    /// Throws a runtime error if the type is unknown.
    static FluidSystemType ReadFluidSystemType(const std::string& yaml_filename);

  protected:
    bool m_verbose;
    FluidSystemType m_type;
    std::string m_output_dir;
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
