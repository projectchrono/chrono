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

#ifndef CH_PARSER_CFD_YAML_H
#define CH_PARSER_CFD_YAML_H

#include <string>

#include "chrono_parsers/yaml/ChParserYAML.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Base class for YAML parsers for fluid systems.
class ChApiParsers ChParserCfdYAML : public ChParserYAML {
  public:
    enum class FluidSystemType { SPH, BEM };

    ChParserCfdYAML(bool verbose = false);
    virtual ~ChParserCfdYAML() {}

    /// Return the fluid system type.
    FluidSystemType GetType() const { return m_type; }

#ifdef CHRONO_VSG
    /// Return a VSG run-visualization plugin.
    virtual std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> GetVisualizationPlugin() const { return nullptr; }
#endif

    /// Peek in specified YAML file and read the fluid system type.
    /// Throws a runtime error if the type is unknown.
    static FluidSystemType ReadFluidSystemType(const std::string& yaml_filename);

  protected:
    FluidSystemType m_type;
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
