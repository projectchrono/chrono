// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Conlain Kelly
// =============================================================================
//
// Parser utility class for OpenSim input files.
//
// =============================================================================

#ifndef CH_PARSER_OPENSIM_H
#define CH_PARSER_OPENSIM_H

#include <functional>
#include <map>

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

namespace chrono {

namespace utils {

/// @addtogroup chrono_utils
/// @{

/// OpenSim input file parser
class ChApi ChParserOpenSim {
  public:
    ChParserOpenSim();
    ~ChParserOpenSim() {}

    void parse(ChSystem& system,            ///< [in] containing Chrono system
               const std::string& filename  ///< [in] OpenSim input file name
               );

    ChSystem* parse(const std::string& filename,  ///< [in] OpenSim input file name
                    ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< [in] contact method
                    );

  private:
    /// Setup lambda table for body parsing
    void initFunctionTable();

    /// Creates ChBody and parses its various properties from its XML child nodes
    bool parseBody(rapidxml::xml_node<>* bodyNode, ChSystem& my_system);

    std::map<std::string, std::function<void(rapidxml::xml_node<>*, ChSystem&, std::shared_ptr<ChBody>)>>
        function_table;
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
