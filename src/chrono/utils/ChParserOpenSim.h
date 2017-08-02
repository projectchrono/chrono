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
#include "chrono/physics/ChBodyAuxRef.h"

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
    enum VisType { PRIMITIVES, MESH };

    void parse(ChSystem& system,             ///< [in] containing Chrono system
               const std::string& filename,  ///< [in] OpenSim input file name
               VisType vis = VisType::PRIMITIVES);

    ChSystem* parse(const std::string& filename,  ///< [in] OpenSim input file name
                    ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC,  ///< [in] contact method
                    VisType vis = VisType::PRIMITIVES);

  private:
    /// Setup lambda table for body parsing
    void initFunctionTable();

    /// Creates body and parses its various properties from its XML child nodes
    bool parseBody(rapidxml::xml_node<>* bodyNode, ChSystem& my_system);

    // Initializes visualization shapes for bodies connected to each link
    void initVisualizations(rapidxml::xml_node<>* node, ChSystem& p_system);

    // Get an STL vector from a string, used to make the xml parsing cleaner
    template <typename T>
    static inline std::vector<T> strToSTLVector(const char* string) {
        std::istringstream buf(string);
        std::istream_iterator<T> beg(buf), end;
        return std::vector<T>(beg, end);
    }
    // Maps child fields of a body node to functions that handle said fields
    std::map<std::string, std::function<void(rapidxml::xml_node<>*, ChSystem&, std::shared_ptr<ChBodyAuxRef>)>>
        function_table;
    VisType m_visType;
    std::vector<std::shared_ptr<ChLink>> m_jointList;
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
