// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// A set of helper functions for pulling data from json structures into Chrono
// or SynChrono objects
//
// =============================================================================

#include <fstream>

#include "chrono_synchrono/utils/SynUtilsJSON.h"

#include "chrono/core/ChLog.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using namespace chrono::vehicle;
using namespace rapidjson;

namespace chrono {
namespace synchrono {

// -----------------------------------------------------------------------------

Document ReadFileJSON(const std::string& filename) {
    Document d;
    std::ifstream ifs(filename);
    if (!ifs.good()) {
        GetLog() << "ERROR: Could not open JSON file: " << filename << "\n";
    } else {
        IStreamWrapper isw(ifs);
        d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
        if (d.IsNull()) {
            GetLog() << "ERROR: Invalid JSON file: " << filename << "\n";
        }
    }
    return d;
}

// -----------------------------------------------------------------------------

ChVector<> ReadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

ChQuaternion<> ReadQuaternionJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

ChCoordsys<> ReadCoordsysJSON(const Value& a, const Value& b) {
    return ChCoordsys<>(ReadVectorJSON(a), ReadQuaternionJSON(b));
}

// -----------------------------------------------------------------------------

VisualizationType ReadVisualizationTypeJSON(const std::string& type) {
    // Determine visualization type.
    VisualizationType visualization_type;
    if (type.compare("None") == 0) {
        visualization_type = VisualizationType::NONE;
    } else if (type.compare("Primitives") == 0) {
        visualization_type = VisualizationType::PRIMITIVES;
    } else if (type.compare("Mesh") == 0) {
        visualization_type = VisualizationType::MESH;
    } else {
        throw ChException("Visualization type \"" + type + "\" is not a supported type in ReadVisualizationTypeJSON.");
    }
    return visualization_type;
}

}  // namespace synchrono
}  // namespace chrono
