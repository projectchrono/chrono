// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility functions for parsing JSON files.
//
// =============================================================================

#ifndef CH_UTILS_JSON_H
#define CH_UTILS_JSON_H

#include <string>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"

#include "chrono/core/ChApiCE.h"
#include "chrono/assets/ChColor.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChFrame.h"
#include "chrono/functions/ChFunctionInterp.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChJoint.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

// -----------------------------------------------------------------------------

/// Load and return a RapidJSON document from the specified file.
/// A Null document is returned if the file cannot be opened.
ChApi void ReadFileJSON(const std::string& filename, rapidjson::Document& d);

// -----------------------------------------------------------------------------

/// Load and return a ChVector3d from the specified JSON array.
ChApi ChVector3d ReadVectorJSON(const rapidjson::Value& a);

///  Load and return a ChQuaternion from the specified JSON array.
ChApi ChQuaternion<> ReadQuaternionJSON(const rapidjson::Value& a);

/// Load and return a coordinate system from the specific JSON value.
ChApi ChCoordsys<> ReadCoordinateSystemJSON(const rapidjson::Value& a);

///  Load and return a ChFrame from the specified JSON array
ChApi ChFrame<> ReadFrameJSON(const rapidjson::Value& a);

///  Load and return a ChColor from the specified JSON array.
ChApi ChColor ReadColorJSON(const rapidjson::Value& a);

// -----------------------------------------------------------------------------

ChApi ChContactMaterialData ReadMaterialInfoJSON(const rapidjson::Value& mat);

ChApi std::shared_ptr<ChJoint::BushingData> ReadBushingDataJSON(const rapidjson::Value& bd);

/// Load and return a joint type from the specific JSON value.
ChApi ChJoint::Type ReadJointTypeJSON(const rapidjson::Value& a);

// -----------------------------------------------------------------------------

/// Load and return a body geometry structure from the specified JSON value.
/// Collision geometry and contact material information is set in the return ChBodyGeometry object if the given JSON
/// object has a member "Contact". Visualization geometry is loaded if the JSON object has a member "Visualization".
/// Note that file names for collision and visualization geometry are assumed to be relative to the Chrono data directory.
ChApi utils::ChBodyGeometry ReadBodyGeometryJSON(const rapidjson::Value& d);

/// Load and return a TSDA geometry structure from the specified JSON value.
ChApi utils::ChTSDAGeometry ReadTSDAGeometryJSON(const rapidjson::Value& d);

// -----------------------------------------------------------------------------

ChApi std::shared_ptr<ChLinkTSDA::ForceFunctor> ReadTSDAFunctorJSON(const rapidjson::Value& td, double& free_length);
ChApi std::shared_ptr<ChLinkRSDA::TorqueFunctor> ReadRSDAFunctorJSON(const rapidjson::Value& td, double& free_angle);

// -----------------------------------------------------------------------------

/// Utility class for reading and setting an (x,y) map.
class ChApi ChMapData {
  public:
    /// Construct a ChMapData with an empty map.
    ChMapData() : m_n(0) {}

    /// Read data from the specified JSON object.
    void Read(const rapidjson::Value& a);

    /// Set the map data to the specified recorder function.
    /// The map data is scaled by the specified factors.
    void Set(ChFunctionInterp& map, double x_factor = 1, double y_factor = 1) const;

    /// Set the map data to the specified vector of pairs.
    /// The map data is scaled by the specified factors.
    void Set(std::vector<std::pair<double, double>>& vec, double x_factor = 1, double y_factor = 1) const;

  private:
    unsigned int m_n;
    std::vector<double> m_x;
    std::vector<double> m_y;
};

/// @} chrono_io

}  // end namespace chrono

#endif
