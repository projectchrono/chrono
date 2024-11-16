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
// Authors: Radu Serban
// =============================================================================
//
// Hitch chassis connector model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef CHASSIS_CONNECTOR_HITCH_H
#define CHASSIS_CONNECTOR_HITCH_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/chassis/ChChassisConnectorHitch.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Hitch chassis connector model constructed with data from file (JSON format).
class CH_VEHICLE_API ChassisConnectorHitch : public ChChassisConnectorHitch {
  public:
    ChassisConnectorHitch(const std::string& filename);
    ChassisConnectorHitch(const rapidjson::Document& d);
    ~ChassisConnectorHitch() {}

  private:
    virtual void Create(const rapidjson::Document& d) override;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
