// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Ian Rust
// =============================================================================
//
// FifthWheel chassis connector model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef CHASSIS_CONNECTOR_FIFTH_WHEEL_H
#define CHASSIS_CONNECTOR_FIFTH_WHEEL_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/chassis/ChChassisConnectorFifthWheel.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Fifth wheel chassis connector model constructed with data from file (JSON format).
class CH_VEHICLE_API ChassisConnectorFifthWheel : public ChChassisConnectorFifthWheel {
  public:
    ChassisConnectorFifthWheel(const std::string& filename);
    ChassisConnectorFifthWheel(const rapidjson::Document& d);
    ~ChassisConnectorFifthWheel() {}

  private:
    virtual void Create(const rapidjson::Document& d) override;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
