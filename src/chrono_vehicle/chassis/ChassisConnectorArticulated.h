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
// Articulated chassis connector model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef CHASSIS_CONNECTOR_ARTICULATED_H
#define CHASSIS_CONNECTOR_ARTICULATED_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/chassis/ChChassisConnectorArticulated.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Articulated chassis connector model constructed with data from file (JSON format).
class CH_VEHICLE_API ChassisConnectorArticulated : public ChChassisConnectorArticulated {
  public:
    ChassisConnectorArticulated(const std::string& filename);
    ChassisConnectorArticulated(const rapidjson::Document& d);
    ~ChassisConnectorArticulated() {}

    ///  Return the maximum steering angle.  The steering input is scaled by this value to produce the angle applied to
    ///  the underlying rotational motor.
    virtual double GetMaxSteeringAngle() const override { return m_maxangle; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_maxangle;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
