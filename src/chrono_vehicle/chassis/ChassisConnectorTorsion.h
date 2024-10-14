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
// Torsion chassis connector model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef CHASSIS_CONNECTOR_TORSION_H
#define CHASSIS_CONNECTOR_TORSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/chassis/ChChassisConnectorTorsion.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Torsion chassis connector model constructed with data from file (JSON format).
class CH_VEHICLE_API ChassisConnectorTorsion : public ChChassisConnectorTorsion {
  public:
    ChassisConnectorTorsion(const std::string& filename);
    ChassisConnectorTorsion(const rapidjson::Document& d);
    ~ChassisConnectorTorsion() {}

    /// Return the torsion stiffness of the chassis.
    virtual double GetTorsionStiffness() const override { return m_torsion_stiffness; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_torsion_stiffness;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
