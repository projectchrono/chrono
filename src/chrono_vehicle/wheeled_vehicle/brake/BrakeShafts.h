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
// Wheeled vehicle shafts-based brake model constructed with data from file
// (JSON format).
//
// =============================================================================

#ifndef BRAKE_SHAFTS_H
#define BRAKE_SHAFTS_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_brake
/// @{

/// Wheeled vehicle shafts-based brake model constructed with data from file (JSON format).
class CH_VEHICLE_API BrakeShafts : public ChBrakeShafts {
  public:
    BrakeShafts(const std::string& filename);
    BrakeShafts(const rapidjson::Document& d);
    ~BrakeShafts() {}

    virtual double GetShaftInertia() override { return m_shaft_inertia; }
    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_shaft_inertia;
    double m_maxtorque;
};

/// @} vehicle_wheeled_brake

}  // end namespace vehicle
}  // end namespace chrono

#endif
