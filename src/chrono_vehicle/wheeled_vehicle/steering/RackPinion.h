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
// Rack-pinion steering model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef RACK_PINION_H
#define RACK_PINION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_steering
/// @{

/// Rack-pinion steering model constructed with data from file (JSON format).
class CH_VEHICLE_API RackPinion : public ChRackPinion {
  public:
    RackPinion(const std::string& filename);
    RackPinion(const rapidjson::Document& d);
    ~RackPinion() {}

    virtual double GetSteeringLinkMass() const override { return m_steeringLinkMass; }
    virtual ChVector<> GetSteeringLinkInertia() const override { return m_steeringLinkInertia; }
    virtual double GetSteeringLinkCOM() const override { return m_steeringLinkCOM; }
    virtual double GetSteeringLinkRadius() const override { return m_steeringLinkRadius; }
    virtual double GetSteeringLinkLength() const override { return m_steeringLinkLength; }

    virtual double GetPinionRadius() const override { return m_pinionRadius; }

    virtual double GetMaxAngle() const override { return m_maxAngle; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_steeringLinkMass;
    ChVector<> m_steeringLinkInertia;
    double m_steeringLinkCOM;
    double m_steeringLinkRadius;
    double m_steeringLinkLength;

    double m_pinionRadius;
    double m_maxAngle;
};

/// @} vehicle_wheeled_steering

}  // end namespace vehicle
}  // end namespace chrono

#endif
