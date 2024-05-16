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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// BMW E90 rack-pinion steering model.
// Vehicle Parameters taken from SAE Paper 2007-01-0818
// Steering trapez behind origin (original in front of origin)
//
// =============================================================================

#ifndef BMW_E90_RACKPINION_H
#define BMW_E90_RACKPINION_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// Rack-pinion steering subsystem for the BMW E90 vehicle.
class CH_MODELS_API BMW_E90_Steering : public ChRackPinion {
  public:
    BMW_E90_Steering(const std::string& name);

    ~BMW_E90_Steering() {}

    virtual double GetSteeringLinkMass() const override { return m_steeringLinkMass; }

    virtual ChVector3d GetSteeringLinkInertia() const override { return m_steeringLinkInertia; }

    virtual double GetSteeringLinkCOM() const override { return m_steeringLinkCOM; }

    virtual double GetSteeringLinkRadius() const override { return m_steeringLinkRadius; }

    virtual double GetSteeringLinkLength() const override { return m_steeringLinkLength; }

    virtual double GetPinionRadius() const override { return m_pinionRadius; }

    virtual double GetMaxAngle() const override { return m_maxAngle; }

  private:
    static const double m_steeringLinkMass;
    static const ChVector3d m_steeringLinkInertia;
    static const double m_steeringLinkCOM;
    static const double m_steeringLinkRadius;
    static const double m_steeringLinkLength;

    static const double m_pinionRadius;

    static const double m_maxAngle;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
