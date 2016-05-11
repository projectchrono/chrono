// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// HMMWV rack-pinion steering model.
//
// =============================================================================

#ifndef HMMWV_RACKPINION_H
#define HMMWV_RACKPINION_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChRackPinion.h"

#include "models/ChApiModels.h"

namespace hmmwv {

class CH_MODELS_API HMMWV_RackPinion : public chrono::vehicle::ChRackPinion {
  public:
    HMMWV_RackPinion(const std::string& name);
    ~HMMWV_RackPinion() {}

    virtual double GetSteeringLinkMass() const override { return m_steeringLinkMass; }
    virtual chrono::ChVector<> GetSteeringLinkInertia() const override { return m_steeringLinkInertia; }
    virtual double GetSteeringLinkCOM() const override { return m_steeringLinkCOM; }
    virtual double GetSteeringLinkRadius() const override { return m_steeringLinkRadius; }
    virtual double GetSteeringLinkLength() const override { return m_steeringLinkLength; }

    virtual double GetPinionRadius() const override { return m_pinionRadius; }

    virtual double GetMaxAngle() const override { return m_maxAngle; }

  private:
    static const double m_steeringLinkMass;
    static const chrono::ChVector<> m_steeringLinkInertia;
    static const double m_steeringLinkCOM;
    static const double m_steeringLinkRadius;
    static const double m_steeringLinkLength;

    static const double m_pinionRadius;

    static const double m_maxAngle;
};

}  // end namespace hmmwv

#endif
