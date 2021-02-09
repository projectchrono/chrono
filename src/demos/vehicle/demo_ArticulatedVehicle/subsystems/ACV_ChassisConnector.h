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
// Actuated articulation between front and rear chassis
//
// =============================================================================

#ifndef ACV_CHASSIS_CONNECTOR_H
#define ACV_CHASSIS_CONNECTOR_H

#include "chrono_vehicle/chassis/ChChassisConnectorArticulated.h"

class ACV_ChassisConnector : public chrono::vehicle::ChChassisConnectorArticulated {
  public:
    ACV_ChassisConnector(const std::string& name);
    ~ACV_ChassisConnector() {}

    virtual double GetMaxSteeringAngle() const override { return m_maxangle; }

  private:
    static const double m_maxangle;
};

#endif
