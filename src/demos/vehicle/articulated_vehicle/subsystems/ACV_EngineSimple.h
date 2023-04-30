// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Generic engine model for a vehicle with a trivial speed-torque curve
//
// =============================================================================

#ifndef ACV_ENGINE_SIMPLE_H
#define ACV_ENGINE_SIMPLE_H

#include "chrono_vehicle/powertrain/ChEngineSimple.h"

class ACV_EngineSimple : public chrono::vehicle::ChEngineSimple {
  public:
    ACV_EngineSimple(const std::string& name);
    ~ACV_EngineSimple() {}

    virtual double GetMaxPower() const override { return m_max_power; }
    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_max_power;
    static const double m_max_torque;
    static const double m_max_speed;
};

#endif
