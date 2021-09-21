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
// Generic simple brake model
//
// =============================================================================

#ifndef ACV_BRAKE_SIMPLE_H
#define ACV_BRAKE_SIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

class ACV_BrakeSimple : public chrono::vehicle::ChBrakeSimple {
  public:
    ACV_BrakeSimple(const std::string& name);
    ~ACV_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

#endif
