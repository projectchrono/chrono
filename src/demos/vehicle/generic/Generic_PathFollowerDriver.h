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
//
// =============================================================================

#ifndef GENERIC_PATHFOLLOWER_DRIVER_H
#define GENERIC_PATHFOLLOWER_DRIVER_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

class Generic_PathFollowerDriver : public chrono::ChDriver {
  public:
    Generic_PathFollowerDriver(chrono::ChVehicle& vehicle, chrono::ChBezierCurve* path);
    ~Generic_PathFollowerDriver() {}

    chrono::ChPathSteeringController& GetSteeringController() { return m_PID; }

    void Reset();
    virtual void Advance(double step) override;

  private:
    chrono::ChVehicle& m_vehicle;
    chrono::ChPathSteeringController m_PID;
};

#endif
