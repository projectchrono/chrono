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
// Authors: Alessandro Tasora
// =============================================================================
//
// G500 simple brake models (front and rear).
//
// =============================================================================

#ifndef G500_BRAKESIMPLE_H
#define G500_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace gclass {

/// @addtogroup vehicle_models_gclass
/// @{

/// Simple UAZBUS front brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API G500_BrakeSimpleFront : public ChBrakeSimple {
  public:
    G500_BrakeSimpleFront(const std::string& name);
    virtual ~G500_BrakeSimpleFront() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

/// Simple UAZBUS rear brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API G500_BrakeSimpleRear : public ChBrakeSimple {
  public:
    G500_BrakeSimpleRear(const std::string& name);
    virtual ~G500_BrakeSimpleRear() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

/// @} vehicle_models_gclass

}  // end namespace gclass
}  // end namespace vehicle
}  // end namespace chrono

#endif
