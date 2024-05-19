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
// Authors: Radu Serban, Justin Madsen, Asher Elmquist
// =============================================================================
//
// BMW E90 (330i 2006) rear wheel subsystem 8x18
// Data from https://www.rsu.de (Tire Dealer)
//
// =============================================================================

#ifndef BMW_E90_REAR_WHEEL_H
#define BMW_E90_REAR_WHEEL_H

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// Sedan wheel (can be used on any axle, left or right).
class CH_MODELS_API BMW_E90_RearWheel : public ChWheel {
  public:
    BMW_E90_RearWheel(const std::string& name);
    ~BMW_E90_RearWheel() {}

    virtual double GetWheelMass() const override { return m_mass; }
    virtual const ChVector3d& GetWheelInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }

  protected:
    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector3d m_inertia;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
