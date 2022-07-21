// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Base class for a force element tire model
//
// =============================================================================

#ifndef CH_FORCEELEMENT_TIRE_H
#define CH_FORCEELEMENT_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Base class for a force lement tire model.
class CH_VEHICLE_API ChForceElementTire : public ChTire {
  public:
    virtual ~ChForceElementTire() {}

  protected:
    /// Construct a tire with the specified name.
    ChForceElementTire(const std::string& name);

    /// Return the tire mass.
    virtual double GetTireMass() const = 0;

    /// Return the tire moments of inertia (in the tire centroidal frame).
    virtual ChVector<> GetTireInertia() const = 0;

    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const = 0;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const = 0;

  private:
    virtual void InitializeInertiaProperties() override final;
    virtual void UpdateInertiaProperties() override final;

    virtual double GetAddedMass() const override final;
    virtual ChVector<> GetAddedInertia() const override final;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
