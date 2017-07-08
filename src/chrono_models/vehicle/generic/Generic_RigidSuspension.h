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
// Generic concrete rigid suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChRigidSuspension) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef GENERIC_RIGID_SUSPENSION_H
#define GENERIC_RIGID_SUSPENSION_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidSuspension.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Rigid suspension for a generic vehicle (spindles directly attached to chassis).
class CH_MODELS_API Generic_RigidSuspension : public ChRigidSuspension {
  public:
    Generic_RigidSuspension(const std::string& name) : ChRigidSuspension(name) {}

    ~Generic_RigidSuspension() {}

    virtual const ChVector<> getLocation(PointId which) override;

    virtual double getSpindleMass() const override { return m_spindleMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

  private:
    static const double m_spindleMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const ChVector<> m_spindleInertia;

    static const double m_axleInertia;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
