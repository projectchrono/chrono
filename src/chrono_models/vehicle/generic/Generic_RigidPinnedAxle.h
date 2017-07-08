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
// Generic concrete rigid-pinned-axle suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChRigidSuspension) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#ifndef GENERIC_RIGID_PINNED_AXLE_H
#define GENERIC_RIGID_PINNED_AXLE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidPinnedAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Rigid pinned axle suspension for a generic vehicle (spindles attached to a rigid axle).
class CH_MODELS_API Generic_RigidPinnedAxle : public ChRigidPinnedAxle {
  public:
    Generic_RigidPinnedAxle(const std::string& name) : ChRigidPinnedAxle(name) {}

    ~Generic_RigidPinnedAxle() {}

    virtual const ChVector<> getLocation(PointId which) override;

    virtual const ChVector<> getAxleTubeCOM() const override { return m_axleTubeCOM; }
    virtual const ChVector<> getAxlePinLocation() const override { return m_axlePinLoc; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

  private:
    static const double m_spindleMass;
    static const double m_axleTubeMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_axleTubeRadius;

    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_axleTubeInertia;

    static const ChVector<> m_axleTubeCOM;
    static const ChVector<> m_axlePinLoc;

    static const double m_axleInertia;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
