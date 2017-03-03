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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Generic multi-link suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origin at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#ifndef GENERIC_MULTILINK_H
#define GENERIC_MULTILINK_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChMultiLink.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

class CH_MODELS_API Generic_MultiLink : public ChMultiLink {
  public:
    Generic_MultiLink(const std::string& name);
    ~Generic_MultiLink();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUpperArmMass() const override { return m_upperArmMass; }
    virtual double getLateralMass() const override { return m_lateralMass; }
    virtual double getTrailingLinkMass() const override { return m_trailingLinkMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUpperArmRadius() const override { return m_upperArmRadius; }
    virtual double getLateralRadius() const override { return m_lateralRadius; }
    virtual double getTrailingLinkRadius() const override { return m_trailingLinkRadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getUpperArmInertia() const override { return m_upperArmInertia; }
    virtual const ChVector<>& getLateralInertia() const override { return m_lateralInertia; }
    virtual const ChVector<>& getTrailingLinkInertia() const override { return m_trailingLinkInertia; }
    virtual const ChVector<>& getUprightInertia() const override { return m_uprightInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual ChSpringForceCallback* getSpringForceCallback() const override { return m_springForceCB; }
    virtual ChSpringForceCallback* getShockForceCallback() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override;
    virtual const ChVector<> getDirection(DirectionId which) override;

    ChSpringForceCallback* m_springForceCB;
    ChSpringForceCallback* m_shockForceCB;

    static const double m_spindleMass;
    static const double m_upperArmMass;
    static const double m_lateralMass;
    static const double m_trailingLinkMass;
    static const double m_uprightMass;

    static const double m_spindleRadius;
    static const double m_spindleWidth;
    static const double m_upperArmRadius;
    static const double m_lateralRadius;
    static const double m_trailingLinkRadius;
    static const double m_uprightRadius;

    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_upperArmInertia;
    static const ChVector<> m_lateralInertia;
    static const ChVector<> m_trailingLinkInertia;
    static const ChVector<> m_uprightInertia;

    static const double m_axleInertia;

    static const double m_springCoefficient;
    static const double m_dampingCoefficient;
    static const double m_springRestLength;
};

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
