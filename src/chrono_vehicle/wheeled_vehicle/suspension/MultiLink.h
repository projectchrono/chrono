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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Multi-link suspension constructed with data from file.
//
// =============================================================================

#ifndef MULTILINK_H
#define MULTILINK_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChMultiLink.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Multi-link suspension constructed with data from file.
class CH_VEHICLE_API MultiLink : public ChMultiLink {
  public:
    MultiLink(const std::string& filename);
    MultiLink(const rapidjson::Document& d);
    ~MultiLink();

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
    virtual ChLinkSpringCB::ForceFunctor* getSpringForceFunctor() const override { return m_springForceCB; }
    virtual ChLinkSpringCB::ForceFunctor* getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector<> getDirection(DirectionId which) override { return m_directions[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChLinkSpringCB::ForceFunctor* m_springForceCB;
    ChLinkSpringCB::ForceFunctor* m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];
    ChVector<> m_directions[NUM_DIRS];

    double m_spindleMass;
    double m_upperArmMass;
    double m_lateralMass;
    double m_trailingLinkMass;
    double m_uprightMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_upperArmRadius;
    double m_lateralRadius;
    double m_trailingLinkRadius;
    double m_uprightRadius;

    ChVector<> m_spindleInertia;
    ChVector<> m_upperArmInertia;
    ChVector<> m_lateralInertia;
    ChVector<> m_trailingLinkInertia;
    ChVector<> m_uprightInertia;

    double m_axleInertia;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
