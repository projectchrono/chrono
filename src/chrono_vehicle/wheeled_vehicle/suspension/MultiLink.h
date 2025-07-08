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

    virtual bool UseTierodBodies() const override { return m_use_tierod_bodies; }

    virtual double getCamberAngle() const override { return m_camber_angle; }
    virtual double getToeAngle() const override { return m_toe_angle; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getUpperArmMass() const override { return m_upperArmMass; }
    virtual double getLateralMass() const override { return m_lateralMass; }
    virtual double getTrailingLinkMass() const override { return m_trailingLinkMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }
    virtual double getTierodMass() const override { return m_tierodMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getUpperArmRadius() const override { return m_upperArmRadius; }
    virtual double getLateralRadius() const override { return m_lateralRadius; }
    virtual double getTrailingLinkRadius() const override { return m_trailingLinkRadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }
    virtual double getTierodRadius() const override { return m_tierodRadius; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getUpperArmInertia() const override { return m_upperArmInertia; }
    virtual const ChVector3d& getLateralInertia() const override { return m_lateralInertia; }
    virtual const ChVector3d& getTrailingLinkInertia() const override { return m_trailingLinkInertia; }
    virtual const ChVector3d& getUprightInertia() const override { return m_uprightInertia; }
    virtual const ChVector3d getTierodInertia() const override { return m_tierodInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual double getShockRestLength() const override { return m_shockRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual std::shared_ptr<ChJoint::BushingData> getTierodBushingData() const override { return m_tierodBushingData; }

  private:
    virtual const ChVector3d getLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector3d getDirection(DirectionId which) override { return m_directions[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    ChVector3d m_points[NUM_POINTS];
    ChVector3d m_directions[NUM_DIRS];

    bool m_use_tierod_bodies;

    double m_camber_angle;
    double m_toe_angle;

    double m_spindleMass;
    double m_upperArmMass;
    double m_lateralMass;
    double m_trailingLinkMass;
    double m_uprightMass;
    double m_tierodMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_upperArmRadius;
    double m_lateralRadius;
    double m_trailingLinkRadius;
    double m_uprightRadius;
    double m_tierodRadius;

    ChVector3d m_spindleInertia;
    ChVector3d m_upperArmInertia;
    ChVector3d m_lateralInertia;
    ChVector3d m_trailingLinkInertia;
    ChVector3d m_uprightInertia;
    ChVector3d m_tierodInertia;

    double m_axleInertia;

    std::shared_ptr<ChJoint::BushingData> m_tierodBushingData;

    double m_springRestLength;
    double m_shockRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
