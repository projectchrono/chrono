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
// Three-link Independent Rear Suspension constructed with data from file.
//
// =============================================================================

#ifndef THREELINK_IRS_H
#define THREELINK_IRS_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChThreeLinkIRS.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Three-link Independent Rear Suspension constructed with data from file.
class CH_VEHICLE_API ThreeLinkIRS : public ChThreeLinkIRS {
  public:
    ThreeLinkIRS(const std::string& filename);
    ThreeLinkIRS(const rapidjson::Document& d);
    ~ThreeLinkIRS();

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getArmMass() const override { return m_armMass; }
    virtual double getUpperLinkMass() const override { return m_upperMass; }
    virtual double getLowerLinkMass() const override { return m_lowerMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getArmRadius() const override { return m_armRadius; }
    virtual double getUpperLinkRadius() const override { return m_upperLinkRadius; }
    virtual double getLowerLinkRadius() const override { return m_lowerLinkRadius; }

    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getArmInertia() const override { return m_armInertia; }
    virtual const ChVector<>& getUpperLinkInertia() const override { return m_upperInertia; }
    virtual const ChVector<>& getLowerLinkInertia() const override { return m_lowerInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    virtual ChSpringForceCallback* getSpringForceCallback() const override { return m_springForceCB; }
    virtual ChSpringForceCallback* getShockForceCallback() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector<> getDirection(DirectionId which) override { return m_dirs[which]; }

    void Create(const rapidjson::Document& d);

    ChSpringForceCallback* m_springForceCB;
    ChSpringForceCallback* m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];
    ChVector<> m_dirs[NUM_DIRS];

    double m_spindleMass;
    double m_armMass;
    double m_upperMass;
    double m_lowerMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_armRadius;
    double m_upperLinkRadius;
    double m_lowerLinkRadius;

    ChVector<> m_spindleInertia;
    ChVector<> m_armInertia;
    ChVector<> m_upperInertia;
    ChVector<> m_lowerInertia;

    double m_axleInertia;

    double m_springRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
