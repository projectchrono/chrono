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

    virtual double getCamberAngle() const override { return m_camber_angle; }
    virtual double getToeAngle() const override { return m_toe_angle; }

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
    virtual double getShockRestLength() const override { return m_shockRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual std::shared_ptr<ChVehicleBushingData> getArmChassisBushingData() const { return m_armChassisBushingData; }
    virtual std::shared_ptr<ChVehicleBushingData> getArmUpperBushingData() const { return m_armUpperBushingData; }
    virtual std::shared_ptr<ChVehicleBushingData> getArmLowerBushingData() const { return m_armLowerBushingData; }
    virtual std::shared_ptr<ChVehicleBushingData> getChassisUpperBushingData() const { return m_chassisUpperBushingData; }
    virtual std::shared_ptr<ChVehicleBushingData> getChassisLowerBushingData() const { return m_chassisLowerBushingData; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector<> getDirection(DirectionId which) override { return m_dirs[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];
    ChVector<> m_dirs[NUM_DIRS];

    double m_camber_angle;
    double m_toe_angle;

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

    std::shared_ptr<ChVehicleBushingData> m_armChassisBushingData;
    std::shared_ptr<ChVehicleBushingData> m_armUpperBushingData;
    std::shared_ptr<ChVehicleBushingData> m_armLowerBushingData;
    std::shared_ptr<ChVehicleBushingData> m_chassisUpperBushingData;
    std::shared_ptr<ChVehicleBushingData> m_chassisLowerBushingData;

    double m_springRestLength;
    double m_shockRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
