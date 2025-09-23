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
// Single A-arm suspension constructed with data from file.
//
// =============================================================================

#ifndef SINGLE_WISHBONE_H
#define SINGLE_WISHBONE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSingleWishbone.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Single A-arm suspension constructed with data from file.
class CH_VEHICLE_API SingleWishbone : public ChSingleWishbone {
  public:
    SingleWishbone(const std::string& filename);
    SingleWishbone(const rapidjson::Document& d);
    ~SingleWishbone();

    virtual bool UseTierodBodies() const override { return m_use_tierod_bodies; }

    virtual double getCamberAngle() const override { return m_camber_angle; }
    virtual double getToeAngle() const override { return m_toe_angle; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getCAMass() const override { return m_CAMass; }
    virtual double getUprightMass() const override { return m_uprightMass; }
    virtual double getTierodMass() const override { return m_tierodMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getCARadius() const override { return m_CARadius; }
    virtual double getUprightRadius() const override { return m_uprightRadius; }
    virtual double getTierodRadius() const override { return m_tierodRadius; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getCAInertiaMoments() const override { return m_CAInertiaMoments; }
    virtual const ChVector3d& getCAInertiaProducts() const override { return m_CAInertiaProducts; }
    virtual const ChVector3d& getUprightInertiaMoments() const override { return m_uprightInertiaMoments; }
    virtual const ChVector3d& getUprightInertiaProducts() const override { return m_uprightInertiaProducts; }
    virtual const ChVector3d getTierodInertia() const override { return m_tierodInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual double getShockRestLength() const override { return m_shockRestLength; }
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual std::shared_ptr<ChJoint::BushingData> getCABushingData() const override { return m_CABushingData; }
    virtual std::shared_ptr<ChJoint::BushingData> getTierodBushingData() const override { return m_tierodBushingData; }

  private:
    virtual const ChVector3d getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    ChVector3d m_points[NUM_POINTS];

    bool m_use_tierod_bodies;

    double m_camber_angle;
    double m_toe_angle;

    double m_spindleMass;
    double m_CAMass;
    double m_uprightMass;
    double m_tierodMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_CARadius;
    double m_uprightRadius;
    double m_tierodRadius;

    ChVector3d m_spindleInertia;
    ChVector3d m_CAInertiaMoments;
    ChVector3d m_CAInertiaProducts;
    ChVector3d m_uprightInertiaMoments;
    ChVector3d m_uprightInertiaProducts;
    ChVector3d m_tierodInertia;

    double m_axleInertia;

    std::shared_ptr<ChJoint::BushingData> m_CABushingData;
    std::shared_ptr<ChJoint::BushingData> m_tierodBushingData;

    double m_shockRestLength;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
