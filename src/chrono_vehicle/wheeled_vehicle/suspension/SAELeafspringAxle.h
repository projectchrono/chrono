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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Leaf-spring solid axle suspension constructed with data from file.
//
// =============================================================================

#ifndef SAE_LEAFSPRINGAXLE_H
#define SAE_LEAFSPRINGAXLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSAELeafspringAxle.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Leaf-spring solid axle suspension constructed with data from file.
class CH_VEHICLE_API SAELeafspringAxle : public ChSAELeafspringAxle {
  public:
    SAELeafspringAxle(const std::string& filename);
    SAELeafspringAxle(const rapidjson::Document& d);
    ~SAELeafspringAxle();

  protected:
    virtual double getCamberAngle() const override { return m_camber_angle; }
    virtual double getToeAngle() const override { return m_toe_angle; }

    /// Return the center of mass of the axle tube.
    virtual const ChVector3d getAxleTubeCOM() const override { return m_axleTubeCOM; }
    /// Return the radius of the spindle body (visualization only).
    virtual double getSpindleRadius() const override { return m_spindleRadius; }

    /// Return the width of the spindle body (visualization only).
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    /// Return the mass of the axle tube body.
    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const override { return m_spindleMass; }

    virtual double getFrontLeafMass() const override { return m_frontleafMass; }
    virtual double getRearLeafMass() const override { return m_rearleafMass; }
    virtual double getClampMass() const override { return m_clampMass; }
    virtual double getShackleMass() const override { return m_clampMass; }

    /// Return the radius of the axle tube body (visualization only).
    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }

    /// Return the moments of inertia of the axle tube body.
    virtual const ChVector3d& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    /// Return the moments of inertia of the spindle body.
    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getFrontLeafInertia() const override { return m_frontleafInertia; }
    virtual const ChVector3d& getRearLeafInertia() const override { return m_rearleafInertia; }
    virtual const ChVector3d& getClampInertia() const override { return m_clampInertia; }
    virtual const ChVector3d& getShackleInertia() const override { return m_shackleInertia; }

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const override { return m_axleInertia; }

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const override { return m_springRestLength; }
    /// Return the free (rest) length of the shock element.
    virtual double getShockRestLength() const override { return m_shockRestLength; }
    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> getLatTorqueFunctorA() const override {
        return m_latRotSpringCBA;
    }
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> getLatTorqueFunctorB() const override {
        return m_latRotSpringCBB;
    }

    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> getVertTorqueFunctorA() const override {
        return m_vertRotSpringCBA;
    }
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> getVertTorqueFunctorB() const override {
        return m_vertRotSpringCBB;
    }

    virtual std::shared_ptr<ChJoint::BushingData> getShackleBushingData() const override {
        return m_shackleBushingData;
    }
    virtual std::shared_ptr<ChJoint::BushingData> getClampBushingData() const override { return m_clampBushingData; }
    virtual std::shared_ptr<ChJoint::BushingData> getLeafspringBushingData() const override {
        return m_leafspringBushingData;
    }

  private:
    virtual const ChVector3d getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_latRotSpringCBA;
    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_latRotSpringCBB;

    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_vertRotSpringCBA;
    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_vertRotSpringCBB;

    ChVector3d m_points[NUM_POINTS];

    ////double m_damperDegressivityCompression;
    ////double m_damperDegressivityExpansion;

    double m_camber_angle;
    double m_toe_angle;

    double m_spindleMass;
    double m_axleTubeMass;

    double m_frontleafMass;
    double m_rearleafMass;
    double m_clampMass;
    double m_shackleMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_axleTubeRadius;

    double m_springRestLength;
    double m_shockRestLength;
    double m_axleInertia;

    ChVector3d m_spindleInertia;
    ChVector3d m_axleTubeInertia;
    ChVector3d m_axleTubeCOM;

    ChVector3d m_frontleafInertia;
    ChVector3d m_rearleafInertia;
    ChVector3d m_clampInertia;
    ChVector3d m_shackleInertia;

    std::shared_ptr<ChJoint::BushingData> m_shackleBushingData;
    std::shared_ptr<ChJoint::BushingData> m_clampBushingData;
    std::shared_ptr<ChJoint::BushingData> m_leafspringBushingData;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
