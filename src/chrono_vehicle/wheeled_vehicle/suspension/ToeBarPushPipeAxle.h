// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Steerable PushPipe axle suspension constructed with data from file.
//
// =============================================================================

#ifndef TOEBAR_PUSHPIPEAXLE_H
#define TOEBAR_PUSHPIPEAXLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChToeBarPushPipeAxle.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Steerable leaf-spring solid axle suspension constructed with data from file.
class CH_VEHICLE_API ToeBarPushPipeAxle : public ChToeBarPushPipeAxle {
  public:
    ToeBarPushPipeAxle(const std::string& filename);
    ToeBarPushPipeAxle(const rapidjson::Document& d);
    ~ToeBarPushPipeAxle();

  protected:
    virtual double getCamberAngle() const override { return m_camber_angle; }
    virtual double getToeAngle() const override { return m_toe_angle; }

    /// Return the mass of the knuckle body.
    virtual double getKnuckleMass() const override { return m_knuckleMass; }

    /// Return the mass of the tierod body.
    virtual double getTierodMass() const override { return m_tierodMass; }

    /// Return the mass of the draglink body.
    virtual double getDraglinkMass() const override { return m_draglinkMass; }

    /// Return the radius of the knuckle body (visualization only).
    virtual double getKnuckleRadius() const override { return m_knuckleRadius; }

    /// Return the radius of the tierod body (visualization only).
    virtual double getTierodRadius() const override { return m_tierodRadius; }

    /// Return the radius of the draglink body (visualization only).
    virtual double getDraglinkRadius() const override { return m_draglinkRadius; }

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
    /// Return the mass of the panhard rod body.
    virtual double getPanhardRodMass() const override { return m_panhardRodMass; }

    /// Return the radius of the axle tube body (visualization only).
    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    /// Return the radius of the axle tube body (visualization only).
    virtual double getPanhardRodRadius() const override { return m_panhardRodRadius; };

    /// return the vertical offset of the axle tube
    virtual double getPortalOffset() const override { return m_portalOffset; }

    /// Return the moments of inertia of the axle tube body.
    virtual const ChVector3d& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    /// Return the moments of inertia of the spindle body.
    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    /// Return the moments of inertia of the knuckle body.
    virtual const ChVector3d& getKnuckleInertia() const override { return m_knuckleInertia; }
    /// Return the moments of inertia of the tierod body.
    virtual const ChVector3d& getTierodInertia() const override { return m_tierodInertia; }
    /// Return the moments of inertia of the draglink body.
    virtual const ChVector3d& getDraglinkInertia() const override { return m_draglinkInertia; }

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const override { return m_axleInertia; }
    /// Return the moments of inertia of the panhard rod body.
    virtual const ChVector3d& getPanhardRodInertia() const override { return m_panhardRodInertia; }

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const override { return m_springRestLength; }
    /// Return the free (rest) length of the shock element.
    virtual double getShockRestLength() const override { return m_shockRestLength; }

    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual bool isLeftKnuckleActuated() override { return m_use_left_knuckle; }

  private:
    virtual const ChVector3d getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    ChVector3d m_points[NUM_POINTS];

    ////double m_damperDegressivityCompression;
    ////double m_damperDegressivityExpansion;

    double m_camber_angle;
    double m_toe_angle;

    double m_spindleMass;
    double m_axleTubeMass;
    double m_knuckleMass;
    double m_tierodMass;
    double m_draglinkMass;
    double m_panhardRodMass;

    double m_portalOffset;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_axleTubeRadius;
    double m_knuckleRadius;
    double m_tierodRadius;
    double m_draglinkRadius;
    double m_panhardRodRadius;

    double m_springRestLength;
    double m_shockRestLength;
    double m_axleInertia;

    ChVector3d m_spindleInertia;
    ChVector3d m_axleTubeInertia;
    ChVector3d m_panhardRodInertia;
    ChVector3d m_axleTubeCOM;
    ChVector3d m_knuckleInertia;
    ChVector3d m_tierodInertia;
    ChVector3d m_draglinkInertia;

    bool m_use_left_knuckle;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
