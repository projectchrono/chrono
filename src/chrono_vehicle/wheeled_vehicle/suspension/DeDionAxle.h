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
// DeDion axle suspension constructed with data from file.
//
// =============================================================================

#ifndef DEDIONAXLE_H
#define DEDIONAXLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDeDionAxle.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Leaf-spring solid axle suspension constructed with data from file.
class CH_VEHICLE_API DeDionAxle : public ChDeDionAxle {
  public:
    DeDionAxle(const std::string& filename);
    DeDionAxle(const rapidjson::Document& d);
    ~DeDionAxle();

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
    /// Return the mass of the Watt center link body.
    virtual double getWattCenterMass() const override { return m_wattCenterMass; };
    /// Return the mass of the Watt side link bodies (same value for both sides).
    virtual double getWattSideMass() const override { return m_wattSideMass; };

    /// Return the radius of the axle tube body (visualization only).
    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    /// Return the radius of the Watt linkage bodies (visualization only, same value for center, left, right).
    virtual double getWattLinkRadius() const override { return m_wattLinkRadius; };

    /// Return the moments of inertia of the axle tube body.
    virtual const ChVector3d& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    /// Return the moments of inertia of the spindle body.
    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    /// Return the moments of inertia of the Watt center body.
    virtual const ChVector3d& getWattCenterInertia() const override { return m_wattCenterInertia; }
    /// Return the moments of inertia of the Watt side bodies (same value for both sides).
    virtual const ChVector3d& getWattSideInertia() const override { return m_wattSideInertia; }

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const override { return m_axleInertia; }

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const override { return m_springRestLength; }

    /// Return the fre (rest) length of the shock element.
    virtual double getShockRestLength() const override { return m_shockRestLength; }

    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

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
    double m_wattCenterMass;
    double m_wattSideMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_axleTubeRadius;
    double m_wattLinkRadius;

    double m_springRestLength;
    double m_shockRestLength;
    double m_axleInertia;

    ChVector3d m_spindleInertia;
    ChVector3d m_axleTubeInertia;
    ChVector3d m_axleTubeCOM;
    ChVector3d m_wattCenterInertia;
    ChVector3d m_wattSideInertia;

    bool m_use_left_knuckle;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
