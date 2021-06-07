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
// Three link solid axle suspension constructed with data from file.
//
// =============================================================================

#ifndef SOLID_THREE_LINK_AXLE_H
#define SOLID_THREE_LINK_AXLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidThreeLinkAxle.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Leaf-spring solid axle suspension constructed with data from file.
class CH_VEHICLE_API SolidThreeLinkAxle : public ChSolidThreeLinkAxle {
  public:
    SolidThreeLinkAxle(const std::string& filename);
    SolidThreeLinkAxle(const rapidjson::Document& d);
    ~SolidThreeLinkAxle();

  protected:
    /// Return the center of mass of the axle tube.
    virtual const ChVector<> getAxleTubeCOM() const override { return m_axleTubeCOM; }
    /// Return the radius of the spindle body (visualization only).
    virtual double getSpindleRadius() const override { return m_spindleRadius; }

    /// Return the width of the spindle body (visualization only).
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    /// Return the mass of the axle tube body.
    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    /// Return the mass of the spindle body.
    virtual double getSpindleMass() const override { return m_spindleMass; }
    /// Return the mass of the triangle body.
    virtual double getTriangleMass() const override { return m_triangleMass; }
    /// Return the mass of the link body.
    virtual double getLinkMass() const override { return m_linkMass; }

    /// Return the radius of the axle tube body (visualization only).
    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }

    /// Return the moments of inertia of the axle tube body.
    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    /// Return the moments of inertia of the spindle body.
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    /// Return the moments of inertia of the triangle body.
    virtual const ChVector<>& getTriangleInertia() const override { return m_triangleInertia; }
    /// Return the moments of inertia of the link body.
    virtual const ChVector<>& getLinkInertia() const override { return m_linkInertia; }

    /// Return the inertia of the axle shaft.
    virtual double getAxleInertia() const override { return m_axleInertia; }

    /// Return the free (rest) length of the spring element.
    virtual double getSpringRestLength() const override { return m_springRestLength; }

    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    virtual const ChVector<> getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    ChVector<> m_points[NUM_POINTS];

    ////double m_damperDegressivityCompression;
    ////double m_damperDegressivityExpansion;

    double m_spindleMass;
    double m_axleTubeMass;
    double m_triangleMass;
    double m_linkMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_axleTubeRadius;

    double m_springRestLength;
    double m_axleInertia;

    ChVector<> m_triangleInertia;
    ChVector<> m_linkInertia;
    ChVector<> m_spindleInertia;
    ChVector<> m_axleTubeInertia;
    ChVector<> m_axleTubeCOM;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
