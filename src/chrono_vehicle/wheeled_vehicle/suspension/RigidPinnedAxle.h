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
// Rigid suspension with pinned axle constructed with data from file.
//
// =============================================================================

#ifndef RIGID_PINNED_AXLE_H
#define RIGID_PINNED_AXLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidPinnedAxle.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Rigid suspension with pinned axle constructed with data from file.
class CH_VEHICLE_API RigidPinnedAxle : public ChRigidPinnedAxle {
  public:
    RigidPinnedAxle(const std::string& filename);
    RigidPinnedAxle(const rapidjson::Document& d);
    ~RigidPinnedAxle() {}

    virtual const ChVector3d getAxleTubeCOM() const override { return m_axleTubeCOM; }
    virtual const ChVector3d getAxlePinLocation() const override { return m_axlePinLoc; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getAxleTubeInertia() const override { return m_axleTubeInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

  private:
    virtual const ChVector3d getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChVector3d m_points[NUM_POINTS];

    double m_spindleMass;
    double m_axleTubeMass;

    double m_spindleRadius;
    double m_spindleWidth;
    double m_axleTubeRadius;

    ChVector3d m_spindleInertia;
    ChVector3d m_axleTubeInertia;

    ChVector3d m_axleTubeCOM;
    ChVector3d m_axlePinLoc;

    double m_axleInertia;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
