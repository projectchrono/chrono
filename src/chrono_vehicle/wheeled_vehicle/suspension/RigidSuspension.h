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
// Rigid suspension constructed with data from file.
//
// =============================================================================

#ifndef RIGID_SUSPENSION_H
#define RIGID_SUSPENSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidSuspension.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Rigid suspension constructed with data from file.
class CH_VEHICLE_API RigidSuspension : public ChRigidSuspension {
  public:
    RigidSuspension(const std::string& filename);
    RigidSuspension(const rapidjson::Document& d);
    ~RigidSuspension() {}

    virtual double getSpindleMass() const override { return m_spindleMass; }

    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

  private:
    virtual const ChVector3d getLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChVector3d m_points[NUM_POINTS];

    double m_spindleMass;

    double m_spindleRadius;
    double m_spindleWidth;

    ChVector3d m_spindleInertia;

    double m_axleInertia;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
