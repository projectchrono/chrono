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
// RotaryArm steering model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef ROTARY_ARM_H
#define ROTARY_ARM_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_steering
/// @{

/// RotaryArm-pinion steering model constructed with data from file (JSON format).
class CH_VEHICLE_API RotaryArm : public ChRotaryArm {
  public:
    RotaryArm(const std::string& filename);
    RotaryArm(const rapidjson::Document& d);
    ~RotaryArm() {}

    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const ChVector3d& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const ChVector3d& getPitmanArmInertiaProducts() const override { return m_pitmanArmInertiaProducts; }

    virtual const ChVector3d getLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector3d getDirection(DirectionId which) override { return m_dirs[which]; }

    virtual double getMaxAngle() const override { return m_maxAngle; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_pitmanArmMass;

    double m_pitmanArmRadius;

    double m_maxAngle;

    ChVector3d m_pitmanArmInertiaMoments;
    ChVector3d m_pitmanArmInertiaProducts;

    ChVector3d m_points[NUM_POINTS];
    ChVector3d m_dirs[NUM_DIRS];
};

/// @} vehicle_wheeled_steering

}  // end namespace vehicle
}  // end namespace chrono

#endif
