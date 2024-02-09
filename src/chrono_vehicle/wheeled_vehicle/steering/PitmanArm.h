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
// Pitman arm steering model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef PITMAN_ARM_H
#define PITMAN_ARM_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArm.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_steering
/// @{

/// Pitman arm steering model constructed with data from file (JSON format).
class CH_VEHICLE_API PitmanArm : public ChPitmanArm {
  public:
    PitmanArm(const std::string& filename);
    PitmanArm(const rapidjson::Document& d);
    ~PitmanArm() {}

    virtual double getSteeringLinkMass() const override { return m_steeringLinkMass; }
    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getSteeringLinkRadius() const override { return m_steeringLinkRadius; }
    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const ChVector3d& getSteeringLinkInertiaMoments() const override { return m_steeringLinkInertiaMoments; }
    virtual const ChVector3d& getSteeringLinkInertiaProducts() const override { return m_steeringLinkInertiaProducts; }
    virtual const ChVector3d& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const ChVector3d& getPitmanArmInertiaProducts() const override { return m_pitmanArmInertiaProducts; }

    virtual double getMaxAngle() const override { return m_maxAngle; }

    virtual const ChVector3d getLocation(PointId which) override { return m_points[which]; }
    virtual const ChVector3d getDirection(DirectionId which) override { return m_dirs[which]; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    ChVector3d m_points[NUM_POINTS];
    ChVector3d m_dirs[NUM_DIRS];

    double m_steeringLinkMass;
    double m_pitmanArmMass;

    double m_steeringLinkRadius;
    double m_pitmanArmRadius;

    double m_maxAngle;

    ChVector3d m_steeringLinkInertiaMoments;
    ChVector3d m_steeringLinkInertiaProducts;
    ChVector3d m_pitmanArmInertiaMoments;
    ChVector3d m_pitmanArmInertiaProducts;
};

/// @} vehicle_wheeled_steering

}  // end namespace vehicle
}  // end namespace chrono

#endif
