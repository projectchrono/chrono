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
// Torsion-bar suspension system using rotational damper constructed with data
// from file (JSON format)
//
// =============================================================================

#ifndef ROTATIONAL_DAMPER_RWA_H
#define ROTATIONAL_DAMPER_RWA_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/suspension/ChRotationalDamperRWAssembly.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Torsion-bar suspension system using linear dampers constructed with data from
/// file (JSON format)
class CH_VEHICLE_API RotationalDamperRWAssembly : public ChRotationalDamperRWAssembly {
  public:
    RotationalDamperRWAssembly(const std::string& filename, bool has_shock);
    RotationalDamperRWAssembly(const rapidjson::Document& d, bool has_shock);
    ~RotationalDamperRWAssembly();

    /// Return the mass of the arm body.
    virtual double GetArmMass() const override { return m_arm_mass; }
    /// Return the moments of inertia of the arm body.
    virtual const ChVector<>& GetArmInertia() const override { return m_arm_inertia; }
    /// Return a visualization radius for the arm body.
    virtual double GetArmVisRadius() const override { return m_arm_radius; }

    /// Return the functor object for the torsional spring torque.
    virtual ChLinkRotSpringCB::TorqueFunctor* GetSpringTorqueFunctor() const override { return m_spring_torqueCB; }

    /// Return the functor object for the torsional shock force.
    virtual ChLinkRotSpringCB::TorqueFunctor* GetShockTorqueCallback() const override { return m_shock_torqueCB; }

  private:
    virtual const ChVector<> GetLocation(PointId which) override { return m_points[which]; }

    virtual void Create(const rapidjson::Document& d) override;

    ChLinkRotSpringCB::TorqueFunctor* m_spring_torqueCB;
    ChLinkRotSpringCB::TorqueFunctor* m_shock_torqueCB;

    ChVector<> m_points[NUM_POINTS];

    double m_arm_mass;
    ChVector<> m_arm_inertia;
    double m_arm_radius;
};

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
