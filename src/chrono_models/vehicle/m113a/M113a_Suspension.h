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
// M113 suspension subsystem.
//
// =============================================================================

#ifndef M113a_SUSPENSION_H
#define M113a_SUSPENSION_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/suspension/ChLinearDamperRWAssembly.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

class CH_MODELS_API M113a_Suspension : public ChLinearDamperRWAssembly {
  public:
    M113a_Suspension(VehicleSide side, bool has_shock);
    ~M113a_Suspension();

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) override;

    /// Return the mass of the arm body.
    virtual double GetArmMass() const override { return m_arm_mass; }
    /// Return the moments of inertia of the arm body.
    virtual const ChVector<>& GetArmInertia() const override { return m_arm_inertia; }
    /// Return a visualization radius for the arm body.
    virtual double GetArmVisRadius() const override { return m_arm_radius; }

    /// Return the functor object for the torsional spring torque.
    virtual ChLinkRotSpringCB::TorqueFunctor* GetSpringTorqueFunctor() const override { return m_spring_torqueCB; }

    /// Return the functor object for the translational shock force.
    virtual ChLinkSpringCB::ForceFunctor* GetShockForceFunctor() const override { return m_shock_forceCB; }

  private:
    VehicleSide m_side;

    ChLinkRotSpringCB::TorqueFunctor* m_spring_torqueCB;
    ChLinkSpringCB::ForceFunctor* m_shock_forceCB;

    static const double m_arm_mass;
    static const ChVector<> m_arm_inertia;
    static const double m_arm_radius;

    static const double m_torsion_a0;
    static const double m_torsion_k;
    static const double m_torsion_c;
    static const double m_torsion_t;
    static const double m_torsion_kstop;
    static const double m_torsion_lowerstop;
    static const double m_torsion_upperstop;

    static const double m_shock_c;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
