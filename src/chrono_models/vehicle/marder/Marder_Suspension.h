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
// Authors: Rainer Gericke
// =============================================================================
//
// Marder suspension subsystem.
//
// =============================================================================

#ifndef MARDER_SUSPENSION_H
#define MARDER_SUSPENSION_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/suspension/ChTranslationalDamperSuspension.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace marder {

/// @addtogroup vehicle_models_marder
/// @{

/// Linear-damper Marder track suspension.
class CH_MODELS_API Marder_Suspension : public ChTranslationalDamperSuspension {
  public:
    Marder_Suspension(const std::string& name, VehicleSide side, int index, bool has_shock);
    ~Marder_Suspension();

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const ChVector<> GetLocation(PointId which) override;

    /// Return the mass of the arm body.
    virtual double GetArmMass() const override { return m_arm_mass; }
    /// Return the moments of inertia of the arm body.
    virtual const ChVector<>& GetArmInertia() const override { return m_arm_inertia; }
    /// Return a visualization radius for the arm body.
    virtual double GetArmVisRadius() const override { return m_arm_radius; }

    /// Return the free (rest) angle of the spring element.
    virtual double GetSpringRestAngle() const override { return 0; }
    /// Return the functor object for the torsional spring torque.
    virtual std::shared_ptr<ChLinkRSDA::TorqueFunctor> GetSpringTorqueFunctor() const override {
        return m_spring_torqueCB;
    }
    /// Return the functor object for the translational shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> GetShockForceFunctor() const override { return m_shock_forceCB; }

  private:
    VehicleSide m_side;

    std::shared_ptr<ChLinkRSDA::TorqueFunctor> m_spring_torqueCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shock_forceCB;

    static const double m_arm_mass;
    static const ChVector<> m_arm_inertia;
    static const double m_arm_radius;

    static const double m_torsion_a0;
    static const double m_torsion_k;
    static const double m_torsion_c;
    static const double m_torsion_t;

    static const double m_shock_c;
};

/// @} vehicle_models_marder

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono

#endif
