// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Template for a rigid tire
//
// =============================================================================

#ifndef CH_RIGIDTIRE_H
#define CH_RIGIDTIRE_H

#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Rigid tire model.
/// This tire is modeled as a rigid cylinder.  Requires a terrain system that
/// supports rigid contact with friction.
class CH_VEHICLE_API ChRigidTire : public ChTire {
  public:
    ChRigidTire(const std::string& name  ///< [in] name of this tire system
                );

    virtual ~ChRigidTire() {}

    /// Set contact material properties
    void SetContactMaterial(float friction_coefficient = 0.6f,    ///< [in] coefficient of friction
                            float restitution_coefficient = 0.1,  ///< [in] coefficient of restitution
                            float young_modulus = 2e5f,           ///< [in] Young's modulus of elasticity
                            float poisson_ratio = 0.3f            ///< [in] Poisson ratio
                            );

    /// Get the tire width.
    virtual double GetWidth() const = 0;

    /// Get the tire force and moment.
    /// A ChRigidTire always returns zero forces and moments if the tire is
    /// simulated together with the associated vehicle (the tire forces are
    /// automatically applied to the associated wheel through Chrono's frictional
    /// contact system). If the tire is co-simulated, the tire force and moments
    /// encapsulate the tire-terrain forces (i.e. the resultant of all contact
    /// forces acting on the tire).
    virtual TireForce GetTireForce(bool cosim = false) const override;

    /// Initialize this tire system.
    /// This function creates the tire contact shape and attaches it to the
    /// associated wheel body.
    virtual void Initialize(std::shared_ptr<ChBody> wheel,  ///< handle to the associated wheel body
                            VehicleSide side                ///< left/right vehicle side
                            ) override;

  protected:
  private:
    float m_friction;
    float m_restitution;
    float m_young_modulus;
    float m_poisson_ratio;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
