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

#include "physics/ChBody.h"

#include "chrono_vehicle/ChTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {

///
/// Rigid tire model.
/// This tire is modeled as a rigid cylinder.  Requires a terrain system that
/// supports rigid contact with friction.
///
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

    /// Get the tire force and moment.
    /// For a rigid tire, the tire forces are automatically applied to the
    /// associated wheel (through Chrono's frictional contact system). The values
    /// returned here are never used.
    virtual ChTireForce GetTireForce() const override;

    /// Initialize this tire system.
    /// This function creates the tire contact shape and attaches it to the
    /// associated wheel body.
    void Initialize(ChSharedPtr<ChBody> wheel  ///< handle to the associated wheel body
                    );

  protected:
    /// Return the tire radius.
    virtual double getRadius() const = 0;

    /// Return the tire width.
    virtual double getWidth() const = 0;

  private:
    float m_friction;
    float m_restitution;
    float m_young_modulus;
    float m_poisson_ratio;
};

}  // end namespace chrono

#endif
