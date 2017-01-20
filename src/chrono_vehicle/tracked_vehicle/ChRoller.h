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
// Base class for a tracked vehicle roller.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_ROLLER_H
#define CH_ROLLER_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

/**
    @addtogroup vehicle_tracked
    @{
        @defgroup vehicle_tracked_roller Roller subsystem
    @}
*/

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_roller
/// @{

/// Base class for a roller wheel subsystem.
class CH_VEHICLE_API ChRoller : public ChPart {
  public:
    ChRoller(const std::string& name  ///< [in] name of the subsystem
             );

    virtual ~ChRoller() {}

    /// Return the type of track shoe consistent with this roller wheel.
    virtual GuidePinType GetType() const = 0;

    /// Get a handle to the roller body.
    std::shared_ptr<ChBody> GetBody() const { return m_wheel; }

    /// Get a handle to the revolute joint.
    std::shared_ptr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

    /// Return the mass of the roller body.
    virtual double GetMass() const = 0;
  
    /// Return the moments of inertia of the roller body.
    virtual const ChVector<>& GetInertia() = 0;

    /// Get the radius of the road wheel.
    virtual double GetRadius() const = 0;

    /// Set coefficient of friction.
    /// The default value is 0.7
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set coefficient of restiturion.
    /// The default value is 0.1
    void SetContactRestitutionCoefficient(float restitution_coefficient) { m_restitution = restitution_coefficient; }

    /// Set contact material properties.
    /// These values are used to calculate contact material coefficients (if the containing
    /// system is so configured and if the DEM-P contact method is being used).
    /// The default values are: Y = 1e8 and nu = 0.3
    void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                      float poisson_ratio   ///< [in] Poisson ratio
                                      );

    /// Set contact material coefficients.
    /// These values are used directly to compute contact forces (if the containing system
    /// is so configured and if the DEM-P contact method is being used).
    /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
    void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                        float gn,  ///< [in] normal contact damping
                                        float kt,  ///< [in] tangential contact stiffness
                                        float gt   ///< [in] tangential contact damping
                                        );

    /// Get coefficient of friction for contact material.
    float GetCoefficientFriction() const { return m_friction; }
    /// Get coefficient of restitution for contact material.
    float GetCoefficientRestitution() const { return m_restitution; }
    /// Get Young's modulus of elasticity for contact material.
    float GetYoungModulus() const { return m_young_modulus; }
    /// Get Poisson ratio for contact material.
    float GetPoissonRatio() const { return m_poisson_ratio; }
    /// Get normal stiffness coefficient for contact material.
    float GetKn() const { return m_kn; }
    /// Get tangential stiffness coefficient for contact material.
    float GetKt() const { return m_kt; }
    /// Get normal viscous damping coefficient for contact material.
    float GetGn() const { return m_gn; }
    /// Get tangential viscous damping coefficient for contact material.
    float GetGt() const { return m_gt; }

    /// Turn on/off collision flag for the road wheel.
    void SetCollide(bool val) { m_wheel->SetCollide(val); }

    /// Initialize this roller subsystem.
    /// The roller subsystem is initialized by attaching it to the chassis body
    /// at the specified location (with respect to and expressed in the reference
    /// frame of the chassis). It is assumed that the roller subsystem reference
    /// frame is always aligned with the chassis reference frame.
    /// A derived roller subsystem template class must extend this default
    /// implementation and specify contact geometry for the roller wheel.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location              ///< [in] location relative to the chassis frame
                            );

    /// Log current constraint violations.
    void LogConstraintViolations();

  protected:
    std::shared_ptr<ChBody> m_wheel;                 ///< handle to the roller body
    std::shared_ptr<ChLinkLockRevolute> m_revolute;  ///< handle to roller revolute joint

    float m_friction;       ///< contact coefficient of friction
    float m_restitution;    ///< contact coefficient of restitution
    float m_young_modulus;  ///< contact material Young modulus
    float m_poisson_ratio;  ///< contact material Poisson ratio
    float m_kn;             ///< normal contact stiffness
    float m_gn;             ///< normal contact damping
    float m_kt;             ///< tangential contact stiffness
    float m_gt;             ///< tangential contact damping
};

/// Vector of handles to roller subsystems.
typedef std::vector<std::shared_ptr<ChRoller> > ChRollerList;

/// @} vehicle_tracked_roller

}  // end namespace vehicle
}  // end namespace chrono

#endif
