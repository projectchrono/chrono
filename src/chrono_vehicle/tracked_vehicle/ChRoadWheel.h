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
// Base class for a road wheel.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_ROAD_WHEEL_H
#define CH_ROAD_WHEEL_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Base class for a road wheel subsystem.
class CH_VEHICLE_API ChRoadWheel {
  public:
    ChRoadWheel(const std::string& name  ///< [in] name of the subsystem
                );

    virtual ~ChRoadWheel() {}

    /// Get the name identifier for this road wheel subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this road wheel subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Return the type of track shoe consistent with this road wheel.
    virtual GuidePinType GetType() const = 0;

    /// Get a handle to the road wheel body.
    std::shared_ptr<ChBody> GetWheelBody() const { return m_wheel; }

    /// Get a handle to the revolute joint.
    std::shared_ptr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

    /// Return the mass of the road wheel body.
    virtual double GetWheelMass() const = 0;
  
    /// Return the moments of inertia of the road wheel body.
    virtual const ChVector<>& GetWheelInertia() = 0;

    /// Get the radius of the road wheel.
    virtual double GetWheelRadius() const = 0;

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

    /// Initialize this road wheel subsystem.
    /// The road wheel subsystem is initialized by attaching it to the specified
    /// carrier body at the specified location (with respect to and expressed in the
    /// reference frame of the chassis). It is assumed that the road wheel subsystem
    /// reference frame is always aligned with the chassis reference frame.
    /// A derived road wheel subsystem template class must extend this default
    /// implementation and specify contact geometry for the road wheel.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            std::shared_ptr<ChBody> carrier,        ///< [in] handle to the carrier body
                            const ChVector<>& location              ///< [in] location relative to the chassis frame
                            );

    /// Log current constraint violations.
    void LogConstraintViolations();

  protected:
    std::string m_name;                              ///< name of the subsystem
    std::shared_ptr<ChBody> m_wheel;                 ///< handle to the road wheel body
    std::shared_ptr<ChLinkLockRevolute> m_revolute;  ///< handle to wheel revolute joint

    float m_friction;       ///< contact coefficient of friction
    float m_restitution;    ///< contact coefficient of restitution
    float m_young_modulus;  ///< contact material Young modulus
    float m_poisson_ratio;  ///< contact material Poisson ratio
    float m_kn;             ///< normal contact stiffness
    float m_gn;             ///< normal contact damping
    float m_kt;             ///< tangential contact stiffness
    float m_gt;             ///< tangential contact damping
};

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
