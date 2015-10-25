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

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChRoadWheel : public ChShared {
  public:
    ChRoadWheel(const std::string& name  ///< [in] name of the subsystem
                )
        : m_name(name) {}

    virtual ~ChRoadWheel() {}

    /// Get the name identifier for this road wheel subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this road wheel subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Return the type of track shoe consistent with this road wheel.
    virtual TrackShoeType GetType() const = 0;

    /// Get a handle to the road wheel body.
    ChSharedPtr<ChBody> GetWheel() const { return m_wheel; }

    /// Get a handle to the revolute joint.
    ChSharedPtr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

    /// Get the radius of the road wheel.
    virtual double GetWheelRadius() const = 0;

    /// Set contact material properties.
    /// This function must be called before Initialize().
    void SetContactMaterial(float friction_coefficient = 0.6f,    ///< [in] coefficient of friction
                            float restitution_coefficient = 0.1,  ///< [in] coefficient of restitution
                            float young_modulus = 2e5f,           ///< [in] Young's modulus of elasticity
                            float poisson_ratio = 0.3f            ///< [in] Poisson ratio
                            );

    /// Initialize this road wheel subsystem.
    /// The road wheel subsystem is initialized by attaching it to the specified
    /// carrier body at the specified location (with respect to and expressed in the
    /// reference frame of the chassis). It is assumed that the road wheel subsystem
    /// reference frame is always aligned with the chassis reference frame.
    /// A derived road wheel subsystem template class must extend this default
    /// implementation and specify contact geometry for the road wheel.
    virtual void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            ChSharedPtr<ChBody> carrier,        ///< [in] handle to the carrier body
                            const ChVector<>& location          ///< [in] location relative to the chassis frame
                            );

    /// Add visualization of the road wheel.
    /// This (optional) function should be called only after a call to Initialize().
    /// Must be implemented by derived classes (templates).
    virtual void AddWheelVisualization() {}

  protected:
    /// Return the mass of the road wheel body.
    virtual double GetWheelMass() const = 0;
    /// Return the moments of inertia of the road wheel body.
    virtual const ChVector<>& GetWheelInertia() = 0;

    std::string m_name;                          ///< name of the subsystem
    ChSharedPtr<ChBody> m_wheel;                 ///< handle to the road wheel body
    ChSharedPtr<ChLinkLockRevolute> m_revolute;  ///< handle to wheel revolute joint

    float m_friction;
    float m_restitution;
    float m_young_modulus;
    float m_poisson_ratio;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
