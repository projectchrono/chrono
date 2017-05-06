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
};

/// Vector of handles to roller subsystems.
typedef std::vector<std::shared_ptr<ChRoller> > ChRollerList;

/// @} vehicle_tracked_roller

}  // end namespace vehicle
}  // end namespace chrono

#endif
