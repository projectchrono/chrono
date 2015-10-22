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
// Base class for a tracked vehicle sprocket. A sprocket is responsible for
// contact processing with the track shoes of the containing track assembly.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_SPROCKET_H
#define CH_SPROCKET_H

#include <vector>

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsBody.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChSprocket : public ChShared {
  public:
    ChSprocket(const std::string& name  ///< [in] name of the subsystem
               )
        : m_name(name) {}

    virtual ~ChSprocket() {}

    /// Get the name identifier for this sprocket subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this sprocket subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Get a handle to the road wheel body.
    ChSharedPtr<ChBody> GetWheel() const { return m_gear; }

    /// Get a handle to the axle shaft.
    ChSharedPtr<ChShaft> GetAxle() const { return m_axle; }

    /// Get a handle to the revolute joint.
    ChSharedPtr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

    /// Get the angular speed of the axle.
    double GetAxleSpeed() const { return m_axle->GetPos_dt(); }

    /// Apply the provided motor torque.
    /// The given torque is applied to the axle. This function provides the interface
    /// to the drivetrain subsystem (intermediated by the vehicle system).
    void ApplyAxleTorque(double torque  ///< [in] value of applied torque
                         );

  protected:
    std::string m_name;  ///< name of the subsystem

    ChSharedPtr<ChBody> m_gear;                   ///< handle to the sprocket gear body
    ChSharedPtr<ChShaft> m_axle;                  ///< handle to gear shafts
    ChSharedPtr<ChShaftsBody> m_axle_to_spindle;  ///< handle to gear-shaft connector
    ChSharedPtr<ChLinkLockRevolute> m_revolute;   ///< handle to sprocket revolute joint
};

/// Vector of handles to sprocket subsystems.
typedef std::vector<ChSharedPtr<ChSprocket> > ChSprocketList;

}  // end namespace vehicle
}  // end namespace chrono

#endif
