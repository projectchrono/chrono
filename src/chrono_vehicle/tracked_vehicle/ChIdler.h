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
// Base class for an idler subsystem.  An idler consists of the idler wheel and
// a connecting body.  The idler wheel is connected through a revolute joint to
// the connecting body which in turn is connected to the chassis through a
// translational joint.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_IDLER_H
#define CH_IDLER_H

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChIdler : public ChShared {
  public:
    ChIdler(const std::string& name  ///< [in] name of the subsystem
            )
        : m_name(name) {}

    virtual ~ChIdler() {}

    /// Get the name identifier for this idler subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this idler subsystem.
    void SetName(const std::string& name) { m_name = name; }
    /// Return the type of track shoe consistent with this idler.
    TrackShoeType GetType() const { return m_type; }

    /// Get a handle to the road wheel body.
    ChSharedPtr<ChBody> GetWheel() const { return m_wheel; }

    /// Get a handle to the revolute joint.
    ChSharedPtr<ChLinkLockRevolute> GetRevolute() const { return m_revolute; }

  protected:
    std::string m_name;  ///< name of the subsystem

    TrackShoeType m_type;                        ///< type of the track shoe matching this idler
    ChSharedPtr<ChBody> m_wheel;                 ///< handle to the idler wheel body
    ChSharedPtr<ChLinkLockRevolute> m_revolute;  ///< handle to idler wheel revolute joint
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
