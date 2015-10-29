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
// Base class for a tracked vehicle driveline.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_DRIVELINE_H
#define CH_TRACK_DRIVELINE_H

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChTrackDriveline : public ChShared {
  public:
    ChTrackDriveline(const std::string& name  ///< [in] name of the subsystem
                     )
        : m_name(name) {}

    virtual ~ChTrackDriveline() {}

    /// Get the name identifier for this driveline subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this driveline subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Get a handle to the driveshaft.
    /// Return a shared pointer to the shaft that connects this driveline to a
    /// powertrain system (i.e., right after the transmission box).
    ChSharedPtr<ChShaft> GetDriveshaft() const { return m_driveshaft; }

    /// Get the angular speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to
    /// the powertrain system. The default implementation returns the driveline's
    /// driveshaft speed.
    virtual double GetDriveshaftSpeed() const { return m_driveshaft->GetPos_dt(); }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the sprockets of the
    /// two track assembly subsystems.
    virtual void Initialize(ChSharedPtr<ChBody> chassis,              ///< handle to the chassis body
                            ChSharedPtr<ChTrackAssembly> track_left,  ///< handle to the left track assembly
                            ChSharedPtr<ChTrackAssembly> track_right  ///< handle to the right track assembly
                            ) = 0;

    /// Update the driveline subsystem.
    /// The motor torque represents the input to the driveline subsystem from the
    /// powertrain system. The default implementation applies this torque to the
    /// driveline's driveshaft.
    /// A derived class must also process the steering input to send appropriate
    /// torques to the sprockets of the two track assemblies.
    virtual void Update(double steering, double torque) { m_driveshaft->SetAppliedTorque(torque); }

  protected:
    std::string m_name;  ///< name of the subsystem

    ChSharedPtr<ChShaft> m_driveshaft;  ///< handle to the shaft connection to the powertrain
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
