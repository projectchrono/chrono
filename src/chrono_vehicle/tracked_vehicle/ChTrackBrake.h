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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Base class for a sprocket brake.
//
// =============================================================================

#ifndef CH_TRACK_BRAKE_H
#define CH_TRACK_BRAKE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/tracked_vehicle/ChSprocket.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_brake
/// @{

/// Base class for a tracked vehicle brake subsystem.
class CH_VEHICLE_API ChTrackBrake : public ChPart {
  public:
    virtual ~ChTrackBrake() {}

    /// Initialize the brake by providing the sprocket's revolute link.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,   ///< associated chassis subsystem
                            std::shared_ptr<ChSprocket> sprocket  ///< associated sprocket subsystem
                            ) = 0;

    /// Update the brake subsystem for the given braking driver input.
    /// <pre>
    ///   braking = 0 : completely free,
    ///   braking = 1 : provide maximum braking torque
    /// </pre>
    virtual void Synchronize(double braking) = 0;

    /// Get the current brake torque.
    virtual double GetBrakeTorque() = 0;

  protected:
    ChTrackBrake(const std::string& name);

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;
};

/// Vector of handles to brake subsystems.
typedef std::vector<std::shared_ptr<ChTrackBrake> > ChTrackBrakeList;

/// @} vehicle_tracked_brake

}  // end namespace vehicle
}  // end namespace chrono

#endif
