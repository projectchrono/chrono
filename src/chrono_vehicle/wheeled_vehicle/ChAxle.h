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
// Authors: Radu Serban
// =============================================================================
//
// Class representing a wheeled vehicle axle. A ChAxle encapsulates a suspension
// subsystem, (optionally) an anti-roll-bar subsystem, two brakes (left/right),
// and a varying number of wheels.
//
// =============================================================================

//// RADU
//// Todo: rename old axle (as in ChSuspension::GetAxle) to something else (axleshaft?)

#ifndef CH_AXLE_H
#define CH_AXLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"
#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/ChAntirollBar.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Base class for a suspension subsystem.
class CH_VEHICLE_API ChAxle {
  public:
    ChAxle();

    ~ChAxle() {}

    /// Enable/disable output for this subsystem.
    void SetOutput(bool state);

    /// Initialize this axle subsystem, by initializing its components.
    /// The suspension subsystem is initialized by attaching it to the specified chassis and (if provided) to the
    /// specified subchassis, at the specified location (with respect to and expressed in the reference frame of the
    /// chassis). It is assumed that the suspension reference frame is always aligned with the chassis reference frame.
    /// If a steering subsystem is provided, the suspension tierods are to be attached to the steering's central link
    /// body (steered suspension); otherwise they are to be attached to the chassis (non-steered suspension).
    /// 'wheel_separation' is the distance between wheel centers on one side of the axle (if using double wheels). The
    /// default value of 0 indicates the common case of a single wheel per side.
    void Initialize(std::shared_ptr<ChChassis> chassis,        ///< associated chassis subsystem
                    std::shared_ptr<ChSubchassis> subchassis,  ///< associated subchassis subsystem (may be null)
                    std::shared_ptr<ChSteering> steering,      ///< associated steering subsystem (may be null)
                    const ChVector<>& susp_location,           ///< suspension location relative to the chassis frame
                    const ChVector<>& arb_location,            ///< antirollbar location relative to chassis frame
                    double wheel_separation = 0,               ///< distance between wheel centers on one side
                    double left_ang_vel = 0,                   ///< initial angular velocity of left wheel
                    double right_ang_vel = 0                   ///< initial angular velocity of right wheel
    );

    /// Synchronize this suspension subsystem.
    void Synchronize(double braking);

    /// Get all wheels from this axle.
    /// The wheels associated with an axle are assumed to be ordered from inner to outer wheels, first left then right.
    /// In other words, for a single-wheel axle the order is left wheel, right wheel.  For a double-wheel axle, the
    /// order is inner left, inner right, outer left, outer right.
    const ChWheelList& GetWheels() const { return m_wheels; }

    /// Get the specified wheel of this axle.
    std::shared_ptr<ChWheel> GetWheel(VehicleSide side, WheelLocation location = SINGLE) const;

    ///  Get the sdpecified brake of this axle.
    std::shared_ptr<ChBrake> GetBrake(VehicleSide side) const;

    std::shared_ptr<ChSuspension> m_suspension;
    std::shared_ptr<ChBrake> m_brake_left;
    std::shared_ptr<ChBrake> m_brake_right;
    std::shared_ptr<ChAntirollBar> m_antirollbar;
    ChWheelList m_wheels;
};

/// Vector of handles to axle subsystems.
typedef std::vector<std::shared_ptr<ChAxle> > ChAxleList;

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
