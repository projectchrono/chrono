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

#include <string>
#include <vector>

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

    /// Get the location of the suspension subsystem relative to the chassis reference frame.
    /// The suspension reference frame is always aligned with the chassis reference frame.
    const ChVector<>& GetSuspensionLocation() const { return m_susp_location; }

    /// Enable/disable output for this subsystem.
    void SetOutput(bool state);

    /// Initialize this axle subsystem, by initializing its components.
    /// The suspension subsystem is initialized by attaching it to the specified chassis body at the specified location
    /// (with respect to and expressed in the reference frame of the chassis). It is assumed that the suspension
    /// reference frame is always aligned with the chassis reference frame.
    /// 'tierod_body' is a handle to the body to which the suspension tierods are to be attached. For a steered
    /// suspension, this will be the steering (central) link of a suspension subsystem.  Otherwise, this is the chassis.
    /// If this suspension is steered, 'steering_index' indicates the index of the associated steering mechanism in the
    /// vehicle's list (-1 for a non-steered suspension).
    /// 'wheel_separation' is the distance between wheel centers on one side of the axle (if using double wheels). The
    /// default value of 0 indicates the common case of a single wheel per side.
    virtual void Initialize(
        std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
        const ChVector<>& susp_location,        ///< [in] suspension location relative to the chassis frame
        const ChVector<>& arb_location,         ///< [in] antirollbar location relative to chassis frame
        std::shared_ptr<ChBody> tierod_body,    ///< [in] body to which tireods are connected
        int steering_index,                     ///< [in] index of the associated steering mechanism
        double wheel_separation = 0,            ///< [in] distance between wheel centers on one side
        double left_ang_vel = 0,                ///< [in] initial angular velocity of left wheel
        double right_ang_vel = 0                ///< [in] initial angular velocity of right wheel
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

    std::shared_ptr<ChSuspension> m_suspension;
    std::shared_ptr<ChBrake> m_brake_left;
    std::shared_ptr<ChBrake> m_brake_right;
    std::shared_ptr<ChAntirollBar> m_antirollbar;
    ChWheelList m_wheels;

  protected:
    ChVector<> m_susp_location;  ///< suspension location relative to chassis
    int m_steering_index;        ///< index of associated steering mechanism

    bool m_double_wheel;        ///< true if doulbe wheels on each side
    double m_wheel_separation;  ///< distance between wheel centers (0 for single wheel)
};

/// Vector of handles to axle subsystems.
typedef std::vector<std::shared_ptr<ChAxle> > ChAxleList;

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
