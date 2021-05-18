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
// Vehicle driver based on controls from some Autonomy Stack / AI.
// Currently, it is assumed that the AI provides a desired longitudinal
// acceleration and wheel angles for the front and rear axles. The underlying
// assumption is of Ackermann steering of a bicycle model.
//
// =============================================================================

#ifndef CH_AI_DRIVER_H
#define CH_AI_DRIVER_H

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/utils/ChSpeedController.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Driver
class CH_VEHICLE_API ChAIDriver : public ChDriver {
  public:
    ChAIDriver(ChVehicle& vehicle);
    virtual ~ChAIDriver() {}

    /// Set speed controller PID gains.
    void SetSpeedControllerGains(double Kp, double Ki, double Kd);

    /// Specify the throttle value below which braking is enabled.
    /// If the vehicle is moving faster than the set speed, the controller attempts to
    /// reduce speed either by reducing the throttle input (if the current throttle input
    /// is above the threshold value) or by applying brakes (otherwise).
    void SetThresholdThrottle(double val) { m_throttle_threshold = val; }

    /// Get the underlying speed controller object.
    ChSpeedController& GetSpeedController() { return m_speedPID; }

    /// Return the value of the vehicle steering input (in [-1,+1]) given the desired front and rear wheel angles.
    /// The underlying assumption is of Ackermann steering of a bicycle model.
    /// Note that the Chrono::Vehicle ISO reference frame convention implies that a positive front angle corresponds
    /// to a turn to the left (i.e., positive value of vehicle steering input).
    virtual double CalculateSteering(double front_axle_angle, double rear_axle_angle) = 0;

    /// Set the controls from the autonomy stack: longitudinal acceleration (m/s2) and the front & rear wheel angles (in
    /// radians).  The underlying assumption is of Ackermann steering of a bicycle model.
    void Synchronize(double time, double long_acc, double front_axle_angle, double rear_axle_angle);

    /// Advance the state of this driver system by the specified duration.
    virtual void Advance(double step) override;

  protected:
    ChSpeedController m_speedPID;  ///< speed controller
    double m_throttle_threshold;   ///< throttle value below which brakes are applied

    double m_target_speed;  ///< current value of target speed
    double m_last_time;     ///< last time
    double m_last_speed;    ///< last target speed

  private:
    virtual void Synchronize(double time) override final;
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
