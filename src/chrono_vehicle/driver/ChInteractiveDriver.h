// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Base class and utilities for interactive vehicle drivers. This class
// implements the common functionality for a driver that accepts user inputs
// from keyboard or a joystick.
//
// =============================================================================

#ifndef CH_INTERACTIVE_DRIVER_H
#define CH_INTERACTIVE_DRIVER_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_vehicle/driver/ChDataDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Interactive driver for the a vehicle.
/// This class implements support for keyboard or joystick control of a vehicle, but is independent of a particular
/// implementation of a keyboard/joystick event handler.
class CH_VEHICLE_API ChInteractiveDriver : public ChDriver {
  public:
    /// Functioning modes for a ChInteractiveDriver.
    enum class InputMode {
        LOCK,      ///< driver inputs locked at current values
        KEYBOARD,  ///< driver inputs from keyboard
        JOYSTICK   ///< driver inputs from joystick
    };

    /// Semantics of the keyboard driving controls (used only in InputMode::KEYBOARD).
    enum class KeyboardMode {
        CUMULATIVE,  ///< each keypress nudges the input by a fixed increment; historical Chrono::Vehicle behavior
        HELD         ///< inputs follow the keys currently held down, as in a driving game (see SetKeyState)
    };

    /// Driving controls tracked in KeyboardMode::HELD.
    enum class InputKey {
        THROTTLE,     ///< accelerator pedal (default key 'W')
        BRAKE,        ///< brake pedal (default key 'S')
        STEER_LEFT,   ///< steer left (default key 'A')
        STEER_RIGHT,  ///< steer right (default key 'D')
        CLUTCH        ///< clutch pedal (default key 'E'; manual transmission only)
    };

    /// Construct an interactive driver.
    ChInteractiveDriver(ChVehicle& vehicle);

    virtual ~ChInteractiveDriver() {}

    /// Check if joystick is supported.
    virtual bool HasJoystick() const { return false; }

    /// Set the current functioning mode.
    void SetInputMode(InputMode mode);

    InputMode GetInputMode() const { return m_mode; }

    /// Select the semantics of the keyboard driving controls.
    /// Takes precedence over any default proposed by an input backend (see SetDefaultKeyboardMode).
    void SetKeyboardMode(KeyboardMode mode);

    KeyboardMode GetKeyboardMode() const { return m_keyboard_mode; }

    /// Propose a keyboard mode, to be called by an input backend that knows which modes it can support.
    /// Has no effect if the application already made an explicit call to SetKeyboardMode.
    void SetDefaultKeyboardMode(KeyboardMode mode);

    /// Report that a driving control key was pressed (true) or released (false).
    /// Used only in KeyboardMode::HELD. Calls are idempotent, so key auto-repeat is harmless.
    void SetKeyState(InputKey key, bool pressed);

    /// Report that all driving control keys are released.
    /// Call this when the input backend can no longer observe the keyboard (e.g., on loss of window focus), so
    /// that inputs do not stay pinned at their last value.
    void ReleaseAllKeys();

    /// Set the increment in throttle input for each recorded keypress (default 1/50).
    /// Used only in KeyboardMode::CUMULATIVE.
    void SetThrottleDelta(double delta) { m_throttle_delta = delta; }

    /// Set the increment in steering input for each recorded keypress (default 1/50).
    /// Used only in KeyboardMode::CUMULATIVE.
    void SetSteeringDelta(double delta) { m_steering_delta = delta; }

    /// Set the increment in braking input for each recorded keypress (default 1/50).
    /// Used only in KeyboardMode::CUMULATIVE.
    void SetBrakingDelta(double delta) { m_braking_delta = delta; }

    /// Set the increment in clutch input for each recorded keypress (default 1/50).
    /// Used only in KeyboardMode::CUMULATIVE.
    void SetClutchDelta(double delta) { m_clutch_delta = delta; }

    /// Set the step size for integration of the internal driver dynamics.
    void SetStepsize(double val) { m_stepsize = val; }

    /// Set gains for internal dynamics (default values are 4.0).
    /// Each input approaches its target exponentially with time constant 1/gain; in KeyboardMode::HELD this also
    /// sets how fast a held key ramps an input up or down.
    void SetGains(double steering_gain = 1, double throttle_gain = 1, double braking_gain = 1, double clutch_gain = 1);

    /// Increase Throttle (KeyboardMode::CUMULATIVE)
    void IncreaseThrottle();

    /// Decrease Throttle (KeyboardMode::CUMULATIVE)
    void DecreaseThrottle();

    /// Steering Left (KeyboardMode::CUMULATIVE)
    void SteeringLeft();

    /// Steering Right (KeyboardMode::CUMULATIVE)
    void SteeringRight();

    /// Increase Clutch (KeyboardMode::CUMULATIVE)
    void IncreaseClutch();

    /// Decrease Clutch (KeyboardMode::CUMULATIVE)
    void DecreaseClutch();

    /// Center Steering
    void SteeringCenter();

    /// Release Pedals
    void ReleasePedals();

    /// Update the state of this driver system at the specified time.
    virtual void Synchronize(double time) override {}

    /// Advance the state of this driver system by the specified time step.
    virtual void Advance(double step) override;

  protected:
    /// Recompute the input targets from the set of keys currently held down (KeyboardMode::HELD).
    void UpdateTargetsFromHeldKeys();

    InputMode m_mode;  ///< current mode of the driver

    // Variables for mode=KEYBOARD
    KeyboardMode m_keyboard_mode;   ///< semantics of the keyboard driving controls
    bool m_keyboard_mode_user_set;  ///< application explicitly selected a keyboard mode
    bool m_key_throttle;            ///< throttle key currently held down
    bool m_key_brake;               ///< brake key currently held down
    bool m_key_steer_left;          ///< steer-left key currently held down
    bool m_key_steer_right;         ///< steer-right key currently held down
    bool m_key_clutch;              ///< clutch key currently held down

    double m_steering_target;  ///< current target value for steering input
    double m_throttle_target;  ///< current target value for throttle input
    double m_braking_target;   ///< current target value for braking input
    double m_clutch_target;    ///< current target value for clutch input

    double m_stepsize;  ///< time step for internal dynamics

    double m_steering_delta;  ///< steering increment on each keypress
    double m_throttle_delta;  ///< throttle increment on each keypress
    double m_braking_delta;   ///< braking increment on each keypress
    double m_clutch_delta;    ///< clutch increment on each keypress

    double m_steering_gain;  ///< gain for steering internal dynamics
    double m_throttle_gain;  ///< gain for throttle internal dynamics
    double m_braking_gain;   ///< gain for braking internal dynamics
    double m_clutch_gain;    ///< gain for clutch internal dynamics
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
