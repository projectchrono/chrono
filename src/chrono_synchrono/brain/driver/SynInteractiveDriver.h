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
// Authors: Radu Serban, Justin Madsen, Conlain Kelly, Aaron Young
// =============================================================================
//
// Interactive driver for a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use that as an input; it will
// otherwise default to a keyboard input.
//
// =============================================================================

#ifndef SYN_INTERACTIVE_DRIVER_H
#define SYN_INTERACTIVE_DRIVER_H

#include <string>

#include "chrono_synchrono/SynApi.h"

#include "chrono_vehicle/ChDriver.h"

#include "chrono_vehicle/driver/ChDataDriver.h"

#include "chrono_thirdparty/joystick/joystick.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

/// Interactive driver model using keyboard inputs.
/// Interactive driver for the a vehicle. This class implements the
/// functionality required by its base ChDriver class using keyboard inputs.
///
/// @sa ChDataDriver
class SYN_API SynInteractiveDriver : public ChDriver {
  public:
    /// Functioning modes for a SynInteractiveDriver
    enum InputMode {
        LOCK,      ///< driver inputs locked at current values
        KEYBOARD,  ///< driver inputs from keyboard
        DATAFILE,  ///< driver inputs from data file
        JOYSTICK   ///< driver inputs from joystick
    };

    /// Construct an interactive driver.
    SynInteractiveDriver(ChVehicle& vehicle  ///< associated vehicle
    );

    virtual ~SynInteractiveDriver() {}

    /// Update the state of this driver system at the specified time.
    virtual void Synchronize(double time) override;

    /// Advance the state of this driver system by the specified time step.
    virtual void Advance(double step) override;

    /// Intercept and process joystick or keyboard inputs.
    bool Sample();

    /// Set the increment in throttle input for each recorded keypress (default 1/50).
    void SetThrottleDelta(double delta);

    /// Set the increment in steering input for each recorded keypress (default 1/50).
    void SetSteeringDelta(double delta);

    /// Set the increment in braking input for each recorded keypress (default 1/50).
    void SetBrakingDelta(double delta);

    /// Set the step size for integration of the internal driver dynamics.
    void SetStepsize(double val) { m_stepsize = val; }

    /// Set gains for internal dynamics.
    /// Default values are 4.0.
    void SetGains(double steering_gain, double throttle_gain, double braking_gain);

    /// Set the input file for the underlying data driver.
    void SetInputDataFile(const std::string& filename);

    /// Set the current functioning mode.
    void SetInputMode(InputMode mode);

    /// Return the current functioning mode as a string.
    std::string GetInputModeAsString() const;

  protected:
    Joystick m_joystick;

    InputMode m_mode;  ///< current mode of the driver

    // Variables for mode=KEYBOARD
    double m_steering_target;  ///< current target value for steering input
    double m_throttle_target;  ///< current target value for throttle input
    double m_braking_target;   ///< current target value for braking input

    double m_stepsize;  ///< time step for internal dynamics

    double m_steering_delta;  ///< steering increment on each keypress
    double m_throttle_delta;  ///< throttle increment on each keypress
    double m_braking_delta;   ///< braking increment on each keypress

    double m_steering_gain;  ///< gain for steering internal dynamics
    double m_throttle_gain;  ///< gain for throttle internal dynamics
    double m_braking_gain;   ///< gain for braking internal dynamics

    // Variables for mode=JOYSTICK
    int m_dT;

    // Variables for mode=DATAFILE
    double m_time_shift;                          ///< time at which mode was switched to DATAFILE
    std::shared_ptr<ChDataDriver> m_data_driver;  ///< embedded data driver (for playback)
};

}  // namespace synchrono
}  // namespace chrono

#endif
