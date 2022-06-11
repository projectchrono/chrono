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
// Authors: Radu Serban, Justin Madsen, Conlain Kelly
// =============================================================================
//
// Irrlicht-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use that as an input; it will
// otherwise default to a keyboard input.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs.
//
// =============================================================================

#ifndef CH_IRRGUIDRIVER_H
#define CH_IRRGUIDRIVER_H

#include <string>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChDriver.h"

#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/utils/ChVehicleVisualSystemIrrlicht.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Interactive driver model using keyboard inputs.
/// Irrlicht-based GUI driver for the a vehicle. This class implements the
/// functionality required by its base ChDriver class using keyboard inputs.
/// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
/// and update the current driver inputs.
///
/// @sa ChDataDriver
class CH_VEHICLE_API ChIrrGuiDriver : public ChDriver, public irr::IEventReceiver {
  public:
    /// Functioning modes for a ChIrrGuiDriver
    enum InputMode {
        LOCK,      ///< driver inputs locked at current values
        KEYBOARD,  ///< driver inputs from keyboard
        DATAFILE,  ///< driver inputs from data file
        JOYSTICK   ///< driver inputs from joystick
    };

    /// Exposes the unnamed enum of irrlicht axes to enforce right values in API usage
    enum JoystickAxes {
        AXIS_X = irr::SEvent::SJoystickEvent::AXIS_X,
        AXIS_Y = irr::SEvent::SJoystickEvent::AXIS_Y,		
        AXIS_Z = irr::SEvent::SJoystickEvent::AXIS_Z,		
        AXIS_R = irr::SEvent::SJoystickEvent::AXIS_R,		
        AXIS_U = irr::SEvent::SJoystickEvent::AXIS_U,
        AXIS_V = irr::SEvent::SJoystickEvent::AXIS_V,
        NONE
    };

    /// Construct an Irrlicht GUI driver.
    ChIrrGuiDriver(ChVehicleVisualSystemIrrlicht& vsys);

    virtual ~ChIrrGuiDriver() {}

    /// Intercept and process keyboard inputs.
    virtual bool OnEvent(const irr::SEvent& event) override;

    /// Update the state of this driver system at the specified time.
    virtual void Synchronize(double time) override;

    /// Advance the state of this driver system by the specified time step.
    virtual void Advance(double step) override;

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

    /// Set joystick axes: throttle, brake, steering, clutch. 
    void SetJoystickAxes(JoystickAxes tr_ax, JoystickAxes br_ax, JoystickAxes st_ax, JoystickAxes cl_ax);

    /// Get joystick axes for the throttle.
    JoystickAxes GetThrottleAxis(JoystickAxes tr_ax) const { return throttle_axis; }

    /// Get joystick axes for the brake.
    JoystickAxes GetBrakeAxis(JoystickAxes br_ax) const { return brake_axis; }

    /// Get joystick axes for the steer.
    JoystickAxes GetSteerAxis(JoystickAxes st_ax) const { return steer_axis; }

    /// Get joystick axes for the clutch.
    JoystickAxes GetClutchAxis(JoystickAxes cl_ax) const { return cl_ax; }

    /// Feed button number and callback function to implement a custom callback.
    void SetButtonCallback(int button, void(*cbfun)()) {cb_fun = cbfun; callbackButton=button; }

  protected:
    ChVehicleVisualSystemIrrlicht& m_vsys;

    InputMode m_mode; ///< current mode of the driver

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
    // Axes for joystick usage
    JoystickAxes throttle_axis = AXIS_Z;
    JoystickAxes brake_axis = AXIS_R;
    JoystickAxes steer_axis = AXIS_X;
    JoystickAxes clutch_axis = AXIS_Y;
    // Joystick button associated to the custom callback
    int callbackButton = -1;
    // Custom callback, can be implemented in the application
    void (*cb_fun)() = nullptr;

    // Variables for mode=DATAFILE
    double m_time_shift;                          ///< time at which mode was switched to DATAFILE
    std::shared_ptr<ChDataDriver> m_data_driver;  ///< embedded data driver (for playback)
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
