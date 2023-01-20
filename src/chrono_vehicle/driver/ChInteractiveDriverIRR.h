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
// Authors: Radu Serban, Justin Madsen, Conlain Kelly, Marcel Offermans
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

#ifndef CH_INTERACTIVE_DRIVER_IRR_H
#define CH_INTERACTIVE_DRIVER_IRR_H

#include <string>

#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/ChVehicleVisualSystemIrrlicht.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Irrlicht interface to a specific joystick axis and its calibration data.
struct CH_VEHICLE_API ChJoystickAxisIRR {
    ChJoystickAxisIRR();

    /// Exposes the unnamed enum of irrlicht axes to enforce right values in API usage.
    enum Axis {
        AXIS_X = irr::SEvent::SJoystickEvent::AXIS_X,
        AXIS_Y = irr::SEvent::SJoystickEvent::AXIS_Y,
        AXIS_Z = irr::SEvent::SJoystickEvent::AXIS_Z,
        AXIS_R = irr::SEvent::SJoystickEvent::AXIS_R,
        AXIS_U = irr::SEvent::SJoystickEvent::AXIS_U,
        AXIS_V = irr::SEvent::SJoystickEvent::AXIS_V,
        NONE
    };

    /// Get value from joystick event (if triggered by this axis).
    double GetValue(const irr::SEvent::SJoystickEvent& joystickEvent);

    /// Read axis configuration from JSON file.
    void Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print);

    int id;                         ///< controller ID
    Axis axis;                      ///< controller axis
    std::string name;               ///< axis name
    double min, max;                ///< input range
    double scaled_min, scaled_max;  ///< output range
    double value;                   ///< current output value
};

/// Irrlicht interface to a specific joystick button.
struct CH_VEHICLE_API ChJoystickButtonIRR {
    ChJoystickButtonIRR();

    /// Return press state for this joystick event (if triggered by this button).
    bool IsPressed(const irr::SEvent::SJoystickEvent& joystickEvent, bool continuous = false);

    /// Read button configuration from JSON file.
    void Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print);

    int id;                  ///< controller ID
    int button;              ///< controller button
    std::string name;        ///< buttoin name
    int buttonPressedCount;  ///< counter to identify continuous press
    bool buttonPressed;      ///< current output value
};

/// Irrlicht-based interactive driver for the a vehicle.
/// This class implements the functionality required by the base ChInteractiveDriver class using keyboard or joystick
/// inputs. As an Irrlicht event receiver, its OnEvent() callback is used to keep track and update the current driver
/// inputs.
class CH_VEHICLE_API ChInteractiveDriverIRR : public ChInteractiveDriver, public irr::IEventReceiver {
  public:
    /// Construct an Irrlicht GUI driver.
    ChInteractiveDriverIRR(ChVehicleVisualSystemIrrlicht& vsys);

    virtual ~ChInteractiveDriverIRR() {}

    /// Check if joystick is supported.
    virtual bool HasJoystick() const override { return m_joystick_info.size() > 0; }

    /// Initialize this driver system.
    virtual void Initialize() override;

    /// Set joystick JSON configuration file name.
    void SetJoystickConfigFile(const std::string& filename);

    /// Enable/disable joystick debugging output (default: false).
    void SetJoystickDebug(bool val) { m_joystick_debug = val; }

    /// Feed button number and callback function to implement a custom callback.
    void SetButtonCallback(int button, void (*cbfun)());

  private:
    /// Intercept and process keyboard and joystick inputs.
    virtual bool OnEvent(const irr::SEvent& event) override;

    bool ProcessJoystickEvents(const irr::SEvent& event);
    bool ProcessKeyboardEvents(const irr::SEvent& event);

    ChVehicleVisualSystemIrrlicht& m_vsys;

    // Variables for mode=JOYSTICK
    irr::core::array<irr::SJoystickInfo> m_joystick_info;  ///< Irrlicht joystick information
    std::string m_joystick_file;                           ///< JSON specification file
    int m_joystick_proccess_frame;                         ///< counter for successive event processing frames
    bool m_joystick_debug;                                 ///< enable/disable debug output
    int m_joystick_debug_frame;                            ///< counter for successive output frames

    ChJoystickAxisIRR steerAxis;
    ChJoystickAxisIRR throttleAxis;
    ChJoystickAxisIRR brakeAxis;
    ChJoystickAxisIRR clutchAxis;
    ChJoystickButtonIRR shiftUpButton;
    ChJoystickButtonIRR shiftDownButton;
    ChJoystickButtonIRR gearReverseButton;
    ChJoystickButtonIRR gear1Button;
    ChJoystickButtonIRR gear2Button;
    ChJoystickButtonIRR gear3Button;
    ChJoystickButtonIRR gear4Button;
    ChJoystickButtonIRR gear5Button;
    ChJoystickButtonIRR gear6Button;
    ChJoystickButtonIRR gear7Button;
    ChJoystickButtonIRR gear8Button;
    ChJoystickButtonIRR gear9Button;
    ChJoystickButtonIRR toggleManualGearboxButton;

    int m_callback_button;          ///< joystick button associated to the custom callback
    void (*m_callback_function)();  ///< custom callback, can be implemented in the application
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
