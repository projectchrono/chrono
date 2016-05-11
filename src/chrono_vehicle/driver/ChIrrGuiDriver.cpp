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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Irrlicht-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard inputs.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs. As such it does not need to override
// the default no-op Advance() virtual method.
//
// =============================================================================

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIrrGuiDriver::ChIrrGuiDriver(ChVehicleIrrApp& app)
    : ChDriver(*app.m_vehicle),
      m_app(app),
      m_throttleDelta(1.0 / 50),
      m_steeringDelta(1.0 / 50),
      m_brakingDelta(1.0 / 50),
      m_mode(KEYBOARD) {
    app.SetUserEventReceiver(this);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChIrrGuiDriver::OnEvent(const SEvent& event) {
    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_A:
                if (m_mode == KEYBOARD)
                    SetSteering(m_steering - m_steeringDelta);
                return true;
            case KEY_KEY_D:
                if (m_mode == KEYBOARD)
                    SetSteering(m_steering + m_steeringDelta);
                return true;
            case KEY_KEY_W:
                if (m_mode == KEYBOARD) {
                    SetThrottle(m_throttle + m_throttleDelta);
                    if (m_throttle > 0)
                        SetBraking(m_braking - m_brakingDelta * 3.0);
                }
                return true;
            case KEY_KEY_S:
                if (m_mode == KEYBOARD) {
                    SetThrottle(m_throttle - m_throttleDelta * 3.0);
                    if (m_throttle <= 0)
                        SetBraking(m_braking + m_brakingDelta);
                }
                return true;
        }
    } else {
        switch (event.KeyInput.Key) {
            case KEY_KEY_L:
                m_mode = LOCK;
                return true;

            case KEY_KEY_K:
                m_throttle = 0;
                m_steering = 0;
                m_braking = 0;
                m_mode = KEYBOARD;
                return true;

            case KEY_KEY_J:
                if (m_data_driver) {
                    m_mode = DATAFILE;
                    m_time_shift = m_app.m_vehicle->GetSystem()->GetChTime();
                }
                return true;

            case KEY_KEY_Z:
                if (m_mode == KEYBOARD)
                    m_app.m_powertrain->SetDriveMode(ChPowertrain::FORWARD);
                return true;
            case KEY_KEY_X:
                if (m_mode == KEYBOARD)
                    m_app.m_powertrain->SetDriveMode(ChPowertrain::NEUTRAL);
                return true;
            case KEY_KEY_C:
                if (m_mode == KEYBOARD)
                    m_app.m_powertrain->SetDriveMode(ChPowertrain::REVERSE);
                return true;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::SetInputDataFile(const std::string& filename) {
    // Embed a DataDriver.
    m_data_driver = std::make_shared<ChDataDriver>(m_vehicle, filename, false);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::SetInputMode(InputMode mode) {
    switch (mode) {
        case LOCK:
            m_mode = LOCK;
            break;
        case KEYBOARD:
            m_throttle = 0;
            m_steering = 0;
            m_braking = 0;
            m_mode = KEYBOARD;
            break;
        case DATAFILE:
            if (m_data_driver) {
                m_mode = DATAFILE;
                m_time_shift = m_app.m_vehicle->GetSystem()->GetChTime();
            }
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::Synchronize(double time) {
    // Do nothing if no embedded DataDriver.
    if (m_mode != DATAFILE || !m_data_driver)
        return;

    // Call the update function of the embedded DataDriver, with shifted time.
    m_data_driver->Synchronize(time - m_time_shift);

    // Use inputs from embedded DataDriver
    m_throttle = m_data_driver->GetThrottle();
    m_steering = m_data_driver->GetSteering();
    m_braking = m_data_driver->GetBraking();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::string ChIrrGuiDriver::GetInputModeAsString() const {
    switch (m_mode) {
        case LOCK:
            return std::string("Input mode: LOCK");
        case KEYBOARD:
            return std::string("Input mode: KEY");
        case DATAFILE:
            return std::string("Input mode: FILE");
    }
    return std::string("");
}

}  // end namespace vehicle
}  // end namespace chrono
