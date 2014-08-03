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
// Irrlicht-based GUI driver for the HMMWV9 model. This class implements the
// functionality required by its base ChDriver class using keyboard inputs.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs. As such it does not need to override
// the default no-op Update() virtual method.
//
// =============================================================================

#include "HMMWV9_IrrGuiDriver.h"


using namespace chrono;
using namespace irr;


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_IrrGuiDriver::HMMWV9_IrrGuiDriver(ChIrrApp& app,
                                         const int tlc_X,
                                         const int tlc_Y)
{
  app.SetUserEventReceiver(this);

  gui::IGUIStaticText* text_inputs = app.GetIGUIEnvironment()->addStaticText(
    L"", core::rect<s32>(tlc_X, tlc_Y, tlc_X + 200, tlc_Y + 55), true, false, 0);
    
  m_text_throttle = app.GetIGUIEnvironment()->addStaticText(
    L"Throttle: 0",
    core::rect<s32>(10, 10, 150, 25), false, false, text_inputs);

  m_text_steering = app.GetIGUIEnvironment()->addStaticText(
    L"Steering: 0",
    core::rect<s32>(10, 30, 150, 45), false, false, text_inputs);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool HMMWV9_IrrGuiDriver::OnEvent(const SEvent& event)
{
  // user hit a key, while not holding it down
  if (event.EventType == EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown)
  {
    char msg[100];

    switch (event.KeyInput.Key) {
    case KEY_KEY_A:
      setSteering(m_steering - 0.1);
      sprintf(msg, "Steering: %+3.3g", m_steering);
      m_text_steering->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_D:
      setSteering(m_steering + 0.1);
      sprintf(msg, "Steering: %+3.3g", m_steering);
      m_text_steering->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_W:
      setThrottle(m_throttle + 0.1);
      sprintf(msg, "Throttle: %+3.3g", m_throttle*100.);
      m_text_throttle->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_S:
      setThrottle(m_throttle - 0.1);
      sprintf(msg, "Throttle: %+3.3g", m_throttle*100.);
      m_text_throttle->setText(core::stringw(msg).c_str());
      return true;
    }
  }

  return false;

}
