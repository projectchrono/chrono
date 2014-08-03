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

#ifndef HMMWV9_IRRGUIDRIVER_H
#define HMMWV9_IRRGUIDRIVER_H


#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChIrrApp.h"

#include "ChDriver.h"


class HMMWV9_IrrGuiDriver : public chrono::ChDriver, public irr::IEventReceiver
{
public:
  HMMWV9_IrrGuiDriver(irr::ChIrrApp&    app,
                      const int         tlc_X = 740,
                      const int         tlc_Y = 20);

  ~HMMWV9_IrrGuiDriver() {}

  bool OnEvent(const irr::SEvent& event);

private:
  irr::gui::IGUIStaticText* m_text_throttle;
  irr::gui::IGUIStaticText* m_text_steering;

};


#endif