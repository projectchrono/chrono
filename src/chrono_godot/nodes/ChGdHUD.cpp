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
// Authors: Asher Elmquist
// =============================================================================
//
// Class to manage adding, changing, and removing godot scene nodes
//
// =============================================================================
#include <core/ustring.h>
#include <scene/gui/button.h>
#include <scene/gui/control.h>
#include <scene/gui/label.h>

// ChGodotIncludes
#include "chrono_godot/nodes/ChGdHUD.h"

namespace chrono {
namespace gd {

ChGdHUD::ChGdHUD() {}
ChGdHUD::~ChGdHUD() {
    // for (auto obj : m_resourceList) {
    //     memdelete(obj);
    // }
    // m_resourceList.clear();
}

void ChGdHUD::Initialize(ChSystem* chsystem, Viewport* root) {
    // save the ChSystem
    m_system = chsystem;

    // initialize a "defualt" HUD
    // m_hud = memnew(Control);
    // m_hud->set_visible(false);
    // m_resourceList.push_back(m_hud);
    // m_hud->set_name("HUD");
    // root->add_child(m_hud);
    //
    // // add text box
    // time_text = memnew(Label);
    // m_resourceList.push_back(time_text);
    // time_text->set_name("SimTime");
    // time_text->set_text("ChTime: ");
    // time_text->set_position({20, 100});
    // m_hud->add_child(time_text);
    //
    // // add button -> can easily add, but need to solve clicking
    // Button* pause_button = memnew(Button);
    // m_resourceList.push_back(pause_button);
    // pause_button->set_name("Button");
    // pause_button->set_text("Pause Simulation");
    // pause_button->set_position({20, 120});
    // pause_button->set_disabled(false);
    // m_hud->add_child(pause_button);
}

void ChGdHUD::SetDisplay(bool display) {
    // m_hud->set_visible(display);
}

void ChGdHUD::Update() {
    // if (m_system) {
    //     // std::ostringstream x_convert;
    //     // x_convert << m_system->GetChTime();
    //
    //     // std::cout<<"R: "<<r<<std::endl;
    //     time_text->set_text("ChTime: " + String::num_real(m_system->GetChTime()));
    // }
}

}  // namespace gd
}  // end namespace chrono
