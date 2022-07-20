// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// =============================================================================

#include "chrono_models/robot/robosimian/RoboSimianVisualSystemIrrlicht.h"

namespace chrono {
namespace robosimian {

// -----------------------------------------------------------------------------

class RS_IEventReceiver : public irr::IEventReceiver {
  public:
    RS_IEventReceiver(RoboSimianVisualSystemIrrlicht* vsys) : m_vsys(vsys), m_vis(VisualizationType::COLLISION) {}

    virtual bool OnEvent(const irr::SEvent& event) override {
        if (event.EventType != irr::EET_KEY_INPUT_EVENT)
            return false;

        if (!event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_C:
                    m_vis = (m_vis == VisualizationType::MESH ? VisualizationType::COLLISION : VisualizationType::MESH);

                    m_vsys->m_robot->SetVisualizationTypeChassis(m_vis);
                    m_vsys->m_robot->SetVisualizationTypeSled(m_vis);
                    m_vsys->m_robot->SetVisualizationTypeLimbs(m_vis);
                    m_vsys->m_robot->SetVisualizationTypeWheels(m_vis);

                    m_vsys->BindAll();

                    return true;

                default:
                    return false;
            }
        }

        return false;
    }

  private:
    VisualizationType m_vis;
    RoboSimianVisualSystemIrrlicht* m_vsys;
};

// -----------------------------------------------------------------------------

RoboSimianVisualSystemIrrlicht::RoboSimianVisualSystemIrrlicht(RoboSimian* robot, RS_Driver* driver)
    : m_robot(robot), m_driver(driver), m_HUD_x(650), m_HUD_y(20) {
    m_systems.push_back(robot->GetSystem());
    m_erecv = new RS_IEventReceiver(this);
    AddUserEventReceiver(m_erecv);
}

RoboSimianVisualSystemIrrlicht::~RoboSimianVisualSystemIrrlicht() {
    delete m_erecv;
}

void RoboSimianVisualSystemIrrlicht::renderTextBox(const std::string& msg,
                                                   int xpos,
                                                   int ypos,
                                                   int length,
                                                   int height,
                                                   irr::video::SColor color) {
    irr::core::rect<irr::s32> mclip(xpos, ypos, xpos + length, ypos + height);
    GetVideoDriver()->draw2DRectangle(irr::video::SColor(90, 60, 60, 60),
                                      irr::core::rect<irr::s32>(xpos, ypos, xpos + length, ypos + height), &mclip);
    irr::gui::IGUIFont* font = GetGUIEnvironment()->getBuiltInFont();
    font->draw(msg.c_str(), irr::core::rect<irr::s32>(xpos + 3, ypos + 3, xpos + length, ypos + height), color);
}

void RoboSimianVisualSystemIrrlicht::Render() {
    ChVisualSystemIrrlicht::Render();

    char msg[100];

    sprintf(msg, "Time %.2f", m_robot->GetSystem()->GetChTime());
    renderTextBox(msg, m_HUD_x, m_HUD_y, 120, 15, irr::video::SColor(255, 250, 200, 00));

    auto type = std::string("Contact method: ") +
                (m_robot->GetSystem()->GetContactMethod() == ChContactMethod::NSC ? "NSC" : "SMC");
    renderTextBox(type.c_str(), m_HUD_x, m_HUD_y + 15, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "Driver phase: %s", m_driver->GetCurrentPhase().c_str());
    renderTextBox(msg, m_HUD_x, m_HUD_y + 30, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega FR: %.2f", std::abs(m_robot->GetWheelOmega(FR)));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 60, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega RR: %.2f", std::abs(m_robot->GetWheelOmega(RR)));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 75, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega FL: %.2f", std::abs(m_robot->GetWheelOmega(FL)));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 90, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega RL: %.2f", std::abs(m_robot->GetWheelOmega(RL)));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 105, 120, 15, irr::video::SColor(255, 250, 200, 00));
}

}  // namespace robosimian
}  // namespace chrono
