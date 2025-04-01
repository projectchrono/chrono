// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// VSG-based visualization for a track test rig.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemVSG.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigInteractiveDriver.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

class ChTTRKeyboardHandlerVSG : public vsg3d::ChEventHandlerVSG {
  public:
    ChTTRKeyboardHandlerVSG(ChTrackTestRigVisualSystemVSG* app) : m_app(app) {}

    // Keyboard events for chase-cam and interactive driver control
    void process(vsg::KeyPressEvent& keyPress) override {
        if (!m_app->m_rig)
            return;
        auto driver = std::dynamic_pointer_cast<ChTrackTestRigInteractiveDriver>(m_app->m_rig->GetDriver());
        if (!driver)
            return;

        switch (keyPress.keyBase) {
            case vsg::KEY_Plus:  // next post
            case vsg::KEY_KP_Add:
                driver->NextPost();
                break;
            case vsg::KEY_Minus:  // previous post
            case vsg::KEY_KP_Subtract:
                driver->PreviousPost();
                break;

            case vsg::KEY_t:  // post up
                driver->IncreasePost();
                break;
            case vsg::KEY_g:  // left post down
                driver->DecreasePost();
                break;

            case vsg::KEY_w:  // right post up
                driver->IncreaseThrottle();
                break;
            case vsg::KEY_s:  // right post down
                driver->DecreaseThrottle();
                break;

            default:
                break;
        }
    }

  private:
    ChTrackTestRigVisualSystemVSG* m_app;
};

// -----------------------------------------------------------------------------

class ChTTRGuiComponentVSG : public vsg3d::ChGuiComponentVSG {
  public:
    ChTTRGuiComponentVSG(ChTrackTestRigVisualSystemVSG* app) : m_app(app) {}
    virtual void render() override;

  private:
    ChTrackTestRigVisualSystemVSG* m_app;
};

void ChTTRGuiComponentVSG::render() {
    auto driver = std::dynamic_pointer_cast<ChTrackTestRigInteractiveDriver>(m_app->m_rig->GetDriver());

    ImGui::SetNextWindowSize(ImVec2(280.0f, 0.0f));
    ImGui::Begin("Test Rig");

    // Only if interactive driver
    if (driver && ImGui::BeginTable("RigTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                    ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Throttle:");
        ImGui::TableNextColumn();
        ImGui::PushItemWidth(150.0f);
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor(200, 100, 20));
        DrawGauge(-driver->GetThrottle(), -1, 1);
        ImGui::PopStyleColor();
        ImGui::PopItemWidth();

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Post:");
        ImGui::TableNextColumn();
        DrawGauge(-driver->GetPost(), -1, 1);
        ImGui::EndTable();
    }

    ImGui::Spacing();

    if (ImGui::BeginTable("Subsystems", 1, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        uint16_t vtag = 0;

        ////ImGui::TableNextColumn();
        ////static bool chassis_visible = true;
        ////if (ImGui::Checkbox("Chassis", &chassis_visible)) {
        ////    int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::CHASSIS);
        ////    m_app->m_chassis_visible = !m_app->m_chassis_visible;
        ////    m_app->SetBodyObjVisibility(m_app->m_chassis_visible, tag);
        ////    m_app->SetLinkObjVisibility(m_app->m_chassis_visible, tag);
        ////}

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        static bool sprocket_visible = true;
        if (ImGui::Checkbox("Sprocket", &sprocket_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::SPROCKET);
            m_app->m_sprocket_visible = !m_app->m_sprocket_visible;
            m_app->SetBodyObjVisibility(m_app->m_sprocket_visible, tag);
            m_app->SetLinkObjVisibility(m_app->m_sprocket_visible, tag);
        }

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        static bool idler_visible = true;
        if (ImGui::Checkbox("Idler", &idler_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::IDLER);
            m_app->m_idler_visible = !m_app->m_idler_visible;
            m_app->SetBodyObjVisibility(m_app->m_idler_visible, tag);
            m_app->SetLinkObjVisibility(m_app->m_idler_visible, tag);
            m_app->SetSpringVisibility(m_app->m_idler_visible, tag);
            m_app->SetSegmentVisibility(m_app->m_idler_visible, tag);
        }

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        static bool suspension_visible = true;
        if (ImGui::Checkbox("Suspension", &suspension_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::TRACK_SUSPENSION);
            m_app->m_suspension_visible = !m_app->m_suspension_visible;
            m_app->SetBodyObjVisibility(m_app->m_suspension_visible, tag);
            m_app->SetLinkObjVisibility(m_app->m_suspension_visible, tag);
            m_app->SetSpringVisibility(m_app->m_suspension_visible, tag);
            m_app->SetSegmentVisibility(m_app->m_suspension_visible, tag);
        }

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        static bool wheel_visible = true;
        if (ImGui::Checkbox("Wheel", &wheel_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::TRACK_WHEEL);
            m_app->m_wheel_visible = !m_app->m_wheel_visible;
            m_app->SetBodyObjVisibility(m_app->m_wheel_visible, tag);
            m_app->SetLinkObjVisibility(m_app->m_wheel_visible, tag);
        }

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        static bool shoe_visible = true;
        if (ImGui::Checkbox("Track shoe", &shoe_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::SHOE);
            m_app->m_shoe_visible = !m_app->m_shoe_visible;
            m_app->SetBodyObjVisibility(m_app->m_shoe_visible, tag);
            m_app->SetLinkObjVisibility(m_app->m_shoe_visible, tag);
            m_app->SetFeaMeshVisibility(m_app->m_shoe_visible, tag);
        }

        ImGui::EndTable();
    }

    ImGui::End();
}

// -----------------------------------------------------------------------------

ChTrackTestRigVisualSystemVSG::ChTrackTestRigVisualSystemVSG()
    : ChVisualSystemVSG(),
      m_rig(nullptr),
      m_chassis_visible(true),
      m_sprocket_visible(true),
      m_idler_visible(true),
      m_suspension_visible(true),
      m_wheel_visible(true),
      m_shoe_visible(true) {
    // Disable global visibility controls
    m_show_visibility_controls = false;
}

void ChTrackTestRigVisualSystemVSG::AttachTTR(ChTrackTestRig* rig) {
    m_rig = rig;
    AttachSystem(rig->GetSystem());
}

void ChTrackTestRigVisualSystemVSG::Initialize() {
    if (!m_initialized) {
        SetCameraVertical(CameraVerticalDir::Z);
        SetCameraAngleDeg(40.0);
        SetLightIntensity(1.0f);
        SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        EnableShadows(true);
        AddCamera(ChVector3d(0, -6, 0.5), ChVector3d(0, 0, 0.5));

        // Add custom GUI
        m_gui.push_back(chrono_types::make_shared<ChTTRGuiComponentVSG>(this));

        // Add keyboard handler
        m_evhandler.push_back(chrono_types::make_shared<ChTTRKeyboardHandlerVSG>(this));

        // Invoke the base Initialize method
        // Note: this must occur only *after* adding custom GUI components and event handlers
        ChVisualSystemVSG::Initialize();
    }

    if (m_rig) {
        auto sprocket_x = m_rig->m_track->GetSprocketLocation().x();
        auto idler_x = m_rig->m_track->GetIdlerLocation().x();
        ChVector3d target = ChVector3d(0.5 * (sprocket_x + idler_x), 0, 0.5);
        ChVector3d position = target - ChVector3d(0, 6, 0);
        SetCameraPosition(position);
        SetCameraTarget(target);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
