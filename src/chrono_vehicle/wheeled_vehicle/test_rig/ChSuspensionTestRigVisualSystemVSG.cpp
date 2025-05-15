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
// Authors: Radu Serban
// =============================================================================
//
// VSG-based visualization for a suspension test rig.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriver.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

class ChSTRKeyboardHandlerVSG : public vsg3d::ChEventHandlerVSG {
  public:
    ChSTRKeyboardHandlerVSG(ChSuspensionTestRigVisualSystemVSG* app) : m_app(app) {}

    // Keyboard events for chase-cam and interactive driver control
    void process(vsg::KeyPressEvent& keyPress) override {
        if (!m_app->m_rig)
            return;
        auto driver = std::dynamic_pointer_cast<ChSuspensionTestRigInteractiveDriver>(m_app->m_rig->GetDriver());
        if (!driver)
            return;

        switch (keyPress.keyBase) {
            case vsg::KEY_Plus:  // next axle
            case vsg::KEY_KP_Add:
                driver->NextAxle();
                break;
            case vsg::KEY_Minus:  // previous axle
            case vsg::KEY_KP_Subtract:
                driver->PreviousAxle();
                break;

            case vsg::KEY_a:
                driver->IncrementSteering();
                break;
            case vsg::KEY_d:
                driver->DecrementSteering();
                break;

            case vsg::KEY_t:  // left post up
                driver->DecrementLeft();
                break;
            case vsg::KEY_g:  // left post down
                driver->IncrementLeft();
                break;

            case vsg::KEY_y:  // right post up
                driver->DecrementRight();
                break;
            case vsg::KEY_h:  // right post down
                driver->IncrementRight();
                break;

            default:
                break;
        }
    }

  private:
    ChSuspensionTestRigVisualSystemVSG* m_app;
};

// -----------------------------------------------------------------------------

class ChSTRGuiComponentVSG : public vsg3d::ChGuiComponentVSG {
  public:
    ChSTRGuiComponentVSG(ChSuspensionTestRigVisualSystemVSG* app) : m_app(app) {}
    virtual void render(vsg::CommandBuffer& cb) override;

  private:
    ChSuspensionTestRigVisualSystemVSG* m_app;
};

void ChSTRGuiComponentVSG::render(vsg::CommandBuffer& cb) {
    auto driver = std::dynamic_pointer_cast<ChSuspensionTestRigInteractiveDriver>(m_app->m_rig->GetDriver());

    ImGui::SetNextWindowSize(ImVec2(280.0f, 0.0f));
    ImGui::Begin("Test Rig");

    // Only if interactive driver
    if (driver && ImGui::BeginTable("RigTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Steering:");
        ImGui::TableNextColumn();
        ImGui::PushItemWidth(150.0f);
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor(200, 100, 20));
        DrawGauge(-driver->GetSteering(), -1, 1);
        ImGui::PopStyleColor();
        ImGui::PopItemWidth();
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Post left:");
        ImGui::TableNextColumn();
        DrawGauge(-driver->GetLeft(), -1, 1);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Post right:");
        ImGui::TableNextColumn();
        DrawGauge(-driver->GetRight(), -1, 1);
        ImGui::EndTable();
    }

    ImGui::Spacing();

    if (ImGui::BeginTable("Subsystems", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
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

        ImGui::TableNextColumn();
        static bool suspension_visible = true;
        if (ImGui::Checkbox("Suspension", &suspension_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::SUSPENSION);
            m_app->m_suspension_visible = !m_app->m_suspension_visible;
            m_app->SetBodyObjVisibility(m_app->m_suspension_visible, tag);
            m_app->SetLinkObjVisibility(m_app->m_suspension_visible, tag);
            m_app->SetSpringVisibility(m_app->m_suspension_visible, tag);
            m_app->SetSegmentVisibility(m_app->m_suspension_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool steering_visible = true;
        if (ImGui::Checkbox("Steering", &steering_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::STEERING);
            m_app->m_steering_visible = !m_app->m_steering_visible;
            m_app->SetBodyObjVisibility(m_app->m_steering_visible, tag);
            m_app->SetLinkObjVisibility(m_app->m_steering_visible, tag);
            m_app->SetSpringVisibility(m_app->m_steering_visible, tag);
            m_app->SetSegmentVisibility(m_app->m_steering_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool wheel_visible = true;
        if (ImGui::Checkbox("Wheel", &wheel_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::WHEEL);
            m_app->m_wheel_visible = !m_app->m_wheel_visible;
            m_app->SetBodyObjVisibility(m_app->m_wheel_visible, tag);
            m_app->SetLinkObjVisibility(m_app->m_wheel_visible, tag);
        }

        ////ImGui::TableNextColumn();
        ////static bool tire_visible = true;
        ////if (ImGui::Checkbox("Tire", &tire_visible)) {
        ////    int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::TIRE);
        ////    m_app->m_tire_visible = !m_app->m_tire_visible;
        ////    m_app->SetBodyObjVisibility(m_app->m_tire_visible, tag);
        ////    m_app->SetLinkObjVisibility(m_app->m_tire_visible, tag);
        ////    m_app->SetFeaMeshVisibility(m_app->m_tire_visible, tag);
        ////}

        ImGui::EndTable();
    }


    ImGui::End();
}

// -----------------------------------------------------------------------------

ChSuspensionTestRigVisualSystemVSG::ChSuspensionTestRigVisualSystemVSG()
    : ChVisualSystemVSG(),
      m_rig(nullptr),
      m_chassis_visible(true),
      m_suspension_visible(true),
      m_steering_visible(true),
      m_wheel_visible(true),
      m_tire_visible(true) {
    // Disable global visibility controls
    m_show_visibility_controls = false;
}

void ChSuspensionTestRigVisualSystemVSG::AttachSTR(ChSuspensionTestRig* rig) {
    m_rig = rig;
    AttachSystem(rig->GetVehicle().GetSystem());
}

void ChSuspensionTestRigVisualSystemVSG::Initialize() {
    if (!m_initialized) {
        SetCameraVertical(CameraVerticalDir::Z);
        SetCameraAngleDeg(40.0);
        SetLightIntensity(1.0f);
        SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        EnableShadows();
        AddCamera(ChVector3d(-2.0, 0, 0.75), ChVector3d(0, 0, 0.5));

        // Add custom GUI
        m_gui.push_back(chrono_types::make_shared<ChSTRGuiComponentVSG>(this));

        // Add keyboard handler
        m_evhandler.push_back(chrono_types::make_shared<ChSTRKeyboardHandlerVSG>(this));

        // Invoke the base Initialize method
        // Note: this must occur only *after* adding custom GUI components and event handlers
        ChVisualSystemVSG::Initialize();
    } 
    
    if (m_rig) {
        ChVector3d target =
            0.5 * (m_rig->GetSpindlePos(0, LEFT) + m_rig->GetSpindlePos(0, RIGHT)) + ChVector3d(0, 0, 0.5);
        ChVector3d position = target - ChVector3d(5, 0, 0);
        SetCameraPosition(position);
        SetCameraTarget(target);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
