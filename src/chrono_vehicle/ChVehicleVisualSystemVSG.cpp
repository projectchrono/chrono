// Copyright (c) 2022 projectchrono.org
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
// VSG-based visualization wrapper for vehicles.  This class is a derived
// from ChVisualSystemVSG and provides the following functionality:
//   - rendering of the entire VSG scene
//   - custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#include "chrono_vsg/ChGuiComponentVSG.h"

#include "chrono_vehicle/ChVehicleVisualSystemVSG.h"
#include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionShafts.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

class ChVehicleKeyboardHandlerVSG : public vsg3d::ChEventHandlerVSG {
  public:
    ChVehicleKeyboardHandlerVSG(ChVehicleVisualSystemVSG* app)
        : m_app(app), m_transmission_auto(nullptr), m_transmission_manual(nullptr) {
        auto transmission = m_app->m_vehicle->GetTransmission();
        if (transmission) {
            m_transmission_auto = transmission->asAutomatic();  // nullptr for a manual transmission
            m_transmission_manual = transmission->asManual();   // nullptr for an automatic transmission
        }
    }

    // Keyboard events for chase-cam and interactive driver control
    void process(vsg::KeyPressEvent& keyPress) override {
        if (!m_app->m_vehicle)
            return;

        switch (keyPress.keyModified) {
            case vsg::KEY_V:
                m_app->m_vehicle->LogConstraintViolations();
                return;
            default:
                break;
        }

        switch (keyPress.keyBase) {
            case vsg::KEY_Left:
                m_app->m_camera->Turn(-1);
                break;
            case vsg::KEY_Right:
                m_app->m_camera->Turn(1);
                break;
            case vsg::KEY_Down:
                m_app->m_camera->Zoom(+1);
                break;
            case vsg::KEY_Up:
                m_app->m_camera->Zoom(-1);
                break;
            case vsg::KEY_Prior:
                m_app->m_camera->Raise(-1);
                return;
            case vsg::KEY_Next:
                m_app->m_camera->Raise(+1);
                return;
            case vsg::KEY_a:
                if (m_app->m_driver)
                    m_app->m_driver->SteeringLeft();
                return;
            case vsg::KEY_d:
                if (m_app->m_driver)
                    m_app->m_driver->SteeringRight();
                return;
            case vsg::KEY_c:
                if (m_app->m_driver)
                    m_app->m_driver->SteeringCenter();
                return;
            case vsg::KEY_w:
                if (m_app->m_driver)
                    m_app->m_driver->IncreaseThrottle();
                return;
            case vsg::KEY_s:
                if (m_app->m_driver)
                    m_app->m_driver->DecreaseThrottle();
                return;
            case vsg::KEY_r:
                if (m_app->m_driver)
                    m_app->m_driver->ReleasePedals();
                return;
            case vsg::KEY_1:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Chase);
                return;
            case vsg::KEY_2:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Follow);
                return;
            case vsg::KEY_3:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Track);
                return;
            case vsg::KEY_4:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Inside);
                return;
            case vsg::KEY_5:
                m_app->SetChaseCameraState(utils::ChChaseCamera::Free);
                return;
            default:
                break;
        }

        if (m_transmission_auto) {
            switch (keyPress.keyBase) {
                case vsg::KEY_z:
                    if (m_transmission_auto->GetDriveMode() != ChAutomaticTransmission::DriveMode::FORWARD)
                        m_transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::FORWARD);
                    else
                        m_transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::REVERSE);
                    return;
                case vsg::KEY_x:
                    m_transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::NEUTRAL);
                    return;
                case vsg::KEY_t:
                    if (m_transmission_auto->GetShiftMode() == ChAutomaticTransmission::ShiftMode::MANUAL)
                        m_transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::AUTOMATIC);
                    else
                        m_transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::MANUAL);
                    return;
                case vsg::KEY_Rightbracket:
                    m_transmission_auto->ShiftUp();
                    return;
                case vsg::KEY_Leftbracket:
                    m_transmission_auto->ShiftDown();
                    return;
                default:
                    break;
            }
        }

        if (m_transmission_manual) {
            switch (keyPress.keyBase) {
                case vsg::KEY_Rightbracket:
                    m_transmission_manual->ShiftUp();
                    return;
                case vsg::KEY_Leftbracket:
                    m_transmission_manual->ShiftDown();
                    return;
                case vsg::KEY_e:
                    if (m_app->m_driver)
                        m_app->m_driver->IncreaseClutch();
                    return;
                case vsg::KEY_q:
                    if (m_app->m_driver)
                        m_app->m_driver->DecreaseClutch();
                    return;
                default:
                    break;
            }
        }
    }

  private:
    ChVehicleVisualSystemVSG* m_app;
    ChAutomaticTransmission* m_transmission_auto;
    ChManualTransmission* m_transmission_manual;
};

// -----------------------------------------------------------------------------

class ChVehicleGuiComponentVSG : public vsg3d::ChGuiComponentVSG {
  public:
    ChVehicleGuiComponentVSG(ChVehicleVisualSystemVSG* app) : m_app(app) {}
    virtual void render() override;

  private:
    ChVehicleVisualSystemVSG* m_app;
};

void DrawGauge(float val, float v_min, float v_max) {
    ImGui::PushItemWidth(150.0f);
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor(200, 100, 20));
    ImGui::SliderFloat("", &val, v_min, v_max, "%.2f");
    ImGui::PopStyleColor();
    ImGui::PopItemWidth();
}

void ShowHelp() {
    if (ImGui::CollapsingHeader("Chase camera controls", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BulletText("Selection of camera mode");
        ImGui::Indent();
        ImGui::Text("1, 2, 3, 4: select camera mode (chase/follow/track/inside)");
        ImGui::Unindent();

        ImGui::BulletText("'Chase' mode");
        ImGui::Indent();
        ImGui::Text("Left/Right: adjust camera chase angle");
        ImGui::Text("Up/Down: adjust camera chase distance");
        ImGui::Unindent();

        ImGui::BulletText("'Follow' mode");
        ImGui::Indent();
        ImGui::Text("Up/Down: adjust camera chase distance");
        ImGui::Unindent();

        ImGui::BulletText("'Track' mode");
        ImGui::BulletText("'Inside' mode");
        ImGui::Indent();
        ImGui::Text("no controls available");
        ImGui::Unindent();
    }

    if (ImGui::CollapsingHeader("Vehicle controls", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::BulletText("Drive and steering controls");
        ImGui::Indent();
        ImGui::Text("W/S: acceleartion/decceleration (combined throttle/brake controls)");
        ImGui::Text("A/D: steering (left/right)");
        ImGui::Text("C: center steering wheel (set steering=0)");
        ImGui::Text("R: release pedals (set throttle=brake=clutch=0)");
        ImGui::Unindent();

        ImGui::BulletText("Automatic transmission vehicles");
        ImGui::Indent();
        ImGui::Text("Z: toggle forward/reverse");
        ImGui::Text("X: shift to neutral");
        ImGui::Text("T: toggle manumatic/full automatic mode");
        ImGui::Text("]: shift up (in manumatic mode)");
        ImGui::Text("[: shift down (in manumatic mode)");
        ImGui::Unindent();

        ImGui::BulletText(
            "Manual transmission vehicles\n(shifting up and down goes from forward gears to neutral and then reverse)");
        ImGui::Indent();
        ImGui::Text("]: shift up");
        ImGui::Text("[: shift down");
        ImGui::Unindent();
    }
}

void ChVehicleGuiComponentVSG::render() {
    char label[64];
    int nstr = sizeof(label) - 1;

    ImGui::SetNextWindowSize(ImVec2(280.0f, 0.0f));
    ////ImGui::SetNextWindowPos(ImVec2(5.0f, 150.0f));
    ImGui::Begin("Vehicle");

    if (ImGui::BeginTable("CamTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextColumn();
        ImGui::Text("Camera State:");
        ImGui::TableNextColumn();
        ImGui::Text(m_app->GetChaseCamera().GetStateName().c_str());
        ImGui::EndTable();
    }

    ImGui::Spacing();

    if (ImGui::BeginTable("VehTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("Vehicle Speed:");
        ImGui::TableNextColumn();
        snprintf(label, nstr, "%8.3f m/s", m_app->GetVehicle().GetSpeed());
        ImGui::Text(label);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("Steering:");
        ImGui::TableNextColumn();
        ImGui::PushItemWidth(150.0f);
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor(200, 100, 20));
        DrawGauge(-m_app->GetSteering(), -1, 1);
        ImGui::PopStyleColor();
        ImGui::PopItemWidth();
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        if (m_app->GetVehicle().GetPowertrainAssembly() &&
            m_app->GetVehicle().GetPowertrainAssembly()->GetTransmission()->IsManual()) {
            ImGui::Text("Clutch:");
            ImGui::TableNextColumn();
            DrawGauge(m_app->GetClutch(), 0, 1);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
        }
        ImGui::Text("Braking:");
        ImGui::TableNextColumn();
        DrawGauge(m_app->GetBraking(), 0, 1);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("Throttle:");
        ImGui::TableNextColumn();
        DrawGauge(m_app->GetThrottle(), 0, 1);
        ImGui::EndTable();
    }

    // Display information from powertrain system
    const auto& powertrain = m_app->GetVehicle().GetPowertrainAssembly();
    if (powertrain) {
        const auto& engine = powertrain->GetEngine();
        const auto& transmission = powertrain->GetTransmission();

        auto transmission_auto = transmission->asAutomatic();  // nullptr for a manual transmission
        auto transmission_manual = transmission->asManual();   // nullptr for an automatic transmission

        ImGui::Spacing();

        if (ImGui::BeginTable("Powertrain", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            ImGui::Text("Engine Speed:");
            ImGui::TableNextColumn();
            snprintf(label, nstr, "%8.1lf RPM", engine->GetMotorSpeed() * 30 / CH_C_PI);
            ImGui::Text(label);
            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            ImGui::Text("Engine Torque:");
            ImGui::TableNextColumn();
            snprintf(label, nstr, "%8.1lf Nm", engine->GetOutputMotorshaftTorque());
            ImGui::Text(label);
            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            char shift_mode = 'M';
            if (transmission->IsAutomatic()) {
                if (transmission_auto->GetShiftMode() == ChAutomaticTransmission::ShiftMode::AUTOMATIC) {
                    shift_mode = 'A';
                }
                switch (transmission_auto->GetDriveMode()) {
                    case ChAutomaticTransmission::DriveMode::FORWARD:
                        snprintf(label, nstr, "[%c] Gear forward:", shift_mode);
                        break;
                    case ChAutomaticTransmission::DriveMode::NEUTRAL:
                        snprintf(label, nstr, "[%c] Gear neutral", shift_mode);
                        break;
                    case ChAutomaticTransmission::DriveMode::REVERSE:
                        snprintf(label, nstr, "[%c] Gear reverse", shift_mode);
                        break;
                }
            }
            else if (transmission->IsManual()) {
                snprintf(label, nstr, "[M] Gear:");
            }
            ImGui::Text(label);
            ImGui::TableNextColumn();
            if (transmission->IsManual() || (transmission->IsAutomatic() && transmission_auto->GetDriveMode() == ChAutomaticTransmission::DriveMode::FORWARD)) {
                snprintf(label, nstr, "%d", transmission->GetCurrentGear());
            }
            else {
                snprintf(label, nstr, "");
            }
            ImGui::Text(label);
            ImGui::TableNextRow();
            ImGui::EndTable();
        }

        if (transmission_auto && transmission_auto->HasTorqueConverter()) {
            ImGui::Spacing();

            if (ImGui::BeginTable("TorqueConverter", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(0.0f, 0.0f))) {
                ImGui::TableNextColumn();
                ImGui::Text("T.conv.slip:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%8.1f", transmission_auto->GetTorqueConverterSlippage());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("T.conv.torque.in:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%8.1f Nm", transmission_auto->GetTorqueConverterInputTorque());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("T.conv.torque.out:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%8.1f Nm", transmission_auto->GetTorqueConverterOutputTorque());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("T.conv.speed.out:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%8.1f RPM", transmission_auto->GetTorqueConverterOutputSpeed() * 30 / CH_C_PI);
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::EndTable();
            }
        }
    }

    m_app->AppendGUIStats();

    static bool show_help = false;
    ImGui::Spacing();
    ImGui::Checkbox("Vehicle controls help", &show_help);

    if (show_help) {
        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always,
                                ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x * 0.75f, io.DisplaySize.y * 0.75f), ImGuiCond_Always);
        ImGui::Begin("Help", &show_help, ImGuiWindowFlags_NoCollapse);
        ShowHelp();
        ImGui::End();
    }

    ImGui::End();
}

// -----------------------------------------------------------------------------

ChVehicleVisualSystemVSG::ChVehicleVisualSystemVSG() : ChVisualSystemVSG(), m_driver(nullptr) {}

ChVehicleVisualSystemVSG::~ChVehicleVisualSystemVSG() {}

void ChVehicleVisualSystemVSG::Initialize() {
    // Create vehicle-specific GUI and let derived classes append to it
    m_gui.push_back(chrono_types::make_shared<ChVehicleGuiComponentVSG>(this));

    // Add keyboard handler
    m_evhandler.push_back(chrono_types::make_shared<ChVehicleKeyboardHandlerVSG>(this));

    // Do not create a VSG camera trackball controller
    m_camera_trackball = false;

    // Invoke the base Initialize method
    ChVisualSystemVSG::Initialize();
}

void ChVehicleVisualSystemVSG::Advance(double step) {
    // Update the ChChaseCamera: take as many integration steps as needed to
    // exactly reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_camera->Update(h);
        t += h;
    }

    // Update the VSG camera
    ChVector<> cam_pos = m_camera->GetCameraPos();
    ChVector<> cam_target = m_camera->GetTargetPos();
    m_vsg_cameraEye.set(cam_pos.x(), cam_pos.y(), cam_pos.z());
    m_vsg_cameraTarget.set(cam_target.x(), cam_target.y(), cam_target.z());
    m_lookAt->eye.set(cam_pos.x(), cam_pos.y(), cam_pos.z());
    m_lookAt->center.set(cam_target.x(), cam_target.y(), cam_target.z());
}

}  // namespace vehicle
}  // namespace chrono
