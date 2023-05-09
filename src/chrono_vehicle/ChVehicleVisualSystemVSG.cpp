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
    ChVehicleKeyboardHandlerVSG(ChVehicleVisualSystemVSG* app) : m_app(app) {}

    // Keyboard events for chase-cam and interactive driver control
    void process(vsg::KeyPressEvent& keyPress) override {
        if (!m_app->m_vehicle)
            return;

        switch (keyPress.keyModified) {
            case vsg::KEY_V:
                m_app->m_vehicle->LogConstraintViolations();
                return;
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
            case vsg::KEY_z:
                if (m_app->m_vehicle->GetTransmission()) {
                    if (m_app->m_vehicle->GetTransmission()->GetDriveMode() != ChTransmission::DriveMode::FORWARD)
                        m_app->m_vehicle->GetTransmission()->SetDriveMode(ChTransmission::DriveMode::FORWARD);
                    else
                        m_app->m_vehicle->GetTransmission()->SetDriveMode(ChTransmission::DriveMode::REVERSE);
                }
                return;
            case vsg::KEY_x:
                if (m_app->m_vehicle->GetTransmission())
                    m_app->m_vehicle->GetTransmission()->SetDriveMode(ChTransmission::DriveMode::NEUTRAL);
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
        }
    }

  private:
    ChVehicleVisualSystemVSG* m_app;
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
        ImGui::Text("Throttle:");
        ImGui::TableNextColumn();
        DrawGauge(m_app->GetThrottle(), 0, 1);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("Braking:");
        ImGui::TableNextColumn();
        DrawGauge(m_app->GetBraking(), 0, 1);
        ImGui::EndTable();
    }

    // Display information from powertrain system
    const auto& powertrain = m_app->GetVehicle().GetPowertrainAssembly();
    if (powertrain) {
        const auto& engine = powertrain->GetEngine();
        const auto& transmission = powertrain->GetTransmission();

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
            char tranny_mode = transmission->GetMode() == ChTransmission::Mode::AUTOMATIC ? 'A' : 'M';
            switch (transmission->GetDriveMode()) {
                case ChTransmission::DriveMode::FORWARD:
                    snprintf(label, nstr, "[%c] Gear forward:", tranny_mode);
                    break;
                case ChTransmission::DriveMode::NEUTRAL:
                    snprintf(label, nstr, "[%c] Gear neutral", tranny_mode);
                    break;
                case ChTransmission::DriveMode::REVERSE:
                    snprintf(label, nstr, "[%c] Gear reverse", tranny_mode);
                    break;
            }
            ImGui::Text(label);
            ImGui::TableNextColumn();
            if (transmission->GetDriveMode() == ChTransmission::DriveMode::FORWARD)
                snprintf(label, nstr, "%d", transmission->GetCurrentGear());
            else
                snprintf(label, nstr, "");
            ImGui::Text(label);
            ImGui::TableNextRow();
            ImGui::EndTable();
        }

        if (m_app->m_has_TC) {
            ImGui::Spacing();

            if (ImGui::BeginTable("TorqueConverter", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(0.0f, 0.0f))) {
                ImGui::TableNextColumn();
                ImGui::Text("T.conv.slip:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%8.1f", transmission->GetTorqueConverterSlippage());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("T.conv.torque.in:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%8.1f Nm", transmission->GetTorqueConverterInputTorque());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("T.conv.torque.out:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%8.1f Nm", transmission->GetTorqueConverterOutputTorque());
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("T.conv.speed.out:");
                ImGui::TableNextColumn();
                snprintf(label, nstr, "%8.1f RPM", transmission->GetTorqueConverterOutputSpeed() * 30 / CH_C_PI);
                ImGui::Text(label);
                ImGui::TableNextRow();
                ImGui::EndTable();
            }
        }
    }

    m_app->AppendGUIStats();

    ImGui::End();
}

// -----------------------------------------------------------------------------

ChVehicleVisualSystemVSG::ChVehicleVisualSystemVSG() : ChVisualSystemVSG(), m_driver(nullptr), m_has_TC(false) {}

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

    // Initialize chase-cam mode
    SetChaseCameraState(utils::ChChaseCamera::State::Chase);

    if (!m_vehicle || !m_vehicle->GetPowertrainAssembly())
        return;

    if (std::dynamic_pointer_cast<ChAutomaticTransmissionShafts>(m_vehicle->GetTransmission()))
        m_has_TC = true;
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
