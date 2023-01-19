// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// VSG-based visualization wrapper for vehicles.  This class is a derived
// from ChVisualSystemVSG and provides the following functionality:
//   - rendering of the entire Irrlicht scene
//   - custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#include "chrono_vsg/ChGuiComponentVSG.h"
#include "chrono_vehicle/driver/ChVSGGuiDriver.h"

namespace chrono {
namespace vehicle {

class FindVertexData : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) { object.traverse(*this); }

    void apply(vsg::BindVertexBuffers& bvd) {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[0]->data->accept(*this);
    }

    void apply(vsg::vec3Array& vertices) {
        if (verticesSet.count(&vertices) == 0) {
            verticesSet.insert(&vertices);
        }
    }

    std::vector<vsg::ref_ptr<vsg::vec3Array>> getVerticesList() {
        std::vector<vsg::ref_ptr<vsg::vec3Array>> verticesList(verticesSet.size());
        auto vertices_itr = verticesList.begin();
        for (auto& vertices : verticesSet) {
            (*vertices_itr++) = const_cast<vsg::vec3Array*>(vertices);
        }

        return verticesList;
    }

    std::set<vsg::vec3Array*> verticesSet;
};

class FindNormalData : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) { object.traverse(*this); }

    void apply(vsg::BindVertexBuffers& bvd) {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[1]->data->accept(*this);
    }

    void apply(vsg::vec3Array& normals) {
        if (normalsSet.count(&normals) == 0) {
            normalsSet.insert(&normals);
        }
    }

    std::vector<vsg::ref_ptr<vsg::vec3Array>> getNormalsList() {
        std::vector<vsg::ref_ptr<vsg::vec3Array>> normalsList(normalsSet.size());
        auto normals_itr = normalsList.begin();
        for (auto& normals : normalsSet) {
            (*normals_itr++) = const_cast<vsg::vec3Array*>(normals);
        }

        return normalsList;
    }

    std::set<vsg::vec3Array*> normalsSet;
};

class FindColorData : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) { object.traverse(*this); }

    void apply(vsg::BindVertexBuffers& bvd) {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[3]->data->accept(*this);
    }

    void apply(vsg::vec4Array& colors) {
        if (colorsSet.count(&colors) == 0) {
            colorsSet.insert(&colors);
        }
    }

    std::vector<vsg::ref_ptr<vsg::vec4Array>> getColorsList() {
        std::vector<vsg::ref_ptr<vsg::vec4Array>> colorsList(colorsSet.size());
        auto colors_itr = colorsList.begin();
        for (auto& colors : colorsSet) {
            (*colors_itr++) = const_cast<vsg::vec4Array*>(colors);
        }

        return colorsList;
    }

    std::set<vsg::vec4Array*> colorsSet;
};

// -----------------------------------------------------------------------------

class VehAppKeyboardHandler : public vsg::Inherit<vsg::Visitor, VehAppKeyboardHandler> {
  public:
    VehAppKeyboardHandler(ChVehicleVisualSystemVSG* app) : m_app(app) {}

#ifdef WIN32
    void apply(vsg::KeyPressEvent& keyPress) override {
        // keyboard events for camara steering
        switch (keyPress.keyModified) {
            case vsg::KEY_Down:
                m_app->CameraZoom(1);
                return;
            case vsg::KEY_Up:
                m_app->CameraZoom(-1);
                return;
            case vsg::KEY_Left:
                m_app->CameraTurn(-1);
                return;
            case vsg::KEY_Right:  // not recognized on the Mac
            case 94:              // Mac hack
                m_app->CameraTurn(1);
                return;
            case vsg::KEY_Next:
                m_app->CameraRaise(-1);
                return;
            case vsg::KEY_Prior:
                m_app->CameraRaise(1);
                return;
            case vsg::KEY_a:
                m_app->SteeringLeft();
                return;
            case vsg::KEY_d:
                m_app->SteeringRight();
                return;
            case vsg::KEY_c:
                m_app->SteeringCenter();
                return;
            case vsg::KEY_w:
                m_app->IncreaseVehicleSpeed();
                return;
            case vsg::KEY_s:
                m_app->DecreaseVehicleSpeed();
                return;
            case vsg::KEY_r:
                m_app->ReleasePedals();
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
            case vsg::KEY_V:
                m_app->LogContraintViolations();
                return;
        }
        switch (keyPress.keyBase) {
            case vsg::KEY_a:
                m_app->SteeringLeft();
                return;
            case vsg::KEY_d:
                m_app->SteeringRight();
                return;
            case vsg::KEY_c:
                m_app->SteeringCenter();
                return;
            case vsg::KEY_w:
                m_app->IncreaseVehicleSpeed();
                return;
            case vsg::KEY_s:
                m_app->DecreaseVehicleSpeed();
                return;
            case vsg::KEY_r:
                m_app->ReleasePedals();
                return;
        }
#else
    void apply(vsg::KeyPressEvent& keyPress) override {
        // keyboard events for camara steering
        switch (keyPress.keyBase) {
            case vsg::KEY_Down:
                m_app->CameraZoom(1);
                return;
            case vsg::KEY_Up:
                m_app->CameraZoom(-1);
                return;
            case vsg::KEY_Left:
                m_app->CameraTurn(-1);
                return;
            case vsg::KEY_Right:  // not recognized on the Mac
    #ifdef __APPLE__
            case 94:              // Mac hack
    #endif
                m_app->CameraTurn(1);
                return;
            case vsg::KEY_Next:
                m_app->CameraRaise(-1);
                return;
            case vsg::KEY_Prior:
                m_app->CameraRaise(1);
                return;
            case vsg::KEY_a:
                m_app->SteeringLeft();
                return;
            case vsg::KEY_d:
                m_app->SteeringRight();
                return;
            case vsg::KEY_c:
                m_app->SteeringCenter();
                return;
            case vsg::KEY_w:
                m_app->IncreaseVehicleSpeed();
                return;
            case vsg::KEY_s:
                m_app->DecreaseVehicleSpeed();
                return;
            case vsg::KEY_r:
                m_app->ReleasePedals();
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
            case vsg::KEY_V:
                m_app->LogContraintViolations();
                return;
        }
#endif
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

void ChVehicleGuiComponentVSG::render() {
    auto powertrain = m_app->GetVehicle().GetPowertrain();

    char label[64];
    int nstr = sizeof(label) - 1;

    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
    ImGui::Begin("Vehicle");

    ImGui::BeginTable("CamTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit, ImVec2(0.0f, 0.0f));
    ImGui::TableNextColumn();
    ImGui::Text("Camera State:");
    ImGui::TableNextColumn();
    ImGui::Text(m_app->GetChaseCamera().GetStateName().c_str());
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("Input Node:");
    ImGui::TableNextColumn();
    ImGui::Text(m_app->GetDriverMsg().c_str());
    ImGui::EndTable();
    ImGui::Spacing();
    ImGui::BeginTable("VehTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit, ImVec2(0.0f, 0.0f));
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("Vehicle Speed:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.3f m/s", m_app->GetVehicle().GetSpeed());
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("Steering:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.3f", m_app->GetSteering());
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("Throttle:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.3f", m_app->GetThrottle());
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("Braking:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.3f", m_app->GetBraking());
    ImGui::Text(label);
    ImGui::EndTable();

    ImGui::Spacing();
    
    ImGui::BeginTable("PowerTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                      ImVec2(0.0f, 0.0f));
    ImGui::TableNextColumn();
    ImGui::Text("Engine Speed:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.1lf RPM", powertrain ? powertrain->GetMotorSpeed() * 30 / CH_C_PI : 0);
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("Engine Torque:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.1lf Nm", powertrain ? powertrain->GetMotorTorque() : 0);
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    switch (m_app->GetDriveMode()) {
        case 'F':
            snprintf(label, nstr, "[%c] Gear forward:", m_app->GetTransmissionMode());
            break;
        case 'N':
            snprintf(label, nstr, "[%c] Gear neutral:", m_app->GetTransmissionMode());
            break;
        case 'R':
            snprintf(label, nstr, "[%c] Gear reverse:", m_app->GetTransmissionMode());
            break;
        default:
            snprintf(label, nstr, "[%c] Gear ?:", m_app->GetTransmissionMode());
            break;
    }
    ImGui::Text(label);
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%d", powertrain ? powertrain->GetCurrentTransmissionGear() : 0);
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::EndTable();

    ImGui::Spacing();

    //// RADU TODO 
    ////  show only if TC exists
    ImGui::BeginTable("ConvTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                      ImVec2(0.0f, 0.0f));
    ImGui::TableNextColumn();
    ImGui::Text("T.conv.slip:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.2f", powertrain ? powertrain->GetTorqueConverterSlippage() : 0);
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("T.conv.torque.in:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.1f Nm", powertrain ? powertrain->GetTorqueConverterInputTorque() : 0);
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("T.conv.torque.out:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.1f Nm", powertrain ? powertrain->GetTorqueConverterOutputTorque() : 0);
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("T.conv.speed.out:");
    ImGui::TableNextColumn();
    snprintf(label, nstr, "%.1f RPM", powertrain ? powertrain->GetTorqueConverterOutputSpeed() * 0 / CH_C_PI : 0);
    ImGui::Text(label);
    ImGui::TableNextRow();
    ImGui::EndTable();

    m_app->AppendGUIStats();

    ImGui::End();
}

// -----------------------------------------------------------------------------

ChVehicleVisualSystemVSG::ChVehicleVisualSystemVSG() : ChVisualSystemVSG(), m_guiDriver(nullptr) {}

ChVehicleVisualSystemVSG::~ChVehicleVisualSystemVSG() {}

void ChVehicleVisualSystemVSG::Initialize() {
    // Create vehicle-specific GUI and let derived classes append to it
    m_gui.push_back(chrono_types::make_shared<ChVehicleGuiComponentVSG>(this));
    //AppendGUIStats();

    // Do not create a VSG camera trackball controller
    m_camera_trackball = false;

    // Invoke the base Initialize method
    ChVisualSystemVSG::Initialize();

    // Add keyboard handler
    auto veh_kbHandler = VehAppKeyboardHandler::create(this);
    m_viewer->addEventHandler(veh_kbHandler);
    
    // Initialize chase-cam mode
    SetChaseCameraState(utils::ChChaseCamera::State::Chase);

    if (!m_vehicle)
        return;
    if (!m_vehicle->GetPowertrain())
        return;
    auto spt = std::dynamic_pointer_cast<ChShaftsPowertrain>(m_vehicle->GetPowertrain());
    if (spt) {
        m_params->show_converter_data = true;
    }
}

void ChVehicleVisualSystemVSG::AttachVehicle(vehicle::ChVehicle* vehicle) {
    ChVehicleVisualSystem::AttachVehicle(vehicle);

    //// RADU TODO 
    ////   Anything else here? If not, eliminate!
}

char ChVehicleVisualSystemVSG::GetTransmissionMode() {
    if (!m_vehicle)
        return '?';
    if (!m_vehicle->GetPowertrain())
        return '?';
    switch (m_vehicle->GetPowertrain()->GetTransmissionMode()) {
        default:
        case ChPowertrain::TransmissionMode::AUTOMATIC:
            return 'A';
        case ChPowertrain::TransmissionMode::MANUAL:
            return 'M';
    }
}

char ChVehicleVisualSystemVSG::GetDriveMode() {
    if (!m_vehicle)
        return '?';
    if (!m_vehicle->GetPowertrain())
        return '?';
    switch (m_vehicle->GetPowertrain()->GetDriveMode()) {
        default:
        case ChPowertrain::DriveMode::FORWARD:
            return 'F';
        case ChPowertrain::DriveMode::NEUTRAL:
            return 'N';
        case ChPowertrain::DriveMode::REVERSE:
            return 'R';
    }
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

    // Update the Irrlicht camera
    ChVector<> cam_pos = m_camera->GetCameraPos();
    ChVector<> cam_target = m_camera->GetTargetPos();
    m_vsg_cameraEye.set(cam_pos.x(), cam_pos.y(), cam_pos.z());
    m_vsg_cameraTarget.set(cam_target.x(), cam_target.y(), cam_target.z());
    m_lookAt->eye.set(cam_pos.x(), cam_pos.y(), cam_pos.z());
    m_lookAt->center.set(cam_target.x(), cam_target.y(), cam_target.z());
}

void ChVehicleVisualSystemVSG::IncreaseVehicleSpeed() {
    // GetLog() << "Speed+\n";
    if (m_guiDriver) {
        m_guiDriver->IncreaseThrottle();
    }
}

void ChVehicleVisualSystemVSG::DecreaseVehicleSpeed() {
    // GetLog() << "Speed-\n";
    if (m_guiDriver) {
        m_guiDriver->DecreaseThrottle();
    }
}

void ChVehicleVisualSystemVSG::SteeringLeft() {
    // GetLog() << "Left\n";
    if (m_guiDriver) {
        m_guiDriver->SteeringLeft();
    }
}

void ChVehicleVisualSystemVSG::SteeringRight() {
    // GetLog() << "Right\n";
    if (m_guiDriver) {
        m_guiDriver->SteeringRight();
    }
}

void ChVehicleVisualSystemVSG::SteeringCenter() {
    // GetLog() << "Center\n";
    if (m_guiDriver) {
        m_guiDriver->SteeringCenter();
    }
}

void ChVehicleVisualSystemVSG::ReleasePedals() {
    // GetLog() << "Release\n";
    if (m_guiDriver) {
        m_guiDriver->ReleasePedals();
    }
}

void ChVehicleVisualSystemVSG::CameraZoom(int how) {
    if (m_vehicle) {
        m_camera->Zoom(how);
    }
}

void ChVehicleVisualSystemVSG::CameraTurn(int how) {
    if (m_vehicle) {
        m_camera->Turn(how);
    }
}

void ChVehicleVisualSystemVSG::CameraRaise(int how) {
    if (m_vehicle) {
        m_camera->Raise(how);
    }
}

void ChVehicleVisualSystemVSG::LogContraintViolations() {
    m_vehicle->LogConstraintViolations();
}

void ChVehicleVisualSystemVSG::SetTargetSymbol(double size, ChColor col) {
    m_target_symbol_size = vsg::dvec3(size, size, size);

    // Is the symbol already present?
    vsg::ref_ptr<vsg::MatrixTransform> transform;
    bool found = false;
    for (auto child : m_symbolScene->children) {
        char sType = ' ';
        if (!child->getValue("SymbolType", sType))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        if (sType != 'T')
            continue;
        // we only set the size
        transform->matrix = vsg::translate(m_target_symbol_position) * vsg::scale(m_target_symbol_size);
        found = true;
    }
    if (found)
        return;

    // Symbol is not found, build it
    transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(m_target_symbol_position) * vsg::scale(m_target_symbol_size);

    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetEmissiveColor(col);
    auto tmpGroup =
        m_shapeBuilder->createShape(vsg3d::ShapeBuilder::SPHERE_SHAPE, material, transform, m_draw_as_wireframe);
    tmpGroup->setValue("SymbolType", 'T');
    m_symbolScene->addChild(tmpGroup);
}

void ChVehicleVisualSystemVSG::SetTargetSymbolPosition(ChVector<> pos) {
    m_target_symbol_position = vsg::dvec3(pos.x(), pos.y(), pos.z());
    for (auto child : m_symbolScene->children) {
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        char sType = ' ';
        if (!child->getValue("SymbolType", sType))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        if (sType != 'T')
            continue;
        transform->matrix = vsg::translate(m_target_symbol_position) * vsg::scale(m_target_symbol_size);
    }
}

void ChVehicleVisualSystemVSG::SetSentinelSymbol(double size, ChColor col) {
    m_sentinel_symbol_size = vsg::dvec3(size, size, size);

    // Is the symbol already present?
    vsg::ref_ptr<vsg::MatrixTransform> transform;
    bool found = false;
    for (auto child : m_symbolScene->children) {
        char sType = ' ';
        if (!child->getValue("SymbolType", sType))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        if (sType != 'S')
            continue;
        // we only set the size
        transform->matrix = vsg::translate(m_sentinel_symbol_position) * vsg::scale(m_sentinel_symbol_size);
        found = true;
    }
    if (found)
        return;

    // Symbol is not found, build it
    transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(m_sentinel_symbol_position) * vsg::scale(m_sentinel_symbol_size);

    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetEmissiveColor(col);
    auto tmpGroup =
        m_shapeBuilder->createShape(vsg3d::ShapeBuilder::SPHERE_SHAPE, material, transform, m_draw_as_wireframe);
    tmpGroup->setValue("SymbolType", 'S');
    m_symbolScene->addChild(tmpGroup);
}

void ChVehicleVisualSystemVSG::SetSentinelSymbolPosition(ChVector<> pos) {
    m_sentinel_symbol_position = vsg::dvec3(pos.x(), pos.y(), pos.z());
    for (auto child : m_symbolScene->children) {
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        char sType = ' ';
        if (!child->getValue("SymbolType", sType))
            continue;
        if (!child->getValue("TransformPtr", transform))
            continue;
        if (sType != 'S')
            continue;
        transform->matrix = vsg::translate(m_sentinel_symbol_position) * vsg::scale(m_sentinel_symbol_size);
    }
}

void ChVehicleVisualSystemVSG::UpdateFromMBS() {
    ChVisualSystemVSG::UpdateFromMBS();

    //// RADU TODO
    ////   Anything else here? If not, eliminate!
}

}  // namespace vehicle
}  // namespace chrono