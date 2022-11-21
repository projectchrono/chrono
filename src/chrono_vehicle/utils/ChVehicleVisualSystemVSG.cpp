#include "ChVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

class FindVertexData : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) { object.traverse(*this); }
    /*
        void apply(vsg::Geometry& geometry)
        {
            if (geometry.arrays.empty()) return;
            geometry.arrays[0]->data->accept(*this);
        }

        void apply(vsg::VertexIndexDraw& vid) {
            if (vid.arrays.empty())
                return;
            vid.arrays[0]->data->accept(*this);
        }
    */
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

class VehAppKeyboardHandler : public vsg::Inherit<vsg::Visitor, VehAppKeyboardHandler> {
  public:
    VehAppKeyboardHandler(vsg::Viewer* viewer) : m_viewer(viewer) {}

    void SetParams(vsg::ref_ptr<ChVehicleVisualSystemVSG::StateParams> params, ChVehicleVisualSystemVSG* appPtr) {
        _params = params;
        m_appPtr = appPtr;
    }
#ifdef WIN32
    void apply(vsg::KeyPressEvent& keyPress) override {
        // keyboard events for camara steering
        switch (keyPress.keyModified) {
            case vsg::KEY_Down:
                m_appPtr->CameraZoom(1);
                return;
            case vsg::KEY_Up:
                m_appPtr->CameraZoom(-1);
                return;
            case vsg::KEY_Left:
                m_appPtr->CameraTurn(-1);
                return;
            case vsg::KEY_Right:  // not recognized on the Mac
            case 94:              // Mac hack
                m_appPtr->CameraTurn(1);
                return;
            case vsg::KEY_Next:
                m_appPtr->CameraRaise(-1);
                return;
            case vsg::KEY_Prior:
                m_appPtr->CameraRaise(1);
                return;
            case vsg::KEY_a:
                m_appPtr->SteeringLeft();
                return;
            case vsg::KEY_d:
                m_appPtr->SteeringRight();
                return;
            case vsg::KEY_c:
                m_appPtr->SteeringCenter();
                return;
            case vsg::KEY_w:
                m_appPtr->IncreaseVehicleSpeed();
                return;
            case vsg::KEY_s:
                m_appPtr->DecreaseVehicleSpeed();
                return;
            case vsg::KEY_r:
                m_appPtr->ReleasePedals();
                return;
            case vsg::KEY_1:
                m_appPtr->CameraState(utils::ChChaseCamera::Chase);
                return;
            case vsg::KEY_2:
                m_appPtr->CameraState(utils::ChChaseCamera::Follow);
                return;
            case vsg::KEY_3:
                m_appPtr->CameraState(utils::ChChaseCamera::Track);
                return;
            case vsg::KEY_4:
                m_appPtr->CameraState(utils::ChChaseCamera::Inside);
                return;
            case vsg::KEY_5:
                m_appPtr->CameraState(utils::ChChaseCamera::Free);
                return;
            case vsg::KEY_V:
                m_appPtr->LogContraintViolations();
                return;
        }
        switch (keyPress.keyBase) {
            case vsg::KEY_a:
                m_appPtr->SteeringLeft();
                return;
            case vsg::KEY_d:
                m_appPtr->SteeringRight();
                return;
            case vsg::KEY_c:
                m_appPtr->SteeringCenter();
                return;
            case vsg::KEY_w:
                m_appPtr->IncreaseVehicleSpeed();
                return;
            case vsg::KEY_s:
                m_appPtr->DecreaseVehicleSpeed();
                return;
            case vsg::KEY_r:
                m_appPtr->ReleasePedals();
                return;
        }
#else
    void apply(vsg::KeyPressEvent& keyPress) override {
        // keyboard events for camara steering
        switch (keyPress.keyBase) {
            case vsg::KEY_Down:
                m_appPtr->CameraZoom(1);
                return;
            case vsg::KEY_Up:
                m_appPtr->CameraZoom(-1);
                return;
            case vsg::KEY_Left:
                m_appPtr->CameraTurn(-1);
                return;
            case vsg::KEY_Right:  // not recognized on the Mac
    #ifdef __APPLE__
            case 94:              // Mac hack
    #endif
                m_appPtr->CameraTurn(1);
                return;
            case vsg::KEY_Next:
                m_appPtr->CameraRaise(-1);
                return;
            case vsg::KEY_Prior:
                m_appPtr->CameraRaise(1);
                return;
            case vsg::KEY_a:
                m_appPtr->SteeringLeft();
                return;
            case vsg::KEY_d:
                m_appPtr->SteeringRight();
                return;
            case vsg::KEY_c:
                m_appPtr->SteeringCenter();
                return;
            case vsg::KEY_w:
                m_appPtr->IncreaseVehicleSpeed();
                return;
            case vsg::KEY_s:
                m_appPtr->DecreaseVehicleSpeed();
                return;
            case vsg::KEY_r:
                m_appPtr->ReleasePedals();
                return;
            case vsg::KEY_1:
                m_appPtr->CameraState(utils::ChChaseCamera::Chase);
                _params->camera_mode = "Chase";
                return;
            case vsg::KEY_2:
                m_appPtr->CameraState(utils::ChChaseCamera::Follow);
                _params->camera_mode = "Follow";
                return;
            case vsg::KEY_3:
                m_appPtr->CameraState(utils::ChChaseCamera::Track);
                _params->camera_mode = "Track";
                return;
            case vsg::KEY_4:
                m_appPtr->CameraState(utils::ChChaseCamera::Inside);
                _params->camera_mode = "Inside";
                return;
            case vsg::KEY_5:
                m_appPtr->CameraState(utils::ChChaseCamera::Free);
                _params->camera_mode = "Free";
                return;
            case vsg::KEY_V:
                m_appPtr->LogContraintViolations();
                return;
        }
#endif
    }

  private:
    vsg::observer_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<ChVehicleVisualSystemVSG::StateParams> _params;
    ChVehicleVisualSystemVSG* m_appPtr;
};

ChVehicleVisualSystemVSG::ChVehicleVisualSystemVSG() : ChVisualSystemVSG(), m_guiDriver(nullptr) {
    m_params->showVehicleState = true;
}

ChVehicleVisualSystemVSG::~ChVehicleVisualSystemVSG() {}

void ChVehicleVisualSystemVSG::Initialize() {
    ChVisualSystemVSG::Initialize();
    // add keyboard handler
    auto veh_kbHandler = VehAppKeyboardHandler::create(m_viewer);
    veh_kbHandler->SetParams(m_params, this);
    m_viewer->addEventHandler(veh_kbHandler);
    CameraState(utils::ChChaseCamera::State::Chase);
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
}

void ChVehicleVisualSystemVSG::AttachGuiDriver(ChVSGGuiDriver* driver) {
    m_guiDriver = driver;
}

void ChVehicleVisualSystemVSG::Synchronize(const std::string& msg, const DriverInputs& driver_inputs) {
    if (m_vehicle) {
        m_params->vehicleSpeed = m_vehicle->GetSpeed();
        m_params->steering = driver_inputs.m_steering;
        m_params->throttle = driver_inputs.m_throttle;
        m_params->braking = driver_inputs.m_braking;
        m_params->input_mode = msg;
    } else {
        m_params->vehicleSpeed = 0.0;
        m_params->steering = 0.0;
        m_params->throttle = 0.0;
        m_params->braking = 0.0;
        m_params->input_mode = "n. a.";
    }
}

int ChVehicleVisualSystemVSG::GetGearPosition() {
    if (!m_vehicle)
        return 0;
    if (!m_vehicle->GetPowertrain())
        return 0;
    return m_vehicle->GetPowertrain()->GetCurrentTransmissionGear();
}

double ChVehicleVisualSystemVSG::GetEngineSpeedRPM() {
    if (!m_vehicle)
        return 0.0;
    if (!m_vehicle->GetPowertrain())
        return 0.0;
    return m_vehicle->GetPowertrain()->GetMotorSpeed() * 30.0 / CH_C_PI;
}

double ChVehicleVisualSystemVSG::GetEngineTorque() {
    if (!m_vehicle)
        return 0.0;
    if (!m_vehicle->GetPowertrain())
        return 0.0;
    return m_vehicle->GetPowertrain()->GetMotorTorque();
}

char ChVehicleVisualSystemVSG::GetTransmissionMode() {
    if (!m_vehicle)
        return '?';
    if (!m_vehicle->GetPowertrain())
        return '?';
    switch (m_vehicle->GetPowertrain()->GetTransmissionMode()) {
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

double ChVehicleVisualSystemVSG::GetTconvSlip() {
    if (!m_vehicle)
        return 0.0;
    if (!m_vehicle->GetPowertrain())
        return 0.0;
    if (m_params->show_converter_data) {
        return m_vehicle->GetPowertrain()->GetTorqueConverterSlippage();
    } else {
        return 0.0;
    }
}

double ChVehicleVisualSystemVSG::GetTconvTorqueInput() {
    if (!m_vehicle)
        return 0.0;
    if (!m_vehicle->GetPowertrain())
        return 0.0;
    if (m_params->show_converter_data) {
        return m_vehicle->GetPowertrain()->GetTorqueConverterInputTorque();
    } else {
        return 0.0;
    }
}

double ChVehicleVisualSystemVSG::GetTconvTorqueOutput() {
    if (!m_vehicle)
        return 0.0;
    if (!m_vehicle->GetPowertrain())
        return 0.0;
    if (m_params->show_converter_data) {
        return m_vehicle->GetPowertrain()->GetTorqueConverterOutputTorque();
    } else {
        return 0.0;
    }
}

double ChVehicleVisualSystemVSG::GetTconvSpeedOutput() {
    if (!m_vehicle)
        return 0.0;
    if (!m_vehicle->GetPowertrain())
        return 0.0;
    if (m_params->show_converter_data) {
        return m_vehicle->GetPowertrain()->GetTorqueConverterOutputSpeed() * 30.0 / CH_C_PI;
    } else {
        return 0.0;
    }
}

void ChVehicleVisualSystemVSG::CameraState(utils::ChChaseCamera::State state) {
    m_camera->SetState(state);
    switch (state) {
        case utils::ChChaseCamera::State::Chase:
            m_params->camera_mode = "Chase";
            break;
        case utils::ChChaseCamera::State::Follow:
            m_params->camera_mode = "Follow";
            break;
        case utils::ChChaseCamera::State::Track:
            m_params->camera_mode = "Track";
            break;
        case utils::ChChaseCamera::State::Inside:
            m_params->camera_mode = "Inside";
            break;
        case utils::ChChaseCamera::State::Free:
            m_params->camera_mode = "Free";
            break;
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

void ChVehicleVisualSystemVSG::BindAll() {
    ChVisualSystemVSG::BindAll();

    for (auto& item : m_systems[0]->Get_otherphysicslist()) {
        if (auto defsoil = std::dynamic_pointer_cast<SCMDeformableSoil>(item)) {
            auto defshape = defsoil->GetVisualShape(0);
            if (!defshape) {
                continue;
            }
            if (!defshape->IsVisible()) {
                continue;
            }
            auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(defshape);
            if (!trimesh) {
                continue;
            }
            GetLog() << "Generating SCM Deformable Terrain....\n";
            auto transform = vsg::MatrixTransform::create();
            if (trimesh->GetNumMaterials() > 0) {
                m_deformableScene->addChild(
                    m_shapeBuilder->createTrimeshMatShape(transform, trimesh->IsWireframe(), trimesh));
            } else {
                m_deformableScene->addChild(
                    m_shapeBuilder->createTrimeshColShapeSCM(transform, trimesh->IsWireframe(), trimesh));
            }
            // find the vertices in the scenegraph
            m_scm_vertices_list = vsg::visit<FindVertexData>(m_deformableScene).getVerticesList();
            for (auto& vertices : m_scm_vertices_list) {
                m_num_scm_vertices += vertices->size();
            }
            GetLog() << "Dynamic SCM vertices = " << m_num_scm_vertices << "\n";
            size_t num_vertices_actual = trimesh->GetMesh()->getCoordsVertices().size();
            GetLog() << "Dynamic Mesh vertices = " << num_vertices_actual << "\n";
            m_scm_z_actual.resize(num_vertices_actual);
            m_scm_vertex_update_ok = true;
        }
    }
}

void ChVehicleVisualSystemVSG::UpdateFromMBS() {
    ChVisualSystemVSG::UpdateFromMBS();

    for (auto& item : m_systems[0]->Get_otherphysicslist()) {
        if (auto defsoil = std::dynamic_pointer_cast<SCMDeformableSoil>(item)) {
            auto defshape = defsoil->GetVisualShape(0);
            if (!defshape) {
                continue;
            }
            if (!defshape->IsVisible()) {
                continue;
            }
            auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshShape>(defshape);
            if (!trimesh) {
                continue;
            }
            if (m_scm_vertex_update_ok) {
                // GetLog() << "Updating SCM Deformable Terrain....\n";
                auto& defvertices = trimesh->GetMesh()->getCoordsVertices();
                for (size_t k = 0; k < defvertices.size(); k++) {
                    m_scm_z_actual[k] = defvertices[k].z();
                }
            }
        }
    }
}

void ChVehicleVisualSystemVSG::Render() {
    if (m_params->frame_number == 0)
        m_params->time_begin = double(clock()) / double(CLOCKS_PER_SEC);

    UpdateFromMBS();

    if (!m_viewer->advanceToNextFrame()) {
        return;
    }

    // pass any events into EventHandlers assigned to the Viewer
    m_viewer->handleEvents();

    m_viewer->update();

    if (m_scm_vertex_update_ok) {
        for (auto& vertices : m_scm_vertices_list) {
            size_t k = 0;
            for (auto& v : *vertices) {
                v.z = m_scm_z_actual[k];
                k++;
            }
            vertices->dirty();
        }
    }

    m_viewer->recordAndSubmit();

    if (m_params->do_image_capture) {
        vsg3d::exportScreenshot(m_window, m_options, m_imageFilename);
        m_params->do_image_capture = false;
    }

    m_viewer->present();
    m_params->frame_number++;
}

}  // namespace vehicle
}  // namespace chrono