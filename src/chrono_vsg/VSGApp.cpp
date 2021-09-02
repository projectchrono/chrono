// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
// Vulkan Scene Graph viewer class
// =============================================================================

#include "chrono_vsg/core/ChApiVSG.h"

#include "VSGApp.h"

using namespace chrono::vsg3d;

struct Params : public vsg::Inherit<vsg::Object, Params> {
    bool showGui = true;  // you can toggle this with your own EventHandler and key
    int drawMode = 0;
};

class AppKeyboardHandler : public vsg::Inherit<vsg::Visitor, AppKeyboardHandler> {
  public:
    AppKeyboardHandler(vsg::Viewer* viewer) : m_viewer(viewer) {}
    void SetParams(vsg::ref_ptr<Params> params, VSGApp* appPtr) {
        _params = params;
        m_appPtr = appPtr;
    }
    void apply(vsg::KeyPressEvent& keyPress) override {
        if (keyPress.keyBase == '1') {
            _params->drawMode = 0;
            m_appPtr->UpdateDrawMode(_params->drawMode);
        }
        if (keyPress.keyBase == '2') {
            _params->drawMode = 1;
            m_appPtr->UpdateDrawMode(_params->drawMode);
        }
        if (keyPress.keyBase == '3') {
            _params->drawMode = 2;
            m_appPtr->UpdateDrawMode(_params->drawMode);
        }
        if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
            _params->showGui = !_params->showGui;
        }
    }

  private:
    vsg::observer_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<Params> _params;
    VSGApp* m_appPtr;
};

class MyGuiComponent {
  public:
    MyGuiComponent(vsg::ref_ptr<Params> params, VSGApp* appPtr) : _params(params), m_appPtr(appPtr) {}

    // Example here taken from the Dear imgui comments (mostly)
    bool operator()() {
        bool visibleComponents = false;
        ImGuiIO& io = ImGui::GetIO();
#ifdef __APPLE__
        io.FontGlobalScale = 2.0;
#else
        io.FontGlobalScale = 1.0;
#endif

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
        if (_params->showGui) {
            ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
            ImGui::Begin("Chrono::VSG GUI");  // Create a window called "Hello, world!" and append into it.

            ImGui::Text("3D Draw Mode (tbd): ");
            ImGui::SameLine();

            if (ImGui::RadioButton("Filled", &_params->drawMode, 0)) {
                m_appPtr->UpdateDrawMode(_params->drawMode);
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Wireframe", &_params->drawMode, 1)) {
                m_appPtr->UpdateDrawMode(_params->drawMode);
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Body CoG Dots", &_params->drawMode, 2)) {
                m_appPtr->UpdateDrawMode(_params->drawMode);
            }

            if (ImGui::Button(
                    "Quit"))  // Buttons return true when clicked (most widgets return true when edited/activated)
                m_appPtr->Quit();

            ImGui::End();
            visibleComponents = true;
        }

        return visibleComponents;
    }

  private:
    vsg::ref_ptr<Params> _params;
    VSGApp* m_appPtr;
};

VSGApp::VSGApp() {
    m_up_vector = vsg::dvec3(0, 0, 1);
}

VSGApp::~VSGApp() {}

bool VSGApp::Initialize(int windowWidth, int windowHeight, const char* windowTitle, ChSystem* system) {
    m_system = system;

    auto options = vsg::Options::create();
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    options->objectCache = vsg::ObjectCache::create();

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = windowTitle;
    windowTraits->width = windowWidth;
    windowTraits->height = windowHeight;

    // create the viewer and assign window(s) to it
    m_viewer = vsg::Viewer::create();

    m_window = vsg::Window::create(windowTraits);
    if (!m_window) {
        std::cout << "Could not create windows." << std::endl;
        return 1;
    }

    m_builder = vsg::Builder::create();
    m_builder->options = options;

    m_viewer->addWindow(m_window);

    // holds whole 3d stuff
    m_scenegraph = vsg::Group::create();

    // subparts, only one active, for easy access
    m_dot_subgraph = vsg::Switch::create();
    m_line_subgraph = vsg::Switch::create();
    m_polygon_subgraph = vsg::Switch::create();

    // add switch nodes to m_scenegraph
    m_scenegraph->addChild(m_dot_subgraph);
    m_scenegraph->addChild(m_line_subgraph);
    m_scenegraph->addChild(m_polygon_subgraph);

    // use chrono multibody system the generate 3d info
    BuildSceneGraph();

    // compute the bounds of the scene graph to help position camera
    vsg::ComputeBounds computeBounds;
    m_scenegraph->accept(computeBounds);
    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;

    // These are set statically because the geometry in the class is expanded in the shader
    double nearFarRatio = 0.01;

    // set up the camera
    m_lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

    m_perspective = vsg::Perspective::create(
        30.0, static_cast<double>(m_window->extent2D().width) / static_cast<double>(m_window->extent2D().height),
        nearFarRatio * radius, radius * 400.5);

    m_camera = vsg::Camera::create(m_perspective, m_lookAt, vsg::ViewportState::create(m_window->extent2D()));

    // The commandGraph will contain a 2 stage renderGraph 1) 3D scene 2) ImGui (by default also includes clear depth
    // buffers)
    m_commandGraph = vsg::CommandGraph::create(m_window);
    m_renderGraph = vsg::RenderGraph::create(m_window);
    m_commandGraph->addChild(m_renderGraph);

    // create the normal 3D view of the scene
    m_renderGraph->addChild(vsg::View::create(m_camera, m_scenegraph));

    // Create the ImGui node and add it to the renderGraph
    auto params = Params::create();
    m_renderGraph->addChild(vsgImGui::RenderImGui::create(m_window, MyGuiComponent(params, this)));

    // Add the ImGui event handler first to handle events early
    m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

    auto kbHandler = AppKeyboardHandler::create(m_viewer);
    kbHandler->SetParams(params, this);
    m_viewer->addEventHandler(kbHandler);

    // add close handler to respond the close window button and pressing escape
    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));

    m_viewer->addEventHandler(vsg::Trackball::create(m_camera));

    m_viewer->assignRecordAndSubmitTaskAndPresentation({m_commandGraph});

    m_viewer->compile();

    return true;
}

void VSGApp::BuildSceneGraph() {
    // look for bodies
    size_t numBodies = m_system->Get_bodylist().size();
    GetLog() << "Bodies found = " << numBodies << "\n";
    for (auto body : m_system->Get_bodylist()) {
        GetLog() << "processing body " << body.get()->GetId() << "\n";
        // position of the body
        const Vector pos = body->GetFrame_REF_to_abs().GetPos();
        // rotation of the body
        Quaternion rot = body->GetFrame_REF_to_abs().GetRot();
        double angle;
        Vector axis;
        rot.Q_to_AngAxis(angle, axis);
        for (int i = 0; i < body->GetAssets().size(); i++) {
            auto asset = body->GetAssets().at(i);

            if (!std::dynamic_pointer_cast<ChVisualization>(asset)) {
                continue;
            }

            GetLog() << "  processing asset# " << i << "\n";
            ChVisualization* visual_asset = ((ChVisualization*)(asset.get()));
            // position of the asset
            Vector center = visual_asset->Pos;
            // rotate asset pos into global frame
            center = rot.Rotate(center);
            // Get the local rotation of the asset
            Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
            // add the local rotation to the rotation of the body
            lrot = rot % lrot;
            lrot.Normalize();
            lrot.Q_to_AngAxis(angle, axis);
            if (ChSphereShape* sphere_shape = dynamic_cast<ChSphereShape*>(asset.get())) {
                double radius = sphere_shape->GetSphereGeometry().rad;
                ChVector<> pos_final = pos + center;
                /*
                 model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
                 model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
                 model = glm::scale(model, glm::vec3(radius, radius, radius));
                 model_sphere.push_back(model);
                 */
                vsg::GeometryInfo geomInfo;
                geomInfo.dx.set(radius, 0.0f, 0.0f);
                geomInfo.dy.set(0.0f, radius, 0.0f);
                geomInfo.dz.set(0.0f, 0.0f, radius);
                geomInfo.position = vsg::vec3(pos_final.x(), pos_final.y(), pos_final.z());
                geomInfo.transform = vsg::rotate(angle, axis.x(), axis.y(), axis.z());
                vsg::StateInfo stateInfo;

                stateInfo.wireframe = false;
                stateInfo.lighting = true;
                m_polygon_subgraph->addChild(true, m_builder->createSphere(geomInfo, stateInfo));

            } else if (ChEllipsoidShape* ellipsoid_shape = dynamic_cast<ChEllipsoidShape*>(asset.get())) {
                Vector radius = ellipsoid_shape->GetEllipsoidGeometry().rad;
                ChVector<> pos_final = pos + center;
                /*
                 model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
                 model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
                 model = glm::scale(model, glm::vec3(radius.x(), radius.y(), radius.z()));
                 model_sphere.push_back(model);
                 */
                vsg::GeometryInfo geomInfo;
                geomInfo.dx.set(radius.x(), 0.0f, 0.0f);
                geomInfo.dy.set(0.0f, radius.y(), 0.0f);
                geomInfo.dz.set(0.0f, 0.0f, radius.z());
                geomInfo.position = vsg::vec3(pos_final.x(), pos_final.y(), pos_final.z());
                geomInfo.transform = vsg::rotate(angle, axis.x(), axis.y(), axis.z());
                vsg::StateInfo stateInfo;

                stateInfo.wireframe = false;
                stateInfo.lighting = true;
                m_polygon_subgraph->addChild(true, m_builder->createSphere(geomInfo, stateInfo));
            } else if (ChBoxShape* box_shape = dynamic_cast<ChBoxShape*>(asset.get())) {
                ChVector<> pos_final = pos + center;
                Vector radius = box_shape->GetBoxGeometry().Size;
                /*
                 model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
                 model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
                 model = glm::scale(model, glm::vec3(radius.x(), radius.y(), radius.z()));
                 model_box.push_back(model);
                 */
                vsg::GeometryInfo geomInfo;
                geomInfo.dx.set(radius.x(), 0.0f, 0.0f);
                geomInfo.dy.set(0.0f, radius.y(), 0.0f);
                geomInfo.dz.set(0.0f, 0.0f, radius.z());
                geomInfo.position = vsg::vec3(pos_final.x(), pos_final.y(), pos_final.z());
                geomInfo.transform = vsg::rotate(angle, axis.x(), axis.y(), axis.z());
                vsg::StateInfo stateInfo;

                stateInfo.wireframe = false;
                stateInfo.lighting = true;
                m_polygon_subgraph->addChild(true, m_builder->createBox(geomInfo, stateInfo));
            } else if (ChCylinderShape* cylinder_shape = dynamic_cast<ChCylinderShape*>(asset.get())) {
                double rad = cylinder_shape->GetCylinderGeometry().rad;
                ChVector<> dir = cylinder_shape->GetCylinderGeometry().p2 - cylinder_shape->GetCylinderGeometry().p1;
                double height = dir.Length();
                dir.Normalize();
                ChVector<> mx, my, mz;
                dir.DirToDxDyDz(my, mz, mx);  // y is axis, in cylinder.obj frame
                ChMatrix33<> mrot;
                mrot.Set_A_axis(mx, my, mz);
                lrot = rot % (visual_asset->Rot.Get_A_quaternion() % mrot.Get_A_quaternion());
                // position of cylinder based on two points
                ChVector<> mpos =
                    0.5 * (cylinder_shape->GetCylinderGeometry().p2 + cylinder_shape->GetCylinderGeometry().p1);

                lrot.Q_to_AngAxis(angle, axis);
                ChVector<> pos_final = pos + rot.Rotate(mpos);
                /*
                 model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
                 model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
                 model = glm::scale(model, glm::vec3(rad, height * .5, rad));
                 model_cylinder.push_back(model);
                 */
                vsg::GeometryInfo geomInfo;
                geomInfo.dx.set(rad, 0.0f, 0.0f);
                geomInfo.dy.set(0.0f, 0.5 * height, 0.0f);
                geomInfo.dz.set(0.0f, 0.0f, rad);
                geomInfo.position = vsg::vec3(pos_final.x(), pos_final.y(), pos_final.z());
                geomInfo.transform =
                    vsg::rotate(CH_C_PI_2, 0.0, 0.0, 1.0) * vsg::rotate(angle, axis.x(), axis.y(), axis.z());
                vsg::StateInfo stateInfo;

                stateInfo.wireframe = false;
                stateInfo.lighting = true;
                m_polygon_subgraph->addChild(true, m_builder->createCylinder(geomInfo, stateInfo));
            } else if (ChConeShape* cone_shape = dynamic_cast<ChConeShape*>(asset.get())) {
                Vector rad = cone_shape->GetConeGeometry().rad;
                ChVector<> pos_final = pos + center;
                /*
                 model = glm::translate(glm::mat4(1), glm::vec3(pos_final.x(), pos_final.y(), pos_final.z()));
                 model = glm::rotate(model, float(angle), glm::vec3(axis.x(), axis.y(), axis.z()));
                 model = glm::scale(model, glm::vec3(rad.x(), rad.y(), rad.z()));
                 model_cone.push_back(model);
                 */
                vsg::GeometryInfo geomInfo;
                geomInfo.dx.set(rad.x(), 0.0f, 0.0f);
                geomInfo.dy.set(0.0f, rad.z(), 0.0f);
                geomInfo.dz.set(0.0f, 0.0f, rad.y());
                geomInfo.position = vsg::vec3(pos_final.x(), pos_final.z(), pos_final.y());
                geomInfo.transform =
                    vsg::rotate(-CH_C_PI_2, 1.0, 0.0, 0.0) * vsg::rotate(angle, axis.x(), axis.y(), axis.z());
                vsg::StateInfo stateInfo;

                stateInfo.wireframe = false;
                stateInfo.lighting = true;
                m_polygon_subgraph->addChild(true, m_builder->createCone(geomInfo, stateInfo));
            }
        }
    }
}

void VSGApp::UpdateSceneGraph() {}

void VSGApp::Render() {
    m_viewer->handleEvents();
    m_viewer->update();
    m_viewer->recordAndSubmit();
    // adjust draw mode, if necessary
    if (m_drawModeChanged) {
        switch (m_drawMode) {
            case 0:
                m_dot_subgraph->setAllChildren(false);
                m_line_subgraph->setAllChildren(false);
                m_polygon_subgraph->setAllChildren(true);
                break;
            case 1:
                m_dot_subgraph->setAllChildren(false);
                m_line_subgraph->setAllChildren(true);
                m_polygon_subgraph->setAllChildren(false);
                break;
            case 2:
                m_dot_subgraph->setAllChildren(true);
                m_line_subgraph->setAllChildren(false);
                m_polygon_subgraph->setAllChildren(false);
                break;
        }
        m_drawModeChanged = false;
    }
    m_viewer->present();
}
