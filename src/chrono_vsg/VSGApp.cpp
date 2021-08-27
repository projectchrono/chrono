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

class AppKeyboardHandler : public vsg::Inherit<vsg::Visitor, AppKeyboardHandler>
{
public:
    AppKeyboardHandler(vsg::Viewer* viewer)
        : m_viewer(viewer)
    {
    }
    void SetParams(vsg::ref_ptr<Params> params, VSGApp* appPtr) {
        _params = params;
        m_appPtr = appPtr;
    }
    void apply(vsg::KeyPressEvent& keyPress) override
    {
        if(keyPress.keyBase == '1') {
            _params->drawMode = 0;
            m_appPtr->UpdateDrawMode(_params->drawMode);
        }
        if(keyPress.keyBase == '2') {
            _params->drawMode = 1;
            m_appPtr->UpdateDrawMode(_params->drawMode);
        }
        if(keyPress.keyBase == '3') {
            _params->drawMode = 2;
            m_appPtr->UpdateDrawMode(_params->drawMode);
        }
        if(keyPress.keyBase == 'm') {
            std::cout << "Key 'm' hit" << std::endl;
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
    MyGuiComponent(vsg::ref_ptr<Params> params, VSGApp* appPtr) : _params(params), m_appPtr(appPtr) {
    }
    
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
            ImGui::SetNextWindowSize(ImVec2(0.0f,0.0f));
            ImGui::Begin("Chrono::VSG GUI");  // Create a window called "Hello, world!" and append into it.

            ImGui::Text("3D Draw Mode (tbd): ");
            ImGui::SameLine();
            
            if(ImGui::RadioButton("Filled", &_params->drawMode, 0)) {
                m_appPtr->UpdateDrawMode(_params->drawMode);
            }
            ImGui::SameLine();
            if(ImGui::RadioButton("Wireframe", &_params->drawMode, 1)) {
                m_appPtr->UpdateDrawMode(_params->drawMode);
            }
            ImGui::SameLine();
            if(ImGui::RadioButton("Body CoG Dots", &_params->drawMode, 2)) {
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
    m_renderGraph->addChild(vsgImGui::RenderImGui::create(m_window, MyGuiComponent(params,this)));

    // Add the ImGui event handler first to handle events early
    m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());
    
    auto kbHandler = AppKeyboardHandler::create(m_viewer);
    kbHandler->SetParams(params,this);
    m_viewer->addEventHandler(kbHandler);

    // add close handler to respond the close window button and pressing escape
    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));

    m_viewer->addEventHandler(vsg::Trackball::create(m_camera));

    m_viewer->assignRecordAndSubmitTaskAndPresentation({m_commandGraph});

    m_viewer->compile();

    return true;
}

void VSGApp::BuildSceneGraph() {}

void VSGApp::UpdateSceneGraph() {}

void VSGApp::Render() {
    m_viewer->handleEvents();
    m_viewer->update();
    // adjust draw mode, if necessary
    if(m_drawModeChanged) {
    switch(m_drawMode) {
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
    m_viewer->recordAndSubmit();
    m_viewer->present();
}
