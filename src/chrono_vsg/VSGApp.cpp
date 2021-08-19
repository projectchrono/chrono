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
    bool showDemoWindow = false;
    bool showSecondWindow = false;
    float clearColor[3]{0.2f, 0.2f, 0.4f};  // Unfortunately, this doesn't change dynamically in vsg
    uint32_t counter = 0;
    float dist = 0.f;
};

class MyGuiComponent {
public:
    MyGuiComponent(vsg::ref_ptr<Params> params) : _params(params) {
    }

    // Example here taken from the Dear imgui comments (mostly)
    bool operator()() {

        bool visibleComponents = false;
        ImGuiIO& io = ImGui::GetIO();
        io.FontGlobalScale = 2;
        
        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
        if (_params->showGui) {
            ImGui::Begin("Hello, world!");  // Create a window called "Hello, world!" and append into it.

            ImGui::Text("Some useful message here.");  // Display some text (you can use a format strings too)
            ImGui::Checkbox("Demo Window", &_params->showDemoWindow);  // Edit bools storing our window open/close state
            ImGui::Checkbox("Another Window", &_params->showSecondWindow);
            ImGui::SliderFloat("float", &_params->dist, 0.0f, 1.0f);  // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::ColorEdit3("clear color", (float*)&_params->clearColor);  // Edit 3 floats representing a color

            if (ImGui::Button(
                    "Button"))  // Buttons return true when clicked (most widgets return true when edited/activated)
                _params->counter++;

            ImGui::SameLine();
            ImGui::Text("counter = %d", _params->counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                        ImGui::GetIO().Framerate);
            ImGui::End();
            visibleComponents = true;
        }

        // 3. Show another simple window.
        if (_params->showSecondWindow) {
            ImGui::Begin("Another Window",
                         &_params->showSecondWindow);  // Pass a pointer to our bool variable (the window will have a
                                                       // closing button that will clear the bool when clicked)
            ImGui::Text("Hello from another window!");
            if (ImGui::Button("Close Me"))
                _params->showSecondWindow = false;
            ImGui::End();

            visibleComponents = true;
        }

        if (_params->showDemoWindow) {
            ImGui::ShowDemoWindow(&_params->showDemoWindow);

            visibleComponents = true;
        }

        return visibleComponents;
    }

  private:
    vsg::ref_ptr<Params> _params;
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
    m_renderGraph->addChild(vsgImGui::RenderImGui::create(m_window, MyGuiComponent(params)));

    // Add the ImGui event handler first to handle events early
    m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

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
    m_viewer->recordAndSubmit();
    m_viewer->present();
}
