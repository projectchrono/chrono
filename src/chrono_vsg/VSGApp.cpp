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

#include <iostream>

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>

#include "chrono_vsg/VSGApp.h"
#include "chrono_vsg/shapes/VSGAsys.h"
#include "chrono_vsg/shapes/VSGBsys.h"
#include "chrono_vsg/shapes/VSGCsys.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCapsuleShape.h"

namespace chrono {
namespace vsg3d {

struct Params : public vsg::Inherit<vsg::Object, Params> {
    bool showGui = true;  // you can toggle this with your own EventHandler and key
};

class AppKeyboardHandler : public vsg::Inherit<vsg::Visitor, AppKeyboardHandler> {
  public:
    AppKeyboardHandler(vsg::Viewer* viewer) : m_viewer(viewer) {}

    void SetParams(vsg::ref_ptr<Params> params, VSGApp* appPtr) {
        _params = params;
        m_appPtr = appPtr;
    }

    void apply(vsg::KeyPressEvent& keyPress) override {
        if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
            _params->showGui = !_params->showGui;
        }
        if (keyPress.keyBase == 'q' || keyPress.keyModified == 'q') {
            m_appPtr->Quit();
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
            ImGui::Begin("App:");  // Create a window called "Hello, world!" and append into it.

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

VSGApp::VSGApp() : m_up_vector(vsg::dvec3(0, 0, 1)), m_system(nullptr) {}

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

    // enable transfer from the colour and depth buffer images
    windowTraits->swapchainPreferences.imageUsage =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    windowTraits->depthImageUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    // create the viewer and assign window(s) to it
    m_viewer = vsg::Viewer::create();

    m_window = vsg::Window::create(windowTraits);
    if (!m_window) {
        std::cout << "Could not create window." << std::endl;
        return false;
    }

    auto device = m_window->getOrCreateDevice();
    if (!device) {
        std::cout << "Could not create device." << std::endl;
        return false;
    }
    m_viewer->addWindow(m_window);

    // holds whole 3d stuff
    m_scenegraph = vsg::Group::create();

    // add ambient light
    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(1.0, 1.0, 1.0);
    ambientLight->intensity = 0.1;

    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "head light";
    directionalLight->color.set(1.0, 1.0, 1.0);
    directionalLight->intensity = 0.9;
    directionalLight->direction.set(0.0, 0.0, -1.0);

    auto absoluteTransform = vsg::AbsoluteTransform::create();
    absoluteTransform->addChild(ambientLight);
    absoluteTransform->addChild(directionalLight);

    m_scenegraph->addChild(absoluteTransform);

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

void VSGApp::Render() {
    m_viewer->handleEvents();
    m_viewer->update();
    m_viewer->recordAndSubmit();
    m_viewer->present();
}

void VSGApp::Quit() {
    m_viewer->close();
}

}  // namespace vsg3d
}  // namespace chrono