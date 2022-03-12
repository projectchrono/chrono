// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban, Rainer Gericke
// =============================================================================

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>
#include "chrono_vsg/tools/createSkybox.h"
#include "ChVisualSystemVSG.h"

using namespace std;
namespace chrono {
namespace vsg3d {

class AppKeyboardHandler : public vsg::Inherit<vsg::Visitor, AppKeyboardHandler> {
  public:
    AppKeyboardHandler(vsg::Viewer* viewer) : m_viewer(viewer) {}

    void SetParams(vsg::ref_ptr<ChVisualSystemVSG::StateParams> params, ChVisualSystemVSG* appPtr) {
        _params = params;
        m_appPtr = appPtr;
    }

    void apply(vsg::KeyPressEvent& keyPress) override {
        if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
            // toggle graphical menu
            _params->showGui = !_params->showGui;
        }
        if (keyPress.keyBase == 't' || keyPress.keyModified == 't') {
            // terminate process
            m_appPtr->Quit();
        }
    }

  private:
    vsg::observer_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> _params;
    ChVisualSystemVSG* m_appPtr;
};

class GuiComponent {
  public:
    GuiComponent(vsg::ref_ptr<ChVisualSystemVSG::StateParams> params, ChVisualSystemVSG* appPtr)
        : _params(params), m_appPtr(appPtr) {}

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
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> _params;
    ChVisualSystemVSG* m_appPtr;
};

ChVisualSystemVSG::ChVisualSystemVSG() {
    m_options = vsg::Options::create();
    m_options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    m_options->paths.push_back(GetChronoDataPath());
    m_options->objectCache = vsg::ObjectCache::create();
#ifdef vsgXchange_all
    // add vsgXchange's support for reading and writing 3rd party file formats
    m_options->add(vsgXchange::all::create());
#endif
    m_options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
}

ChVisualSystemVSG::~ChVisualSystemVSG() {}

void ChVisualSystemVSG::Initialize() {
    m_windowTraits = vsg::WindowTraits::create();
    m_windowTraits->windowTitle = m_windowTitle;
    m_windowTraits->width = m_windowWidth;
    m_windowTraits->height = m_windowHeight;
    m_windowTraits->deviceExtensionNames = {VK_KHR_MULTIVIEW_EXTENSION_NAME, VK_KHR_MAINTENANCE2_EXTENSION_NAME,
                                            VK_KHR_CREATE_RENDERPASS_2_EXTENSION_NAME,
                                            VK_KHR_DEPTH_STENCIL_RESOLVE_EXTENSION_NAME};

    // enable transfer from the colour and depth buffer images
    m_windowTraits->swapchainPreferences.imageUsage =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    m_windowTraits->depthImageUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    // create the viewer and assign window(s) to it
    m_viewer = vsg::Viewer::create();

    m_window = vsg::Window::create(m_windowTraits);
    if (!m_window) {
        std::cout << "Could not create window." << std::endl;
        return;
    }

    auto device = m_window->getOrCreateDevice();
    if (!device) {
        std::cout << "Could not create device." << std::endl;
        return;
    }
    m_viewer->addWindow(m_window);

    // holds whole 3d stuff
    m_scenegraph = vsg::Group::create();
    if (m_use_skybox) {
        // build node from cubemap texture file
        if (!m_skyboxFilename.empty()) {
            if (auto node = createSkybox(m_skyboxFilename, m_options); node) {
                m_scenegraph->addChild(node);
            } else {
                cout << "Couldn't load " << m_skyboxFilename << endl;
            }
        }
    }


    // compute the bounds of the scene graph to help position camera
    vsg::ComputeBounds computeBounds;
    m_scenegraph->accept(computeBounds);
    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;

    // These are set statically because the geometry in the class is expanded in the shader
    double nearFarRatio = 0.01;

    // set up the camera
    m_lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, m_up_vector);
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

    // Imgui graphical menu handler
    m_renderGraph->addChild(vsgImGui::RenderImGui::create(m_window, GuiComponent(m_params, this)));
    // Add the ImGui event handler first to handle events early
    m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

    // add keyboard handler
    auto kbHandler = AppKeyboardHandler::create(m_viewer);
    kbHandler->SetParams(m_params, this);
    m_viewer->addEventHandler(kbHandler);

    // add close handler to respond the close window button and pressing escape
    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));
    m_viewer->addEventHandler(vsg::Trackball::create(m_camera));

    m_viewer->assignRecordAndSubmitTaskAndPresentation({m_commandGraph});

    m_viewer->compile();
}

void ChVisualSystemVSG::Render() {
    m_viewer->handleEvents();
    m_viewer->update();
    m_viewer->recordAndSubmit();
    m_viewer->present();
}

bool ChVisualSystemVSG::Run() {
    return m_viewer->advanceToNextFrame();
}

void ChVisualSystemVSG::Quit() {
    m_viewer->close();
}

void ChVisualSystemVSG::SetWindowSize(const ChVector2<int>& win_size) {
    m_windowWidth = win_size[0];
    m_windowHeight = win_size[1];
}

void ChVisualSystemVSG::SetWindowTitle(const std::string& win_title) {
    m_windowTitle = string(win_title);
}

}  // namespace vsg3d
}  // namespace chrono