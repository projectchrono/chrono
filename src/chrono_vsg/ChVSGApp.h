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
// Vulkan Scene Graph viewer, this class will hopefully draw the system to the
// screen and handle input some day
// =============================================================================

#ifndef CH_VSG_APP_H
#define CH_VSG_APP_H

#include <iostream>
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono_vsg/resources/ChVSGSettings.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChVector.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono_vsg/tools/ChVSGScreenshotHandler.h"

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

/// @addtogroup vsg_module
/// @{

class CH_VSG_API ChVSGApp {
  public:
    ChVSGApp();
    ~ChVSGApp();
    vsg::ref_ptr<vsg::Viewer> GetViewer() { return m_viewer; }
    bool Initialize(int windowWidth, int windowHeight, const char* windowTitle, ChSystem* system);
    void Render();
    void setClearColor(float r, float g, float b, float a) {
        m_clearColor[0] = r;
        m_clearColor[1] = g;
        m_clearColor[2] = b;
        m_clearColor[3] = a;
    }
    void setClearColor(float r, float g, float b) { setClearColor(r, g, b, 1.0); }
    void setTimeStep(double tStep) { m_timeStep = tStep; }
    void setOutputStep(double outStep) { m_outputStep = outStep; }
    void setUpVector(ChVector<> up);

    void setupTexPool(vsg::ref_ptr<vsg::Window> window, vsg::ViewportState* viewport, uint32_t maxNumTextures = 32);
    void compile(vsg::ref_ptr<vsg::Node> subgraph);

    void doTimeStep();
    void Quit() { m_viewer->close(); }

  protected:
    void BuildSceneGraph();
    void UpdateSceneGraph();
    vsg::ref_ptr<vsg::MatrixTransform> GetTransform(std::shared_ptr<ChBody> body, std::shared_ptr<ChAsset> asset);
    void IncreaseWaitCounter();
    vsg::ref_ptr<vsg::RenderPass> createRenderPassCompatibleWithReadingDepthBuffer(vsg::Device* device,
                                                                                   VkFormat imageFormat,
                                                                                   VkFormat depthFormat);

  private:
    uint32_t _allocatedTextureCount = 0;
    uint32_t _maxNumTextures = 0;
    vsg::ref_ptr<vsg::CompileTraversal> _compile;

    ChSystem* m_system;

    double m_timeStep;
    double m_outputStep;

    DrawMode m_drawMode;

    bool m_build_graph;

    float m_horizonMountainHeight;

    vsg::ref_ptr<vsg::WindowTraits> m_windowTraits;

    vsg::Paths m_searchPaths;

    std::string m_fontFilename;
    vsg::ref_ptr<vsg::Font> m_font;

    vsg::ref_ptr<vsg::Group> m_scenegraph;

    vsg::ref_ptr<vsg::Viewer> m_viewer;

    vsg::ref_ptr<vsg::Window> m_window;

    vsg::ref_ptr<ChVSGScreenshotHandler> m_screenshotHandler;

    float m_clearColor[4];

    vsg::vec3 m_light_position;  // for Blinn-Phong

    vsg::dvec3 m_up_vector;

    size_t m_wait_counter;
    size_t m_wait_counter_max;
};

}  // namespace vsg3d
}  // namespace chrono
#endif
