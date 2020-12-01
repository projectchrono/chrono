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

#ifndef CH_VSG_CHRONO_APP_H
#define CH_VSG_CHRONO_APP_H

#include <iostream>
#include "chrono_vsg/core/ChApiVSG.h"
#include "chrono_vsg/resources/ChVSGSettings.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChVector.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

/// @addtogroup vsg_module
/// @{

class CH_VSG_API ChVSGChronoApp {
  public:
    ChVSGChronoApp(ChSystem* sys, std::string& windowTitle, size_t windowWidth = 800, size_t windowHeight = 600);
    ~ChVSGChronoApp();
    ChSystem* GetSystem() { return m_system; }
    bool OpenWindow();
    vsg::ref_ptr<vsg::Viewer> GetViewer() { return m_viewer; }
    void Render();
    void GenerateText();

  private:
    void BuildSceneGraph();
    vsg::ref_ptr<vsg::Camera> createMainCamera(int32_t x, int32_t y, uint32_t width, uint32_t height);
    vsg::ref_ptr<vsg::Camera> createTextCamera(int32_t x, int32_t y, uint32_t width, uint32_t height);

    bool m_build_graph;

    ChSystem* m_system;

    ChColor m_clearColor;

    vsg::ref_ptr<vsg::WindowTraits> m_windowTraits;

    vsg::ref_ptr<vsg::Viewer> m_viewer;

    vsg::ref_ptr<vsg::Window> m_window;

    vsg::Paths m_searchPaths;

    std::string m_titleFontFilename;
    vsg::ref_ptr<vsg::Font> m_titleFont;

    std::string m_infoFontFilename;
    vsg::ref_ptr<vsg::Font> m_infoFont;

    vsg::ref_ptr<vsg::Group> m_scenegraph;
    vsg::ref_ptr<vsg::Camera> m_mainCamera;

    vsg::ref_ptr<vsg::Group> m_scenegraphText;
    vsg::ref_ptr<vsg::Camera> m_textCamera;

    vsg::time_point m_start_point;
    size_t m_frameCount = 0;
};

}  // namespace vsg3d
}  // namespace chrono
#endif
