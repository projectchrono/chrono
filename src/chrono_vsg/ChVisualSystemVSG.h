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

#ifndef CH_VISUAL_SYSTEM_VSG_H
#define CH_VISUAL_SYSTEM_VSG_H

#include <iostream>
#include <string>
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono/assets/ChVisualSystem.h"

namespace chrono {
namespace vsg3d {

/// Vertical direction
enum class CameraVerticalDir { Y, Z };

class CH_VSG_API ChVisualSystemVSG : virtual public ChVisualSystem {
  public:
    ChVisualSystemVSG();
    ~ChVisualSystemVSG();
    void Initialize();

    /// Set the window size (default 640x480).
    /// Must be called before Initialize().
    void SetWindowSize(const ChVector2<int>& win_size);

    /// Set the windoiw title (default "").
    /// Must be called before Initialize().
    void SetWindowTitle(const std::string& win_title);

    // activate skybox (converted from Irrlicht skybox)
    void SetUseSkyBox(bool yesno=false) { m_use_skybox = yesno;}

    // renders the whole scene
    void Render();

    // still running?
    bool Run();

    // terminate
    void Quit();

    struct StateParams : public vsg::Inherit<vsg::Object, StateParams> {
        bool showGui = true;  // (don't) show the imgui menu
    };

private:
    std::string m_windowTitle = "VSG Visual System";
    int m_windowWidth = 640;
    int m_windowHeight = 480;
    //
    vsg::ref_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<vsg::Window> m_window;
    vsg::ref_ptr<vsg::Options> m_options;
    vsg::ref_ptr<vsg::WindowTraits> m_windowTraits;
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> m_params = StateParams::create();
    //
    bool m_use_skybox = false;
    std::string m_skyboxFilename = "vsg/textures/chrono_skybox.ktx2"; // "vsg/models/chrono_sky.vsgb";

    //
    vsg::ref_ptr<vsg::Group> m_scenegraph;
    //
    vsg::ref_ptr<vsg::LookAt> m_lookAt;
    vsg::ref_ptr<vsg::Camera> m_camera;
    vsg::ref_ptr<vsg::ProjectionMatrix> m_perspective;
    vsg::dvec3 m_up_vector = vsg::dvec3(0,0,1);
    //
    vsg::ref_ptr<vsg::CommandGraph> m_commandGraph;
    vsg::ref_ptr<vsg::RenderGraph> m_renderGraph;

};
}  // namespace vsg3d
}  // namespace chrono
#endif
