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

    /// Set the window title (default "").
    /// Must be called before Initialize().
    void SetWindowTitle(const std::string& win_title);

    // activate skybox (converted from Irrlicht skybox)
    void SetUseSkyBox(bool yesno=false) { m_use_skybox = yesno;}

    /// Add a logo in a 3D scene.
    void AddLogo(std::string logoName);

    /// Use Z-up camera rendering (default CameraVerticalDir::Z).
    /// Must be called before Initialize().
    void SetCameraVertical(CameraVerticalDir vert);

    // Set Clear Color
    /// Must be called before Initialize().
    void SetClearColor(ChColor cc);

    // renders the whole scene
    void Render();

    // still running?
    bool Run();

    // terminate
    void Quit();

    /// Create a snapshot of the last rendered frame and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) override;

    struct StateParams : public vsg::Inherit<vsg::Object, StateParams> {
        bool showGui = true;  // (don't) show the imgui menu
        bool do_image_capture = false; // mark image capturing as needed
    };

private:
    std::string m_windowTitle = "VSG Visual System";
    int m_windowWidth = 640;
    int m_windowHeight = 480;
    //
    bool m_initialized = false;
    vsg::ref_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<vsg::Window> m_window;
    vsg::ref_ptr<vsg::Options> m_options;
    vsg::ref_ptr<vsg::WindowTraits> m_windowTraits;
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> m_params = StateParams::create();
    std::string m_imageFilename;
    //
    bool m_use_skybox = false;
    std::string m_skyboxFilename = "vsg/textures/chrono_skybox.ktx2";
    std::string m_logo_fileName;
    ChColor m_bg_color = ChColor(1,1,1,1);
    //
    vsg::ref_ptr<vsg::Group> m_scenegraph;
    //
    vsg::ref_ptr<vsg::LookAt> m_lookAt;
    vsg::ref_ptr<vsg::Camera> m_camera;
    vsg::ref_ptr<vsg::ProjectionMatrix> m_perspective;
    vsg::dvec3 m_eye_point = vsg::dvec3(-10,-5,1);
    vsg::dvec3 m_center_point = vsg::dvec3(0,0,0);
    vsg::dvec3 m_up_vector = vsg::dvec3(0,0,1);
    bool m_yup = false;
    //
    vsg::ref_ptr<vsg::CommandGraph> m_commandGraph;
    vsg::ref_ptr<vsg::RenderGraph> m_renderGraph;
    //
    void export_image();
};
}  // namespace vsg3d
}  // namespace chrono
#endif
