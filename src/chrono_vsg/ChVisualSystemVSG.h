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
#include "chrono/assets/ChVisualModel.h"

#include "tools/createSkybox.h"
#include "shapes/ShapeBuilder.h"

namespace chrono {
namespace vsg3d {

/// Vertical direction
enum class CameraVerticalDir { Y, Z };

class CH_VSG_API ChVisualSystemVSG : virtual public ChVisualSystem {
  public:
    ChVisualSystemVSG();
    ~ChVisualSystemVSG();

    void Initialize();
    void Render();
    bool Run();
    // terminate
    void Quit();
    void WriteImageToFile(const std::string& filename) override;
    void SetWindowSize(ChVector2<int> size);
    void SetWindowPosition(ChVector2<int> pos);
    void SetWindowTitle(std::string title);
    void SetClearColor(ChColor color);
    void SetUseSkyBox(bool yesno);
    // Draw scene as wireframes
    void SetWireFrameMode(bool mode = true) { m_draw_as_wireframe = mode; }
    void SetCameraVertical(chrono::vsg3d::CameraVerticalDir upDir);
    void AddCamera(const ChVector<>& pos, ChVector<> targ = VNULL);
    void SetLightIntensity(double intensity) { m_lightIntensity = ChClamp(intensity, 0.0, 1.0); }
    void SetLightDirection(double acimut, double elevation);
    void SetCameraAngleDeg(double angleDeg) { m_cameraAngleDeg = angleDeg; }
    void ShowAllCoGs(double size);
    void SetGuiFontSize(float theSize = 20.f) { m_guiFontSize = theSize; }
    void SetDecoGrid(double ustep, double vstep, int nu, int nv, ChCoordsys<> pos, ChColor col);
    void BindAll() override;
    void OnUpdate(ChSystem* sys) override;

    struct StateParams : public vsg::Inherit<vsg::Object, StateParams> {
        bool showGui = true;            // (don't) show the imgui menu, actually unused
        bool do_image_capture = false;  // mark image capturing as needed
        double cogSymbolSize = 0.0;
    };

  private:
    vsg::ref_ptr<vsg::Viewer> m_viewer;
    vsg::ref_ptr<vsg::Window> m_window;
    int m_windowWidth = 800;
    int m_windowHeight = 600;
    int m_windowX = 0;
    int m_windowY = 0;
    std::string m_windowTitle;
    ChColor m_clearColor;
    //
    vsg::ref_ptr<vsg::Options> m_options;
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> m_params = StateParams::create();
    int m_numThreads = 16;
    vsg::ref_ptr<vsg::OperationThreads> m_loadThreads;
    //
    //  m_scene +- skybox, lights +- m_bodyScene
    //                            |
    //                            +- m_cogScene
    //                            |
    //                            +- m_linkScene
    //                            |
    //                            +- m_particleScene
    //                            |
    //                            +- m_decoScene
    vsg::ref_ptr<vsg::Group> m_scene;
    vsg::ref_ptr<vsg::Group> m_bodyScene;
    vsg::ref_ptr<vsg::Group> m_cogScene;
    vsg::ref_ptr<vsg::Group> m_linkScene;
    vsg::ref_ptr<vsg::Group> m_particleScene;
    vsg::ref_ptr<vsg::Group> m_decoScene;
    std::string m_skyboxPath;
    bool m_useSkybox;
    //
    vsg::dvec3 m_cameraUpVector;
    bool m_yup;
    vsg::dvec3 m_cameraEye;
    vsg::dvec3 m_cameraTarget;
    double m_cameraAngleDeg = 30.0;
    //
    double m_lightIntensity = 1.0;
    double m_elevation = 0;
    double m_acimut = 0;
    //
    bool m_draw_as_wireframe = false;
    vsg::ref_ptr<ShapeBuilder> m_shapeBuilder;
    //
    std::string m_imageFilename;
    // bool m_do_image_export = false;
    float m_guiFontSize = 25.0f;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
