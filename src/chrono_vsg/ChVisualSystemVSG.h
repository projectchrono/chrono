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

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>

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

    virtual void Initialize();
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
    void SetGuiFontSize(float theSize);
    void SetDecoGrid(double ustep, double vstep, int nu, int nv, ChCoordsys<> pos, ChColor col);
    void BeginScene() {};
    void EndScene() {};
    double GetModelTime();
    size_t GetFrameNumber();
    double GetWallclockTime();
    double GetRealtimeFactor();
    virtual int GetGearPosition() { return 0; }
    virtual double GetEngineSpeedRPM() { return 0.0; }
    virtual double GetEngineTorque() { return 0.0; }
    virtual char GetTransmissionMode() { return '?'; }
    virtual char GetDriveMode() { return '?'; }
    virtual double GetTconvSlip() { return 0.0; }
    virtual double GetTconvTorqueInput() { return 0.0; }
    virtual double GetTconvTorqueOutput() { return 0.0; }
    virtual double GetTconvSpeedOutput() { return 0.0; }
    virtual int GetNumDrivenAxles() { return 0; }
    virtual double GetTireTorque(int axle, int side) { return 0.0; }
    virtual void AttachGui();

    struct StateParams : public vsg::Inherit<vsg::Object, StateParams> {
        bool showGui = true;            // (don't) show the imgui menu, actually unused
        bool do_image_capture = false;  // mark image capturing as needed
        double cogSymbolSize = 0.0;
        size_t frame_number = 0;  // updated in Run() loop
        double time_begin = 0.0;  // wallclock time at begin of Run() loop
        bool showVehicleState = false;
        double vehicleSpeed = 0;
        double steering = 0;
        double throttle = 0;
        double braking = 0;
        std::string camera_mode = "VSG";
        std::string input_mode = "";
        int gear_position = 0;
        double engine_rpm = 0.0;
        double engine_torque = 0.0;
        char transmission_mode = '?';
        char drive_mode = '?';
        bool show_converter_data = false;
    };

  protected:
    void BindAll() override;
    void UpdateFromMBS();
    // collect some often used calulations (not for Cylinders!)
    void Point2PointHelperAbs(ChVector<>& P1,
                              ChVector<>& P2,
                              double& height,
                              ChVector<>& pos,
                              double& rotAngle,
                              ChVector<>& rotAxis);
    vsg::ref_ptr<vsgImGui::RenderImGui> m_renderGui;
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> m_params = StateParams::create();
    vsg::ref_ptr<vsg::Window> m_window;
    vsg::ref_ptr<vsg::Viewer> m_viewer;
    vsg::dvec3 m_vsg_cameraEye = vsg::dvec3(-10.0, 0.0, 0.0);
    vsg::dvec3 m_vsg_cameraTarget = vsg::dvec3(0.0, 0.0, 0.0);
    vsg::ref_ptr<vsg::LookAt> m_lookAt;
    vsg::ref_ptr<vsg::Camera> m_vsg_camera;

  private:
    std::map<std::size_t, vsg::ref_ptr<vsg::Node>> m_objCache;
    std::hash<std::string> m_stringHash;
    int m_windowWidth = 800;
    int m_windowHeight = 600;
    int m_windowX = 0;
    int m_windowY = 0;
    std::string m_windowTitle;
    ChColor m_clearColor;
    //
    vsg::ref_ptr<vsg::Options> m_options;
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
    // cache for particle shape
    vsg::ref_ptr<vsg::Group> m_particlePattern;
    //
    std::string m_skyboxPath;
    bool m_useSkybox;
    //
    vsg::dvec3 m_cameraUpVector;
    bool m_yup;
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
    float m_guiFontSize = 20.0f;
};
}  // namespace vsg3d
}  // namespace chrono
#endif
