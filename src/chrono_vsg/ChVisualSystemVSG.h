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
#include "chrono_vsg/ChApiVSG.h"

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/physics/ChLoadContainer.h"

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/imgui.h>

#include "tools/createSkybox.h"
#include "tools/exportScreenshot.h"
#include "shapes/ShapeBuilder.h"

namespace chrono {
namespace vsg3d {

/// @addtogroup vsg_module
/// @{

class CH_VSG_API ChVisualSystemVSG : virtual public ChVisualSystem {
  public:
    ChVisualSystemVSG();
    ~ChVisualSystemVSG();

    virtual void Initialize();

    virtual void BindAll() override;

    virtual bool Run() override;
    virtual void BeginScene() override {}
    virtual void EndScene() override {}
    virtual void Render() override;
    virtual void RenderCOGFrames(double axis_length = 1) override;

    void WriteImageToFile(const std::string& filename) override;

    // Terminate the VSG visualization.
    void Quit();

    void SetWindowSize(ChVector2<int> size);
    void SetWindowSize(int width, int height);
    void SetWindowPosition(ChVector2<int> pos);
    void SetWindowPosition(int from_left, int from_top);
    void SetWindowTitle(std::string title);
    void SetClearColor(ChColor color);
    void SetOutputScreen(int screenNum = 0);
    void SetFullscreen(bool yesno = false);
    void SetUseSkyBox(bool yesno);

    /// Draw the scene objects as wireframes.
    void SetWireFrameMode(bool mode = true) { m_draw_as_wireframe = mode; }

    void SetCameraVertical(CameraVerticalDir upDir);

    /// Add a camera to the VSG scene.
    /// Note that currently only one camera is supported.
    virtual int AddCamera(const ChVector<>& pos, ChVector<> targ = VNULL) override;

    /// Set the location of the specified camera.
    virtual void SetCameraPosition(int id, const ChVector<>& pos) override;

    /// Set the target (look-at) point of the specified camera.
    virtual void SetCameraTarget(int id, const ChVector<>& target) override;

    /// Set the location of the current (active) camera.
    virtual void SetCameraPosition(const ChVector<>& pos) override;

    /// Set the target (look-at) point of the current (active) camera.
    virtual void SetCameraTarget(const ChVector<>& target) override;

    void SetLightIntensity(double intensity) { m_lightIntensity = ChClamp(intensity, 0.0, 1.0); }
    void SetLightDirection(double acimut, double elevation);
    void SetCameraAngleDeg(double angleDeg) { m_cameraAngleDeg = angleDeg; }
    void ShowAllCoGs(double size);
    void SetGuiFontSize(float theSize);
    void SetDecoGrid(double ustep, double vstep, int nu, int nv, ChCoordsys<> pos, ChColor col);
    int AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) override;
    int AddVisualModel(std::shared_ptr<ChVisualShape> model, const ChFrame<>& frame) override;
    void UpdateVisualModel(int id, const ChFrame<>& frame) override;
    void SetSystemSymbol(double size);
    void SetSystemSymbolPosition(ChVector<> pos);
    void SetColorBar(std::string title, double min_val, double max_val);
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
    virtual double GetSprocketTorque(int side) { return 0.0; }
    virtual double GetSprocketSpeed(int side) { return 0.0; }
    virtual void AttachGui();

    struct StateParams : public vsg::Inherit<vsg::Object, StateParams> {
        bool showGui = true;            // (don't) show the imgui menu, actually unused
        bool do_image_capture = false;  // mark image capturing as needed
        double cogSymbolSize = 0.0;
        size_t frame_number = 0;  // updated in Run() loop
        double time_begin = 0.0;  // wallclock time at begin of Run() loop
        bool showVehicleState = false;
        bool showTrackedVehicleState = false;
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
        bool show_color_bar = false;
        std::string cb_title;
        double cb_min = 0.0;
        double cb_max = 1.0;
    };

  protected:
    virtual void UpdateFromMBS();
    // collect some often used calulations (not for Cylinders!)
    void Point2PointHelperAbs(ChVector<>& P1,
                              ChVector<>& P2,
                              double& height,
                              ChVector<>& pos,
                              double& rotAngle,
                              ChVector<>& rotAxis);
    bool m_initialized = false;
    int m_screen_num = -1;
    bool m_use_fullscreen = false;
    vsg::ref_ptr<vsgImGui::RenderImGui> m_renderGui;
    vsg::ref_ptr<ChVisualSystemVSG::StateParams> m_params = StateParams::create();
    vsg::ref_ptr<vsg::Window> m_window;
    vsg::ref_ptr<vsg::Viewer> m_viewer;
    vsg::dvec3 m_vsg_cameraEye = vsg::dvec3(-10.0, 0.0, 0.0);
    vsg::dvec3 m_vsg_cameraTarget = vsg::dvec3(0.0, 0.0, 0.0);
    vsg::ref_ptr<vsg::LookAt> m_lookAt;
    vsg::ref_ptr<vsg::Camera> m_vsg_camera;
    vsg::dvec3 m_system_symbol_position = vsg::dvec3(0.0, 0.0, 0.0);
    vsg::dvec3 m_system_symbol_size = vsg::dvec3(1.0, 1.0, 1.0);
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
    //                            |
    //                            +- m_symbolScene
    //                            |
    //                            +- m_deformableScene
    vsg::ref_ptr<vsg::Group> m_scene;
    vsg::ref_ptr<vsg::Group> m_bodyScene;
    vsg::ref_ptr<vsg::Group> m_cogScene;
    vsg::ref_ptr<vsg::Group> m_linkScene;
    vsg::ref_ptr<vsg::Group> m_particleScene;
    vsg::ref_ptr<vsg::Group> m_decoScene;
    vsg::ref_ptr<vsg::Group> m_symbolScene;
    vsg::ref_ptr<vsg::Group> m_deformableScene;
    //
    vsg::ref_ptr<ShapeBuilder> m_shapeBuilder;
    vsg::ref_ptr<vsg::Builder> m_vsgBuilder;
    //
    bool m_draw_as_wireframe = false;
    vsg::ref_ptr<vsg::Options> m_options;
    //
    std::string m_imageFilename;
    //============================================================
    size_t m_num_vsgVertexList = 0;
    bool m_allowVertexTransfer = false;
    bool m_allowNormalsTransfer = false;
    bool m_allowColorsTransfer = false;
    std::vector<vsg::ref_ptr<vsg::vec3Array>> m_vsgVerticesList;
    std::vector<vsg::ref_ptr<vsg::vec3Array>> m_vsgNormalsList;
    std::vector<vsg::ref_ptr<vsg::vec4Array>> m_vsgColorsList;
    std::shared_ptr<ChTriangleMeshShape> m_mbsMesh;
    //============================================================
    // scenery object helper
    std::vector<vsg::ref_ptr<vsg::Group>> m_sceneryPtr;
    //============================================================
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
    int m_numThreads = 16;
    vsg::ref_ptr<vsg::OperationThreads> m_loadThreads;
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
    // bool m_do_image_export = false;
    float m_guiFontSize = 20.0f;
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono
#endif
