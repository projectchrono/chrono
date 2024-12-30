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

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/Texture.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/implot.h>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualModel.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeBarrel.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeSurface.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePath.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChParticleCloud.h"

#include "chrono_vsg/ChApiVSG.h"
#include "chrono_vsg/ChGuiComponentVSG.h"
#include "chrono_vsg/ChEventHandlerVSG.h"
#include "chrono_vsg/shapes/ShapeBuilder.h"

namespace chrono {
namespace vsg3d {

/// @addtogroup vsg_module
/// @{

/// VSG-based Chrono run-time visualization system.
class CH_VSG_API ChVisualSystemVSG : virtual public ChVisualSystem {
  public:
    /// Create the Chrono::VSG run-time visualization system.
    /// Optionally, specify the resolution used for tesselation of primitive shapes, by providing the number of
    /// divisions used to discretize a full circle. The default value of 24 corresponds to 15-degree divisions.
    ChVisualSystemVSG(int num_divs = 24);
    ~ChVisualSystemVSG();

    /// Initialize the visualization system.
    virtual void Initialize() override;

    /// Process all visual assets in the associated ChSystem.
    /// This function is called by default by Initialize(), but can also be called later if further modifications to
    /// visualization assets occur.
    virtual void BindAll() override;

    /// Process the visual assets for the specified physics item.
    /// This function must be called if a new physics item is added to the system or if changes to its visual model
    /// occur after the call to Initialize().
    virtual void BindItem(std::shared_ptr<ChPhysicsItem> item) override;

    /// Check if rendering is running.
    /// Returns `false` if the viewer was closed.
    virtual bool Run() override;

    // Terminate the VSG visualization.
    virtual void Quit() override;

    /// Perform any necessary operations at the beginning of each rendering frame.
    virtual void BeginScene() override {}

    /// Draw all 3D shapes and GUI elements at the current frame.
    /// This function is typically called inside a loop such as
    /// <pre>
    ///    while(vis->Run()) {...}
    /// </pre>
    virtual void Render() override;

    /// Render COG frames for all bodies in the system.
    virtual void RenderCOGFrames(double axis_length = 1) override;

    void SetCOGFrameScale(double axis_length);
    void ToggleCOGFrameVisibility();

    /// Render joint frames for all links in the system.
    void RenderJointFrames(double axis_length = 1);

    void SetJointFrameScale(double axis_length);
    void ToggleJointFrameVisibility();

    /// End the scene draw at the end of each animation frame.
    virtual void EndScene() override {}

    /// Create a snapshot of the frame to be rendered and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) override;

    void SetWindowSize(const ChVector2i& size);
    void SetWindowSize(int width, int height);
    void SetWindowPosition(const ChVector2i& pos);
    void SetWindowPosition(int from_left, int from_top);
    void SetWindowTitle(const std::string& title);
    void SetClearColor(const ChColor& color);
    void SetOutputScreen(int screenNum = 0);
    void SetFullscreen(bool yesno = false);
    void SetUseSkyBox(bool yesno);

    /// Draw the scene objects as wireframes.
    void SetWireFrameMode(bool mode = true) { m_wireframe = mode; }

    /// Set the camera up vector (default: Z).
    void SetCameraVertical(CameraVerticalDir upDir);

    /// Add a camera to the VSG scene.
    /// Note that currently only one camera is supported.
    virtual int AddCamera(const ChVector3d& pos, ChVector3d targ = VNULL) override;

    /// Set the location of the specified camera.
    virtual void SetCameraPosition(int id, const ChVector3d& pos) override;

    /// Set the target (look-at) point of the specified camera.
    virtual void SetCameraTarget(int id, const ChVector3d& target) override;

    /// Set the location of the current (active) camera.
    virtual void SetCameraPosition(const ChVector3d& pos) override;

    /// Set the target (look-at) point of the current (active) camera.
    virtual void SetCameraTarget(const ChVector3d& target) override;

    /// Get the location of the current (active) camera.
    virtual ChVector3d GetCameraPosition() const override;

    /// Get the target (look-at) point of the current (active) camera.
    virtual ChVector3d GetCameraTarget() const override;

    /// Get estimated FPS.
    double GetRenderingFPS() const { return m_fps; }

    /// Enable/disable rendering of shadows.
    /// This function must be called before Initialize().
    void SetShadows(bool yesno = false) { m_use_shadows = yesno; }

    void SetLightIntensity(float intensity);
    void SetLightDirection(double azimuth, double elevation);
    void SetCameraAngleDeg(double angleDeg) { m_cameraAngleDeg = angleDeg; }
    void SetGuiFontSize(float theSize);

    virtual void AddGrid(double x_step,
                         double y_step,
                         int nx,
                         int ny,
                         ChCoordsys<> pos = CSYSNORM,
                         ChColor col = ChColor(0.1f, 0.1f, 0.1f)) override;
    virtual int AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) override;
    virtual int AddVisualModel(std::shared_ptr<ChVisualShape> model, const ChFrame<>& frame) override;
    virtual void UpdateVisualModel(int id, const ChFrame<>& frame) override;

    /// Add a user-defined GUI component.
    /// Returns the index of the new component. This function must be called before Initialize().
    size_t AddGuiComponent(std::shared_ptr<ChGuiComponentVSG> gc);

    /// Add a colorbar as a GUI component.
    /// Returns the index of the new component. This function must be called before Initialize().
    size_t AddGuiColorbar(const std::string& title, double min_val, double max_val);

    /// Access the specified GUI component.
    /// Identify the GUI component with the index returned by AddGuiComponent.
    std::shared_ptr<ChGuiComponentVSG> GetGuiComponent(size_t id);

    /// Set visibility for all GUI components (default: true).
    void SetGuiVisibility(bool show_gui) { m_show_gui = show_gui; }

    /// Toggle GUI visibility for all GUI components.
    void ToggleGuiVisibility() { m_show_gui = !m_show_gui; }

    /// Return boolean indicating whether or not GUI are visible.
    bool IsGuiVisible() const { return m_show_gui; }

    /// Set visibility for the default (base) GUI component (default: true).
    void SetBaseGuiVisibility(bool show_gui);

    /// Toggle GUI visibility for the default (base) GUI component.
    void ToggleBaseGuiVisibility();

    /// Return boolean indicating whether or not the default (base) GUI is visible.
    bool IsBaseGuiVisible() const { return m_show_base_gui; }

    /// Set logo visible (default: true).
    void SetLogoVisible(bool yesno) { m_show_logo = yesno; }

    /// Set logo display height (in pixels, default: 64).
    void SetLogoHeight(float height) { m_logo_height = height; }

    /// Set logo position (default: [10,10]).
    /// This is the position of the right-top corner of the logo image (in pixels)
    /// relative to the right-top corner of the rendering window.
    void SetLogoPosition(const ChVector2f& position) { m_logo_pos = position; }

    /// Return boolean indicating whether or not logo is visible.
    bool IsLogoVisible() const { return m_show_logo; }

    /// Add a user-defined VSG event handler.
    void AddEventHandler(std::shared_ptr<ChEventHandlerVSG> eh);

  protected:
    /// Perform necessary setup operations at the beginning of a time step.
    virtual void OnSetup(ChSystem* sys) override;

    void UpdateFromMBS();

    int m_screen_num = -1;
    bool m_use_fullscreen = false;
    bool m_use_shadows = false;

    vsg::ref_ptr<vsg::Window> m_window;
    vsg::ref_ptr<vsg::Viewer> m_viewer;  ///< high-level VSG rendering manager
    vsg::ref_ptr<vsg::RenderGraph> m_renderGraph;

    bool m_show_logo;
    float m_logo_height;
    ChVector2f m_logo_pos;
    std::string m_logo_filename;

    bool m_show_gui;                                              ///< flag to toggle global GUI visibility
    bool m_show_base_gui;                                         ///< flag to toggle base GUI visibility
    size_t m_camera_gui;                                          ///< identifier for the camera info GUI component
    std::shared_ptr<ChGuiComponentVSG> m_base_gui;                ///< default (base) GUI component
    std::vector<std::shared_ptr<ChGuiComponentVSG>> m_gui;        ///< list of all additional GUI components
    std::vector<std::shared_ptr<ChEventHandlerVSG>> m_evhandler;  ///< list of all additional event handlers

    vsg::dvec3 m_vsg_cameraEye = vsg::dvec3(-10.0, 0.0, 0.0);
    vsg::dvec3 m_vsg_cameraTarget = vsg::dvec3(0.0, 0.0, 0.0);
    vsg::ref_ptr<vsg::LookAt> m_lookAt;
    vsg::ref_ptr<vsg::Camera> m_vsg_camera;
    bool m_camera_trackball;  ///< create a camera trackball control?

    //  m_scene +- skybox, lights +- m_bodyScene
    //                            |
    //                            +- m_cogScene
    //                            |
    //                            +- m_pointpointScene
    //                            |
    //                            +- m_particleScene
    //                            |
    //                            +- m_decoScene
    //                            |
    //                            +- m_deformableScene
    vsg::ref_ptr<vsg::Group> m_scene;
    vsg::ref_ptr<vsg::Group> m_bodyScene;
    vsg::ref_ptr<vsg::Group> m_pointpointScene;
    vsg::ref_ptr<vsg::Group> m_particleScene;
    vsg::ref_ptr<vsg::Group> m_deformableScene;
    vsg::ref_ptr<vsg::Group> m_decoScene;

    vsg::ref_ptr<vsg::Switch> m_cogFrameScene;
    vsg::ref_ptr<vsg::Switch> m_jointFrameScene;

    vsg::ref_ptr<vsg::Options> m_options;  ///< I/O related options for vsg::read/write calls
    vsg::ref_ptr<vsg::Builder> m_vsgBuilder;
    vsg::ref_ptr<ShapeBuilder> m_shapeBuilder;

    bool m_wireframe;             ///< draw as wireframes
    bool m_capture_image;         ///< export current frame to image file
    std::string m_imageFilename;  ///< name of file to export current frame

    /// Data related to deformable meshes (FEA and SCM).
    struct DeformableMesh {
        std::shared_ptr<ChTriangleMeshConnected> trimesh;  ///< reference to the Chrono triangle mesh
        vsg::ref_ptr<vsg::vec3Array> vertices;             ///< mesh vertices
        vsg::ref_ptr<vsg::vec3Array> normals;              ///< mesh normals
        vsg::ref_ptr<vsg::vec4Array> colors;               ///< mesh vertex colors
        bool mesh_soup;                                    ///< true if using separate triangles
        bool dynamic_vertices;                             ///< mesh vertices change
        bool dynamic_normals;                              ///< mesh normals change
        bool dynamic_colors;                               ///< mesh vertex colors change
    };
    std::vector<DeformableMesh> m_def_meshes;

    /// Data for particle clouds.
    struct ParticleCloud {
        std::shared_ptr<ChParticleCloud> pcloud;  ///< reference to the Chrono physics item
        vsg::ref_ptr<vsg::vec3Array> positions;   ///< particle positions
        vsg::ref_ptr<vsg::vec4Array> colors;      ///< particle colors
        bool dynamic_positions;                   ///< particle positions change
        bool dynamic_colors;                      ///< particle colors change
    };
    std::vector<ParticleCloud> m_clouds;

    /// export screen image as file (png, bmp, tga, jpg)
    void exportScreenImage();

  private:
    /// Bind the visual model associated with a body.
    void BindBody(const std::shared_ptr<ChBody>& body);

    /// Bind deformable meshes in the visual model associated with the given physics item.
    void BindDeformableMesh(const std::shared_ptr<ChPhysicsItem>& item);

    /// Bind point-point visual assets in the visual model associated with the given physics item.
    void BindPointPoint(const std::shared_ptr<ChPhysicsItem>& item);

    /// Bind the visual model assoicated with a particle cloud.
    void BindParticleCloud(const std::shared_ptr<ChParticleCloud>& pcloud);

    /// Bind the body COG frame.
    void BindBodyFrame(const std::shared_ptr<ChBody>& body);

    /// Bind the joint frames.
    void BindLinkFrame(const std::shared_ptr<ChLink>& link);

    /// Utility function to populate a VSG group with shape groups (from the given visual model).
    /// The visual model may or may not be associated with a Chrono physics item.
    void PopulateGroup(vsg::ref_ptr<vsg::Group> group,
                       std::shared_ptr<ChVisualModel> model,
                       std::shared_ptr<ChPhysicsItem> phitem);

    std::map<std::size_t, vsg::ref_ptr<vsg::Node>> m_objCache;
    std::hash<std::string> m_stringHash;
    int m_windowWidth = 800;
    int m_windowHeight = 600;
    int m_windowX = 0;
    int m_windowY = 0;
    std::string m_windowTitle;
    ChColor m_clearColor;

    int m_numThreads = 16;
    vsg::ref_ptr<vsg::OperationThreads> m_loadThreads;

    bool m_useSkybox;
    std::string m_skyboxPath;

    vsg::dvec3 m_cameraUpVector;
    bool m_yup;
    double m_cameraAngleDeg = 30.0;

    double m_lightIntensity = 1.0f;
    double m_elevation = 0;
    double m_azimuth = 0;
    float m_guiFontSize = 20.0f;

    bool m_show_cog_frames;    ///< flag to toggle COG frame visibility
    double m_cog_frame_scale;  ///< current COG frame scale

    bool m_show_joint_frames;    ///< flag to toggle COG frame visibility
    double m_joint_frame_scale;  ///< current joint frame scale

    unsigned int m_frame_number;                      ///< current number of rendered frames
    double m_start_time;                              ///< wallclock time at first render
    ChTimer m_timer_render;                           ///< timer for rendering speed
    double m_old_time, m_current_time, m_time_total;  ///< render times
    double m_fps;                                     ///< estimated FPS (moving average)

    friend class ChMainGuiVSG;
    friend class ChBaseGuiComponentVSG;
    friend class ChBaseEventHandlerVSG;
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
