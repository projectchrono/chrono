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
#include <unordered_map>

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>
#include <vsgImGui/Texture.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/implot.h>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChColormap.h"

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
#include "chrono_vsg/utils/ChConversionsVSG.h"

namespace chrono {
namespace vsg3d {

class ChVisualSystemVSGPlugin;

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

    /// Attach a custom plugin.
    /// Plugins offer a mechanism for extending a base VSG visual system with custom functionality; e.g., for rendering,
    /// controlling, and displaying information for specific types of Chrono systems. An arbitrary number of plugins can
    /// be attached to a VSG visual system. Attaching plugins muct be done *before* initialization of the VSG system.
    void AttachPlugin(std::shared_ptr<ChVisualSystemVSGPlugin> plugin);

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

    /// End the scene draw at the end of each animation frame.
    virtual void EndScene() override {}

    // --- Model components

    /// Set the visibility of bodies with specified tag.
    /// A tag value of -1 indicates that the visibility flag should be applied to all bodies.
    void SetBodyObjVisibility(bool vis, int tag = -1);

    /// Set the visibility of links with specified tag.
    /// A tag value of -1 indicates that the visibility flag should be applied to all links.
    void SetLinkObjVisibility(bool vis, int tag = -1);

    /// Set the visibility of FEA meshes with specified tag.
    /// A tag value of -1 indicates that the visibility flag should be applied to all meshes.
    void SetFeaMeshVisibility(bool vis, int tag = -1);

    /// Set the visibility of springs with specified tag.
    /// A tag value of -1 indicates that the visibility flag should be applied to all springs.
    void SetSpringVisibility(bool vis, int tag = -1);

    /// Set the visibility of segments with specified tag.
    /// A tag value of -1 indicates that the visibility flag should be applied to all segments.
    void SetSegmentVisibility(bool vis, int tag = -1);

    /// Set the visibility of particle clouds with specified tag to the provided value.
    /// A tag value of -1 indicates that the visibility flag should be applied to all particle clouds.
    void SetParticleCloudVisibility(bool vis, int tag = -1);

    /// --- Collision and contact

    /// Set visibility of collision shapes for objects with specified tag.
    /// A tag of -1 indicates that the visibility flag should be applied to all collision shapes.
    void SetCollisionVisibility(bool vis, int tag = -1);

    /// Set color for rendering wireframe collision shapes.
    void SetCollisionColor(const ChColor& color);

    /// Enable/disable visualization of contact normals (default: false).
    /// A tag value of -1 indicates that the visibility flag should be applied to all collision objects.
    void SetContactNormalsVisibility(bool vis, int tag = -1);
    /// Set color for rendering contact normals.
    void SetContactNormalsColor(const ChColor& color);
    /// Set scale for rendering contact normals.
    void SetContactNormalsScale(double length);

    /// Enable/disable visualization of contact forces (default: false).
    /// A tag value of -1 indicates that the visibility flag should be applied to all collision objects.
    void SetContactForcesVisibility(bool vis, int tag = -1);
    /// Set color for rendering contact forces.
    void SetContactForcesColor(const ChColor& color);
    /// Set scale for rendering contact forces.
    void SetContactForcesScale(double length);

    // --- Reference frames

    /// Render the absolute (global) reference frame
    void SetAbsFrameScale(double axis_length);
    void ToggleAbsFrameVisibility();

    /// Render ref frames for all objects in the system.
    void RenderRefFrames(double axis_length = 1);
    /// Set scale for rendering reference frames.
    void SetRefFrameScale(double axis_length);
    /// Toggle on/off visibility of reference frames.
    void ToggleRefFrameVisibility();

    /// Render COM frames for all bodies in the system.
    void SetCOMFrameScale(double axis_length);
    /// Toggle on/off visibilityy of COM frames.
    void ToggleCOMFrameVisibility();

    /// Render COM symbol for all bodies in the system.
    void ToggleCOMSymbolVisibility();

    /// Render link frames for all links in the system.
    void SetLinkFrameScale(double axis_length);
    /// Toggle on/off visibility of link frames.
    void ToggleLinkFrameVisibility();

    // --- Labels

    /// Toggle visibility of body labels.
    void ToggleBodyLabelVisibility();

    /// Set color for body labels.
    void SetBodyLabelsColor(const ChColor& color);

    /// Set rendering scale for body labels.
    void SetBodyLabelsScale(double length);

    /// Toggle visibility of link labels.
    void ToggleLinkLabelVisibility();

    /// Set color for link labels.
    void SetLinkLabelsColor(const ChColor& color);

    /// Set rendering scale for link labels.
    void SetLinkLabelsScale(double length);

    // ---

    /// Create a snapshot of the frame to be rendered and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) override;

    /// Set window size (default: 800x600).
    void SetWindowSize(const ChVector2i& size);

    /// Set window size (default: 800x600).
    void SetWindowSize(int width, int height);

    /// Set window position (default: 50,50).
    void SetWindowPosition(const ChVector2i& pos);

    /// Set window position (default: 50,50).
    void SetWindowPosition(int from_left, int from_top);

    /// Set window title (default: "").
    void SetWindowTitle(const std::string& title);

    /// Set default output screen.
    void SetOutputScreen(int screenNum = 0);

    /// Enable full-screen mode (default: false).
    /// This function must be called before Initialize().
    void EnableFullscreen(bool val = true);

    /// Enable/disable use of a sky box background (default: false).
    /// This function must be called before Initialize().
    void EnableSkyBox(bool val = true);

    /// Set the sky box texture.
    void SetSkyBoxTexture(const std::string& filename);

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

    /// Get estimated rendering FPS.
    double GetRenderingFPS() const { return m_fps; }

    /// Set target render frame rate (default: 0 means render every frame)
    /// When set to, for example, 60fps, it limits rendering to approximately that many FPS
    /// even if physics is running faster. Dramatically improves performance for VSG since
    /// recordAndSubmit() is expensive
    void SetTargetRenderFPS(double fps) { m_target_render_fps = fps; }

    /// Enable/disable rendering of shadows (default: false).
    /// This function must be called before Initialize().
    void EnableShadows(bool val = true) { m_use_shadows = val; }

    /// Indicate whether or not shadows are enabled.
    bool AreShadowsEnabled() const { return m_use_shadows; }

    /// Set light intensity (default: 1).
    /// The light intensity is clamped in [0,1].
    /// Directional light intensity is set to 100% of this value, unless shadow are enabled, in which case it is set to
    /// 80%. Ambient light intensity is set to 10% of this value.
    void SetLightIntensity(float intensity);

    /// Set azimuth and elevation for directional light (default: 3pi/4 and pi/4) .
    /// The azimuth is measured conter-clockwise from the x-axis and is clamped in [-pi, pi].
    /// The elevation is measured from the (xy)-plane and is clamped in [-pi/2, pi/2].
    void SetLightDirection(double azimuth, double elevation);

    /// Set the camera field of view in degrees (default: 40).
    void SetCameraAngleDeg(double angleDeg) { m_camera_angle_deg = angleDeg; }

    /// Set GUI font size (default: 13).
    void SetGuiFontSize(float size);

    /// Add a wireframe grid with specified resolution at the specified position.
    virtual void AddGrid(double x_step,                                      ///< spacing in x direction
                         double y_step,                                      ///< spacing in y direction
                         int nx,                                             ///< number of divisions in x direction
                         int ny,                                             ///< number of divisions in y direction
                         ChCoordsys<> pos = CSYSNORM,                        ///< position of grid center
                         ChColor col = ChColor(0.1f, 0.1f, 0.1f)) override;  ///< line color

    /// Add a visual model not bound to a Chrono object.
    /// The return value is the index of the new visual model.
    virtual int AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) override;
    /// Add a visual shape not bound to a Chrono object.
    /// The return value is the index of the new visual model.
    virtual int AddVisualModel(std::shared_ptr<ChVisualShape> model, const ChFrame<>& frame) override;
    /// Modify the position of the specified un-bound visual model.
    virtual void UpdateVisualModel(int id, const ChFrame<>& frame) override;

    /// Add a user-defined GUI component.
    /// Returns the index of the new component. This function must be called before Initialize().
    size_t AddGuiComponent(std::shared_ptr<ChGuiComponentVSG> gc);

    /// Add a colorbar as a GUI component.
    /// Returns the index of the new component. This function must be called before Initialize().
    size_t AddGuiColorbar(const std::string& title,  ///< GUI window title
                          const ChVector2d& range,   ///< data range
                          ChColormap::Type type,     ///< colormap
                          bool bimodal = false,      ///< negative/positive
                          float width = 400          ///< texture width in pixels
    );

    /// Access the specified GUI component.
    /// Identify the GUI component with the index returned by AddGuiComponent.
    std::shared_ptr<ChGuiComponentVSG> GetGuiComponent(size_t id);

    /// Set visibility for all GUI components (default: true).
    void SetGuiVisibility(bool show_gui) { m_show_gui = show_gui; }

    /// Toggle GUI visibility for all GUI components.
    void ToggleGuiVisibility() { m_show_gui = !m_show_gui; }

    /// Indicate whether or not GUI is visible.
    bool IsGuiVisible() const { return m_show_gui; }

    /// Set visibility for the default (base) GUI component (default: true).
    void SetBaseGuiVisibility(bool show_gui);

    /// Toggle GUI visibility for the default (base) GUI component.
    void ToggleBaseGuiVisibility();

    /// Indicate whether or not the default (base) GUI is visible.
    bool IsBaseGuiVisible() const { return m_show_base_gui; }

    /// Change logo image.
    void SetLogo(const std::string& filename) { m_logo_filename = filename; }

    /// Disable showing the Chrono logo (default: true).
    void HideLogo() { m_show_logo = false; }

    /// Set logo display height (in pixels, default: 64).
    void SetLogoHeight(float height) { m_logo_height = height; }

    /// Set logo position (default: [10,10]).
    /// This is the position of the right-top corner of the logo image (in pixels)
    /// relative to the right-top corner of the rendering window.
    void SetLogoPosition(const ChVector2f& position) { m_logo_pos = position; }

    /// Indicate whether or not logo is visible.
    bool IsLogoVisible() const { return m_show_logo; }

    /// Add a user-defined VSG event handler.
    void AddEventHandler(std::shared_ptr<ChEventHandlerVSG> eh);

    /// Get a reference to the underlying VSG scene.
    vsg::ref_ptr<vsg::Group> GetVSGScene() const { return m_scene; }

    /// Get a reference to the underlying shape builder.
    vsg::ref_ptr<ShapeBuilder> GetVSGShapeBuilder() const { return m_shapeBuilder; }

    /// Get the ImGui texture for the specified colormap.
    vsg::ref_ptr<vsgImGui::Texture> GetColormapTexture(ChColormap::Type type) const {
        return m_colormap_textures.at(type);
    }

    /// Data for particle clouds managed by the visual system.
    struct ParticleCloud {
        std::shared_ptr<ChParticleCloud> pcloud;            ///< reference to the Chrono physics item
        vsg::ref_ptr<vsg::vec3Array> positions;             ///< particle positions
        vsg::ref_ptr<vsg::vec4Array> colors;                ///< particle colours
        bool dynamic_positions;                             ///< particle positions change
        bool dynamic_colors;                                ///< particle colours change
        vsg::ref_ptr<vsg::Node> geometry_node;              ///< owning scene graph node
        vsg::ref_ptr<vsg::BufferInfo> position_bufferInfo;  ///< instance positions buffer
        vsg::ref_ptr<vsg::BufferInfo> color_bufferInfo;     ///< instance colours buffer
        bool use_compute_colors = false;                    ///< GPU compute overrides CPU updates
        vsg::ref_ptr<vsg::Commands> compute_commands;       ///< compute dispatch commands
    };

    /// Access particle cloud metadata.
    std::vector<ParticleCloud>& GetParticleClouds() { return m_clouds; }
    const std::vector<ParticleCloud>& GetParticleClouds() const { return m_clouds; }

    /// Register commands that must run on the compute queue before rendering.
    void AddComputeCommands(vsg::ref_ptr<vsg::Commands> commands);

    /// Access command graphs used for compute and rendering work.
    vsg::ref_ptr<vsg::CommandGraph> GetComputeCommandGraph() const { return m_computeCommandGraph; }
    vsg::ref_ptr<vsg::CommandGraph> GetRenderCommandGraph() const { return m_renderCommandGraph; }

    /// Access the underlying window.
    vsg::ref_ptr<vsg::Window> GetWindow() const { return m_window; }

    /// Access the VSG options object used for resource loading.
    vsg::ref_ptr<vsg::Options> GetOptions() const { return m_options; }

  protected:
    /// Perform necessary setup operations at the beginning of a time step.
    virtual void OnSetup(ChSystem* sys) override;

    /// Update all VSG scenes with the current state of the associated Chrono systems.
    void Update();

    bool GetDesiredCloudVisibility(int tag) const;

    int m_screen_num = -1;
    bool m_use_fullscreen;
    bool m_use_shadows;

    vsg::ref_ptr<vsg::Window> m_window;
    vsg::ref_ptr<vsg::Viewer> m_viewer;  ///< high-level VSG rendering manager
    vsg::ref_ptr<vsg::RenderGraph> m_renderGraph;
    vsg::ref_ptr<vsg::CommandGraph> m_renderCommandGraph;   ///< graphics submit path
    vsg::ref_ptr<vsg::CommandGraph> m_computeCommandGraph;  ///< compute submit path (particle colouring)

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

    vsg::ref_ptr<vsg::Group> m_scene;
    vsg::ref_ptr<vsg::Switch> m_pointpointScene;
    vsg::ref_ptr<vsg::Switch> m_particleScene;
    vsg::ref_ptr<vsg::Switch> m_visFixedScene;
    vsg::ref_ptr<vsg::Switch> m_visMutableScene;
    vsg::ref_ptr<vsg::Switch> m_collFixedScene;
    vsg::ref_ptr<vsg::Switch> m_collMutableScene;
    vsg::ref_ptr<vsg::Switch> m_contactNormalsScene;
    vsg::ref_ptr<vsg::Switch> m_contactForcesScene;
    vsg::ref_ptr<vsg::Switch> m_absFrameScene;
    vsg::ref_ptr<vsg::Switch> m_refFrameScene;
    vsg::ref_ptr<vsg::Switch> m_linkFrameScene;
    vsg::ref_ptr<vsg::Switch> m_comFrameScene;
    vsg::ref_ptr<vsg::Switch> m_comSymbolScene;
    vsg::ref_ptr<vsg::Switch> m_bodyLabelScene;
    vsg::ref_ptr<vsg::Switch> m_linkLabelScene;
    vsg::ref_ptr<vsg::Group> m_decoScene;

    vsg::ref_ptr<vsg::Options> m_options;  ///< I/O related options for vsg::read/write calls
    vsg::ref_ptr<vsg::Builder> m_vsgBuilder;
    vsg::ref_ptr<ShapeBuilder> m_shapeBuilder;

    bool m_capture_image;          ///< export current frame to image file
    std::string m_image_filename;  ///< name of file to export current frame

    /// Data related to deformable (mutable) meshes, both visualization and collision.
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

    std::vector<ParticleCloud> m_clouds;     ///< particle cloud metadata cached for VSG interop
    bool m_default_cloud_visibility = true;  ///< fallback visibility before a specific tag is toggled
    std::unordered_map<int, bool> m_cloud_visibility_overrides;  ///< per-tag visibility overrides

    bool m_show_visibility_controls;  ///< enable/disable global visibility controls

    std::vector<std::shared_ptr<ChVisualSystemVSGPlugin>> m_plugins;

  private:
    enum class ObjectType { BODY, LINK, FEA, OTHER };
    enum class PointPointType { SPRING, SEGMENT };

    /// Custom contact reporter to create contact normals and forces VSG nodes.
    class CreateContactsVSG : public ChContactContainer::ReportContactCallback {
      public:
        CreateContactsVSG(ChVisualSystemVSG* app);

        void Reset();

        virtual bool OnReportContact(const ChVector3d& pA,
                                     const ChVector3d& pB,
                                     const ChMatrix33<>& plane_coord,
                                     double distance,
                                     double eff_Radius,
                                     const ChVector3d& react_forces,
                                     const ChVector3d& react_torques,
                                     ChContactable* modA,
                                     ChContactable* modB,
                                     int constraint_offset) override;

      private:
        ChVisualSystemVSG* m_app;
        size_t m_crt_contact;
    };

    /// Create a buffer of VSG nodes for contact normals and forces.
    void CreateContacts();

    /*
     *
     * TODO: this version does not work with current VSG if shadows are enabled.
     * This is because there are issues with creating nodes after initialization of the shadow processing!
     *
    class CreateContactsVSG : public ChContactContainer::ReportContactCallback {
      public:
        CreateContactsVSG(ChVisualSystemVSG* app);

        void Reset();

        virtual bool OnReportContact(const ChVector3d& pA,
                                     const ChVector3d& pB,
                                     const ChMatrix33<>& plane_coord,
                                     double distance,
                                     double eff_Radius,
                                     const ChVector3d& react_forces,
                                     const ChVector3d& react_torques,
                                     ChContactable* modA,
                                     ChContactable* modB,
                                     int constraint_offset) override;

      private:
        ChVisualSystemVSG* m_app;
        std::shared_ptr<ChVisualMaterial> m_mat_normals;
        std::shared_ptr<ChVisualMaterial> m_mat_forces;
        size_t m_num_existing_normals_nodes;
        size_t m_num_existing_forces_nodes;
        size_t m_crt_normals_node;
        size_t m_crt_forces_node;
    };
    */

    /// Bind assets associated with a ChBody.
    void BindBody(const std::shared_ptr<ChBody>& body);

    /// Bind assets associated with a ChLink.
    void BindLink(const std::shared_ptr<ChLinkBase>& link);

    /// Bind assets associated with a ChMesh.
    void BindMesh(const std::shared_ptr<fea::ChMesh>& mesh);

    /// Bind all assets associated with the given ChAssembly.
    void BindAssembly(const ChAssembly& assembly);

    /// Bind the non-mutable shapes in the visual model associated with the given Chrono object.
    void BindVisualShapesFixed(const std::shared_ptr<ChObj>& obj, ObjectType type);

    /// Bind mutable shapes (deformable meshes) in the visual model associated with the given Chrono object.
    void BindVisualShapesMutable(const std::shared_ptr<ChObj>& obj, ObjectType type);

    /// Bind the non-mutable shapes in the collision model associated with the given contactable.
    void BindCollisionShapesFixed(const std::shared_ptr<ChContactable>& obj, int tag);

    /// Bind the mutable shapes in the collision model associated with the given contactable.
    void BindCollisionShapesMutable(const std::shared_ptr<ChContactable>& obj, int tag);

    /// Bind point-point visual assets in the visual model associated with the given Chrono object.
    void BindPointPoint(const std::shared_ptr<ChObj>& item);

    /// Bind the visual model assoicated with a particle cloud.
    void BindParticleCloud(const std::shared_ptr<ChParticleCloud>& pcloud);

    /// Bind the reference frame for the given ChObj.
    void BindReferenceFrame(const std::shared_ptr<ChObj>& obj);

    /// Bind the body COM frame.
    void BindCOMFrame(const std::shared_ptr<ChBody>& body);

    /// Bind the body COM symbols.
    void BindCOMSymbols();

    /// Bind the link frames.
    void BindLinkFrame(const std::shared_ptr<ChLinkBase>& link);

    /// Bind the body and link labels.
    void BindLabels();

    /// Populate a VSG group with non-mutable visualization shapes (from the given visual model).
    void PopulateVisualShapesFixed(vsg::ref_ptr<vsg::Group> group, std::shared_ptr<ChVisualModel> model);

    /// Populate a VSG group with mutable visualization shapes (from the given visual model).
    void PopulateVisualShapesMutable(vsg::ref_ptr<vsg::Group> group, std::shared_ptr<ChVisualModel> model);

    /// Populate a VSG group with non-mutable collision shapes (from the given collision model).
    /// The VSG shapes are always rendered wireframe.
    void PopulateCollisionShapeFixed(vsg::ref_ptr<vsg::Group> group, std::shared_ptr<ChCollisionModel> model);

    /// Populate a VSG group with mutable collision shapes (from the given collision model).
    /// The VSG shapes are always rendered wireframe.
    void PopulateCollisionShapeMutable(vsg::ref_ptr<vsg::Group> group, std::shared_ptr<ChCollisionModel> model);

    /// Utility function to collect active body positions from all assemblies in all systems.
    static void CollectActiveBodyCOMPositions(const ChAssembly& assembly, std::vector<ChVector3d>& positions);

    /// Utility function to collect link frame positions from all assemblies in all systems.
    /// We always use the 2nd link reference frame.
    static void CollectLinkFramePositions(const ChAssembly& assembly, std::vector<ChVector3d>& positions);

    /// Utility function to convert a vector of Chrono positions to VSG positions.
    static void ConvertPositions(const std::vector<ChVector3d>& c, vsg::ref_ptr<vsg::vec4Array> v, double w);

    /// Export screen image as file (png, bmp, tga, jpg).
    void ExportScreenImage();

    std::map<std::size_t, vsg::ref_ptr<vsg::Node>> m_objCache;
    std::hash<std::string> m_stringHash;
    int m_windows_width = 800;
    int m_windows_height = 600;
    int m_windows_x = 0;
    int m_windows_y = 0;
    std::string m_windows_title;

    int m_numThreads = 16;
    vsg::ref_ptr<vsg::OperationThreads> m_loadThreads;

    bool m_use_skybox;
    std::string m_skybox_path;

    vsg::dvec3 m_camera_up_vector;
    bool m_yup;
    double m_camera_angle_deg;

    double m_light_intensity;
    double m_elevation;
    double m_azimuth;
    float m_gui_font_size = 20.0f;

    // Component rendering
    bool m_show_body_objs;       ///< flag to toggle body asset visibility
    bool m_show_link_objs;       ///< flag to toggle link asset visibility
    bool m_show_spring_dampers;  ///< flag to toggle spring-damper visibility
    bool m_show_fea_meshes;      ///< flag to toggle FEA mesh visibility

    // Collision rendering
    bool m_show_collision;           ///< flag to toggle collision shape visibility
    ChColor m_collision_color;       ///< current color for rendering collision shapes
    bool m_collision_color_changed;  ///< flag indicating a change in collision color
    std::vector<vsg::ref_ptr<vsg::vec4Array>> m_collision_colors;

    // Contact rendering
    bool m_show_contact_normals;
    ChColor m_contact_normals_color;
    bool m_contact_normals_color_changed;
    double m_contact_normals_scale;
    std::vector<vsg::ref_ptr<vsg::vec3Array>> m_contact_normals_colors;

    std::shared_ptr<CreateContactsVSG> m_contact_creator;
    unsigned int m_max_num_contacts;

    bool m_show_contact_forces;
    ChColor m_contact_forces_color;
    bool m_contact_forces_color_changed;
    double m_contact_forces_scale;
    std::vector<vsg::ref_ptr<vsg::vec3Array>> m_contact_forces_colors;

    // Frame rendering
    bool m_show_abs_frame;      ///< flag to toggle absolute frame visibility
    bool m_show_ref_frames;     ///< flag to toggle object reference frame visibility
    bool m_show_com_frames;     ///< flag to toggle COM frame visibility
    bool m_show_com_symbols;    ///< flag to toggle COM symbol visibility
    bool m_show_link_frames;    ///< flag to toggle link frame visibility
    double m_abs_frame_scale;   ///< current absolute frame scale
    double m_ref_frame_scale;   ///< current reference frame scale
    double m_com_frame_scale;   ///< current COM frame scale
    double m_com_symbol_ratio;  ///< COM symbol scale relative to current COM frame scale
    double m_link_frame_scale;  ///< current link frame scale

    vsg::ref_ptr<vsg::vec3Array> m_com_symbol_vertices;
    vsg::ref_ptr<vsg::vec4Array> m_com_symbol_positions;
    bool m_com_size_changed;
    bool m_com_symbols_empty;

    // Labels
    std::string m_label_font_path;         ///< path to label font
    vsg::ref_ptr<vsg::Font> m_label_font;  ///< font for body and link labels
    double m_label_size;                   ///< base label text size

    bool m_show_body_labels;      ///< flag to toggle body label visibility
    double m_body_labels_scale;   ///< current body label size scale
    ChColor m_body_labels_color;  ///< current color for body labels

    bool m_show_link_labels;      ///< flag to toggle link label visibility
    double m_link_labels_scale;   ///< current link label size scale
    ChColor m_link_labels_color;  ///< current color for link labels

    std::vector<vsg::ref_ptr<vsg::stringValue>> m_body_labels;
    std::vector<vsg::ref_ptr<vsg::StandardLayout>> m_body_labels_layout;
    std::vector<vsg::ref_ptr<vsg::Text>> m_body_labels_text;

    std::vector<vsg::ref_ptr<vsg::stringValue>> m_link_labels;
    std::vector<vsg::ref_ptr<vsg::StandardLayout>> m_link_labels_layout;
    std::vector<vsg::ref_ptr<vsg::Text>> m_link_labels_text;

    unsigned int m_frame_number;                      ///< current number of rendered frames
    double m_start_time;                              ///< wallclock time at first render
    ChTimer m_timer_render;                           ///< timer for rendering speed
    double m_old_time, m_current_time, m_time_total;  ///< render times
    double m_fps;                                     ///< estimated FPS (moving average)

    double m_target_render_fps;  ///< target rendering framerate (0 = unlimited)
    double m_last_render_time;   ///< simulation time of last render

    // ImGui textures
    vsg::ref_ptr<vsgImGui::Texture> m_logo_texture;
    std::unordered_map<ChColormap::Type, vsg::ref_ptr<vsgImGui::Texture>> m_colormap_textures;

    friend class ChMainGuiVSG;
    friend class ChBaseGuiComponentVSG;
    friend class ChBaseEventHandlerVSG;
    ////friend class ChDrawContactsVSG;
    ////friend class ChDeferredDeleteVSG;
};

// -----------------------------------------------------------------------------

/// Base class for a plugin for a VSG visual system.
/// Plugins offer a mechanism for extending a base VSG visual system with custom functionality; e.g., for rendering,
/// controlling, and displaying information for specific types of Chrono systems. An arbitrary number of plugins can
/// be attached to a VSG visual system.
class ChVisualSystemVSGPlugin {
  public:
    virtual ~ChVisualSystemVSGPlugin() {}

    /// TODO - remove AddEventHandler?
    /// A plugin can call the VSG system's AddEventHandler in its OnAttach() function.

    /// Add custom event handlers for this plugin.
    void AddEventHandler(std::shared_ptr<ChEventHandlerVSG> eh) { m_evhandler.push_back(eh); }

    /// Get a reference to the VSG visual system to which the plugin was attached.
    ChVisualSystemVSG& GetVisualSystemVSG() const { return *m_vsys; }

  protected:
    ChVisualSystemVSGPlugin() {}

    /// Allow this plugin to perform any operations when it is attached to a VSG visual system.
    /// The pointer `m_vsys` to the associated VSG visual system is set before calling OnAttach.
    virtual void OnAttach() {}

    /// Allow this plugin to perform any pre-initialization operations.
    /// This function is called before the initialization of the associated VSG visual system.
    virtual void OnInitialize() {}

    /// Allow this plugin to perform any pre-binding operations.
    /// This function is called during initialization of the associated VSG visual system, after the scene was created
    /// and before binding assets for the associated VSG visual system. A plugin can create and populate its own
    /// children in the VSG scene.
    virtual void OnBindAssets() {}

    /// Allow this plugin to perform any pre-rendering operations.
    /// This function is called before updating and rendering the associated VSG visual system.
    virtual void OnRender() {}

    std::vector<std::shared_ptr<ChEventHandlerVSG>> m_evhandler;  ///< list of all additional event handlers
    ChVisualSystemVSG* m_vsys;                                    ///< associated VSG visual system

    friend class ChVisualSystemVSG;
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
