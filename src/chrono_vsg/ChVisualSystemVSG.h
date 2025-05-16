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

    void SetContactNormalsVisibility(bool vis, int tag = -1);
    void SetContactNormalsColor(const ChColor& color);
    void SetContactNormalsScale(double length);

    void SetContactForcesVisibility(bool vis, int tag = -1);
    void SetContactForcesColor(const ChColor& color);
    void SetContactForcesScale(double length);

    // --- Reference frames

    /// Render the absolute (global) reference frame
    void SetAbsFrameScale(double axis_length);
    void ToggleAbsFrameVisibility();

    /// Render ref frames for all objects in the system.
    void RenderRefFrames(double axis_length = 1);
    void SetRefFrameScale(double axis_length);
    void ToggleRefFrameVisibility();

    /// Render COM frames for all bodies in the system.
    void SetCOMFrameScale(double axis_length);
    void ToggleCOMFrameVisibility();

    /// Render COM symbol for all bodies in the system.
    void ToggleCOMSymbolVisibility();

    /// Render joint frames for all links in the system.
    void SetJointFrameScale(double axis_length);
    void ToggleJointFrameVisibility();

    /// Create a snapshot of the frame to be rendered and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) override;

    void SetWindowSize(const ChVector2i& size);
    void SetWindowSize(int width, int height);
    void SetWindowPosition(const ChVector2i& pos);
    void SetWindowPosition(int from_left, int from_top);
    void SetWindowTitle(const std::string& title);
    void SetOutputScreen(int screenNum = 0);

    /// Enable full-screen mode (default: false).
    /// This function must be called before Initialize().
    void EnableFullscreen(bool val = true);

    /// Enable/disable use of a sky box background (default: false).
    /// This function must be called before Initialize().
    void EnableSkyBox(bool val = true);

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

    /// Enable/disable rendering of shadows (default: false).
    /// This function must be called before Initialize().
    void EnableShadows(bool val = true) { m_use_shadows = val; }

    /// Indicate whether or not shadows are enabled.
    bool AreShadowsEnabled() const { return m_use_shadows; }

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
    vsg::ref_ptr<vsgImGui::Texture> GetColormapTexture(ChColormap::Type type) const { return m_colormap_textures.at(type); }

  protected:
    /// Perform necessary setup operations at the beginning of a time step.
    virtual void OnSetup(ChSystem* sys) override;

    /// Update all VSG scenes with the current state of the associated Chrono systems.
    void Update();

    int m_screen_num = -1;
    bool m_use_fullscreen;
    bool m_use_shadows;

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

    vsg::ref_ptr<vsg::Group> m_scene;
    vsg::ref_ptr<vsg::Switch> m_objScene;
    vsg::ref_ptr<vsg::Switch> m_pointpointScene;
    vsg::ref_ptr<vsg::Switch> m_deformableScene;
    vsg::ref_ptr<vsg::Switch> m_particleScene;
    vsg::ref_ptr<vsg::Switch> m_collisionScene;
    vsg::ref_ptr<vsg::Switch> m_contactNormalsScene;
    vsg::ref_ptr<vsg::Switch> m_contactForcesScene;
    vsg::ref_ptr<vsg::Switch> m_absFrameScene;
    vsg::ref_ptr<vsg::Switch> m_refFrameScene;
    vsg::ref_ptr<vsg::Switch> m_comFrameScene;
    vsg::ref_ptr<vsg::Switch> m_comSymbolScene;
    vsg::ref_ptr<vsg::Switch> m_jointFrameScene;
    vsg::ref_ptr<vsg::Group> m_decoScene;

    vsg::ref_ptr<vsg::Options> m_options;  ///< I/O related options for vsg::read/write calls
    vsg::ref_ptr<vsg::Builder> m_vsgBuilder;
    vsg::ref_ptr<ShapeBuilder> m_shapeBuilder;

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

    bool m_show_visibility_controls;  ///< enable/disable global visibility controls

    std::vector<std::shared_ptr<ChVisualSystemVSGPlugin>> m_plugins;

  private:
    enum class ObjectType { BODY, LINK, OTHER };
    enum class PointPointType { SPRING, SEGMENT };
    enum class DeformableType { FEA, OTHER };

    /// Custom contact reporter to create contact normals and forces VSG nodes.
    class CreateContactsVSG : public ChContactContainer::ReportContactCallback {
      public:
        CreateContactsVSG(ChVisualSystemVSG* app);

        void Reset();

        virtual bool OnReportContact(const ChVector3d& pA,
                                     const ChVector3d& pB,
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const double& eff_Radius,
                                     const ChVector3d& react_forces,
                                     const ChVector3d& react_torques,
                                     ChContactable* modA,
                                     ChContactable* modB) override;

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
                                     const double& distance,
                                     const double& eff_Radius,
                                     const ChVector3d& react_forces,
                                     const ChVector3d& react_torques,
                                     ChContactable* modA,
                                     ChContactable* modB) override;

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

    /// Bind the visual model associated with a ChObj object (body, link, or other).
    void BindObjectVisualModel(const std::shared_ptr<ChObj>& obj, ObjectType type);

    /// Bind the collision model associated with a ChContactable object.
    void BindObjectCollisionModel(const std::shared_ptr<ChContactable>& obj, int tag);

    /// Bind deformable meshes in the visual model associated with the given physics item.
    void BindDeformableMesh(const std::shared_ptr<ChPhysicsItem>& item, DeformableType type);

    /// Bind point-point visual assets in the visual model associated with the given physics item.
    void BindPointPoint(const std::shared_ptr<ChPhysicsItem>& item);

    /// Bind the visual model assoicated with a particle cloud.
    void BindParticleCloud(const std::shared_ptr<ChParticleCloud>& pcloud);

    /// Bind the reference frame for the given ChObj.
    void BindReferenceFrame(const std::shared_ptr<ChObj>& obj);

    /// Bind the body COM frame.
    void BindCOMFrame(const std::shared_ptr<ChBody>& body);

    /// Bind the joint frames.
    void BindLinkFrame(const std::shared_ptr<ChLinkBase>& link);

    void BindCOMSymbols();

    /// Utility function to populate a VSG group with visualization shapes (from the given visual model).
    void PopulateVisGroup(vsg::ref_ptr<vsg::Group> group, std::shared_ptr<ChVisualModel> model);

    /// Utility function to populate a VSG group with collision shapes (from the given collision model).
    /// The VSG shapes are always rendered wireframe.
    void PopulateCollGroup(vsg::ref_ptr<vsg::Group> group, std::shared_ptr<ChCollisionModel> model);

    /// Utility functions to collect active body positions from all assemblies in all systems.
    static void CollectActiveBodyCOMPositions(const ChAssembly& assembly, std::vector<ChVector3d>& positions);
    static void ConvertCOMPositions(const std::vector<ChVector3d>& c, vsg::ref_ptr<vsg::vec4Array> v, double w);

    /// Export screen image as file (png, bmp, tga, jpg).
    void ExportScreenImage();

    std::map<std::size_t, vsg::ref_ptr<vsg::Node>> m_objCache;
    std::hash<std::string> m_stringHash;
    int m_windowWidth = 800;
    int m_windowHeight = 600;
    int m_windowX = 0;
    int m_windowY = 0;
    std::string m_windowTitle;

    int m_numThreads = 16;
    vsg::ref_ptr<vsg::OperationThreads> m_loadThreads;

    bool m_use_skybox;
    std::string m_skyboxPath;

    vsg::dvec3 m_cameraUpVector;
    bool m_yup;
    double m_cameraAngleDeg = 30.0;

    double m_lightIntensity = 1.0f;
    double m_elevation = 0;
    double m_azimuth = 0;
    float m_guiFontSize = 20.0f;

    // Component rendering
    bool m_show_body_objs;   ///< flag to toggle body asset visibility
    bool m_show_link_objs;   ///< flag to toggle link asset visibility
    bool m_show_springs;     ///< flag to toggle spring visibility
    bool m_show_fea_meshes;  ///< flag to toggle FEA mesh visibility

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
    bool m_show_abs_frame;       ///< flag to toggle absolute frame visibility
    bool m_show_ref_frames;      ///< flag to toggle object reference frame visibility
    bool m_show_com_frames;      ///< flag to toggle COM frame visibility
    bool m_show_com_symbols;     ///< flag to toggle COM symbol visibility
    bool m_show_joint_frames;    ///< flag to toggle link frame visibility
    double m_abs_frame_scale;    ///< current absolute frame scale
    double m_ref_frame_scale;    ///< current reference frame scale
    double m_com_frame_scale;    ///< current COM frame scale
    double m_com_symbol_ratio;   ///< COM symbol scale relative to current COM frame scale
    double m_joint_frame_scale;  ///< current joint frame scale

    vsg::ref_ptr<vsg::vec3Array> m_com_symbol_vertices;
    vsg::ref_ptr<vsg::vec4Array> m_com_symbol_positions;
    bool m_com_size_changed;
    bool m_com_symbols_empty;

    unsigned int m_frame_number;                      ///< current number of rendered frames
    double m_start_time;                              ///< wallclock time at first render
    ChTimer m_timer_render;                           ///< timer for rendering speed
    double m_old_time, m_current_time, m_time_total;  ///< render times
    double m_fps;                                     ///< estimated FPS (moving average)

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
