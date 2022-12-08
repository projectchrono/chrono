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
// Radu Serban, Alessandro Tasora
// =============================================================================

//// RADU TODO
//// Allow attaching more than one ChSystem to the same Irrlicht visualization

#ifndef CH_VISUAL_SYSTEM_IRRLICHT_H
#define CH_VISUAL_SYSTEM_IRRLICHT_H

#include <string>
#include <unordered_set>

#include <irrlicht.h>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChObjFileShape.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChLineShape.h"

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrNodeModel.h"
#include "chrono_irrlicht/ChIrrEffects.h"
#include "chrono_irrlicht/ChIrrGUI.h"

namespace chrono {
namespace irrlicht {

/// Irrlicht-based Chrono run-time visualization system.
class ChApiIrr ChVisualSystemIrrlicht : virtual public ChVisualSystem {
  public:
    ChVisualSystemIrrlicht();
    virtual ~ChVisualSystemIrrlicht();

    /// Attach another Chrono system to the run-time visualization system.
    /// Currently only the first associated Chrono system is rendered. 
    virtual void AttachSystem(ChSystem* sys) override;

    /// Enable/disable antialias (default true).
    /// Must be called before Initialize().
    void SetAntialias(bool val);

    /// Enable/disable full-screen mode (default false).
    /// Must be called before Initialize().
    void SetFullscreen(bool val);

    /// Enable/disable shadows (default false).
    /// Must be called before Initialize().
    void SetShadows(bool val);

    /// Set the device driver type (default irr::video::EDT_DIRECT3D9).
    /// Must be called before Initialize().
    void SetDriverType(irr::video::E_DRIVER_TYPE driver_type);

    /// Set the window size (default 640x480).
    /// Must be called before Initialize().
    void SetWindowSize(unsigned int width, unsigned int height);

    /// Set the windoiw title (default "").
    /// Must be called before Initialize().
    void SetWindowTitle(const std::string& win_title);

    /// Use Y-up camera rendering (default CameraVerticalDir::Y).
    /// Must be called before Initialize().
    void SetCameraVertical(CameraVerticalDir vert);

    /// Set the Irrlicht logging level (default irr::ELL_INFORMATION).
    /// Must be called before Initialize().
    void SetLogLevel(irr::ELOG_LEVEL log_level);

    /// Set the scale for symbol drawing (default: 1).
    void SetSymbolScale(double scale);

    /// Initialize the visualization system.
    /// This creates the Irrlicht device using the current values for the optional device parameters.
    virtual void Initialize();

    /// Add a logo in a 3D scene.
    /// Has no effect, unles called after Initialize().
    void AddLogo(const std::string& logo_filename = GetChronoDataFile("logo_chronoengine_alpha.png"));

    /// Add a camera in an Irrlicht 3D scene.
    /// The camera rotation/pan is controlled by mouse left and right buttons, the zoom is controlled by mouse wheel or
    /// rmb+lmb+mouse, the position can be changed also with keyboard up/down/left/right arrows, the height can be
    /// changed with keyboard 'PgUp' and 'PgDn' keys. Optional parameters are position and target.
    /// Has no effect, unles called after Initialize().
    void AddCamera(const ChVector<>& pos, ChVector<> targ = VNULL);

    /// Add a sky box in a 3D scene.
    /// Note: it is assumed that the specified "texture_dir" directory contains the following three texture images:
    /// sky_lf.jpg, sky_up.jpg, sky_dn.jpg.
    /// Has no effect, unless called after Initialize().
    void AddSkyBox(const std::string& texture_dir = GetChronoDataFile("skybox/"));

    /// Add a directional light to the scene.
    /// Has no effect, unles called after Initialize().
    /// Light direction is defined by:
    /// - elevation (deg) between 0 (at the horizon) and 90 (above)
    /// - azimuth (deg) between 0 and 360, with 0 = South (-X), 90 = East (-Y), 180 = North (+X), 270 = West (+Y)
    irr::scene::ILightSceneNode* AddLightDirectional(double elevation = 60,                         ///< light elevation
                                                     double azimuth = 60,                            ///< light azimuth
                                                     ChColor ambient = ChColor(0.5f, 0.5f, 0.5f),   ///< ambient color
                                                     ChColor specular = ChColor(0.2f, 0.2f, 0.2f),  ///< specular color
                                                     ChColor diffuse = ChColor(1.0f, 1.0f, 1.0f)    ///< diffuse color
    );

    /// Add a point light to the scene.
    /// Has no effect, unles called after Initialize().
    irr::scene::ILightSceneNode* AddLight(const ChVector<>& pos,
                                          double radius,
                                          ChColor color = ChColor(0.7f, 0.7f, 0.7f));

    /// Add a point light that cast shadow (using soft shadows/shadow maps)
    /// Note that the quality of the shadow strictly depends on how close 'near_value' and 'far_value' are to the
    /// bounding box of the scene. Use AddShadow() to enable shadows for an object or for the entire scene.
    /// Has no effect, unless called after Initialize().
    irr::scene::ILightSceneNode* AddLightWithShadow(const ChVector<>& pos,
                                                    const ChVector<>& aim,
                                                    double radius,
                                                    double near_value,
                                                    double far_value,
                                                    double angle,
                                                    unsigned int resolution = 512,
                                                    ChColor color = ChColor(1, 1, 1),
                                                    bool directional = false,
                                                    bool clipborder = true);

    /// Simple shortcut to set two point lights in the scene.
    /// For more precise control on lights, use AddLight() or just use Irrlicht directly.
    /// Has no effect, unless called after Initialize().
    void AddTypicalLights();

    /// Attach a custom event receiver to the application.
    void AddUserEventReceiver(irr::IEventReceiver* receiver);

    /// Enable shadow maps for all visual models in a scene or only for a single physics item.
    /// A shadow-enabled light must be added to the scene with
    /// Shadow maps in Irrlicht may slow visualization a bit. Also, one must remember to add shadow-enabled light, using
    /// AddLightWithShadow().
    /// Has no effect, unless called after Initialize().
    void EnableShadows(std::shared_ptr<ChPhysicsItem> item = nullptr);

    /// Enable contact rendering (default: none).
    /// Has no effect, unless called after the visual system is initialized and attached.
    void EnableContactDrawing(ContactsDrawMode mode);

    /// Enable rendering of link (joint) frames (default: none).
    /// Has no effect, unless called after the visual system is initialized and attached.
    void EnableLinkDrawing(LinkDrawMode mode);

    /// Enable rendering of body frames (default: false).
    /// Has no effect, unless called after the visual system is initialized and attached.
    void EnableBodyFrameDrawing(bool val);

    /// Enable rendering of link frames (default: false).
    /// Has no effect, unless called after the visual system is initialized and attached.
    void EnableLinkFrameDrawing(bool val);

    /// Enable rendering of collision shapes (default: false).
    /// Has no effect, unless called after the visual system is initialized and attached.
    void EnableCollisionShapeDrawing(bool val);

    /// Enable modal analysis visualization (default: false).
    /// If true, visualize an oscillatory motion of the n-th mode (only if some ChModalAssembly is found).
    /// Otherwise, visualize the dynamic evolution of the associated system.
    virtual void EnableModalAnalysis(bool val) override;

    /// Set the mode to be shown (only if some ChModalAssembly is found).
    virtual void SetModalModeNumber(int val) override;

    /// Set the amplitude of the shown mode (only if some ChModalAssembly is found).
    virtual void SetModalAmplitude(double val) override;

    /// Set the speed of the shown mode (only if some ChModalAssembly is found).
    virtual void SetModalSpeed(double val) override;

    /// Show the realtime profiler in the 3D view.
    void ShowProfiler(bool val);

    /// Show the object explorer.
    void ShowExplorer(bool val);

    /// Show the info panel in the 3D view.
    void ShowInfoPanel(bool val);

    /// Set the active tab on the info panel.
    /// Has no effect, unless called after the visual system is initialized and attached.
    void SetInfoTab(int ntab);

    irr::IrrlichtDevice* GetDevice() { return m_device; }
    irr::video::IVideoDriver* GetVideoDriver() { return m_device->getVideoDriver(); }
    irr::scene::ISceneManager* GetSceneManager() { return m_device->getSceneManager(); }
    irr::scene::ICameraSceneNode* GetActiveCamera() { return m_device->getSceneManager()->getActiveCamera(); }
    irr::gui::IGUIEnvironment* GetGUIEnvironment() { return m_device->getGUIEnvironment(); }

    /// Process all visual assets in the associated ChSystem.
    /// This function is called by default by Initialize(), but can also be called later if further modifications to
    /// visualization assets occur.
    virtual void BindAll() override;

    /// Process the visual assets for the spcified physics item.
    /// This function must be called if a new physics item is added to the system or if changes to its visual model
    /// occur after the call to Initialize().
    virtual void BindItem(std::shared_ptr<ChPhysicsItem> item) override;

    /// Run the Irrlicht device.
    /// Returns `false` if the device wants to be deleted.
    bool Run();

    /// Clean the canvas at the beginning of each animation frame.
    virtual void BeginScene(bool backBuffer = true, bool zBuffer = true, ChColor color = ChColor(0, 0, 0));

    /// Draw all 3D shapes and GUI elements at the current frame.
    /// This function is typically called inside a loop such as
    /// <pre>
    ///    while(vis->Run()) {...}
    /// </pre>
    virtual void Render();

    /// End the scene draw at the end of each animation frame.
    virtual void EndScene();

    /// Return a fixed-size font for rendering GUI.
    irr::gui::IGUIFont* GetMonospaceFont() const { return m_monospace_font; }

    /// Create a snapshot of the last rendered frame and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) override;

  private:
    /// Create the ChIrrNodes for all visual model instances in the specified assembly.
    void CreateIrrNodes(const ChAssembly* assembly, std::unordered_set<const ChAssembly*>& trace);

    /// Create the ChIrrNodeModel for the visual model instance of the specified physics item.
    void CreateIrrNode(std::shared_ptr<ChPhysicsItem> item);

    /// Populate the ChIrrNodeModel for the visual model instance of the specified physics item.
    void PopulateIrrNode(irr::scene::ISceneNode* node,
                         std::shared_ptr<ChVisualModel> model,
                         const ChFrame<>& parent_frame);

    /// Purge Irrlicht nodes associated with a deleted physics item or with a deleted visual model.
    void PurgeIrrNodes();

    /// Add shadow to an Irrlicht node.
    void AddShadowToIrrNode(irr::scene::ISceneNode* node);

    /// Perform necessary setup operations at the beginning of a time step.
    virtual void OnSetup(ChSystem* sys) override;

    /// Perform necessary update operations at the end of a time step.
    virtual void OnUpdate(ChSystem* sys) override;

    /// Remove all visualization objects from this visualization system.
    virtual void OnClear(ChSystem* sys) override;

    std::unordered_map<ChPhysicsItem*, std::shared_ptr<ChIrrNodeModel>> m_nodes;

    bool m_yup;                                        ///< use Y-up if true, Z-up if false
    std::string m_win_title;                           ///< window title
    irr::SIrrlichtCreationParameters m_device_params;  ///< Irrlicht device parameters
    irr::IrrlichtDevice* m_device;                     ///< Irrlicht visualization device
    irr::gui::IGUIFont* m_monospace_font;              ///< Fixed-size font
    irr::scene::ISceneNode* m_container;               ///< Irrlicht scene container
    std::unique_ptr<ChIrrGUI> m_gui;                   ///< associated Irrlicht GUI and event receiver
    std::unique_ptr<EffectHandler> m_effect_handler;   ///< effect handler for shadow maps
    bool m_use_effects;                                ///< flag to enable/disable effects
    bool m_modal;                                      ///< visualize modal analysis

    // shared meshes
    irr::scene::IAnimatedMesh* sphereMesh;
    irr::scene::IMesh* cubeMesh;
    irr::scene::IMesh* cylinderMesh;
    irr::scene::IMesh* capsuleMesh;
};

}  // namespace irrlicht
}  // namespace chrono

#endif
