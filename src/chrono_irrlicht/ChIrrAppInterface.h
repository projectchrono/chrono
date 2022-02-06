// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHIRRAPPINTERFACE_H
#define CHIRRAPPINTERFACE_H

#include <vector>

#include <irrlicht.h>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrEffects.h"
#include "chrono_irrlicht/ChIrrTools.h"
#include "chrono_irrlicht/ChIrrCamera.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChPovRay.h"
#endif

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

// Forward reference
class ChIrrAppEventReceiver;

/// Vertical direction
enum class VerticalDir { Y, Z };

/// Class to add some GUI to Irrlicht + Chrono applications.
/// This basic GUI can be used to monitor solver timings, to easily change physical system settings, etc.
class ChApiIrr ChIrrAppInterface {
  public:
    /// Create the Irrlicht context (device, etc.)
    ChIrrAppInterface(ChSystem* sys,
                      const std::wstring& title = L"Chrono",
                      const irr::core::dimension2d<irr::u32>& dimens = irr::core::dimension2d<irr::u32>(640, 480),
                      VerticalDir vert = VerticalDir::Y,
                      bool do_fullscreen = false,
                      bool do_shadows = false,
                      bool do_antialias = true,
                      irr::video::E_DRIVER_TYPE mydriver = irr::video::EDT_DIRECT3D9,
                      irr::ELOG_LEVEL log_level = irr::ELL_INFORMATION);

    /// Delete all Irrlicht items (including the Irrlicht scene nodes)
    virtual ~ChIrrAppInterface();

    //// Accessor functions
    irr::IrrlichtDevice* GetDevice() { return device; }
    irr::video::IVideoDriver* GetVideoDriver() { return device->getVideoDriver(); }
    irr::scene::ISceneManager* GetSceneManager() { return device->getSceneManager(); }
    irr::scene::ICameraSceneNode* GetActiveCamera() { return device->getSceneManager()->getActiveCamera(); }
    irr::gui::IGUIEnvironment* GetIGUIEnvironment() { return device->getGUIEnvironment(); }
    EffectHandler* GetEffects() { return effect.get(); }
    irr::scene::ISceneNode* GetContainer() { return container; }
    ChSystem* GetSystem() { return system; }

    /// Show the info panel in the 3D view
    void SetShowInfos(bool val) { show_infos = val; }
    bool GetShowInfos() { return show_infos; }

    void SetInfosTab(int ntab) { this->gad_tabbed->setActiveTab(ntab); }

    /// Show the realtime profiler in the 3D view
    void SetShowProfiler(bool val) { show_profiler = val; }
    bool GetShowProfiler() { return show_profiler; }

    /// Show the object explorer
    void SetShowExplorer(bool val) { show_explorer = val; }
    bool GetShowExplorer() { return show_explorer; }

    /// Set/Get the time step for time integration. This value is used when
    /// calling DoStep() in a loop, to advance the simulation by one timestep.
    void SetTimestep(double val);
    double GetTimestep() { return timestep; }

    /// If set to true, you can use DoStep() in the simulation loop to advance the
    /// simulation by one timestep. Otherwise, you have to handle the time
    /// stepping by yourself, e.g. by calling ChSystem::DoStepDynamics().
    /// Default: true.
    void SetStepManage(bool val) { step_manage = val; }

    /// If enabled, the function DoStep() will enforce soft real-time, by spinning in place until simulation time
    /// catches up with real time.
    void SetTryRealtime(bool val) { try_realtime = val; }

    /// Set/Get the simulation state (running or paused)
    void SetPaused(bool val) { pause_step = val; }
    bool GetPaused() { return pause_step; }

    /// If set to true, each frame of the animation will be saved on the disk
    /// as snapshot0001.bmp, snapshot0002.bmp, etc.
    void SetVideoframeSave(bool val) { videoframe_save = val; }
    bool GetVideoframeSave() { return videoframe_save; }

    /// Set to 1 if you need to save on disk all simulation steps, set to 2 for
    /// saving each 2 steps, etc.
    void SetVideoframeSaveInterval(int val) { videoframe_each = val; }
    int GetVideoframeSaveInterval() { return videoframe_each; }

    /// If set true, instead of doing time integration in myapplication.DoStep() it 
    /// just shows an oscillatory motion of the nth mode (only if some ChModalAssembly is found)
    void SetModalShow(bool val) { modal_show = val; }
    bool GetModalShow() { return modal_show; }

    /// When in SetModalShow(true), use this to pick the n-th mode to show (only if some ChModalAssembly is found)
    void SetModalModeNumber(int val) { modal_mode_n = val; }
    int  GetModalModeNumber() { return modal_mode_n; }

    /// When in SetModalShow(true), this sets the amplitude of shown mode (only if some ChModalAssembly is found)
    void SetModalAmplitude(double val) { modal_amplitude = val; }
    double GetModalAmplitude() { return modal_amplitude; }

    /// When in SetModalShow(true), this sets the speed of shown mode (only if some ChModalAssembly is found)
    void SetModalSpeed(double val) { modal_speed = val; }
    double GetModalSpeed() { return modal_speed; }

#ifdef CHRONO_POSTPROCESS

    /// If set to true, each frame of the animation will be saved on the disk
    /// as a sequence of scripts to be rendered via POVray. Only if solution build with ENABLE_MODULE_POSTPROCESS.
    void SetPOVraySave(bool val);
    bool GetPOVraySave() { return povray_save; }

    /// Set to 1 if you need to save on disk all simulation steps, set to 2 for
    /// saving each 2 steps, etc.
    void SetPOVraySaveInterval(int val) { povray_each = val; }
    int GetPOVrayframeSaveInterval() { return povray_each; }

    /// Access the internal ChPovRay exporter, for advanced tweaking.
    /// Returns 0 if not yet started (use SetPOVraySave(true) to start it)
    postprocess::ChPovRay* GetPOVrayexporter() { return pov_exporter.get(); }

#endif

    /// Set the label mode for contacts
    void SetContactsLabelMode(IrrContactsLabelMode mm) { this->gad_labelcontacts->setSelected((int)mm); }
    /// Set the draw mode for contacts
    void SetContactsDrawMode(IrrContactsDrawMode mm) { this->gad_drawcontacts->setSelected((int)mm); }
    /// Set the label mode for links
    void SetLinksLabelMode(IrrLinkLabelMode mm) { this->gad_labellinks->setSelected((int)mm); }
    /// Set the draw mode for links
    void SetLinksDrawMode(IrrLinkDrawMode mm) { this->gad_drawlinks->setSelected((int)mm); }
    /// Set if the AABB collision shapes will be plotted
    void SetPlotAABB(bool val) { this->gad_plot_aabb->setChecked(val); }
    /// Set if the COG frames will be plotted
    void SetPlotCOGFrames(bool val) { this->gad_plot_cogs->setChecked(val); }
    /// Set if the collision shapes will be plotted
    void SetPlotCollisionShapes(bool val) { this->gad_plot_collisionshapes->setChecked(val); }
    /// Set if the link frames will be plotted
    void SetPlotLinkFrames(bool val) { this->gad_plot_linkframes->setChecked(val); }
    /// Set if the COG frames will be plotted
    void SetPlotConvergence(bool val) { this->gad_plot_convergence->setChecked(val); }

    /// Set the scale for symbol drawing (link frames, COGs, etc.)
    void SetSymbolscale(double val);
    double GetSymbolscale() { return symbolscale; }

    /// Use this function to hook a custom event receiver to the application.
    void SetUserEventReceiver(irr::IEventReceiver* mreceiver) { user_receivers.push_back(mreceiver); }

    /// Set the fonts to be used from now on. Note that the font must be in the
    /// XML format of Irrlicht - this can be generated using a tool provided with
    /// Irrlicht.
    void SetFonts(const std::string& mfontdir = GetChronoDataFile("fonts/arial8.xml"));

    /// Call this to clean the canvas at the beginning of each animation frame
    virtual void BeginScene(bool backBuffer = true,
                            bool zBuffer = true,
                            irr::video::SColor color = irr::video::SColor(255, 0, 0, 0));

    /// Call this function inside a loop such as
    /// <pre>
    ///    while(application.GetDevice()->run()) {...}
    /// </pre>
    /// in order to advance the dynamics by one timestep. The value of the timestep can be set via SetTimestep().
    /// Optionally, you can use SetTryRealtime(true) if your simulation can run in realtime; this will enforce soft
    /// real-time. Alternatively, to use ChSystem::DoStepDynamics() directly in the loop, use SetStepManage(false).
    virtual void DoStep();

    /// Call this function inside a loop such as
    /// <pre>
    ///    while(application.GetDevice()->run()) {...}
    /// </pre>
    /// to draw all 3D shapes and GUI elements at the current frame.
    virtual void DrawAll();

    /// Call this to end the scene draw at the end of each animation frame
    virtual void EndScene();

    /// Dump the last used system matrices and vectors in the current directory,
    /// as 'dump_xxxx.dat' files that can be loaded with Matlab for debugging,
    /// benchmarking etc. It saves M mass matrix, Cq jacobians, E compliance
    /// as Matlab sparse matrix format, and known vectors fb, bi as column Matlab
    /// matrices.
    void DumpSystemMatrices();

    /// Add a logo in a 3D scene.
    void AddLogo(const std::string& mlogofilename = GetChronoDataFile("logo_chronoengine_alpha.png"));

    /// Add a Maya-like camera in an Irrlicht 3D scene.
    /// The camera rotation/pan is controlled by mouse left and right buttons, the zoom is controlled by mouse wheel or
    /// rmb+lmb+mouse, the position can be changed also with keyboard up/down/left/right arrows, the height can be
    /// changed with keyboard 'PgUp' and 'PgDn' keys. Optional parameters are position and target. Note: if you want
    /// more precise control on camera specs, just use plain commands of Irrlicht.
    void AddCamera(irr::core::vector3df pos = irr::core::vector3df(0, 0, -8),
                   irr::core::vector3df targ = irr::core::vector3df(0, 0, 0));

    /// Add a sky box in a 3D scene.
    /// Note: it is assumed that the specified "texturedir" directory contains the following three texture images:
    /// sky_lf.jpg, sky_up.jpg, sky_dn.jpg
    void AddSkyBox(const std::string& texturedir = GetChronoDataFile("skybox/"));

    /// Add a point light to the scene.
    irr::scene::ILightSceneNode* AddLight(irr::core::vector3df pos,
                                          double radius,
                                          irr::video::SColorf color = irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f));

    /// Add a point light that cast shadow (using soft shadows/shadow maps)
    /// Note that the quality of the shadow strictly depends on how you set 'mnear'
    /// and 'mfar' parameters as close as possible to the bounding box of the scene.
    /// NOTE: use myapplication.AddShadow(myitem) to enable shadow for an object!
    /// Otherwise, use myapplication.AddShadowAll().
    irr::scene::ILightSceneNode* AddLightWithShadow(irr::core::vector3df pos,
                                                    irr::core::vector3df aim,
                                                    double radius,
                                                    double mnear,
                                                    double mfar,
                                                    double angle,
                                                    irr::u32 resolution = 512,
                                                    irr::video::SColorf color = irr::video::SColorf(1.f, 1.f, 1.f, 1.f),
                                                    bool directional = false,
                                                    bool clipborder = true);

    /// Simple shortcut to set two point lights in the scene.
    /// Note: if you want more precise control on lights, use AddLight() or just use Irrlicht directly.
    void AddTypicalLights();

  private:
    void DrawCollisionShapes(irr::video::SColor color);

    ChSystem* system;                                  ///< associated Chrono system
    irr::IrrlichtDevice* device;                       ///< Irrlicht visualization device
    std::unique_ptr<EffectHandler> effect;             ///< effect handler for shadow maps
    bool use_effects;                                  ///< flag to enable/disable effects
    std::unique_ptr<ChIrrAppEventReceiver> receiver;   ///< default event receiver
    std::vector<irr::IEventReceiver*> user_receivers;  ///< optional user-defined receivers
    irr::scene::ISceneNode* container;                 ///< Irrliicht scene container

    /// Collision visualization callback object
    std::shared_ptr<collision::ChCollisionSystem::VisualizationCallback> m_drawer;

    bool y_up;

    bool show_infos;
    bool show_profiler;
    bool show_explorer;

    bool step_manage;
    bool pause_step;
    bool try_realtime;
    double timestep;
    bool do_single_step;
    bool videoframe_save;
    int videoframe_num;
    int videoframe_each;

    bool modal_show;
    int  modal_mode_n;
    double modal_amplitude;
    double modal_speed;
    double modal_phi;
    double modal_current_mode_n;
    double modal_current_freq;

#ifdef CHRONO_POSTPROCESS
    bool povray_save;
    std::unique_ptr<postprocess::ChPovRay> pov_exporter;
    int povray_num;
    int povray_each;
#endif

    double symbolscale;

    double camera_auto_rotate_speed;

    ChRealtimeStepTimer m_realtime_timer;

    irr::gui::IGUITabControl* gad_tabbed;
    irr::gui::IGUITab* gad_tab1;
    irr::gui::IGUITab* gad_tab2;
    irr::gui::IGUITab* gad_tab3;

    irr::gui::IGUIStaticText* gad_textFPS;
    irr::gui::IGUIComboBox* gad_drawcontacts;
    irr::gui::IGUIComboBox* gad_labelcontacts;
    irr::gui::IGUIComboBox* gad_drawlinks;
    irr::gui::IGUIComboBox* gad_labellinks;
    irr::gui::IGUICheckBox* gad_plot_aabb;
    irr::gui::IGUICheckBox* gad_plot_cogs;
    irr::gui::IGUICheckBox* gad_plot_collisionshapes;
    irr::gui::IGUICheckBox* gad_plot_linkframes;
    irr::gui::IGUICheckBox* gad_plot_convergence;

    irr::gui::IGUIScrollBar* gad_speed_iternumber;
    irr::gui::IGUIStaticText* gad_speed_iternumber_info;
    irr::gui::IGUIScrollBar* gad_clamping;
    irr::gui::IGUIStaticText* gad_clamping_info;
    irr::gui::IGUIScrollBar* gad_minbounce;
    irr::gui::IGUIStaticText* gad_minbounce_info;
    irr::gui::IGUICheckBox* gad_usesleep;
    irr::gui::IGUIComboBox* gad_ccpsolver;
    irr::gui::IGUIComboBox* gad_stepper;
    irr::gui::IGUIEditBox* gad_timestep;
    irr::gui::IGUIStaticText* gad_timestep_info;
    irr::gui::IGUICheckBox* gad_try_realtime;
    irr::gui::IGUICheckBox* gad_pause_step;
    irr::gui::IGUIEditBox* gad_symbolscale;
    irr::gui::IGUIStaticText* gad_symbolscale_info;
    irr::gui::IGUIStaticText* gad_textHelp;

    irr::gui::IGUICheckBox* gad_modal_show;
    irr::gui::IGUIScrollBar* gad_modal_mode_n;
    irr::gui::IGUIStaticText* gad_modal_mode_n_info;

    irr::gui::IGUITreeView* gad_treeview;

    friend class ChIrrAppEventReceiver;
};

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
