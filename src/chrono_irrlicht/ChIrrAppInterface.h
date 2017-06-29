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
#include "chrono_irrlicht/ChIrrWizard.h"

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

// Forward reference
class ChIrrAppEventReceiver;

/// Class to add some GUI to Irrlicht + ChronoEngine applications.
/// This basic GUI can be used to monitor solver timings, to easily change
/// physical system settings, etc.
class ChApiIrr ChIrrAppInterface {
  public:
    /// Create the IRRLICHT context (device, etc.)
    ChIrrAppInterface(ChSystem* psystem,
                      const wchar_t* title = 0,
                      irr::core::dimension2d<irr::u32> dimens = irr::core::dimension2d<irr::u32>(640, 480),
                      bool do_fullscreen = false,
                      bool do_shadows = false,
                      bool do_antialias = true,
                      irr::video::E_DRIVER_TYPE mydriver = irr::video::EDT_DIRECT3D9);

    /// Safely delete all Irrlicht items (including the Irrlicht scene nodes)
    virtual ~ChIrrAppInterface();

    //// Accessor functions
    irr::IrrlichtDevice* GetDevice() { return device; }
    irr::video::IVideoDriver* GetVideoDriver() { return device->getVideoDriver(); }
    irr::scene::ISceneManager* GetSceneManager() { return device->getSceneManager(); }
    irr::gui::IGUIEnvironment* GetIGUIEnvironment() { return device->getGUIEnvironment(); }
    EffectHandler* GetEffects() { return effect; }
    irr::scene::ISceneNode* GetContainer() { return container; }
    ChSystem* GetSystem() { return system; }

    /// Show the info panel in the 3D view
    void SetShowInfos(bool val) { show_infos = val; }
    bool GetShowInfos() { return show_infos; }

    /// Set/Get the time step for time integration. This value is used when
    /// calling DoStep() in a loop, to advance the simulation by one timestep.
    void SetTimestep(double val);
    double GetTimestep() { return timestep; }

    /// If set to true, you can use DoStep() in the simulation loop to advance the
    /// simulation by one timestep. Otherwise, you have to handle the time
    /// stepping by yourself, e.g. by calling ChSystem::DoStepDynamics().
    void SetStepManage(bool val) { step_manage = val; }
    bool GetStepManage() { return step_manage; }

    /// If set to true, the function DoStep() will try to use a timestep that is
    /// the same as that used to refresh the interface and compute physics (i.e.,
    /// it attempts to keep soft-realtime performance).
    /// Note: the realtime step is bounded above by the timestep specified through
    /// SetTimestep()! This clamping will happen if there's too much load.
    void SetTryRealtime(bool val) { try_realtime = val; }
    bool GetTryRealtime() { return try_realtime; }

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

    /// Set the label mode for contacts
    void SetContactsLabelMode(ChIrrTools::eCh_ContactsLabelMode mm) { this->gad_labelcontacts->setSelected((int)mm); }
    /// Set the draw mode for contacts
    void SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode mm) { this->gad_drawcontacts->setSelected((int)mm); }
    /// Set the label mode for links
    void SetLinksLabelMode(ChIrrTools::eCh_LinkLabelMode mm) { this->gad_labellinks->setSelected((int)mm); }
    /// Set the draw mode for links
    void SetLinksDrawMode(ChIrrTools::eCh_LinkDrawMode mm) { this->gad_drawlinks->setSelected((int)mm); }
	/// Set if the AABB collision shapes will be plotted
    void SetPlotAABB(bool val) { this->gad_plot_aabb->setChecked(val); }
	/// Set if the COG frames will be plotted
	void SetPlotCOGFrames(bool val) { this->gad_plot_cogs->setChecked(val); }
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

    /// Call this important function inside a cycle like
    ///    while(application.GetDevice()->run()) {...}
    /// in order to advance the physics by one timestep. The value of the timestep
    /// can be set via SetTimestep(). Optionally, you can use SetTryRealtime(true)
    /// if your simulation can run in realtime.
    /// Alternatively, if you want to use ChSystem::DoStepDynamics() directly in
    /// your loop, use SetStepManage(false).
    virtual void DoStep();

    /// Call this important function inside a loop like
    ///    while(application.GetDevice()->run()) {...}
    /// in order to get the redrawing of all 3D shapes and all the GUI elements.
    virtual void DrawAll();

    /// Call this to end the scene draw at the end of each animation frame
    virtual void EndScene();

    /// Dump the last used system matrices and vectors in the current directory,
    /// as 'dump_xxxx.dat' files that can be loaded with Matlab for debugging,
    /// benchmarking etc. It saves M mass matrix, Cq jacobians, E compliance
    /// as Matlab sparse matrix format, and known vectors fb, bi as column Matlab
    /// matrices.
    void DumpSystemMatrices();

    //
    // Some wizard functions for 'easy setup' of the application window:
    //

    void AddTypicalLogo(const std::string& mlogofilename = GetChronoDataFile("logo_chronoengine_alpha.png")) {
        ChIrrWizard::add_typical_Logo(GetDevice(), mlogofilename);
    }

    void AddTypicalCamera(irr::core::vector3df mpos = irr::core::vector3df(0, 0, -8),
                          irr::core::vector3df mtarg = irr::core::vector3df(0, 0, 0)) {
        ChIrrWizard::add_typical_Camera(GetDevice(), mpos, mtarg);
    }

    void AddTypicalLights(irr::core::vector3df pos1 = irr::core::vector3df(30.f, 100.f, 30.f),
                          irr::core::vector3df pos2 = irr::core::vector3df(30.f, 80.f, -30.f),
                          double rad1 = 290,
                          double rad2 = 190,
                          irr::video::SColorf col1 = irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f),
                          irr::video::SColorf col2 = irr::video::SColorf(0.7f, 0.8f, 0.8f, 1.0f)) {
        ChIrrWizard::add_typical_Lights(GetDevice(), pos1, pos2, rad1, rad2, col1, col2);
    }

    void AddTypicalSky(const std::string& mtexturedir = GetChronoDataFile("skybox/")) {
        ChIrrWizard::add_typical_Sky(GetDevice(), mtexturedir);
    }

    /// Add a point light to the scene
    irr::scene::ILightSceneNode* AddLight(irr::core::vector3df pos,
                                          double radius,
                                          irr::video::SColorf color = irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f)) {
        irr::scene::ILightSceneNode* mlight =
            device->getSceneManager()->addLightSceneNode(0, pos, color, (irr::f32)radius);
        return mlight;
    }

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
                                                    bool clipborder = true) {
        irr::scene::ILightSceneNode* mlight =
            device->getSceneManager()->addLightSceneNode(0, pos, color, (irr::f32)radius);
        effect->addShadowLight(SShadowLight(resolution, pos, aim, color, (irr::f32)mnear, (irr::f32)mfar,
                                            ((irr::f32)angle * irr::core::DEGTORAD), directional));
        if (clipborder == false) {
            effect->getShadowLight(effect->getShadowLightCount() - 1).setClipBorder(clipborder);
        }
        use_effects = true;
        return mlight;
    }

  private:
    // The Irrlicht engine:
    irr::IrrlichtDevice* device;

    // Xeffects for shadow maps!
    EffectHandler* effect;
    bool use_effects;

    // The ChronoEngine system:
    ChSystem* system;

    ChIrrAppEventReceiver* receiver;

    std::vector<irr::IEventReceiver*> user_receivers;

    irr::scene::ISceneNode* container;

    bool show_infos;

    bool step_manage;
    bool pause_step;
    bool try_realtime;
    double timestep;
    bool do_single_step;
    bool videoframe_save;
    int videoframe_num;
    int videoframe_each;
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
    irr::gui::IGUICheckBox* gad_plot_linkframes;
    irr::gui::IGUICheckBox* gad_plot_convergence;

    irr::gui::IGUIScrollBar* gad_speed_iternumber;
    irr::gui::IGUIStaticText* gad_speed_iternumber_info;
    irr::gui::IGUIScrollBar* gad_pos_iternumber;
    irr::gui::IGUIStaticText* gad_pos_iternumber_info;
    irr::gui::IGUIScrollBar* gad_omega;
    irr::gui::IGUIStaticText* gad_omega_info;
    irr::gui::IGUIScrollBar* gad_lambda;
    irr::gui::IGUIStaticText* gad_lambda_info;
    irr::gui::IGUIScrollBar* gad_clamping;
    irr::gui::IGUIStaticText* gad_clamping_info;
    irr::gui::IGUIScrollBar* gad_minbounce;
    irr::gui::IGUIStaticText* gad_minbounce_info;
    irr::gui::IGUIScrollBar* gad_dt;
    irr::gui::IGUIStaticText* gad_dt_info;
    irr::gui::IGUICheckBox* gad_warmstart;
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

  public:
    std::shared_ptr<ChLinkSpring>* selectedspring;
    std::shared_ptr<ChBody>* selectedtruss;
    std::shared_ptr<ChBody>* selectedmover;
    ChVector<> selectedpoint;
    double selecteddist;

    friend class ChIrrAppEventReceiver;
};

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
