//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHIRRAPPINTERFACE_H
#define CHIRRAPPINTERFACE_H


#include <irrlicht.h>

#include "core/ChRealtimeStep.h"
#include "physics/ChSystem.h"

#include "unit_IRRLICHT/ChApiIrr.h"
#include "unit_IRRLICHT/ChIrrWizard.h"
#include "unit_IRRLICHT/ChIrrTools.h"
#include "unit_IRRLICHT/ChIrrEffects.h"

namespace irr
{

// Forward reference
class ChIrrAppEventReceiver;

/// Class to add some GUI to Irrlicht + Chrono::Engine applications.
/// This basic GUI can be used to monitor solver timings, to easily change
/// physical system settings, etc.

class ChApiIrr ChIrrAppInterface
{
public:

  /// Create the IRRLICHT context (device, etc.)
  ChIrrAppInterface(chrono::ChSystem*      psystem,
                    const wchar_t*         title =0,
                    core::dimension2d<u32> dimens = core::dimension2d<u32>(640,480),
                    bool                   do_fullscreen = false,
                    bool                   do_shadows = false,
                    video::E_DRIVER_TYPE   mydriver = video::EDT_DIRECT3D9);

  /// Safely delete all Irrlicht items (including the Irrlicht scene nodes)
  ~ChIrrAppInterface();

  //// Accessor functions
  IrrlichtDevice*       GetDevice()          {return device;}
  video::IVideoDriver*  GetVideoDriver()     {return device->getVideoDriver();}
  scene::ISceneManager* GetSceneManager()    {return device->getSceneManager();}
  gui::IGUIEnvironment* GetIGUIEnvironment() {return device->getGUIEnvironment();}
  EffectHandler*        GetEffects()         {return effect;}
  scene::ISceneNode*    GetContainer()       {return container;}
  chrono::ChSystem*     GetSystem()          {return system;}

  /// Show the info panel in the 3D view
  void SetShowInfos(bool val) {show_infos= val;}
  bool GetShowInfos()         {return show_infos;}

  /// Set/Get the time step for time integration. This value is used when
  /// calling DoStep() in a loop, to advance the simulation by one timestep.
  void SetTimestep(double val);
  double GetTimestep()          {return timestep;}

  /// If set to true, you can use DoStep() in the simulation loop to advance the
  /// simulation by one timestep. Otherwise, you have to handle the time
  /// stepping by yourself, e.g. by calling ChSystem::DoStepDynamics().
  void SetStepManage(bool val)  {step_manage = val;}
  bool GetStepManage()          {return step_manage;}

  /// If set to true, the function DoStep() will try to use a timestep that is
  /// the same as that used to refresh the interface and compute physics (i.e.,
  /// it attempts to keep soft-realtime performance).
  /// Note: the realtime step is bounded above by the timestep specified through
  /// SetTimestep()! This clamping will happen if there's too much load.
  void SetTryRealtime(bool val) {try_realtime = val;}
  bool GetTryRealtime()         {return try_realtime;}

  /// Set/Get the simulation state (running or paused)
  void SetPaused(bool val)  {pause_step = val;}
  bool GetPaused()          {return pause_step;}

  /// If set to true, each frame of the animation will be saved on the disk
  /// as snapshot0001.bmp, snapshot0002.bmp, etc.
  void SetVideoframeSave(bool val)  {videoframe_save = val;}
  bool GetVideoframeSave()          {return videoframe_save;}

  /// Set to 1 if you need to save on disk all simulation steps, set to 2 for
  /// saving each 2 steps, etc.
  void SetVideoframeSaveInterval(int val) {videoframe_each = val;}
  int  GetVideoframeSaveInterval()        {return videoframe_each;}

  /// Set the scale for symbol drawing (link frames, COGs, etc.)
  void SetSymbolscale(double val);
  double GetSymbolscale()           {return symbolscale;}

  /// Use this function to hook a custom event receiver to the application.
  void SetUserEventReceiver(IEventReceiver* mreceiver) {user_receiver = mreceiver;}

  /// Set the fonts to be used from now on. Note that the font must be in the
  /// XML format of Irrlicht - this can be generated using a tool provided with
  /// Irrlicht.
  void SetFonts(const char* mfontdir = "../data/fonts/arial8.xml");

  /// Call this to clean the canvas at the beginning of each animation frame
  void BeginScene(bool          backBuffer=true,
                  bool          zBuffer=true,
                  video::SColor color=video::SColor(255,0,0,0));

  /// Call this important function inside a cycle like
  ///    while(application.GetDevice()->run()) {...}
  /// in order to advance the physics by one timestep. The value of the timestep
  /// can be set via SetTimestep(). Optionally, you can use SetTryRealtime(true)
  /// if your simulation can run in realtime.
  /// Alternatively, if you want to use ChSystem::DoStepDynamics() directly in
  /// your loop, use SetStepManage(false).
  void DoStep();

  /// Call this important function inside a loop like
  ///    while(application.GetDevice()->run()) {...}
  /// in order to get the redrawing of all 3D shapes and all the GUI elements.
  void DrawAll();

  /// Call this to end the scene draw at the end of each animation frame
  void EndScene();

  /// Dump the last used system matrices and vectors in the current directory,
  /// as 'dump_xxxx.dat' files that can be loaded with Matlab for debugging,
  /// benchmarking etc. It saves M mass matrix, Cq jacobians, E compliance
  /// as Matlab sparse matrix format, and known vectors fb, bi as column Matlab
  /// matrices.
  void DumpMatrices();

  //
  // Some wizard functions for 'easy setup' of the application window:
  //

  void AddTypicalLogo(const char* mtexturedir = "../data/",
                      const char* mlogofilename = "logo_chronoengine_alpha.png")
  {
    ChIrrWizard::add_typical_Logo(GetDevice(), mtexturedir,mlogofilename);
  }

  void AddTypicalCamera(core::vector3df mpos = core::vector3df(0,0,-8),
                        core::vector3df mtarg = core::vector3df(0,0,0))
  {
    ChIrrWizard::add_typical_Camera(GetDevice(), mpos, mtarg);
  }

  void AddTypicalLights(core::vector3df pos1 = core::vector3df(30.f, 100.f,  30.f),
                        core::vector3df pos2 = core::vector3df(30.f, 80.f, -30.f),
                        double rad1 = 290, double rad2 = 190,
                        video::SColorf col1  = video::SColorf(0.7f,0.7f,0.7f,1.0f),
                        video::SColorf col2  = video::SColorf(0.7f,0.8f,0.8f,1.0f))
  {
    ChIrrWizard::add_typical_Lights(GetDevice(), pos1,pos2, rad1, rad2, col1, col2);
  }

  void AddTypicalSky(const char* mtexturedir = "../data/skybox/")
  {
    ChIrrWizard::add_typical_Sky(GetDevice(), mtexturedir);
  }

  /// Add a point light to the scene
  scene::ILightSceneNode* AddLight(core::vector3df pos,
                                   double radius,
                                   video::SColorf color = video::SColorf(0.7f,0.7f,0.7f,1.0f))
  {
    scene::ILightSceneNode* mlight = device->getSceneManager()->addLightSceneNode( 0,pos, color, (f32)radius);
    return mlight;
  }

  /// Add a point light that cast shadow (using soft shadows/shadow maps)
  /// Note that the quality of the shadow strictly depends on how you set 'mnear'
  /// and 'mfar' parameters as close as possible to the bounding box of the scene. 
  /// NOTE: use myapplication.AddShadow(myitem) to enable shadow for an object!
  /// Otherwise, use myapplication.AddShadowAll().
  scene::ILightSceneNode* AddLightWithShadow(core::vector3df pos,
                                             core::vector3df aim,
                                             double radius,
                                             double mnear,
                                             double mfar,
                                             double angle,
                                             u32 resolution=512,
                                             video::SColorf color = video::SColorf(1.f,1.f,1.f,1.f))
  {
    scene::ILightSceneNode* mlight =  device->getSceneManager()->addLightSceneNode( 0,pos, color, (f32)radius);
    effect->addShadowLight(SShadowLight(resolution, pos, aim, color, (f32)mnear , (f32)mfar, ((f32)angle * core::DEGTORAD)));
    use_effects = true;
    return mlight;
  }


private:
  // The Irrlicht engine:
  IrrlichtDevice* device;

  // Xeffects for shadow maps!
  EffectHandler* effect; 
  bool use_effects;

  // The Chrono::Engine system:
  chrono::ChSystem* system;

  ChIrrAppEventReceiver* receiver;

  IEventReceiver* user_receiver;

  scene::ISceneNode* container;

  bool show_infos;

  bool step_manage;
  bool pause_step;
  bool try_realtime;
  double timestep;
  bool do_single_step;
  bool videoframe_save;
  int  videoframe_num;
  int  videoframe_each;
  double symbolscale;

  chrono::ChRealtimeStepTimer m_realtime_timer;

  gui::IGUITabControl* gad_tabbed;
  gui::IGUITab*        gad_tab1;
  gui::IGUITab*        gad_tab2;
  gui::IGUITab*        gad_tab3;

  gui::IGUIStaticText* gad_textFPS;
  gui::IGUIComboBox*   gad_drawcontacts;
  gui::IGUIComboBox*   gad_labelcontacts;
  gui::IGUICheckBox*   gad_plot_aabb;
  gui::IGUICheckBox*   gad_plot_cogs;
  gui::IGUICheckBox*   gad_plot_linkframes;
  gui::IGUICheckBox*   gad_plot_convergence;

  gui::IGUIScrollBar*  gad_speed_iternumber;
  gui::IGUIStaticText* gad_speed_iternumber_info;
  gui::IGUIScrollBar*  gad_pos_iternumber;
  gui::IGUIStaticText* gad_pos_iternumber_info;
  gui::IGUIScrollBar*  gad_omega;
  gui::IGUIStaticText* gad_omega_info;
  gui::IGUIScrollBar*  gad_lambda;
  gui::IGUIStaticText* gad_lambda_info;
  gui::IGUIScrollBar*  gad_clamping;
  gui::IGUIStaticText* gad_clamping_info;
  gui::IGUIScrollBar*  gad_minbounce;
  gui::IGUIStaticText* gad_minbounce_info;
  gui::IGUIScrollBar*  gad_dt;
  gui::IGUIStaticText* gad_dt_info;
  gui::IGUICheckBox*   gad_warmstart;
  gui::IGUICheckBox*   gad_usesleep;
  gui::IGUIComboBox*   gad_ccpsolver;
  gui::IGUIComboBox*   gad_stepper;
  gui::IGUIEditBox*    gad_timestep;
  gui::IGUIStaticText* gad_timestep_info;
  gui::IGUICheckBox*   gad_try_realtime;
  gui::IGUICheckBox*   gad_pause_step;
  gui::IGUIEditBox*    gad_symbolscale;
  gui::IGUIStaticText* gad_symbolscale_info;
  gui::IGUIStaticText* gad_textHelp;

public:
  chrono::ChSharedPtr<chrono::ChLinkSpring>* selectedspring;
  chrono::ChSharedPtr<chrono::ChBody>* selectedtruss;
  chrono::ChSharedPtr<chrono::ChBody>* selectedmover;
  chrono::ChVector<> selectedpoint;
  double selecteddist;

friend class ChIrrAppEventReceiver;
};



} // END_OF_NAMESPACE____

#endif

