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

#include "chrono/collision/ChCModelBullet.h"
#include "chrono/core/ChStream.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveExplorer.h"
#include "chrono/utils/ChProfiler.h"

#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrCamera.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace irrlicht {

    // -----------------------------------------------------------------------------
    // ChIrrAppEventReceiver
    //
    // A custom Irrlicht Event Receiver class.
    // -----------------------------------------------------------------------------

    class ChIrrAppEventReceiver : public irr::IEventReceiver
    {
    public:
        ChIrrAppEventReceiver(ChIrrAppInterface* m_app)
            : app(m_app)
        {
        }
        bool OnEvent(const irr::SEvent& event);

    private:
        ChIrrAppInterface* app;
    };

    bool ChIrrAppEventReceiver::OnEvent(const irr::SEvent& event)
    {
        // Check if there are any user-specified event receivers. Give them the
        // first chance to process the event (in the order in which the user-specified
        // event receivers were registered with the application.
        if(app) {
            for(size_t ir = 0; ir < app->user_receivers.size(); ir++) {
                if(app->user_receivers[ir]->OnEvent(event))
                    return true;
            }
        }

        // Process keyboard events.
        if(event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch(event.KeyInput.Key) {
            case irr::KEY_KEY_I:
                app->SetShowInfos(!app->GetShowInfos());
                return true;
            case irr::KEY_KEY_O:
                app->SetShowProfiler(!app->GetShowProfiler());
                return true;
            case irr::KEY_KEY_U:
                app->SetShowExplorer(!app->GetShowExplorer());
                return true;
            case irr::KEY_SPACE:
                app->pause_step = !app->pause_step;
                return true;
            case irr::KEY_KEY_P:
                app->pause_step = true;
                app->do_single_step = true;
                return true;
            case irr::KEY_F11:
                GetLog() << "---Computing linear static solution---\n";
                app->GetSystem()->DoStaticLinear();
                return true;
            case irr::KEY_F10:
                GetLog() << "---Computing NONlinear static solution, 20 steps---\n";
                app->GetSystem()->DoStaticNonlinear(20);
                return true;
            case irr::KEY_F8: {
                GetLog() << "Saving system in JSON format to dump.json file \n";
                ChStreamOutAsciiFile mfileo("dump.json");
                ChArchiveOutJSON marchiveout(mfileo);
                marchiveout.SetUseVersions(false);
                marchiveout << CHNVP(app->GetSystem(), "System");

                GetLog() << "Saving system in ASCII format to dump.txt file \n";
                ChStreamOutAsciiFile mfileo2("dump.txt");
                ChArchiveAsciiDump marchiveout2(mfileo2);
                marchiveout2.SetUseVersions(false);
                marchiveout2 << CHNVP(app->GetSystem(), "System");
            }
            case irr::KEY_F6:
                GetLog() << "Saving system vector and matrices to dump_xxyy.dat files.\n";
                app->DumpSystemMatrices();
                return true;
            case irr::KEY_F7:
                if(!app->system->GetDumpSolverMatrices()) {
                    GetLog() << "Start saving system vector and matrices to dump_xxxx_yy.dat files...\n";
                    app->system->SetDumpSolverMatrices(true);
                } else {
                    GetLog() << "Stop saving system vector and matrices to dump_xxxx_yy.dat files.\n";
                    app->system->SetDumpSolverMatrices(false);
                }
                return true;
            case irr::KEY_SNAPSHOT:
                if(app->videoframe_save == false) {
                    app->videoframe_save = true;
                    GetLog() << "Start saving frames in /video_capture/snapshotnnnnn.bmp pictures...\n";
                } else {
                    app->videoframe_save = false;
                    GetLog() << "Stop saving frames in /video_capture directory.\n";
                }
                return true;
            case irr::KEY_F4:
                if(app->camera_auto_rotate_speed <= 0)
                    app->camera_auto_rotate_speed = 0.02;
                else
                    app->camera_auto_rotate_speed *= 1.5;
                return true;
            case irr::KEY_F3:
                app->camera_auto_rotate_speed = 0;
                return true;
            case irr::KEY_F2:
                if(app->camera_auto_rotate_speed >= 0)
                    app->camera_auto_rotate_speed = -0.02;
                else
                    app->camera_auto_rotate_speed *= 1.5;
                return true;
            case irr::KEY_ESCAPE:
                app->GetDevice()->closeDevice();
                return true;
            default:
                break;
            }
        }

        irr::core::dimension2d<irr::u32> ssize = app->GetVideoDriver()->getScreenSize();

        // Check if user moved the sliders with mouse.
        if(event.EventType == irr::EET_GUI_EVENT) {
            irr::s32 id = event.GUIEvent.Caller->getID();

            switch(event.GUIEvent.EventType) {
            case irr::gui::EGET_SCROLL_BAR_CHANGED:
                switch(id) {
                case 9904:
                    app->GetSystem()->SetSolverMaxIterations(
                        ((irr::gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                    break;
                case 9911:
                    app->GetSystem()->SetMaxPenetrationRecoverySpeed(
                        (3.0 / 50.0) * ((irr::gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                    break;
                case 9912:
                    app->GetSystem()->SetMinBounceSpeed(
                        (1.0 / 200.0) * ((irr::gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                    break;
                }
                break;

            case irr::gui::EGET_COMBO_BOX_CHANGED:
                if(id == 9907) {
                    int sel = ((irr::gui::IGUIComboBox*)event.GUIEvent.Caller)->getSelected();
                    switch(sel) {
                    case 0:
                        app->GetSystem()->SetSolverType(ChSolver::Type::PSOR);
                        break;
                    case 1:
                        app->GetSystem()->SetSolverType(ChSolver::Type::PSSOR);
                        break;
                    case 2:
                        app->GetSystem()->SetSolverType(ChSolver::Type::PJACOBI);
                        break;
                    case 3:
                        app->GetSystem()->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
                        break;
                    case 4:
                        app->GetSystem()->SetSolverType(ChSolver::Type::APGD);
                        break;
                    case 5:
                        GetLog() << "WARNING.\nYou cannot change to a custom solver using the GUI. Use C++ instead.\n";
                        break;
                    }
                    break;
                }

                if(id == 9908) {
                    int sel = ((irr::gui::IGUIComboBox*)event.GUIEvent.Caller)->getSelected();
                    switch(sel) {
                    case 0:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
                        break;
                    case 1:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
                        break;
                    case 2:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
                        break;
                    case 3:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::TRAPEZOIDAL);
                        break;
                    case 4:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED);
                        break;
                    case 5:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
                        break;
                    case 6:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::HEUN);
                        break;
                    case 7:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);
                        break;
                    case 8:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
                        break;
                    case 9:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::LEAPFROG);
                        break;
                    case 10:
                        app->GetSystem()->SetTimestepperType(ChTimestepper::Type::NEWMARK);
                        break;
                    case 11:
                        GetLog() << "WARNING.\nYou cannot change to a custom timestepper using the GUI. Use C++ "
                                    "instead.\n";
                        break;
                    }
                    break;
                }

            case irr::gui::EGET_CHECKBOX_CHANGED:
                switch(id) {
                case 9913:
                    app->GetSystem()->SetUseSleeping(((irr::gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked());
                    break;
                case 9916:
                    app->SetTryRealtime(((irr::gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked());
                    break;
                case 9917:
                    app->pause_step = ((irr::gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked();
                    break;
                }
                break;

            case irr::gui::EGET_EDITBOX_ENTER:
                if(id == 9918) {
                    double dt = 0.01;
                    dt = atof(irr::core::stringc(((irr::gui::IGUIEditBox*)event.GUIEvent.Caller)->getText()).c_str());
                    app->SetTimestep(dt);
                    break;
                }

                if(id == 9921) {
                    double scale = 0.01;
                    scale =
                        atof(irr::core::stringc(((irr::gui::IGUIEditBox*)event.GUIEvent.Caller)->getText()).c_str());
                    app->SetSymbolscale(scale);
                    break;
                }

            default:
                break;
            }
        }

        return false;
    }

    // -----------------------------------------------------------------------------
    // Implementation of ChIrrAppInterface methods
    // -----------------------------------------------------------------------------

    // Create the IRRLICHT context (device, etc.)
    ChIrrAppInterface::ChIrrAppInterface(ChSystem* psystem,
        const wchar_t* title,
        irr::core::dimension2d<irr::u32> dimens,
        bool do_fullscreen,
        bool do_shadows,
        bool do_antialias,
        irr::video::E_DRIVER_TYPE mydriver,
        irr::ELOG_LEVEL log_level)
        : step_manage(true)
        , try_realtime(false)
        , pause_step(false)
        , timestep(0.01)
        , do_single_step(false)
        , videoframe_save(false)
        , videoframe_num(0)
        , videoframe_each(1)
        , symbolscale(1.0)
        , camera_auto_rotate_speed(0.0)
    {
        irr::SIrrlichtCreationParameters params = irr::SIrrlichtCreationParameters();
        params.AntiAlias = do_antialias;
        params.Bits = 32;
        params.Fullscreen = do_fullscreen;
        params.DriverType = mydriver;
        params.WindowSize = dimens;
        params.Stencilbuffer = do_shadows;
        params.LoggingLevel = log_level;

        device = irr::createDeviceEx(params);

        if(device == 0) {
            GetLog() << "Cannot use default video driver - fall back to OpenGL \n";
            params.DriverType = irr::video::EDT_OPENGL;

            device = irr::createDeviceEx(params);

            if(!device)
                return;
        }

        // Xeffects for shadow maps!
        if(do_antialias)
            effect = new EffectHandler(device, device->getVideoDriver()->getScreenSize() * 2, true, false, true);
        else
            effect = new EffectHandler(device, device->getVideoDriver()->getScreenSize(), true, false, true);
        // note: Irrlicht antialiasing does not work with Xeffects, but we could fake AA in Xeffects
        // by doubling the size of its buffer:  EffectHandler(device, device->getVideoDriver()->getScreenSize()*2
        effect->setAmbientColor(irr::video::SColor(255, 122, 122, 122));
        use_effects = false; // will be true as sson as a light with shadow is added.

        if(title)
            device->setWindowCaption(title);
        else
            device->setWindowCaption(L"Chrono::Engine");

        irr::gui::IGUISkin* skin = GetIGUIEnvironment()->getSkin();
        irr::gui::IGUIFont* font = GetIGUIEnvironment()->getFont(GetChronoDataFile("fonts/arial8.xml").c_str());
        if(font)
            skin->setFont(font);
        skin->setColor(irr::gui::EGDC_BUTTON_TEXT, irr::video::SColor(255, 40, 50, 50));
        skin->setColor(irr::gui::EGDC_HIGH_LIGHT, irr::video::SColor(255, 40, 70, 250));
        skin->setColor(irr::gui::EGDC_FOCUSED_EDITABLE, irr::video::SColor(255, 0, 255, 255));
        skin->setColor(irr::gui::EGDC_3D_HIGH_LIGHT, irr::video::SColor(200, 210, 210, 210));

        gad_tabbed = GetIGUIEnvironment()->addTabControl(irr::core::rect<irr::s32>(2, 70, 220, 510), 0, true, true);
        gad_tab1 = gad_tabbed->addTab(L"Stats");
        gad_tab2 = gad_tabbed->addTab(L"System");
        gad_tab3 = gad_tabbed->addTab(L"Help");

        // create GUI gadgets
        gad_textFPS = GetIGUIEnvironment()->addStaticText(
            L"FPS", irr::core::rect<irr::s32>(10, 10, 200, 230), true, true, gad_tab1);

        gad_labelcontacts =
            GetIGUIEnvironment()->addComboBox(irr::core::rect<irr::s32>(10, 240, 200, 240 + 20), gad_tab1, 9901);
        gad_labelcontacts->addItem(L"Contact distances");
        gad_labelcontacts->addItem(L"Contact force modulus");
        gad_labelcontacts->addItem(L"Contact force (normal)");
        gad_labelcontacts->addItem(L"Contact force (tangent)");
        gad_labelcontacts->addItem(L"Contact torque modulus");
        gad_labelcontacts->addItem(L"Contact torque (spinning)");
        gad_labelcontacts->addItem(L"Contact torque (rolling)");
        gad_labelcontacts->addItem(L"Don't print contact values");
        gad_labelcontacts->setSelected(7);

        gad_drawcontacts =
            GetIGUIEnvironment()->addComboBox(irr::core::rect<irr::s32>(10, 260, 200, 260 + 20), gad_tab1, 9901);
        gad_drawcontacts->addItem(L"Contact normals");
        gad_drawcontacts->addItem(L"Contact distances");
        gad_drawcontacts->addItem(L"Contact N forces");
        gad_drawcontacts->addItem(L"Contact forces");
        gad_drawcontacts->addItem(L"Don't draw contacts");
        gad_drawcontacts->setSelected(4);

        gad_labellinks =
            GetIGUIEnvironment()->addComboBox(irr::core::rect<irr::s32>(10, 280, 200, 280 + 20), gad_tab1, 9923);
        gad_labellinks->addItem(L"Link react.force modulus");
        gad_labellinks->addItem(L"Link react.force X");
        gad_labellinks->addItem(L"Link react.force Y");
        gad_labellinks->addItem(L"Link react.force Z");
        gad_labellinks->addItem(L"Link react.torque modulus");
        gad_labellinks->addItem(L"Link react.torque X");
        gad_labellinks->addItem(L"Link react.torque Y");
        gad_labellinks->addItem(L"Link react.torque Z");
        gad_labellinks->addItem(L"Don't print link values");
        gad_labellinks->setSelected(8);

        gad_drawlinks =
            GetIGUIEnvironment()->addComboBox(irr::core::rect<irr::s32>(10, 300, 200, 300 + 20), gad_tab1, 9924);
        gad_drawlinks->addItem(L"Link reaction forces");
        gad_drawlinks->addItem(L"Link reaction torques");
        gad_drawlinks->addItem(L"Don't draw link vectors");
        gad_drawlinks->setSelected(2);

        gad_plot_aabb = GetIGUIEnvironment()->addCheckBox(
            false, irr::core::rect<irr::s32>(10, 330, 200, 330 + 15), gad_tab1, 9914, L"Draw AABB");

        gad_plot_cogs = GetIGUIEnvironment()->addCheckBox(
            false, irr::core::rect<irr::s32>(10, 345, 200, 345 + 15), gad_tab1, 9915, L"Draw COGs");

        gad_plot_linkframes = GetIGUIEnvironment()->addCheckBox(
            false, irr::core::rect<irr::s32>(10, 360, 200, 360 + 15), gad_tab1, 9920, L"Draw link frames");

        gad_plot_collisionshapes = GetIGUIEnvironment()->addCheckBox(
            false, irr::core::rect<irr::s32>(10, 375, 200, 375 + 15), gad_tab1, 9902, L"Draw collision shapes");

        gad_plot_convergence = GetIGUIEnvironment()->addCheckBox(
            false, irr::core::rect<irr::s32>(10, 390, 200, 390 + 15), gad_tab1, 9902, L"Plot convergence");

        gad_symbolscale = GetIGUIEnvironment()->addEditBox(
            L"", irr::core::rect<irr::s32>(170, 330, 200, 330 + 15), true, gad_tab1, 9921);
        gad_symbolscale_info = GetIGUIEnvironment()->addStaticText(
            L"Symbols scale", irr::core::rect<irr::s32>(110, 330, 170, 330 + 15), false, false, gad_tab1);
        SetSymbolscale(symbolscale);

        // --

        gad_speed_iternumber =
            GetIGUIEnvironment()->addScrollBar(true, irr::core::rect<irr::s32>(10, 10, 150, 10 + 20), gad_tab2, 9904);
        gad_speed_iternumber->setMax(120);
        gad_speed_iternumber_info = GetIGUIEnvironment()->addStaticText(
            L"", irr::core::rect<irr::s32>(155, 10, 220, 10 + 20), false, false, gad_tab2);

        gad_usesleep = GetIGUIEnvironment()->addCheckBox(
            false, irr::core::rect<irr::s32>(10, 100, 200, 100 + 20), gad_tab2, 9913, L"Enable sleeping");

        gad_ccpsolver =
            GetIGUIEnvironment()->addComboBox(irr::core::rect<irr::s32>(10, 130, 200, 130 + 20), gad_tab2, 9907);
        gad_ccpsolver->addItem(L"Projected SOR");
        gad_ccpsolver->addItem(L"Projected SSOR");
        gad_ccpsolver->addItem(L"Projected Jacobi");
        gad_ccpsolver->addItem(L"Projected BB");
        gad_ccpsolver->addItem(L"Projected APGD");
        gad_ccpsolver->addItem(L"(custom)");
        gad_ccpsolver->setSelected(5);

        gad_stepper =
            GetIGUIEnvironment()->addComboBox(irr::core::rect<irr::s32>(10, 160, 200, 160 + 20), gad_tab2, 9908);
        gad_stepper->addItem(L"Euler implicit");
        gad_stepper->addItem(L"Euler semimplicit (linearized)");
        gad_stepper->addItem(L"Euler semimplicit projected");
        gad_stepper->addItem(L"Trapezoidal");
        gad_stepper->addItem(L"Trapezoidal (linearized)");
        gad_stepper->addItem(L"HHT");
        gad_stepper->addItem(L"Heun explicit");
        gad_stepper->addItem(L"RungeKutta45 explicit");
        gad_stepper->addItem(L"Euler explicit");
        gad_stepper->addItem(L"Leapfrog");
        gad_stepper->addItem(L"Newmark");
        gad_stepper->addItem(L"(custom)");

        gad_stepper->setSelected(0);

        gad_clamping =
            GetIGUIEnvironment()->addScrollBar(true, irr::core::rect<irr::s32>(10, 250, 150, 250 + 20), gad_tab2, 9911);
        gad_clamping->setMax(100);
        gad_clamping_info = GetIGUIEnvironment()->addStaticText(
            L"", irr::core::rect<irr::s32>(155, 250, 220, 250 + 20), false, false, gad_tab2);

        gad_minbounce =
            GetIGUIEnvironment()->addScrollBar(true, irr::core::rect<irr::s32>(10, 280, 150, 280 + 20), gad_tab2, 9912);
        gad_minbounce->setMax(100);
        gad_minbounce_info = GetIGUIEnvironment()->addStaticText(
            L"", irr::core::rect<irr::s32>(155, 280, 220, 280 + 20), false, false, gad_tab2);

        gad_timestep = GetIGUIEnvironment()->addEditBox(
            L"", irr::core::rect<irr::s32>(140, 320, 200, 320 + 15), true, gad_tab2, 9918);
        gad_timestep_info = GetIGUIEnvironment()->addStaticText(
            L"Time step", irr::core::rect<irr::s32>(10, 320, 130, 320 + 15), false, false, gad_tab2);

        gad_try_realtime = GetIGUIEnvironment()->addCheckBox(
            false, irr::core::rect<irr::s32>(10, 340, 200, 340 + 15), gad_tab2, 9916, L"Realtime step");
        gad_pause_step = GetIGUIEnvironment()->addCheckBox(
            false, irr::core::rect<irr::s32>(10, 355, 200, 355 + 15), gad_tab2, 9917, L"Pause physics");

        gad_textHelp = GetIGUIEnvironment()->addStaticText(
            L"FPS", irr::core::rect<irr::s32>(10, 10, 200, 380), true, true, gad_tab3);
        irr::core::stringw hstr = "Instructions for interface.\n\n";
        hstr += "MOUSE \n\n";
        hstr += " left button: camera rotation \n";
        hstr += " righ button: camera translate \n";
        hstr += " wheel rotation: camera forward \n";
        hstr += " wheel button: drag collision shapes\n";
        hstr += "\nKEYBOARD\n\n";
        hstr += " 'i' key: show/hide settings\n";
        hstr += " 'o' key: show/hide profiler\n";
        hstr += " 'u' key: show/hide property tree\n";
        hstr += " arrows keys: camera X/Z translate\n";
        hstr += " Pg Up/Dw keys: camera Y translate\n";
        hstr += " 'spacebar' key: stop/start simul.\n";
        hstr += " 'p' key: advance single step\n";
        hstr += " 'Print Scr' key: video capture to .bmp's\n";
        hstr += " 'F6' key: single dump sys. matrices.\n";
        hstr += " 'F7' key: continuous dump sys. matrices.\n";
        hstr += " 'F8' key: dump a .json file.\n";
        hstr += " 'F10' key: non-linear statics.\n";
        hstr += " 'F11' key: linear statics.\n";
        hstr += " 'F2-F3-F4' key: auto rotate camera.\n";
        gad_textHelp->setText(hstr.c_str());

        gad_treeview =
            GetIGUIEnvironment()->addTreeView(irr::core::rect<irr::s32>(2, 80,
                                                  300, // this->device->getVideoDriver()->getScreenSize().Width
                                                  this->device->getVideoDriver()->getScreenSize().Height - 4),
                0, 9919, true, true, true);
        auto child = gad_treeview->getRoot()->addChildBack(L"System", 0);
        child->setExpanded(true);

        system = psystem;

        show_infos = false;
        show_profiler = false;
        show_explorer = false;

        // the container, a level that contains all chrono nodes
        container = device->getSceneManager()->addEmptySceneNode();

        // the event receiver, taking care of user interaction
        ChIrrAppEventReceiver* receiver = new ChIrrAppEventReceiver(this);
        device->setEventReceiver(receiver);
    }

    // ChIrrAppInterface destructor. This safely delete every Irrlicht item
    // (including the Irrlicht scene nodes)
    ChIrrAppInterface::~ChIrrAppInterface()
    {
        device->drop();
        // delete (receiver);
    }

    // Set integration time step.
    void ChIrrAppInterface::SetTimestep(double val)
    {
        timestep = ChMax(10e-9, val);
        char message[50];
        sprintf(message, "%g", timestep);
        gad_timestep->setText(irr::core::stringw(message).c_str());
    }

    // Set the scale for drawing symbols.
    void ChIrrAppInterface::SetSymbolscale(double val)
    {
        symbolscale = ChMax(10e-12, val);
        char message[50];
        sprintf(message, "%g", symbolscale);
        gad_symbolscale->setText(irr::core::stringw(message).c_str());
    }

    // Set the fonts to be used from now on.
    void ChIrrAppInterface::SetFonts(const std::string& mfontdir)
    {
        irr::gui::IGUISkin* skin = GetIGUIEnvironment()->getSkin();
        irr::gui::IGUIFont* font = GetIGUIEnvironment()->getFont(mfontdir.c_str());
        if(font)
            skin->setFont(font);
    }

    // Clean canvas at beginning of scene.
    void ChIrrAppInterface::BeginScene(bool backBuffer, bool zBuffer, irr::video::SColor color)
    {

        utils::ChProfileManager::Reset();
        utils::ChProfileManager::Start_Profile("Irrlicht loop");
        utils::ChProfileManager::Increment_Frame_Counter();

        GetVideoDriver()->beginScene(backBuffer, zBuffer, color);

        if(camera_auto_rotate_speed) {
            irr::core::vector3df pos = GetSceneManager()->getActiveCamera()->getPosition();
            irr::core::vector3df target = GetSceneManager()->getActiveCamera()->getTarget();
            pos.rotateXZBy(camera_auto_rotate_speed, target);
            GetSceneManager()->getActiveCamera()->setPosition(pos);
            GetSceneManager()->getActiveCamera()->setTarget(target);
        }
    }

    // Call this to end the scene draw at the end of each animation frame.
    void ChIrrAppInterface::EndScene()
    {

        utils::ChProfileManager::Stop_Profile();

        if(show_profiler)
            ChIrrTools::drawProfiler(this->GetDevice());

        GetVideoDriver()->endScene();
    }

    // Advance physics by one time step.
    void ChIrrAppInterface::DoStep()
    {
        CH_PROFILE("DoStep");

        if(!step_manage)
            return;

        if(pause_step) {
            if(do_single_step)
                do_single_step = false;
            else
                return;
        }

        if(videoframe_save) {
            if(videoframe_num % videoframe_each == 0) {
                filesystem::create_directory(filesystem::path("video_capture"));
                irr::video::IImage* image = GetVideoDriver()->createScreenShot();
                char filename[100];
                sprintf(filename, "video_capture/screenshot%05d.bmp", (videoframe_num + 1) / videoframe_each);
                if(image)
                    device->getVideoDriver()->writeImageToFile(image, filename);
                image->drop();
            }
            videoframe_num++;
        }

        double dt;
        if(try_realtime)
            dt = m_realtime_timer.SuggestSimulationStep(timestep);
        else
            dt = timestep;

        try {
            system->DoStepDynamics(dt);
        } catch(ChException my_exception) {
            GetLog() << my_exception.what() << "\n";
        }
    }

    void recurse_update_tree_node(ChValue* mvalue, irr::gui::IGUITreeViewNode* mnode)
    {
        ChArchiveExplorer mexplorer2;
        mexplorer2.FetchValues(*mvalue, "*");
        int ni = 0;
        auto subnode = mnode->getFirstChild();
        for(auto j : mexplorer2.GetFetchResults()) {
            ++ni;
            if(!subnode) {
                subnode = mnode->addChildBack(L"_to_set_");
                subnode->setExpanded(false);
            }
            // update the subnode visual info:
            irr::core::stringw jstr(j->name());
            if(j->HasArchiveContainerName()) {
                jstr = L"'";
                jstr += j->CallArchiveContainerName().c_str();
                jstr += "'";
            }
            if(j->GetClassRegisteredName() != "") {
                jstr += L",  [";
                jstr += irr::core::stringw(j->GetClassRegisteredName().c_str());
                jstr += L"] ";
            }
            if(auto mydouble = j->PointerUpCast<double>()) {
                jstr += " =";
                auto stringval = std::to_string(*mydouble);
                jstr += irr::core::stringw(stringval.c_str());
            }
            if(auto myfloat = j->PointerUpCast<float>()) {
                jstr += " =";
                auto stringval = std::to_string(*myfloat);
                jstr += irr::core::stringw(stringval.c_str());
            }
            if(auto myint = j->PointerUpCast<int>()) {
                jstr += " =";
                auto stringval = std::to_string(*myint);
                jstr += irr::core::stringw(stringval.c_str());
            }
            if(auto mybool = j->PointerUpCast<bool>()) {
                jstr += " =";
                auto stringval = std::to_string(*mybool);
                jstr += irr::core::stringw(stringval.c_str());
            }
            subnode->setText(jstr.c_str());

            // recursion to update children nodes
            if(subnode->getExpanded())
                recurse_update_tree_node(j, subnode);

            // this to show the "+" symbol for not yet explored nodes
            ChArchiveExplorer mexplorer3;
            mexplorer3.FetchValues(*j, "*");
            if(subnode->getChildCount() == 0 && mexplorer3.GetFetchResults().size()) {
                subnode->addChildBack(L"_foo_to_set_");
            }

            // process next sub property and corresponding sub node
            subnode = subnode->getNextSibling();
        }
        auto subnode_to_remove = subnode;
        while(subnode_to_remove) {
            auto othernode = subnode_to_remove->getNextSibling();
            mnode->deleteChild(subnode_to_remove);
            subnode_to_remove = othernode;
        }
    }

    // Redraw all 3D shapes and GUI elements
    void ChIrrAppInterface::DrawAll()
    {
        CH_PROFILE("DrawAll");

        irr::core::stringw str = "World time   =";
        str += (int)(1000 * system->GetChTime());
        str += " ms  \n\nCPU step (total)      =";
        str += (int)(1000 * system->GetTimerStep());
        str += " ms \n  CPU Collision time =";
        str += (int)(1000 * system->GetTimerCollision());
        str += " ms \n  CPU Solver time         =";
        str += (int)(1000 * system->GetTimerSolver());
        str += " ms \n  CPU Update time      =";
        str += (int)(1000 * system->GetTimerUpdate());
        str += "\n\nN.of active bodies  : ";
        str += system->GetNbodies();
        str += "\nN.of sleeping bodies  : ";
        str += system->GetNbodiesSleeping();
        str += "\nN.of contacts  : ";
        str += system->GetNcontacts();
        str += "\nN.of coords    : ";
        str += system->GetNcoords_w();
        str += "\nN.of constr.   : ";
        str += system->GetNdoc_w();
        str += "\nN.of variables : ";
        str += system->GetNsysvars_w();
        gad_textFPS->setText(str.c_str());

        if(use_effects)
            effect->update(); // DRAW 3D SCENE using Xeffects for shadow maps, if used!
        else
            GetSceneManager()->drawAll(); // DRAW 3D SCENE the usual way, if no shadow maps

        int dmode = gad_drawcontacts->getSelected();
        ChIrrTools::drawAllContactPoints(
            system->GetContactContainer(), GetVideoDriver(), symbolscale, (ChIrrTools::eCh_ContactsDrawMode)dmode);

        int lmode = gad_labelcontacts->getSelected();
        ChIrrTools::drawAllContactLabels(
            system->GetContactContainer(), GetDevice(), (ChIrrTools::eCh_ContactsLabelMode)lmode);

        int dmodeli = gad_drawlinks->getSelected();
        ChIrrTools::drawAllLinks(*system, GetVideoDriver(), symbolscale, (ChIrrTools::eCh_LinkDrawMode)dmodeli);

        int lmodeli = gad_labellinks->getSelected();
        ChIrrTools::drawAllLinkLabels(*system, GetDevice(), (ChIrrTools::eCh_LinkLabelMode)lmodeli);

        if(gad_plot_aabb->isChecked())
            ChIrrTools::drawAllBoundingBoxes(*system, GetVideoDriver());

        if(gad_plot_cogs->isChecked())
            ChIrrTools::drawAllCOGs(*system, GetVideoDriver(), symbolscale);

        if(gad_plot_linkframes->isChecked())
            ChIrrTools::drawAllLinkframes(*system, GetVideoDriver(), symbolscale);

        if(gad_plot_collisionshapes->isChecked())
            ChIrrTools::drawCollisionShapes(*system, GetDevice());

        if(gad_plot_convergence->isChecked())
            ChIrrTools::drawHUDviolation(GetVideoDriver(), GetDevice(), *system, 240, 370, 300, 100, 100.0);

        gad_tabbed->setVisible(show_infos);
        gad_treeview->setVisible(show_explorer);
        if(show_explorer) {
            chrono::ChValueSpecific<ChSystem> root(*this->system, "system", 0);
            recurse_update_tree_node(&root, gad_treeview->getRoot());
        }

        if(gad_speed_iternumber_info->isVisible()) {
            gad_usesleep->setChecked(GetSystem()->GetUseSleeping());

            char message[50];

            gad_speed_iternumber->setPos(GetSystem()->GetSolverMaxIterations());
            sprintf(message, "%i iters", GetSystem()->GetSolverMaxIterations());
            gad_speed_iternumber_info->setText(irr::core::stringw(message).c_str());

            gad_clamping->setPos((irr::s32)((50.0 / 3.0) * (GetSystem()->GetMaxPenetrationRecoverySpeed())));
            sprintf(message, "%g stab.clamp", GetSystem()->GetMaxPenetrationRecoverySpeed());
            gad_clamping_info->setText(irr::core::stringw(message).c_str());

            gad_minbounce->setPos((irr::s32)(200.0 * (GetSystem()->GetMinBounceSpeed())));
            sprintf(message, "%g min.bounce v", GetSystem()->GetMinBounceSpeed());
            gad_minbounce_info->setText(irr::core::stringw(message).c_str());

            switch(GetSystem()->GetSolverType()) {
            case ChSolver::Type::PSOR:
                gad_ccpsolver->setSelected(0);
                break;
            case ChSolver::Type::PSSOR:
                gad_ccpsolver->setSelected(1);
                break;
            case ChSolver::Type::PJACOBI:
                gad_ccpsolver->setSelected(2);
                break;
            case ChSolver::Type::BARZILAIBORWEIN:
                gad_ccpsolver->setSelected(3);
                break;
            case ChSolver::Type::APGD:
                gad_ccpsolver->setSelected(4);
                break;
            default:
                gad_ccpsolver->setSelected(5);
                break;
            }

            switch(GetSystem()->GetTimestepperType()) {
            case ChTimestepper::Type::EULER_IMPLICIT:
                gad_stepper->setSelected(0);
                break;
            case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED:
                gad_stepper->setSelected(1);
                break;
            case ChTimestepper::Type::EULER_IMPLICIT_PROJECTED:
                gad_stepper->setSelected(2);
                break;
            case ChTimestepper::Type::TRAPEZOIDAL:
                gad_stepper->setSelected(3);
                break;
            case ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED:
                gad_stepper->setSelected(4);
                break;
            case ChTimestepper::Type::HHT:
                gad_stepper->setSelected(5);
                break;
            case ChTimestepper::Type::HEUN:
                gad_stepper->setSelected(6);
                break;
            case ChTimestepper::Type::RUNGEKUTTA45:
                gad_stepper->setSelected(7);
                break;
            case ChTimestepper::Type::EULER_EXPLICIT:
                gad_stepper->setSelected(8);
                break;
            case ChTimestepper::Type::LEAPFROG:
                gad_stepper->setSelected(9);
                break;
            case ChTimestepper::Type::NEWMARK:
                gad_stepper->setSelected(10);
                break;
            default:
                gad_stepper->setSelected(11);
                break;
            }

            gad_try_realtime->setChecked(GetTryRealtime());
            gad_pause_step->setChecked(pause_step);

            if(!GetStepManage()) {
                sprintf(message, "%g", GetSystem()->GetStep());
                gad_timestep->setText(irr::core::stringw(message).c_str());
            }

            // disable timestep-related gadgets if dt not handled by application object
            gad_try_realtime->setEnabled(GetStepManage());
            gad_pause_step->setEnabled(GetStepManage());
            gad_timestep->setEnabled(GetStepManage());
        }

        // if(show_infos)
        GetIGUIEnvironment()->drawAll();
    }

    // Dump the last used system matrices and vectors in the current directory,
    void ChIrrAppInterface::DumpSystemMatrices()
    {
        // For safety
        GetSystem()->Setup();
        GetSystem()->Update();

        try {
            // Save M mass matrix, K stiffness matrix, R damping matrix, Cq jacobians:
            GetSystem()->DumpSystemMatrices(true, true, true, true, "dump_");

        } catch(ChException myexc) {
            GetLog() << myexc.what();
        }
    }

} // end namespace irrlicht
} // end namespace chrono
