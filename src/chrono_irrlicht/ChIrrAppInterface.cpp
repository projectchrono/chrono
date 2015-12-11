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

#include "physics/ChLinkSpring.h"
#include "collision/ChCModelBullet.h"
#include "serialization/ChArchiveAsciiDump.h"
#include "serialization/ChArchiveJSON.h"
#include "core/ChStream.h"

#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrCamera.h"

namespace irr {

// -----------------------------------------------------------------------------
// ChIrrAppEventReceiver
//
// A custom Irrlicht Event Receiver class.
// -----------------------------------------------------------------------------

class ChIrrAppEventReceiver : public IEventReceiver {
  public:
    ChIrrAppEventReceiver(ChIrrAppInterface* m_app) : app(m_app) {}
    bool OnEvent(const SEvent& event);

  private:
    ChIrrAppInterface* app;
};

bool ChIrrAppEventReceiver::OnEvent(const SEvent& event) {
    // Check if there are any user-specified event receivers. Give them the
    // first chance to process the event (in the order in which the user-specified
    // event receivers were registered with the application.
    if (app) {
        for (size_t ir = 0; ir < app->user_receivers.size(); ir++) {
            if (app->user_receivers[ir]->OnEvent(event))
                return true;
        }
    }

    // Process keyboard events.
    if (event.EventType == EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_I:
                app->SetShowInfos(!app->GetShowInfos());
                return true;
            case KEY_SPACE:
                app->pause_step = !app->pause_step;
                return true;
            case KEY_KEY_P:
                app->pause_step = true;
                app->do_single_step = true;
                return true;
            case KEY_F11:
                chrono::GetLog() << "---Computing linear static solution---\n";
                app->GetSystem()->DoStaticLinear();
                return true;
            case KEY_F10:
                chrono::GetLog() << "---Computing NONlinear static solution, 20 steps---\n";
                app->GetSystem()->DoStaticNonlinear(20);
                return true;
            case KEY_F8:
            {
                chrono::GetLog() << "Saving system in JSON format to dump.json file \n";
                chrono::ChStreamOutAsciiFile mfileo("dump.json");
                chrono::ChArchiveOutJSON marchiveout(mfileo);
                marchiveout.SetUseVersions(false);
                marchiveout << CHNVP(app->GetSystem(),"System");

                chrono::GetLog() << "Saving system in ASCII format to dump.txt file \n";
                chrono::ChStreamOutAsciiFile mfileo2("dump.txt");
                chrono::ChArchiveAsciiDump marchiveout2(mfileo2);
                marchiveout2.SetUseVersions(false);
                marchiveout2 << CHNVP(app->GetSystem(),"System");
            }
            case KEY_F6:
                chrono::GetLog() << "Saving system vector and matrices to dump_xxyy.dat files.\n";
                app->DumpMatrices();
                return true;
            case KEY_F7:
                if (!app->system->GetDumpMatrices()) {
                    chrono::GetLog() << "Start saving system vector and matrices to dump_xxxx_yy.dat files...\n";
                    app->system->SetDumpMatrices(true);
                }
                else {
                    chrono::GetLog() << "Stop saving system vector and matrices to dump_xxxx_yy.dat files.\n";
                    app->system->SetDumpMatrices(false);
                }
                return true;
            case KEY_SNAPSHOT:
                if (app->videoframe_save == false) {
                    app->videoframe_save = true;
                    chrono::GetLog() << "Start saving frames to snapshotnnnnn.bmp pictures...\n";
                } else {
                    app->videoframe_save = false;
                    chrono::GetLog() << "Stop saving frames.\n";
                }
                return true;
            case KEY_ESCAPE:
                app->GetDevice()->closeDevice();
                return true;
        }
    }

    core::dimension2d<u32> ssize = app->GetVideoDriver()->getScreenSize();

    // Process mouse events.
    if (event.EventType == EET_MOUSE_INPUT_EVENT) {
        switch (event.MouseInput.Event) {
            case EMIE_MMOUSE_PRESSED_DOWN: {
                core::line3d<f32> mline =
                    app->GetSceneManager()->getSceneCollisionManager()->getRayFromScreenCoordinates(
                        app->GetDevice()->getCursorControl()->getPosition());
                chrono::ChVector<> mfrom(mline.start.X, mline.start.Y, mline.start.Z);
                chrono::ChVector<> mto(mline.end.X, mline.end.Y, mline.end.Z);
                chrono::collision::ChCollisionSystem::ChRayhitResult mresult;
                app->GetSystem()->GetCollisionSystem()->RayHit(mfrom, mto, mresult);
                if (mresult.hit) {
                    if (chrono::ChBody* mbo =
                            dynamic_cast<chrono::ChBody*>(mresult.hitModel->GetContactable())) {
                        app->selectedmover = new chrono::ChSharedPtr<chrono::ChBody>(mbo);
                        app->selectedpoint = (*(app->selectedmover))->Point_World2Body(mresult.abs_hitPoint);
                        app->selecteddist = (mfrom - mresult.abs_hitPoint).Length();
                        app->selectedspring = new chrono::ChSharedPtr<chrono::ChLinkSpring>(new chrono::ChLinkSpring);
                        app->selectedtruss = new chrono::ChSharedPtr<chrono::ChBody>(new chrono::ChBody);
                        (*(app->selectedtruss))->SetBodyFixed(true);
                        app->GetSystem()->AddBody(*(app->selectedtruss));
                        (*(app->selectedspring))
                            ->Initialize(*app->selectedtruss, *app->selectedmover, false, mresult.abs_hitPoint,
                                         mresult.abs_hitPoint);
                        app->GetSystem()->AddLink(*(app->selectedspring));
                    }
                }
                break;
            }
            case EMIE_MMOUSE_LEFT_UP:
                if (app->selectedtruss) {
                    app->GetSystem()->RemoveBody((*(app->selectedtruss)));
                    app->GetSystem()->RemoveLink((*(app->selectedspring)));
                    delete (app->selectedtruss);
                    delete (app->selectedspring);
                    app->selectedtruss = 0;
                    app->selectedspring = 0;
                }
                break;
            case EMIE_MOUSE_MOVED:
                if (app->selectedtruss) {
                    core::line3d<f32> mline =
                        app->GetSceneManager()->getSceneCollisionManager()->getRayFromScreenCoordinates(
                            app->GetDevice()->getCursorControl()->getPosition());
                    chrono::ChVector<> mfrom(mline.start.X, mline.start.Y, mline.start.Z);
                    chrono::ChVector<> mto(mline.end.X, mline.end.Y, mline.end.Z);
                    chrono::ChVector<> mdir = mto - mfrom;
                    mdir.Normalize();
                    chrono::ChVector<> springP1 = mfrom + mdir * app->selecteddist;
                    chrono::ChVector<> springP2 = (*(app->selectedmover))->Point_Body2World(app->selectedpoint);
                    (*(app->selectedspring))->SetEndPoint1Abs(springP1);
                    (*(app->selectedspring))->SetEndPoint2Abs(springP2);
                    (*(app->selectedspring))->Set_SpringK(25 * (*(app->selectedmover))->GetMass());
                    (*(app->selectedspring))->Set_SpringR(3 * (*(app->selectedmover))->GetMass());
                }
                break;
        }
    }

    // Check if user moved the sliders with mouse.
    if (event.EventType == EET_GUI_EVENT) {
        s32 id = event.GUIEvent.Caller->getID();

        switch (event.GUIEvent.EventType) {
            case gui::EGET_SCROLL_BAR_CHANGED:
                switch (id) {
                    case 9904:
                        app->GetSystem()->SetIterLCPmaxItersSpeed(
                            ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                        break;
                    case 9905:
                        app->GetSystem()->SetIterLCPmaxItersStab(
                            ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                        break;
                    case 9909:
                        app->GetSystem()->SetIterLCPomega((1.0 / 50.0) *
                                                          ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                        break;
                    case 9910:
                        app->GetSystem()->SetIterLCPsharpnessLambda(
                            (1.0 / 50.0) * ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                        break;
                    case 9911:
                        app->GetSystem()->SetMaxPenetrationRecoverySpeed(
                            (3.0 / 50.0) * ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                        break;
                    case 9912:
                        app->GetSystem()->SetMinBounceSpeed((1.0 / 200.0) *
                                                            ((gui::IGUIScrollBar*)event.GUIEvent.Caller)->getPos());
                        break;
                }
                break;

            case gui::EGET_COMBO_BOX_CHANGED:
                if (id == 9907) {
                    int sel = ((gui::IGUIComboBox*)event.GUIEvent.Caller)->getSelected();
                    switch (sel) {
                        case 0:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_SOR);
                            break;
                        case 1:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_SYMMSOR);
                            break;
                        case 2:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_JACOBI);
                            break;
                        case 3:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);
                            break;
                        case 4:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN);
                            break;
                        case 5:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_PCG);
                            break;
                        case 6:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_PMINRES);
                            break;
                        case 7:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_APGD);
                            break;
                        case 8:
                            app->GetSystem()->SetLcpSolverType(chrono::ChSystem::LCP_ITERATIVE_MINRES);
                            break;
                        case 9:
                            chrono::GetLog() << "WARNING.\nYou cannot change to a custom solver using the GUI. Use C++ instead.\n";
                            break;
                    }
                    break;
                }

                if (id == 9908) {
                    int sel = ((gui::IGUIComboBox*)event.GUIEvent.Caller)->getSelected();
                    switch (sel) {
                        case 0:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_ANITESCU);
                            break;
                        case 1:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_TASORA);
                            break;
                        case 2:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT);
                            break;
                        case 3:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);
                            break;
                        case 4:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_PROJECTED);
                            break;
                        case 5:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_TRAPEZOIDAL);
                            break;
                        case 6:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_TRAPEZOIDAL_LINEARIZED);
                            break;
                        case 7:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_HHT);
                            break;
                        case 8:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_HEUN);
                            break;
                        case 9:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_RUNGEKUTTA45);
                            break;
                        case 10:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_EULER_EXPLICIT);
                            break;
                        case 11:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_LEAPFROG);
                            break;
                        case 12:
                            app->GetSystem()->SetIntegrationType(chrono::ChSystem::INT_NEWMARK);
                            break;
                        case 13:
                            chrono::GetLog() << "WARNING.\nYou cannot change to a custom timestepper using the GUI. Use C++ instead.\n";
                            break;
                    }
                    break;
                }

            case gui::EGET_CHECKBOX_CHANGED:
                switch (id) {
                    case 9906:
                        app->GetSystem()->SetIterLCPwarmStarting(
                            ((gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked());
                        break;
                    case 9913:
                        app->GetSystem()->SetUseSleeping(((gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked());
                        break;
                    case 9916:
                        app->SetTryRealtime(((gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked());
                        break;
                    case 9917:
                        app->pause_step = ((gui::IGUICheckBox*)event.GUIEvent.Caller)->isChecked();
                        break;
                }
                break;

            case gui::EGET_EDITBOX_ENTER:
                if (id == 9918) {
                    double dt = 0.01;
                    dt = atof(core::stringc(((gui::IGUIEditBox*)event.GUIEvent.Caller)->getText()).c_str());
                    app->SetTimestep(dt);
                    break;
                }

                if (id == 9921) {
                    double scale = 0.01;
                    scale = atof(core::stringc(((gui::IGUIEditBox*)event.GUIEvent.Caller)->getText()).c_str());
                    app->SetSymbolscale(scale);
                    break;
                }
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// Implementation of ChIrrAppInterface methods
// -----------------------------------------------------------------------------

// Create the IRRLICHT context (device, etc.)
ChIrrAppInterface::ChIrrAppInterface(chrono::ChSystem* psystem,
                                     const wchar_t* title,
                                     core::dimension2d<u32> dimens,
                                     bool do_fullscreen,
                                     bool do_shadows,
                                     video::E_DRIVER_TYPE mydriver)
    : step_manage(true),
      try_realtime(false),
      pause_step(false),
      timestep(0.01),
      do_single_step(false),
      videoframe_save(false),
      videoframe_num(0),
      videoframe_each(1),
      symbolscale(1.0),
      selectedtruss(0),
      selectedspring(0),
      selectedmover(0) {
    device = createDevice(mydriver, dimens, 32, do_fullscreen, do_shadows);

    if (device == 0) {
        chrono::GetLog() << "Cannot use default video driver - fall back to OpenGL \n";
        device = createDevice(video::EDT_OPENGL, dimens, 32, do_fullscreen, do_shadows);
        if (!device)
            return;
    }

    // Xeffects for shadow maps!
    effect = new EffectHandler(device, device->getVideoDriver()->getScreenSize(), true, false, true);
    effect->setAmbientColor(video::SColor(255, 122, 122, 122));
    use_effects = false;  // will be true as sson as a lightwith shadow is added.

    if (title)
        device->setWindowCaption(title);
    else
        device->setWindowCaption(L"Chrono::Engine");

    gui::IGUISkin* skin = GetIGUIEnvironment()->getSkin();
    gui::IGUIFont* font = GetIGUIEnvironment()->getFont(chrono::GetChronoDataFile("fonts/arial8.xml").c_str());
    if (font)
        skin->setFont(font);
    skin->setColor(gui::EGDC_BUTTON_TEXT, video::SColor(255, 40, 50, 50));

    gad_tabbed = GetIGUIEnvironment()->addTabControl(core::rect<s32>(2, 70, 220, 496), 0, true, true);
    gad_tab1 = gad_tabbed->addTab(L"Stats");
    gad_tab2 = gad_tabbed->addTab(L"System");
    gad_tab3 = gad_tabbed->addTab(L"Help");

    // create GUI gadgets
    gad_textFPS = GetIGUIEnvironment()->addStaticText(L"FPS", core::rect<s32>(10, 10, 200, 230), true, true, gad_tab1);

    gad_labelcontacts = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10, 240, 200, 240 + 20), gad_tab1, 9901);
    gad_labelcontacts->addItem(L"Contact distances");
    gad_labelcontacts->addItem(L"Contact force modulus");
    gad_labelcontacts->addItem(L"Contact force (normal)");
    gad_labelcontacts->addItem(L"Contact force (tangent)");
    gad_labelcontacts->addItem(L"Contact torque modulus");
    gad_labelcontacts->addItem(L"Contact torque (spinning)");
    gad_labelcontacts->addItem(L"Contact torque (rolling)");
    gad_labelcontacts->addItem(L"Don't print contact values");
    gad_labelcontacts->setSelected(7);

    gad_drawcontacts = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10, 260, 200, 260 + 20), gad_tab1, 9901);
    gad_drawcontacts->addItem(L"Contact normals");
    gad_drawcontacts->addItem(L"Contact distances");
    gad_drawcontacts->addItem(L"Contact N forces");
    gad_drawcontacts->addItem(L"Contact forces");
    gad_drawcontacts->addItem(L"Don't draw contacts");
    gad_drawcontacts->setSelected(4);

    gad_labellinks = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10, 280, 200, 280 + 20), gad_tab1, 9923);
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

    gad_drawlinks = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10, 300, 200, 300 + 20), gad_tab1, 9924);
    gad_drawlinks->addItem(L"Link reaction forces");
    gad_drawlinks->addItem(L"Link reaction torques");
    gad_drawlinks->addItem(L"Don't draw link vectors");
    gad_drawlinks->setSelected(2);

    gad_plot_aabb =
        GetIGUIEnvironment()->addCheckBox(false, core::rect<s32>(10, 330, 200, 330 + 15), gad_tab1, 9914, L"Draw AABB");

    gad_plot_cogs =
        GetIGUIEnvironment()->addCheckBox(false, core::rect<s32>(10, 345, 200, 345 + 15), gad_tab1, 9915, L"Draw COGs");

    gad_plot_linkframes = GetIGUIEnvironment()->addCheckBox(false, core::rect<s32>(10, 360, 200, 360 + 15), gad_tab1,
                                                            9920, L"Draw link frames");

    gad_symbolscale =
        GetIGUIEnvironment()->addEditBox(L"", core::rect<s32>(170, 330, 200, 330 + 15), true, gad_tab1, 9921);
    gad_symbolscale_info = GetIGUIEnvironment()->addStaticText(
        L"Symbols scale", core::rect<s32>(110, 330, 170, 330 + 15), false, false, gad_tab1);
    SetSymbolscale(symbolscale);

    gad_plot_convergence = GetIGUIEnvironment()->addCheckBox(false, core::rect<s32>(10, 375, 200, 375 + 15), gad_tab1,
                                                             9902, L"Plot convergence");

    // --

    gad_speed_iternumber =
        GetIGUIEnvironment()->addScrollBar(true, core::rect<s32>(10, 10, 150, 10 + 20), gad_tab2, 9904);
    gad_speed_iternumber->setMax(120);
    gad_speed_iternumber_info =
        GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155, 10, 220, 10 + 20), false, false, gad_tab2);

    gad_pos_iternumber =
        GetIGUIEnvironment()->addScrollBar(true, core::rect<s32>(10, 40, 150, 40 + 20), gad_tab2, 9905);
    gad_pos_iternumber->setMax(120);
    gad_pos_iternumber_info =
        GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155, 40, 220, 40 + 20), false, false, gad_tab2);

    gad_warmstart = GetIGUIEnvironment()->addCheckBox(false, core::rect<s32>(10, 70, 200, 70 + 20), gad_tab2, 9906,
                                                      L"Warm starting");

    gad_usesleep = GetIGUIEnvironment()->addCheckBox(false, core::rect<s32>(10, 100, 200, 100 + 20), gad_tab2, 9913,
                                                     L"Enable sleeping");

    gad_ccpsolver = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10, 130, 200, 130 + 20), gad_tab2, 9907);
    gad_ccpsolver->addItem(L"Projected SOR");
    gad_ccpsolver->addItem(L"Projected SSOR");
    gad_ccpsolver->addItem(L"Projected Jacobi");
    gad_ccpsolver->addItem(L"Multithreaded SOR");
    gad_ccpsolver->addItem(L"Projected BB");
    gad_ccpsolver->addItem(L"Projected PCG");
    gad_ccpsolver->addItem(L"Projected MINRES");
    gad_ccpsolver->addItem(L"APGD");
    gad_ccpsolver->addItem(L"MINRES");
    gad_ccpsolver->addItem(L"(custom)");
    gad_ccpsolver->setSelected(5);

    gad_stepper = GetIGUIEnvironment()->addComboBox(core::rect<s32>(10, 160, 200, 160 + 20), gad_tab2, 9908);
    gad_stepper->addItem(L"Anitescu stepper");
    gad_stepper->addItem(L"Tasora stepper");
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

    gad_omega = GetIGUIEnvironment()->addScrollBar(true, core::rect<s32>(10, 190, 150, 190 + 20), gad_tab2, 9909);
    gad_omega->setMax(100);
    gad_omega_info =
        GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155, 190, 220, 190 + 20), false, false, gad_tab2);

    gad_lambda = GetIGUIEnvironment()->addScrollBar(true, core::rect<s32>(10, 220, 150, 220 + 20), gad_tab2, 9910);
    gad_lambda->setMax(100);
    gad_lambda_info =
        GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155, 220, 220, 220 + 20), false, false, gad_tab2);

    gad_clamping = GetIGUIEnvironment()->addScrollBar(true, core::rect<s32>(10, 250, 150, 250 + 20), gad_tab2, 9911);
    gad_clamping->setMax(100);
    gad_clamping_info =
        GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155, 250, 220, 250 + 20), false, false, gad_tab2);

    gad_minbounce = GetIGUIEnvironment()->addScrollBar(true, core::rect<s32>(10, 280, 150, 280 + 20), gad_tab2, 9912);
    gad_minbounce->setMax(100);
    gad_minbounce_info =
        GetIGUIEnvironment()->addStaticText(L"", core::rect<s32>(155, 280, 220, 280 + 20), false, false, gad_tab2);

    gad_timestep =
        GetIGUIEnvironment()->addEditBox(L"", core::rect<s32>(140, 320, 200, 320 + 15), true, gad_tab2, 9918);
    gad_timestep_info = GetIGUIEnvironment()->addStaticText(L"Time step", core::rect<s32>(10, 320, 130, 320 + 15),
                                                            false, false, gad_tab2);

    gad_try_realtime = GetIGUIEnvironment()->addCheckBox(false, core::rect<s32>(10, 340, 200, 340 + 15), gad_tab2, 9916,
                                                         L"Realtime step");
    gad_pause_step = GetIGUIEnvironment()->addCheckBox(false, core::rect<s32>(10, 355, 200, 355 + 15), gad_tab2, 9917,
                                                       L"Pause physics");

    gad_textHelp = GetIGUIEnvironment()->addStaticText(L"FPS", core::rect<s32>(10, 10, 200, 350), true, true, gad_tab3);
    core::stringw hstr = "Instructions for interface.\n\n";
    hstr += "MOUSE \n\n";
    hstr += " left button: camera rotation \n";
    hstr += " righ button: camera translate \n";
    hstr += " wheel rotation: camera forward \n";
    hstr += " wheel button: drag collision shapes\n";
    hstr += "\nKEYBOARD\n\n";
    hstr += " 'i' key: show/hide settings\n";
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
    gad_textHelp->setText(hstr.c_str());

    ///

    system = psystem;

    system->AddRef();  // so that it works as with shared ptr

    show_infos = false;

    // the container, a level that contains all chrono nodes
    container = device->getSceneManager()->addEmptySceneNode();

    // the event receiver, taking care of user interaction
    ChIrrAppEventReceiver* receiver = new ChIrrAppEventReceiver(this);
    device->setEventReceiver(receiver);
}

// ChIrrAppInterface destructor. This safely delete every Irrlicht item
// (including the Irrlicht scene nodes)
ChIrrAppInterface::~ChIrrAppInterface() {
    device->drop();
    // delete (receiver);

    system->RemoveRef();
}

// Set integration time step.
void ChIrrAppInterface::SetTimestep(double val) {
    timestep = chrono::ChMax(10e-9, val);
    char message[50];
    sprintf(message, "%g", timestep);
    gad_timestep->setText(core::stringw(message).c_str());
}

// Set the scale for drawing symbols.
void ChIrrAppInterface::SetSymbolscale(double val) {
    symbolscale = chrono::ChMax(10e-12, val);
    char message[50];
    sprintf(message, "%g", symbolscale);
    gad_symbolscale->setText(core::stringw(message).c_str());
}

// Set the fonts to be used from now on.
void ChIrrAppInterface::SetFonts(const std::string& mfontdir) {
    gui::IGUISkin* skin = GetIGUIEnvironment()->getSkin();
    gui::IGUIFont* font = GetIGUIEnvironment()->getFont(mfontdir.c_str());
    if (font)
        skin->setFont(font);
}

// Clean canvas at beginning of scene.
void ChIrrAppInterface::BeginScene(bool backBuffer, bool zBuffer, video::SColor color) {
    GetVideoDriver()->beginScene(backBuffer, zBuffer, color);
}

// Call this to end the scene draw at the end of each animation frame.
void ChIrrAppInterface::EndScene() {
    GetVideoDriver()->endScene();
}

// Advance physics by one time step.
void ChIrrAppInterface::DoStep() {
    if (!step_manage)
        return;

    if (pause_step) {
        if (do_single_step)
            do_single_step = false;
        else
            return;
    }

    if (videoframe_save) {
        if (videoframe_num % videoframe_each == 0) {
            video::IImage* image = GetVideoDriver()->createScreenShot();
            char filename[100];
            sprintf(filename, "screenshot%05d.bmp", (videoframe_num + 1) / videoframe_each);
            if (image)
                device->getVideoDriver()->writeImageToFile(image, filename);
            image->drop();
        }
        videoframe_num++;
    }

    double dt;
    if (try_realtime)
        dt = m_realtime_timer.SuggestSimulationStep(timestep);
    else
        dt = timestep;

    system->DoStepDynamics(dt);
}

// Redraw all 3D shapes and GUI elements
void ChIrrAppInterface::DrawAll() {
    core::stringw str = "World time   =";
    str += (int)(1000 * system->GetChTime());
    str += " ms  \n\nCPU step (total)      =";
    str += (int)(1000 * system->GetTimerStep());
    str += " ms \n  CPU Collision time =";
    str += (int)(1000 * system->GetTimerCollisionBroad());
    str += " ms \n  CPU LCP time         =";
    str += (int)(1000 * system->GetTimerLcp());
    str += " ms \n  CPU Update time      =";
    str += (int)(1000 * system->GetTimerUpdate());
    str += " ms \n\nLCP vel.iters : ";
    str += system->GetIterLCPmaxItersSpeed();
    str += "\nLCP pos.iters : ";
    str += system->GetIterLCPmaxItersStab();
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

    if (use_effects)
        effect->update();  // DRAW 3D SCENE using Xeffects for shadow maps, if used!
    else
        GetSceneManager()->drawAll();  // DRAW 3D SCENE the usual way, if no shadow maps

    int dmode = gad_drawcontacts->getSelected();
    ChIrrTools::drawAllContactPoints(*system, GetVideoDriver(), symbolscale, (ChIrrTools::eCh_ContactsDrawMode)dmode);

    int lmode = gad_labelcontacts->getSelected();
    ChIrrTools::drawAllContactLabels(*system, GetDevice(), (ChIrrTools::eCh_ContactsLabelMode)lmode);

    int dmodeli = gad_drawlinks->getSelected();
    ChIrrTools::drawAllLinks(*system, GetVideoDriver(), symbolscale, (ChIrrTools::eCh_LinkDrawMode)dmodeli);

    int lmodeli = gad_labellinks->getSelected();
    ChIrrTools::drawAllLinkLabels(*system, GetDevice(), (ChIrrTools::eCh_LinkLabelMode)lmodeli);

    if (gad_plot_aabb->isChecked())
        ChIrrTools::drawAllBoundingBoxes(*system, GetVideoDriver());

    if (gad_plot_cogs->isChecked())
        ChIrrTools::drawAllCOGs(*system, GetVideoDriver(), symbolscale);

    if (gad_plot_linkframes->isChecked())
        ChIrrTools::drawAllLinkframes(*system, GetVideoDriver(), symbolscale);

    if (gad_plot_convergence->isChecked())
        ChIrrTools::drawHUDviolation(GetVideoDriver(), GetDevice(), *system, 240, 370, 300, 100, 100.0, 500.0);

    gad_tabbed->setVisible(show_infos);

    if (gad_speed_iternumber_info->isVisible()) {
        gad_warmstart->setChecked(GetSystem()->GetIterLCPwarmStarting());
        gad_usesleep->setChecked(GetSystem()->GetUseSleeping());

        char message[50];

        gad_speed_iternumber->setPos(GetSystem()->GetIterLCPmaxItersSpeed());
        sprintf(message, "%i vel.iters", GetSystem()->GetIterLCPmaxItersSpeed());
        gad_speed_iternumber_info->setText(core::stringw(message).c_str());

        gad_pos_iternumber->setPos(GetSystem()->GetIterLCPmaxItersStab());
        sprintf(message, "%i pos.iters", GetSystem()->GetIterLCPmaxItersStab());
        gad_pos_iternumber_info->setText(core::stringw(message).c_str());

        gad_omega->setPos((s32)(50.0 * (GetSystem()->GetIterLCPomega())));
        sprintf(message, "%g omega", GetSystem()->GetIterLCPomega());
        gad_omega_info->setText(core::stringw(message).c_str());

        gad_lambda->setPos((s32)(50.0 * (GetSystem()->GetIterLCPsharpnessLambda())));
        sprintf(message, "%g lambda", GetSystem()->GetIterLCPsharpnessLambda());
        gad_lambda_info->setText(core::stringw(message).c_str());

        gad_clamping->setPos((s32)((50.0 / 3.0) * (GetSystem()->GetMaxPenetrationRecoverySpeed())));
        sprintf(message, "%g stab.clamp", GetSystem()->GetMaxPenetrationRecoverySpeed());
        gad_clamping_info->setText(core::stringw(message).c_str());

        gad_minbounce->setPos((s32)(200.0 * (GetSystem()->GetMinBounceSpeed())));
        sprintf(message, "%g min.bounce v", GetSystem()->GetMinBounceSpeed());
        gad_minbounce_info->setText(core::stringw(message).c_str());

        switch (GetSystem()->GetLcpSolverType()) {
            case chrono::ChSystem::LCP_ITERATIVE_SOR:
                gad_ccpsolver->setSelected(0);
                break;
            case chrono::ChSystem::LCP_ITERATIVE_SYMMSOR:
                gad_ccpsolver->setSelected(1);
                break;
            case chrono::ChSystem::LCP_ITERATIVE_JACOBI:
                gad_ccpsolver->setSelected(2);
                break;
            case chrono::ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD:
                gad_ccpsolver->setSelected(3);
                break;
            case chrono::ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN:
                gad_ccpsolver->setSelected(4);
                break;
            case chrono::ChSystem::LCP_ITERATIVE_PCG:
                gad_ccpsolver->setSelected(5);
                break;
            case chrono::ChSystem::LCP_ITERATIVE_PMINRES:
                gad_ccpsolver->setSelected(6);
                break;
            case chrono::ChSystem::LCP_ITERATIVE_APGD:
                gad_ccpsolver->setSelected(7);
                break;
            case chrono::ChSystem::LCP_ITERATIVE_MINRES:
                gad_ccpsolver->setSelected(8);
                break;
            default:
                gad_ccpsolver->setSelected(9);
                break;
        }

        switch (GetSystem()->GetIntegrationType()) {
            case chrono::ChSystem::INT_ANITESCU:
                gad_stepper->setSelected(0);
                break;
            case chrono::ChSystem::INT_TASORA:
                gad_stepper->setSelected(1);
                break;
            case chrono::ChSystem::INT_EULER_IMPLICIT:
                gad_stepper->setSelected(2);
                break;
            case chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED:
                gad_stepper->setSelected(3);
                break;
            case chrono::ChSystem::INT_EULER_IMPLICIT_PROJECTED:
                gad_stepper->setSelected(4);
                break;
            case chrono::ChSystem::INT_TRAPEZOIDAL:
                gad_stepper->setSelected(5);
                break;
            case chrono::ChSystem::INT_TRAPEZOIDAL_LINEARIZED:
                gad_stepper->setSelected(6);
                break;
            case chrono::ChSystem::INT_HHT:
                gad_stepper->setSelected(7);
                break;
            case chrono::ChSystem::INT_HEUN:
                gad_stepper->setSelected(8);
                break;
            case chrono::ChSystem::INT_RUNGEKUTTA45:
                gad_stepper->setSelected(9);
                break;
            case chrono::ChSystem::INT_EULER_EXPLICIT:
                gad_stepper->setSelected(10);
                break;
            case chrono::ChSystem::INT_LEAPFROG:
                gad_stepper->setSelected(11);
                break;
            case chrono::ChSystem::INT_NEWMARK:
                gad_stepper->setSelected(12);
                break;
            default:
                gad_stepper->setSelected(13);
                break;
        }

        gad_try_realtime->setChecked(GetTryRealtime());
        gad_pause_step->setChecked(pause_step);

        if (!GetStepManage()) {
            sprintf(message, "%g", GetSystem()->GetStep());
            gad_timestep->setText(core::stringw(message).c_str());
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
void ChIrrAppInterface::DumpMatrices() {
    // For safety
    GetSystem()->Setup();
    GetSystem()->Update();

    // Save the current speeds, maybe these are needed.
    try {
        chrono::ChMatrixDynamic<double> mvold;
        GetSystem()->GetLcpSystemDescriptor()->FromVariablesToVector(mvold);
        chrono::ChStreamOutAsciiFile file_vold("dump_v_old.dat");
        mvold.StreamOUTdenseMatlabFormat(file_vold);
    } catch (chrono::ChException myexc) {
        chrono::GetLog() << myexc.what();
    }

    // This DoStep() is necessary because we want to get the matrices as they
    // are set-up for the time stepping LCP/CCP problem.
    // (If we avoid this, the previous 'mvold' vector won't be in-sync.)
    DoStep();

    // Now save the matrices - as they were setup by the previous time stepping scheme.
    GetSystem()->GetLcpSystemDescriptor()->DumpLastMatrices("dump_");
}

}  // END_OF_NAMESPACE____
