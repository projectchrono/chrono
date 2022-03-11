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

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChProfiler.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveExplorer.h"

#include "chrono_irrlicht/ChIrrGUI.h"

namespace chrono {
namespace irrlicht {

// -----------------------------------------------------------------------------
// Custom Irrlicht event receiver class.
class ChIrrEventReceiver : public irr::IEventReceiver {
  public:
    ChIrrEventReceiver(ChIrrGUI* gui) : m_gui(gui) {}
    ~ChIrrEventReceiver() {}
    bool OnEvent(const irr::SEvent& event);

  private:
    ChIrrGUI* m_gui;
};

bool ChIrrEventReceiver::OnEvent(const irr::SEvent& event) {
    // Check if there are any user-specified event receivers. Give them the first chance to process the event (in the
    // order in which the user-specified event receivers were registered with the application.
    if (m_gui) {
        for (auto recv : m_gui->m_user_receivers) {
            if (recv->OnEvent(event))
                return true;
        }
    }

    // Process keyboard events
    if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case irr::KEY_KEY_I:
                m_gui->SetShowInfos(!m_gui->GetShowInfos());
                return true;
            case irr::KEY_KEY_O:
                m_gui->SetShowProfiler(!m_gui->GetShowProfiler());
                return true;
            case irr::KEY_KEY_U:
                m_gui->SetShowExplorer(!m_gui->GetShowExplorer());
                return true;
            case irr::KEY_F8: {
                GetLog() << "Saving system in JSON format to dump.json file \n";
                ChStreamOutAsciiFile mfileo("dump.json");
                ChArchiveOutJSON marchiveout(mfileo);
                marchiveout.SetUseVersions(false);
                marchiveout << CHNVP(m_gui->m_system, "System");

                GetLog() << "Saving system in ASCII format to dump.txt file \n";
                ChStreamOutAsciiFile mfileo2("dump.txt");
                ChArchiveAsciiDump marchiveout2(mfileo2);
                marchiveout2.SetUseVersions(false);
                marchiveout2 << CHNVP(m_gui->m_system, "System");
            }
            case irr::KEY_F6:
                GetLog() << "Saving system vector and matrices to dump_xxyy.dat files.\n";
                m_gui->DumpSystemMatrices();
                return true;
            case irr::KEY_F7:
                if (!m_gui->m_system->IsSolverMatrixWriteEnabled()) {
                    GetLog() << "Start saving system vector and matrices to *.dat files...\n";
                    m_gui->m_system->EnableSolverMatrixWrite(true);
                } else {
                    GetLog() << "Stop saving system vector and matrices to *.dat files.\n";
                    m_gui->m_system->EnableSolverMatrixWrite(false);
                }
                return true;
            case irr::KEY_F4:
                if (m_gui->camera_auto_rotate_speed <= 0)
                    m_gui->camera_auto_rotate_speed = 0.02;
                else
                    m_gui->camera_auto_rotate_speed *= 1.5;
                return true;
            case irr::KEY_F3:
                m_gui->camera_auto_rotate_speed = 0;
                return true;
            case irr::KEY_F2:
                if (m_gui->camera_auto_rotate_speed >= 0)
                    m_gui->camera_auto_rotate_speed = -0.02;
                else
                    m_gui->camera_auto_rotate_speed *= 1.5;
                return true;
            case irr::KEY_ESCAPE:
                m_gui->GetDevice()->closeDevice();
                return true;
            default:
                break;
        }
    }

    // Process GUI events
    if (event.EventType == irr::EET_GUI_EVENT) {
        irr::s32 id = event.GUIEvent.Caller->getID();

        switch (event.GUIEvent.EventType) {
            case irr::gui::EGET_EDITBOX_ENTER:
                if (id == 9921) {
                    double scale = 0.01;
                    scale =
                        atof(irr::core::stringc(((irr::gui::IGUIEditBox*)event.GUIEvent.Caller)->getText()).c_str());
                    m_gui->SetSymbolscale(scale);
                    break;
                }

            default:
                break;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------

class DebugDrawer : public collision::ChCollisionSystem::VisualizationCallback {
  public:
    explicit DebugDrawer(irr::video::IVideoDriver* driver)
        : m_driver(driver), m_debugMode(0), m_linecolor(255, 255, 0, 0) {}
    ~DebugDrawer() {}

    virtual void DrawLine(const ChVector<>& from, const ChVector<>& to, const ChColor& color) override {
        m_driver->draw3DLine(irr::core::vector3dfCH(from), irr::core::vector3dfCH(to), m_linecolor);
    }

    virtual double GetNormalScale() const override { return 1.0; }

    void SetLineColor(irr::video::SColor& mcolor) { m_linecolor = mcolor; }

  private:
    irr::video::IVideoDriver* m_driver;
    int m_debugMode;
    irr::video::SColor m_linecolor;
};

// -----------------------------------------------------------------------------

ChIrrGUI::ChIrrGUI(irr::IrrlichtDevice* device, ChSystem* sys)
    : m_device(device),
      m_system(sys),
      show_explorer(false),
      show_infos(false),
      show_profiler(false),
      symbolscale(1),
      camera_auto_rotate_speed(0) {
    // Set the default event receiver
    m_receiver = new ChIrrEventReceiver(this);
    m_device->setEventReceiver(m_receiver);

    // Create the collision visualization callback object
    m_drawer = chrono_types::make_shared<DebugDrawer>(GetVideoDriver());
    if (m_system->GetCollisionSystem())
        m_system->GetCollisionSystem()->RegisterVisualizationCallback(m_drawer);

    // Grab the GUI environment
    auto guienv = m_device->getGUIEnvironment();

    irr::gui::IGUISkin* skin = guienv->getSkin();
    irr::gui::IGUIFont* font = guienv->getFont(GetChronoDataFile("fonts/arial8.xml").c_str());
    if (font)
        skin->setFont(font);
    skin->setColor(irr::gui::EGDC_BUTTON_TEXT, irr::video::SColor(255, 40, 50, 50));
    skin->setColor(irr::gui::EGDC_HIGH_LIGHT, irr::video::SColor(255, 40, 70, 250));
    skin->setColor(irr::gui::EGDC_FOCUSED_EDITABLE, irr::video::SColor(255, 0, 255, 255));
    skin->setColor(irr::gui::EGDC_3D_HIGH_LIGHT, irr::video::SColor(200, 210, 210, 210));

    // Create GUI gadgets
    gad_tabbed = guienv->addTabControl(irr::core::rect<irr::s32>(2, 70, 220, 510), 0, true, true);
    auto gad_tab1 = gad_tabbed->addTab(L"Stats");
    auto gad_tab2 = gad_tabbed->addTab(L"Help");

    gad_textFPS = guienv->addStaticText(L"FPS", irr::core::rect<irr::s32>(10, 10, 200, 230), true, true, gad_tab1);

    gad_labelcontacts = guienv->addComboBox(irr::core::rect<irr::s32>(10, 240, 200, 240 + 20), gad_tab1, 9901);
    gad_labelcontacts->addItem(L"Contact distances");
    gad_labelcontacts->addItem(L"Contact force modulus");
    gad_labelcontacts->addItem(L"Contact force (normal)");
    gad_labelcontacts->addItem(L"Contact force (tangent)");
    gad_labelcontacts->addItem(L"Contact torque modulus");
    gad_labelcontacts->addItem(L"Contact torque (spinning)");
    gad_labelcontacts->addItem(L"Contact torque (rolling)");
    gad_labelcontacts->addItem(L"Don't print contact values");
    gad_labelcontacts->setSelected(7);

    gad_drawcontacts = guienv->addComboBox(irr::core::rect<irr::s32>(10, 260, 200, 260 + 20), gad_tab1, 9901);
    gad_drawcontacts->addItem(L"Contact normals");
    gad_drawcontacts->addItem(L"Contact distances");
    gad_drawcontacts->addItem(L"Contact N forces");
    gad_drawcontacts->addItem(L"Contact forces");
    gad_drawcontacts->addItem(L"Don't draw contacts");
    gad_drawcontacts->setSelected(4);

    gad_labellinks = guienv->addComboBox(irr::core::rect<irr::s32>(10, 280, 200, 280 + 20), gad_tab1, 9923);
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

    gad_drawlinks = guienv->addComboBox(irr::core::rect<irr::s32>(10, 300, 200, 300 + 20), gad_tab1, 9924);
    gad_drawlinks->addItem(L"Link reaction forces");
    gad_drawlinks->addItem(L"Link reaction torques");
    gad_drawlinks->addItem(L"Don't draw link vectors");
    gad_drawlinks->setSelected(2);

    gad_plot_aabb =
        guienv->addCheckBox(false, irr::core::rect<irr::s32>(10, 330, 200, 330 + 15), gad_tab1, 9914, L"Draw AABB");

    gad_plot_cogs =
        guienv->addCheckBox(false, irr::core::rect<irr::s32>(10, 345, 200, 345 + 15), gad_tab1, 9915, L"Draw COGs");

    gad_plot_linkframes = guienv->addCheckBox(false, irr::core::rect<irr::s32>(10, 360, 200, 360 + 15), gad_tab1, 9920,
                                              L"Draw link frames");

    gad_plot_collisionshapes = guienv->addCheckBox(false, irr::core::rect<irr::s32>(10, 375, 200, 375 + 15), gad_tab1,
                                                   9902, L"Draw collision shapes");

    gad_plot_convergence = guienv->addCheckBox(false, irr::core::rect<irr::s32>(10, 390, 200, 390 + 15), gad_tab1, 9902,
                                               L"Plot convergence");

    gad_symbolscale = guienv->addEditBox(L"", irr::core::rect<irr::s32>(170, 330, 200, 330 + 15), true, gad_tab1, 9921);
    gad_symbolscale_info = guienv->addStaticText(L"Symbols scale", irr::core::rect<irr::s32>(110, 330, 170, 330 + 15),
                                                 false, false, gad_tab1);
    SetSymbolscale(symbolscale);

    // -- gad_tab2

    gad_textHelp = guienv->addStaticText(L"FPS", irr::core::rect<irr::s32>(10, 10, 200, 380), true, true, gad_tab2);
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

    gad_treeview = guienv->addTreeView(
        irr::core::rect<irr::s32>(2, 80, 300, GetVideoDriver()->getScreenSize().Height - 4), 0, 9919, true, true, true);
    auto child = gad_treeview->getRoot()->addChildBack(L"System", 0);
    child->setExpanded(true);
}

ChIrrGUI::~ChIrrGUI() {
    delete m_receiver;
}

void ChIrrGUI::AddUserEventReceiver(irr::IEventReceiver* receiver) {
    m_user_receivers.push_back(receiver);
}

void ChIrrGUI::SetSymbolscale(double val) {
    symbolscale = ChMax(10e-12, val);
    char message[50];
    sprintf(message, "%g", symbolscale);
    gad_symbolscale->setText(irr::core::stringw(message).c_str());
}

// -----------------------------------------------------------------------------

void ChIrrGUI::DumpSystemMatrices() {
    // For safety
    m_system->Setup();
    m_system->Update();

    try {
        // Save M mass matrix, K stiffness matrix, R damping matrix, Cq jacobians:
        m_system->DumpSystemMatrices(true, true, true, true, "dump_");

    } catch (const ChException& myexc) {
        std::cerr << myexc.what() << std::endl;
    }
}

// -----------------------------------------------------------------------------

static void recurse_update_tree_node(ChValue* value, irr::gui::IGUITreeViewNode* mnode) {
    ChArchiveExplorer mexplorer2;
    mexplorer2.FetchValues(*value, "*");
    int ni = 0;
    auto subnode = mnode->getFirstChild();
    for (auto j : mexplorer2.GetFetchResults()) {
        ++ni;
        if (!subnode) {
            subnode = mnode->addChildBack(L"_to_set_");
            subnode->setExpanded(false);
        }
        // update the subnode visual info:
        irr::core::stringw jstr(j->name());
        if (j->HasArchiveContainerName()) {
            jstr = L"'";
            jstr += j->CallArchiveContainerName().c_str();
            jstr += "'";
        }
        if (j->GetClassRegisteredName() != "") {
            jstr += L",  [";
            jstr += irr::core::stringw(j->GetClassRegisteredName().c_str());
            jstr += L"] ";
        }
        if (auto mydouble = j->PointerUpCast<double>()) {
            jstr += " =";
            auto stringval = std::to_string(*mydouble);
            jstr += irr::core::stringw(stringval.c_str());
        }
        if (auto myfloat = j->PointerUpCast<float>()) {
            jstr += " =";
            auto stringval = std::to_string(*myfloat);
            jstr += irr::core::stringw(stringval.c_str());
        }
        if (auto myint = j->PointerUpCast<int>()) {
            jstr += " =";
            auto stringval = std::to_string(*myint);
            jstr += irr::core::stringw(stringval.c_str());
        }
        if (auto mybool = j->PointerUpCast<bool>()) {
            jstr += " =";
            auto stringval = std::to_string(*mybool);
            jstr += irr::core::stringw(stringval.c_str());
        }
        subnode->setText(jstr.c_str());

        // recursion to update children nodes
        if (subnode->getExpanded())
            recurse_update_tree_node(j, subnode);

        // this to show the "+" symbol for not yet explored nodes
        ChArchiveExplorer mexplorer3;
        mexplorer3.FetchValues(*j, "*");
        if (subnode->getChildCount() == 0 && mexplorer3.GetFetchResults().size()) {
            subnode->addChildBack(L"_foo_to_set_");
        }

        // process next sub property and corresponding sub node
        subnode = subnode->getNextSibling();
    }
    auto subnode_to_remove = subnode;
    while (subnode_to_remove) {
        auto othernode = subnode_to_remove->getNextSibling();
        mnode->deleteChild(subnode_to_remove);
        subnode_to_remove = othernode;
    }
}

void ChIrrGUI::DrawAll() {
    CH_PROFILE("DrawAll");

    irr::core::stringw str = "World time:  ";
    str += (int)(1000 * m_system->GetChTime());
    str += " ms";
    str += "\n\nCPU step (total):  ";
    str += (int)(1000 * m_system->GetTimerStep());
    str += " ms";
    str += "\n  CPU Collision time:  ";
    str += (int)(1000 * m_system->GetTimerCollision());
    str += " ms";
    str += "\n  CPU Solver time:  ";
    str += (int)(1000 * m_system->GetTimerLSsolve());
    str += " ms";
    str += "\n  CPU Update time:  ";
    str += (int)(1000 * m_system->GetTimerUpdate());
    str += " ms";
    str += "\n\nN.of active bodies:  ";
    str += m_system->GetNbodies();
    str += "\nN.of sleeping bodies:  ";
    str += m_system->GetNbodiesSleeping();
    str += "\nN.of contacts:  ";
    str += m_system->GetNcontacts();
    str += "\nN.of coords:  ";
    str += m_system->GetNcoords_w();
    str += "\nN.of constr:  ";
    str += m_system->GetNdoc_w();
    str += "\nN.of variables:  ";
    str += m_system->GetNsysvars_w();
    gad_textFPS->setText(str.c_str());

    int dmode = gad_drawcontacts->getSelected();
    tools::drawAllContactPoints(m_system->GetContactContainer(), GetVideoDriver(), symbolscale,
                                (IrrContactsDrawMode)dmode);

    int lmode = gad_labelcontacts->getSelected();
    tools::drawAllContactLabels(m_system->GetContactContainer(), GetDevice(), (IrrContactsLabelMode)lmode);

    int dmodeli = gad_drawlinks->getSelected();
    tools::drawAllLinks(*m_system, GetVideoDriver(), symbolscale, (IrrLinkDrawMode)dmodeli);

    int lmodeli = gad_labellinks->getSelected();
    tools::drawAllLinkLabels(*m_system, GetDevice(), (IrrLinkLabelMode)lmodeli);

    if (gad_plot_aabb->isChecked())
        tools::drawAllBoundingBoxes(*m_system, GetVideoDriver());

    if (gad_plot_cogs->isChecked())
        tools::drawAllCOGs(*m_system, GetVideoDriver(), symbolscale);

    if (gad_plot_linkframes->isChecked())
        tools::drawAllLinkframes(*m_system, GetVideoDriver(), symbolscale);

    if (gad_plot_collisionshapes->isChecked())
        DrawCollisionShapes(irr::video::SColor(50, 0, 0, 110));

    if (gad_plot_convergence->isChecked())
        tools::drawHUDviolation(GetVideoDriver(), GetDevice(), *m_system, 240, 370, 300, 100, 100.0);

    gad_tabbed->setVisible(show_infos);
    gad_treeview->setVisible(show_explorer);
    if (show_explorer) {
        chrono::ChValueSpecific<ChSystem> root(*m_system, "system", 0);
        recurse_update_tree_node(&root, gad_treeview->getRoot());
    }

    GetGUIEnvironment()->drawAll();
}

void ChIrrGUI::DrawCollisionShapes(irr::video::SColor color) {
    if (!m_drawer)
        return;

    std::static_pointer_cast<DebugDrawer>(m_drawer)->SetLineColor(color);

    GetVideoDriver()->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
    irr::video::SMaterial mattransp;
    mattransp.ZBuffer = true;
    mattransp.Lighting = false;
    GetVideoDriver()->setMaterial(mattransp);

    m_system->GetCollisionSystem()->Visualize(collision::ChCollisionSystem::VIS_Shapes);
}

void ChIrrGUI::BeginScene() {
    if (camera_auto_rotate_speed) {
        irr::core::vector3df pos = GetActiveCamera()->getPosition();
        irr::core::vector3df target = GetActiveCamera()->getTarget();
        pos.rotateXZBy(camera_auto_rotate_speed, target);
        GetActiveCamera()->setPosition(pos);
        GetActiveCamera()->setTarget(target);
    }
}

void ChIrrGUI::EndScene() {
    if (show_profiler)
        tools::drawProfiler(m_device);
}

}  // end namespace irrlicht
}  // end namespace chrono
