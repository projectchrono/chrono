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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - using IRRLICHT as a realtime 3D viewer of a four-bar mechanism
// - using the IRRLICHT graphical user interface (GUI) to handle user-input
//   via mouse.
//
// =============================================================================

#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// Some static data (this is a simple application, we can do this ;) just to allow easy GUI manipulation
IGUIStaticText* text_motorspeed = 0;

// The MyEventReceiver class will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChSystemNSC* system, IrrlichtDevice* device, std::shared_ptr<ChLinkMotorRotationSpeed> motor) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        msystem = system;
        mdevice = device;
        mmotor = motor;
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();

            switch (event.GUIEvent.EventType) {
                case EGET_SCROLL_BAR_CHANGED:
                    if (id == 101)  // id of 'motor speed' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newspeed = 10 * (double)pos / 100.0;
                        // set the speed into motor object
                        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(mmotor->GetSpeedFunction()))
                            mfun->Set_yconst(newspeed);
                        // show speed as formatted text in interface screen
                        char message[50];
                        sprintf(message, "Motor speed: %g [rad/s]", newspeed);
                        text_motorspeed->setText(core::stringw(message).c_str());
                    }
                    break;
                default:
                    break;
            }
        }

        return false;
    }

  private:
    ChSystemNSC* msystem;
    IrrlichtDevice* mdevice;
    std::shared_ptr<ChLinkMotorRotationSpeed> mmotor;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // 1- Create a Chrono physical system
    ChSystemNSC sys;

    // 2- Create the rigid bodies of the four-bar mechanical system
    //   (a flywheel, a rod, a rocker, a truss), maybe setting
    //   position/mass/inertias of their center of mass (COG) etc.

    // ..the truss
    auto my_body_A = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_A);
    my_body_A->SetBodyFixed(true);  // truss does not move!

    // ..the flywheel
    auto my_body_B = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_B);
    my_body_B->SetPos(ChVector<>(0, 0, 0));  // position of COG of flywheel

    // ..the rod
    auto my_body_C = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_C);
    my_body_C->SetPos(ChVector<>(4, 0, 0));  // position of COG of rod

    // ..the rocker
    auto my_body_D = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_D);
    my_body_D->SetPos(ChVector<>(8, -4, 0));  // position of COG of rod

    // 3- Create constraints: the mechanical joints between the
    //    rigid bodies. Doesn't matter if some constraints are redundant.

    // .. a motor between flywheel and truss
    auto my_link_AB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_AB->Initialize(my_body_A, my_body_B, ChFrame<>(ChVector<>(0, 0, 0)));
    sys.AddLink(my_link_AB);
    auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI);  // speed w=3.145 rad/sec
    my_link_AB->SetSpeedFunction(my_speed_function);

    // .. a revolute joint between flywheel and rod
    auto my_link_BC = chrono_types::make_shared<ChLinkLockRevolute>();
    my_link_BC->Initialize(my_body_B, my_body_C, ChCoordsys<>(ChVector<>(2, 0, 0)));
    sys.AddLink(my_link_BC);

    // .. a revolute joint between rod and rocker
    auto my_link_CD = chrono_types::make_shared<ChLinkLockRevolute>();
    my_link_CD->Initialize(my_body_C, my_body_D, ChCoordsys<>(ChVector<>(8, 0, 0)));
    sys.AddLink(my_link_CD);

    // .. a revolute joint between rocker and truss
    auto my_link_DA = chrono_types::make_shared<ChLinkLockRevolute>();
    my_link_DA->Initialize(my_body_D, my_body_A, ChCoordsys<>(ChVector<>(8, -8, 0)));
    sys.AddLink(my_link_DA);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Four-bar mechanism");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, -8));
    vis->AddTypicalLights();

    // ..add a GUI text and GUI slider to control motor of mechanism via mouse
    text_motorspeed = vis->GetGUIEnvironment()->addStaticText(L"Motor speed:", rect<s32>(300, 85, 400, 100), false);
    IGUIScrollBar* scrollbar = vis->GetGUIEnvironment()->addScrollBar(true, rect<s32>(300, 105, 450, 120), 0, 101);
    scrollbar->setMax(100);

    // ..Finally create the custom event receiver
    MyEventReceiver receiver(&sys, vis->GetDevice(), my_link_AB);
    vis->AddUserEventReceiver(&receiver);

    //
    // Configure the solver with non-default settings
    //

    // By default, the solver uses the EULER_IMPLICIT_LINEARIZED stepper, that is very fast,
    // but may allow some geometric error in constraints (because it is based on constraint
    // stabilization). Alternatively, the timestepper EULER_IMPLICIT_PROJECTED is slower,
    // but it is based on constraint projection, so gaps in constraints are less noticeable
    // (hence avoids the 'spongy' behaviour of the default stepper, which operates only
    // on speed-impulse level and keeps constraints'closed' by a continuous stabilization).
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    //
    // Simulation loop
    //

    // use this array of points to store trajectory of a rod-point
    std::vector<chrono::ChVector<> > mtrajectory;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        // .. draw a grid
        tools::drawGrid(vis.get(), 0.5, 0.5);
        // .. draw a circle representing flywheel
        tools::drawCircle(vis.get(), 2.1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
        // .. draw a small circle representing joint BC
        tools::drawCircle(vis.get(), 0.06, ChCoordsys<>(my_link_BC->GetMarker1()->GetAbsCoord().pos, QUNIT));
        // .. draw a small circle representing joint CD
        tools::drawCircle(vis.get(), 0.06, ChCoordsys<>(my_link_CD->GetMarker1()->GetAbsCoord().pos, QUNIT));
        // .. draw a small circle representing joint DA
        tools::drawCircle(vis.get(), 0.06, ChCoordsys<>(my_link_DA->GetMarker1()->GetAbsCoord().pos, QUNIT));
        // .. draw the rod (from joint BC to joint CD)
        tools::drawSegment(vis.get(), my_link_BC->GetMarker1()->GetAbsCoord().pos,
                           my_link_CD->GetMarker1()->GetAbsCoord().pos, ChColor(0, 1, 0));
        // .. draw the rocker (from joint CD to joint DA)
        tools::drawSegment(vis.get(), my_link_CD->GetMarker1()->GetAbsCoord().pos,
                           my_link_DA->GetMarker1()->GetAbsCoord().pos, ChColor(1, 0, 0));
        // .. draw the trajectory of the rod-point
        tools::drawPolyline(vis.get(), mtrajectory, ChColor(0, 0.5f, 0));

        // We need to add another point to the array of 3d points describing the trajectory to be drawn..
        mtrajectory.push_back(my_body_C->Point_Body2World(ChVector<>(1, 1, 0)));
        // keep only last 150 points..
        if (mtrajectory.size() > 150)
            mtrajectory.erase(mtrajectory.begin());

        // Perform time integration
        sys.DoStepDynamics(0.001);

        vis->EndScene();
    }

    return 0;
}
