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
// Demonstration of the conveyor belt primitive.
// For more advanced features regarding feeding devices, look at demo_IRR_feeder.cpp
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChConveyor.h"
#include "chrono/core/ChRealtimeStep.h"

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

// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)

double STATIC_flow = 100;
double STATIC_speed = 2;
std::vector<std::shared_ptr<ChBody> > particlelist;

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChVisualSystemIrrlicht& vsys) : vis(vsys), paused(false) {
        // ..add a GUI slider to control particles flow
        scrollbar_flow = vis.GetGUIEnvironment()->addScrollBar(true, rect<s32>(510, 85, 650, 100), 0, 101);
        scrollbar_flow->setMax(300);
        scrollbar_flow->setPos(150);
        text_flow = vis.GetGUIEnvironment()->addStaticText(L"Flow [particles/s]", rect<s32>(650, 85, 750, 100), false);

        // ..add GUI slider to control the speed
        scrollbar_speed = vis.GetGUIEnvironment()->addScrollBar(true, rect<s32>(510, 125, 650, 140), 0, 102);
        scrollbar_speed->setMax(100);
        scrollbar_speed->setPos(100);
        text_speed =
            vis.GetGUIEnvironment()->addStaticText(L"Conveyor speed [m/s]:", rect<s32>(650, 125, 750, 140), false);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();

            switch (event.GUIEvent.EventType) {
                case EGET_SCROLL_BAR_CHANGED:
                    if (id == 101)  // id of 'flow' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        STATIC_flow = (double)pos;
                    }
                    if (id == 102)  // id of 'speed' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        STATIC_speed = (((double)pos) / 100) * 2;
                    }
                    break;
                default:
                    break;
            }
        }

        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_SPACE:
                    paused = !paused;
                    return true;
                default:
                    break;
            }
        }

        return false;
    }

    bool paused;

  private:
    ChVisualSystemIrrlicht& vis;
    IGUIScrollBar* scrollbar_flow;
    IGUIStaticText* text_flow;
    IGUIScrollBar* scrollbar_speed;
    IGUIStaticText* text_speed;
};

// Function that creates debris that fall on the conveyor belt, to be called at each dt.
// In this example, all debris particles share the same contact material.
void create_debris(ChVisualSystemIrrlicht& vis, ChSystem& sys, double dt, double particles_second) {
    double xnozzlesize = 0.2;
    double znozzlesize = 0.56;
    double ynozzle = 0.2;

    double box_fraction = 0.3;  // 30% cubes
    double cyl_fraction = 0.4;  // 40% cylinders

    double sphrad = 0.013;

    double exact_particles_dt = dt * particles_second;
    double particles_dt = floor(exact_particles_dt);
    double remaind = exact_particles_dt - particles_dt;
    if (remaind > ChRandom())
        particles_dt += 1;

    auto sphere_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    sphere_mat->SetFriction(0.2f);
    sphere_mat->SetRestitution(0.8f);

    auto box_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    box_mat->SetFriction(0.4f);

    auto cyl_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    cyl_mat->SetFriction(0.2f);

    for (int i = 0; i < particles_dt; i++) {
        double rand_fract = ChRandom();

        if (rand_fract < box_fraction) {
            auto rigidBody = chrono_types::make_shared<ChBodyEasySphere>(sphrad,       // size
                                                                         1000,         // density
                                                                         true,         // visualization?
                                                                         true,         // collision?
                                                                         sphere_mat);  // contact material
            rigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize, ynozzle + i * 0.005,
                                         -0.5 * znozzlesize + ChRandom() * znozzlesize));
            rigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));

            sys.Add(rigidBody);

            vis.BindItem(rigidBody);
            sys.GetCollisionSystem()->BindItem(rigidBody);

            particlelist.push_back(rigidBody);
        }

        if ((rand_fract > box_fraction) && (rand_fract < box_fraction + cyl_fraction)) {
            double xscale = 1.3 * (1 - 0.4 * ChRandom());  // for oddly-shaped boxes..
            double yscale = 1.3 * (1 - 0.4 * ChRandom());
            double zscale = 1.3 * (1 - 0.4 * ChRandom());

            auto rigidBody =
                chrono_types::make_shared<ChBodyEasyBox>(sphrad * 2 * xscale, sphrad * 2 * yscale, sphrad * 2 * zscale,
                                                         1000,      // density
                                                         true,      // visualization?
                                                         true,      // collision?
                                                         box_mat);  // contact material
            rigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize, ynozzle + i * 0.005,
                                         -0.5 * znozzlesize + ChRandom() * znozzlesize));
            rigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_bluewhite.png"));

            sys.Add(rigidBody);

            vis.BindItem(rigidBody);
            sys.GetCollisionSystem()->BindItem(rigidBody);

            particlelist.push_back(rigidBody);
        }

        if (rand_fract > box_fraction + cyl_fraction) {
            auto rigidBody = chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y,  //
                                                                           sphrad, sphrad * 2,   // rad, height
                                                                           1000,                 // density
                                                                           true,                 // visualization?
                                                                           true,                 // collision?
                                                                           cyl_mat);             // contact material
            rigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize, ynozzle + i * 0.005,
                                         -0.5 * znozzlesize + ChRandom() * znozzlesize));
            rigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/pinkwhite.png"));

            sys.Add(rigidBody);

            vis.BindItem(rigidBody);
            sys.GetCollisionSystem()->BindItem(rigidBody);

            particlelist.push_back(rigidBody);
        }
    }
}

// Function that deletes old debris (to avoid infinite creation that fills memory)

void purge_debris(ChSystem& sys, int nmaxparticles = 100) {
    while (particlelist.size() > nmaxparticles) {
        sys.Remove(particlelist[0]);               // remove from physical simulation
        particlelist.erase(particlelist.begin());  // remove also from our particle list (will also automatically delete
                                                   // object thank to shared pointer)
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;

    // Set small collision envelopes for objects that will be created from now on
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
    ChCollisionModel::SetDefaultSuggestedMargin(0.002);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create two conveyor fences
    auto fence_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    fence_mat->SetFriction(0.1f);

    auto fence1 = chrono_types::make_shared<ChBodyEasyBox>(2, 0.11, 0.04, 1000, true, true, fence_mat);
    sys.Add(fence1);
    fence1->SetPos(ChVector<>(0, 0, -0.325));
    fence1->SetBodyFixed(true);

    auto fence2 = chrono_types::make_shared<ChBodyEasyBox>(2, 0.11, 0.04, 1000, true, true, fence_mat);
    sys.Add(fence2);
    fence2->SetPos(ChVector<>(0, 0, 0.325));
    fence2->SetBodyFixed(true);

    // Create the conveyor belt (this is a pure Chrono::Engine object,
    // because an Irrlicht 'SceneNode' wrapper is not yet available, so it is invisible - no 3D preview)
    auto conveyor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    conveyor_mat->SetFriction(0.35f);

    auto conveyor = chrono_types::make_shared<ChConveyor>(2, 0.05, 0.6);
    conveyor->SetBodyFixed(true);
    conveyor->SetMaterialSurface(conveyor_mat);
    conveyor->SetConveyorSpeed(STATIC_speed);
    conveyor->SetPos(ChVector<>(0, 0, 0));

    sys.Add(conveyor);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Conveyor belt");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(1.5, 0.4, -1.0), ChVector<>(0.5, 0.0, 0.0));
    vis->AddTypicalLights();

    // Add the custom event receiver to the default interface
    MyEventReceiver receiver(*vis);
    vis->AddUserEventReceiver(&receiver);

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double timestep = 0.005;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(timestep);

        if (!receiver.paused) {
            // Continuosly create debris that fall on the conveyor belt
            create_debris(*vis, sys, timestep, STATIC_flow);

            // Limit the max number of debris particles on the scene, deleting the oldest ones, for performance
            purge_debris(sys, 300);

            // Maybe the user played with the slider and changed STATIC_speed...
            conveyor->SetConveyorSpeed(STATIC_speed);
        }

        realtime_timer.Spin(timestep);
    }

    return 0;
}
