// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChConveyor.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::collision;
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
    MyEventReceiver(ChIrrAppInterface* myapp) {
        // store pointer applicaiton
        application = myapp;

        // ..add a GUI slider to control particles flow
        scrollbar_flow = application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 85, 650, 100), 0, 101);
        scrollbar_flow->setMax(300);
        scrollbar_flow->setPos(150);
        text_flow = application->GetIGUIEnvironment()->addStaticText(L"Flow [particles/s]",
                                                                     rect<s32>(650, 85, 750, 100), false);

        // ..add GUI slider to control the speed
        scrollbar_speed = application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 125, 650, 140), 0, 102);
        scrollbar_speed->setMax(100);
        scrollbar_speed->setPos(100);
        text_speed = application->GetIGUIEnvironment()->addStaticText(L"Conveyor speed [m/s]:",
                                                                      rect<s32>(650, 125, 750, 140), false);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();
            IGUIEnvironment* env = application->GetIGUIEnvironment();

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

        return false;
    }

  private:
    ChIrrAppInterface* application;

    IGUIScrollBar* scrollbar_flow;
    IGUIStaticText* text_flow;
    IGUIScrollBar* scrollbar_speed;
    IGUIStaticText* text_speed;
};

// Function that creates debris that fall on the conveyor belt, to be called at each dt

void create_debris(ChIrrApp& application, double dt, double particles_second) {
    double xnozzlesize = 0.2;
    double znozzlesize = 0.56;
    double ynozzle = 0.2;

    double box_fraction = 0.3;  // 30% cubes
    double cyl_fraction = 0.4;  // 40% cylinders
    double sph_fraction = 1 - box_fraction - cyl_fraction;

    double density = 1;
    double sphrad = 0.013;
    double sphmass = (4 / 3) * CH_C_PI * pow(sphrad, 3) * density;
    double sphinertia = pow(sphrad, 2) * sphmass;

    double exact_particles_dt = dt * particles_second;
    double particles_dt = floor(exact_particles_dt);
    double remaind = exact_particles_dt - particles_dt;
    if (remaind > ChRandom())
        particles_dt += 1;

    video::ITexture* bluwhiteMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("bluwhite.png").c_str());
    video::ITexture* pinkwhiteMap =
        application.GetVideoDriver()->getTexture(GetChronoDataFile("pinkwhite.png").c_str());

    for (int i = 0; i < particles_dt; i++) {
        double rand_fract = ChRandom();

        if (rand_fract < box_fraction) {
            auto mrigidBody = std::make_shared<ChBodyEasySphere>(sphrad,  // size
                                                                 1000,    // density
                                                                 true,    // collide enable?
                                                                 true);   // visualization?
            mrigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize, ynozzle + i * 0.005,
                                          -0.5 * znozzlesize + ChRandom() * znozzlesize));
            mrigidBody->GetMaterialSurface()->SetFriction(0.2f);
            mrigidBody->GetMaterialSurface()->SetRestitution(0.8f);
            mrigidBody->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("bluwhite.png")));

            application.GetSystem()->Add(mrigidBody);

            // This will make particle's visualization assets visible in Irrlicht:
            application.AssetBind(mrigidBody);
            application.AssetUpdate(mrigidBody);

            particlelist.push_back(mrigidBody);
        }

        if ((rand_fract > box_fraction) && (rand_fract < box_fraction + cyl_fraction)) {
            double xscale = 1.3 * (1 - 0.4 * ChRandom());  // for oddly-shaped boxes..
            double yscale = 1.3 * (1 - 0.4 * ChRandom());
            double zscale = 1.3 * (1 - 0.4 * ChRandom());

            auto mrigidBody =
                std::make_shared<ChBodyEasyBox>(sphrad * 2 * xscale, sphrad * 2 * yscale, sphrad * 2 * zscale,
                                                1000,   // density
                                                true,   // collide enable?
                                                true);  // visualization?
            mrigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize, ynozzle + i * 0.005,
                                          -0.5 * znozzlesize + ChRandom() * znozzlesize));
            mrigidBody->GetMaterialSurface()->SetFriction(0.4f);
            mrigidBody->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("cubetexture_bluwhite.png")));

            application.GetSystem()->Add(mrigidBody);

            // This will make particle's visualization assets visible in Irrlicht:
            application.AssetBind(mrigidBody);
            application.AssetUpdate(mrigidBody);

            particlelist.push_back(mrigidBody);
        }

        if (rand_fract > box_fraction + cyl_fraction) {
            auto mrigidBody = std::make_shared<ChBodyEasyCylinder>(sphrad, sphrad * 2,  // rad, height
                                                                   1000,                // density
                                                                   true,                // collide enable?
                                                                   true);               // visualization?
            mrigidBody->SetPos(ChVector<>(-0.5 * xnozzlesize + ChRandom() * xnozzlesize, ynozzle + i * 0.005,
                                          -0.5 * znozzlesize + ChRandom() * znozzlesize));
            mrigidBody->GetMaterialSurface()->SetFriction(0.2f);
            mrigidBody->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("pinkwhite.png")));

            application.GetSystem()->Add(mrigidBody);

            // This will make particle's visualization assets visible in Irrlicht:
            application.AssetBind(mrigidBody);
            application.AssetUpdate(mrigidBody);

            particlelist.push_back(mrigidBody);
        }
    }
}

// Function that deletes old debris (to avoid infinite creation that fills memory)

void purge_debris(ChIrrAppInterface& application, int nmaxparticles = 100) {
    while (particlelist.size() > nmaxparticles) {
        application.GetSystem()->Remove(particlelist[0]);  // remove from physical simulation
        particlelist.erase(particlelist.begin());  // remove also from our particle list (will also automatically delete
                                                   // object thank to shared pointer)
    }
}

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Conveyor belt", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df((f32)1.5, (f32)0.4, -1),
                                    core::vector3df((f32)0.5, 0, 0));

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application);
    // note how to add the custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    // Set small collision envelopes for objects that will be created from now on..
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
    ChCollisionModel::SetDefaultSuggestedMargin(0.002);

    // Create two conveyor fences
   
    auto mfence1 = std::make_shared<ChBodyEasyBox>(2, 0.11, 0.04, 1000, true, true);
    mphysicalSystem.Add(mfence1);
    mfence1->SetPos(ChVector<>(0, 0, -0.325));
    mfence1->SetBodyFixed(true);
    mfence1->GetMaterialSurface()->SetFriction(0.1f);
    
    auto mfence2 = std::make_shared<ChBodyEasyBox>(2, 0.11, 0.04, 1000, true, true);
    mphysicalSystem.Add(mfence2);
    mfence2->SetPos(ChVector<>(0, 0,  0.325));
    mfence2->SetBodyFixed(true);
    mfence2->GetMaterialSurface()->SetFriction(0.1f);


    // Create the conveyor belt (this is a pure Chrono::Engine object,
    // because an Irrlicht 'SceneNode' wrapper is not yet available, so it is invisible - no 3D preview)

    auto mconveyor = std::make_shared<ChConveyor>(2, 0.05, 0.6);
    mconveyor->SetBodyFixed(true);
    mconveyor->GetMaterialSurface()->SetFriction(0.35f);
    mconveyor->SetConveyorSpeed(STATIC_speed);
    mconveyor->SetPos(ChVector<>(0, 0, 0));

    mphysicalSystem.Add(mconveyor);



    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();


    //
    // THE SOFT-REAL-TIME CYCLE
    //

    application.SetStepManage(true);
    application.SetTimestep(0.005);

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        if (!application.GetPaused()) {
            // Continuosly create debris that fall on the conveyor belt
            create_debris(application, application.GetTimestep(), STATIC_flow);

            // Limit the max number of debris particles on the scene, deleting the oldest ones, for performance
            purge_debris(application, 300);

            // Maybe the user played with the slider and changed STATIC_speed...
            mconveyor->SetConveyorSpeed(STATIC_speed);
        }

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
