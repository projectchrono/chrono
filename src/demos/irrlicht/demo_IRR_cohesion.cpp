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
// Demo code about advanced contact feature: cohesion (using complementarity
// contact method)
//
// =============================================================================

#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"

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

float GLOBAL_friction = 0.3f;
float GLOBAL_cohesion = 0;
float GLOBAL_compliance = 0;
float GLOBAL_dampingf = 0.1f;

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChIrrAppInterface* myapp) {
        // store pointer application
        application = myapp;

        // ..add a GUI slider to control friction
        scrollbar_friction =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 85, 650, 100), 0, 101);
        scrollbar_friction->setMax(100);
        scrollbar_friction->setPos(30);
        text_friction = application->GetIGUIEnvironment()->addStaticText(L"Friction coefficient:",
                                                                         rect<s32>(650, 85, 750, 100), false);

        // ..add GUI slider to control the speed
        scrollbar_cohesion =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 125, 650, 140), 0, 102);
        scrollbar_cohesion->setMax(100);
        scrollbar_cohesion->setPos(0);
        text_cohesion =
            application->GetIGUIEnvironment()->addStaticText(L"Cohesion [N]:", rect<s32>(650, 125, 750, 140), false);

        // ..add GUI slider to control the compliance
        scrollbar_compliance =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 165, 650, 180), 0, 103);
        scrollbar_compliance->setMax(100);
        scrollbar_compliance->setPos(0);
        text_compliance = application->GetIGUIEnvironment()->addStaticText(L"Compliance [mm/N]:",
                                                                           rect<s32>(650, 165, 750, 180), false);
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
                        GLOBAL_friction = (float)pos / 100;
                    }
                    if (id == 102)  // id of 'speed' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        GLOBAL_cohesion = (((float)pos) / 100) * 200000.0f;
                    }
                    if (id == 103)  // id of 'compliance' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        GLOBAL_compliance = (((float)pos) / 100) / 1000000.0f;
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

    IGUIScrollBar* scrollbar_friction;
    IGUIStaticText* text_friction;
    IGUIScrollBar* scrollbar_cohesion;
    IGUIStaticText* text_cohesion;
    IGUIScrollBar* scrollbar_compliance;
    IGUIStaticText* text_compliance;
};

void create_some_falling_items(ChSystemNSC& mphysicalSystem) {
    // From now on, all created collision models will have a large outward envelope (needed
    // to allow some compliance with the plastic deformation of cohesive bounds
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.3);

    for (int bi = 0; bi < 400; bi++) {
        // Create a bunch of ChronoENGINE rigid bodies which will fall..
        auto mrigidBody = chrono_types::make_shared<ChBodyEasySphere>(0.81,   // radius
                                                             1000,   // density
                                                             true,   // collide enable?
                                                             true);  // visualization?
        mrigidBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        mrigidBody->GetMaterialSurfaceNSC()->SetFriction(0.3f);

        mphysicalSystem.Add(mrigidBody);

        // optional, attach a texture for better visualization
        auto mtexture = chrono_types::make_shared<ChTexture>();
        mtexture->SetTextureFilename(GetChronoDataFile("rock.jpg"));
        mrigidBody->AddAsset(mtexture);
    }

    // Create the five walls of the rectangular container, using
    // fixed rigid bodies of 'box' type:

    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20,  // x,y,z size
                                                     1000,       // density
                                                     true,       // collide enable?
                                                     true);      // visualization?
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

    auto wallBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody1->SetPos(ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody1);

    auto wallBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody2->SetPos(ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody2);

    auto wallBody3 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody3->SetPos(ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody3);

    auto wallBody4 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody4->SetPos(ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody4);

    // optional, attach  textures for better visualization
    auto mtexturewall = chrono_types::make_shared<ChTexture>();
    mtexturewall->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    wallBody1->AddAsset(mtexturewall);  // note: most assets can be shared
    wallBody2->AddAsset(mtexturewall);
    wallBody3->AddAsset(mtexturewall);
    wallBody4->AddAsset(mtexturewall);
    floorBody->AddAsset(mtexturewall);

    // Add the rotating mixer
    auto rotatingBody = chrono_types::make_shared<ChBodyEasyBox>(10, 5, 1,  // x,y,z size
                                                        4000,      // density
                                                        true,      // collide enable?
                                                        true);     // visualization?
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    rotatingBody->GetMaterialSurfaceNSC()->SetFriction(0.4f);

    mphysicalSystem.Add(rotatingBody);

    // .. a motor between mixer and truss
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(rotatingBody, floorBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2.0));
    mphysicalSystem.AddLink(motor);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Contacts with cohesion", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));

    // Create all the rigid bodies.

    create_some_falling_items(mphysicalSystem);

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application);
    // note how to add the custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    // Modify some setting of the physical system for the simulation, if you want

    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    mphysicalSystem.SetSolverMaxIterations(20);

    // Cohesion in a contact depends on the cohesion in the surface property of the
    // touching bodies, but the user can override this value when each contact is created,
    // by instancing a callback as in the following example:

    class MyContactCallback : public ChContactContainer::AddContactCallback {
      public:
        virtual void OnAddContact(const collision::ChCollisionInfo& contactinfo,
                                  ChMaterialComposite* const material) override {
            // Downcast to appropriate composite material type
            auto mat = static_cast<ChMaterialCompositeNSC* const>(material);

            // Set friction according to user setting:
            mat->static_friction = GLOBAL_friction;

            // Set compliance (normal and tangential at once)
            mat->compliance = GLOBAL_compliance;
            mat->complianceT = GLOBAL_compliance;
            mat->dampingf = GLOBAL_dampingf;

            // Set cohesion according to user setting:
            // Note that we must scale the cohesion force value by time step, because
            // the material 'cohesion' value has the dimension of an impulse.
            float my_cohesion_force = GLOBAL_cohesion;
            mat->cohesion = (float)msystem->GetStep() * my_cohesion_force;  //<- all contacts will have this cohesion!

            if (contactinfo.distance > 0.12)
                mat->cohesion = 0;

            // Note that here you might decide to modify the cohesion
            // depending on object sizes, type, time, position, etc. etc.
            // For example, after some time disable cohesion at all, just
            // add here:
            //    if (msystem->GetChTime() > 10) mat->cohesion = 0;
        };
        ChSystemNSC* msystem;
    };

    MyContactCallback mycontact_callback;           // create the callback object
    mycontact_callback.msystem = &mphysicalSystem;  // will be used by callback

    // Use the above callback to process each contact as it is created.
    mphysicalSystem.GetContactContainer()->RegisterAddContactCallback(&mycontact_callback);

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    application.SetStepManage(true);
    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
