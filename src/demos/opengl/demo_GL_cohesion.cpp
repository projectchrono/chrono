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
// Authors: Hammad Mazhar
// =============================================================================
//
// Demo code about advanced contact feature: cohesion (using complementarity
// contact method)
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_opengl/ChOpenGLWindow.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::collision;

// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)

float GLOBAL_friction = 0.3f;
float GLOBAL_cohesion = 0;
float GLOBAL_compliance = 0;
float GLOBAL_dampingf = 0.1f;

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface

void create_some_falling_items(ChSystemNSC& mphysicalSystem) {
    // From now on, all created collision models will have a large outward envelope (needed
    // to allow some compliance with the plastic deformation of cohesive bounds
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.3);

    // Create a bunch of Chrono rigid bodies which will fall..
    for (int bi = 0; bi < 400; bi++) {
        auto mrigidBody = std::make_shared<ChBodyEasySphere>(0.81,   // radius
                                                             1000,   // density
                                                             true,   // collide enable?
                                                             true);  // visualization?
        mrigidBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        mrigidBody->GetMaterialSurfaceNSC()->SetFriction(0.3f);

        mphysicalSystem.Add(mrigidBody);
    }

    // Create the five walls of the rectangular container, using
    // fixed rigid bodies of 'box' type:

    auto floorBody = std::make_shared<ChBodyEasyBox>(20, 1, 20,  // x,y,z size
                                                     1000,       // density
                                                     true,       // collide enable?
                                                     true);      // visualization?
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

    auto wallBody1 = std::make_shared<ChBodyEasyBox>(1, 10, 20.99,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody1->SetPos(ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody1);

    auto wallBody2 = std::make_shared<ChBodyEasyBox>(1, 10, 20.99,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody2->SetPos(ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody2);

    auto wallBody3 = std::make_shared<ChBodyEasyBox>(20.99, 10, 1,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody3->SetPos(ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody3);

    auto wallBody4 = std::make_shared<ChBodyEasyBox>(20.99, 10, 1,  // x,y,z size
                                                     1000,          // density
                                                     true,          // collide enable?
                                                     true);         // visualization?
    wallBody4->SetPos(ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody4);

    // Add the rotating mixer
    auto rotatingBody = std::make_shared<ChBodyEasyBox>(10, 5, 1,  // x,y,z size
                                                        4000,      // density
                                                        true,      // collide enable?
                                                        true);     // visualization?
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    rotatingBody->GetMaterialSurfaceNSC()->SetFriction(0.4f);

    mphysicalSystem.Add(rotatingBody);

    // .. a motor between mixer and truss
    auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(rotatingBody, floorBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    motor->SetSpeedFunction(std::make_shared<ChFunction_Const>(CH_C_PI / 2.0));
    mphysicalSystem.AddLink(motor);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono physical system
    ChSystemNSC mphysicalSystem;

    // Create all the rigid bodies.

    create_some_falling_items(mphysicalSystem);

    // Modify some setting of the physical system for the simulation, if you want

    mphysicalSystem.SetSolverType(ChSolver::Type::SOR);
    mphysicalSystem.SetMaxItersSolverSpeed(20);
    // mphysicalSystem.SetMaxItersSolverStab(5);

    // Cohesion in a contact depends on the cohesion in the surface property of
    // the touching bodies, but the user can override this value when each contact is created,
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

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Demo_Cohesion_GL", &mphysicalSystem);
    gl_window.SetCamera(ChVector<>(0, 0, -10), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0));
    gl_window.Pause();
    while (gl_window.Active()) {
        if (gl_window.DoStepDynamics(0.01)) {
            // Add code here that will only run after a step is taken
        }
        gl_window.Render();
    }

    return 0;
}
