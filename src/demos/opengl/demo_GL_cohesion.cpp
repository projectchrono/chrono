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

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_opengl/ChVisualSystemOpenGL.h"

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

void create_some_falling_items(ChSystemNSC& sys) {
    // From now on, all created collision models will have a large outward envelope (needed
    // to allow some compliance with the plastic deformation of cohesive bounds
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.3);

    // contact material shared by all falling objects
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.3f);

    // Create falling bodies
    for (int bi = 0; bi < 400; bi++) {
        auto mrigidBody = chrono_types::make_shared<ChBodyEasySphere>(0.81,  // radius
                                                                      1000,  // density
                                                                      true,  // visualization?
                                                                      true,  // collision?
                                                                      mat);  // contact material
        mrigidBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        sys.Add(mrigidBody);
    }

    // Create the five walls of the rectangular container, using fixed rigid bodies of 'box' type:

    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20,   // x,y,z size
                                                              1000,        // density
                                                              true,        // visualization?
                                                              true,        // collision?
                                                              floor_mat);  // contact material
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);
    sys.Add(floorBody);

    auto wallBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99,  // x,y,z size
                                                              1000,          // density
                                                              true,          // visualization?
                                                              true,          // collision?
                                                              floor_mat);    // contact material
    wallBody1->SetPos(ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);
    sys.Add(wallBody1);

    auto wallBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99,  // x,y,z size
                                                              1000,          // density
                                                              true,          // visualization?
                                                              true,          // collision?
                                                              floor_mat);    // contact material
    wallBody2->SetPos(ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);
    sys.Add(wallBody2);

    auto wallBody3 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1,  // x,y,z size
                                                              1000,          // density
                                                              true,          // visualization?
                                                              true,          // collision?
                                                              floor_mat);    // contact material
    wallBody3->SetPos(ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);
    sys.Add(wallBody3);

    auto wallBody4 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1,  // x,y,z size
                                                              1000,          // density
                                                              true,          // visualization?
                                                              true,          // collision?
                                                              floor_mat);    // contact material
    wallBody4->SetPos(ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);
    sys.Add(wallBody4);

    // Add the rotating mixer
    auto mixer_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mixer_mat->SetFriction(0.4f);

    auto rotatingBody = chrono_types::make_shared<ChBodyEasyBox>(10, 5, 1,    // x,y,z size
                                                                 4000,        // density
                                                                 true,        // visualization?
                                                                 true,        // collision?
                                                                 mixer_mat);  // contact material
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    sys.Add(rotatingBody);

    // .. a motor between mixer and truss
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(rotatingBody, floorBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2.0));
    sys.AddLink(motor);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono physical system
    ChSystemNSC sys;

    // Create all the rigid bodies.

    create_some_falling_items(sys);

    // Modify some setting of the physical system for the simulation, if you want

    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(20);

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

    auto mycontact_callback = chrono_types::make_shared<MyContactCallback>();  // create the callback object
    mycontact_callback->msystem = &sys;                            // will be used by callback

    // Use the above callback to process each contact as it is created.
    sys.GetContactContainer()->RegisterAddContactCallback(mycontact_callback);

    // Create run-time visualization system
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Demo cohesion");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::SOLID);
    vis.Initialize();
    vis.AddCamera(ChVector<>(0, 0, -10), ChVector<>(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Y);

    std::function<void()> step_iter = [&]() {
        if (sys.DoStepDynamics(0.01)) {
            // Add code here that will only run after a step is taken
        }
        vis.Render();
    };

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(&opengl::ChVisualSystemOpenGL::WrapRenderStep, (void*)&step_iter, 50, true);
#else
    while (vis.Run()) {
        step_iter();
    }
#endif

    return 0;
}
