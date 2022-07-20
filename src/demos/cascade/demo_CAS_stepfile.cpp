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
//   Show how to use the OpenCASCADE features
//   implemented in the unit_CASCADE:
//
//   - load a 3D model saved in STEP format from a CAD
//   - select some sub assemblies from the STEP model
//   - make Chrono::Engine objects out of those parts
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_cascade/ChCascadeBodyEasy.h"
#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeVisualShape.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the namespace with OpenCascade stuff
using namespace cascade;
int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system: all bodies and constraints
    // will be handled by this ChSystemNSC object.
    ChSystemNSC sys;

    //
    // Load a STEP file, containing a mechanism. The demo STEP file has been
    // created using a 3D CAD (in this case, SolidEdge v.18).
    //

    // Create the ChCascadeDoc, a container that loads the STEP model
    // and manages its subassembles
    ChCascadeDoc mydoc;

    // load the STEP model using this command:
    bool load_ok = mydoc.Load_STEP(GetChronoDataFile("cascade/assembly.stp").c_str());
    // or specify abs.path: ("C:\\data\\cascade\\assembly.stp");

    // print the contained shapes
    mydoc.Dump(GetLog());

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.001);

    // In most CADs the Y axis is horizontal, but we want it vertical.
    // So define a root transformation for rotating all the imported objects.
    ChQuaternion<> rotation1;
    rotation1.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));  // 1: rotate 90° on X axis
    ChQuaternion<> rotation2;
    rotation2.Q_from_AngAxis(CH_C_PI, ChVector<>(0, 1, 0));  // 2: rotate 180° on vertical Y axis
    ChQuaternion<> tot_rotation = rotation2 % rotation1;     // rotate on 1 then on 2, using quaternion product
    ChFrameMoving<> root_frame(ChVector<>(0, 0, 0), tot_rotation);

    // Retrieve some sub shapes from the loaded model, using
    // the GetNamedShape() function, that can use path/subpath/subsubpath/part
    // syntax and * or ? wildcards, etc.

    std::shared_ptr<ChCascadeBodyEasy> body1;
    std::shared_ptr<ChCascadeBodyEasy> body2;

    if (load_ok) {
        TopoDS_Shape shape1;
        if (mydoc.GetNamedShape(shape1, "Assem1/body1")) {
            // Create the ChBody using the ChCascadeBodyEasy helper:
            body1 = chrono_types::make_shared<ChCascadeBodyEasy>(shape1,
                                                                 1000,  // density
                                                                 true,  // add a visualization
                                                                 false  // add a collision model
            );
            sys.Add(body1);
            body1->SetBodyFixed(true);
            // Move the body as for global displacement/rotation (also mbody1 %= root_frame; )
            body1->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape2;
        if (mydoc.GetNamedShape(shape2, "Assem1/body2")) {
            // Create the ChBody using the ChCascadeBodyEasy helper (with more detailed visualization tesselation):
            auto vis_params = chrono_types::make_shared<ChCascadeTriangulate>(  //
                0.02,                                                           // chordal deflection for triangulation
                false,                                                          // chordal deflection is relative
                0.5                                                             // angular deflection for triangulation
            );
            body2 = chrono_types::make_shared<ChCascadeBodyEasy>(shape2,
                                                                 1000,        // density
                                                                 vis_params,  // add a visualization
                                                                 false        // no collision model
            );
            sys.Add(body2);
            // Move the body as for global displacement/rotation  (also mbody2 %= root_frame; )
            body2->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

    } else
        GetLog() << "Warning. Desired STEP file could not be opened/parsed \n";

    // Create a revolute joint between the two parts
    // as in a pendulum. We assume we already know in advance
    // the aboslute position of the joint (ex. we used measuring tools in the 3D CAD)
    ChVector<> measured_joint_pos_mm(0, 48, 120);
    double scale = 1. / 1000.;  // because we use meters instead of mm
    ChVector<> joint_pos =
        ((ChFrame<>)root_frame) * (measured_joint_pos_mm * scale);  // transform because we rotated everything

    if (body1 && body2) {
        std::shared_ptr<ChLinkLockRevolute> my_link(new ChLinkLockRevolute);
        my_link->Initialize(body1, body2, ChCoordsys<>(joint_pos));
        sys.AddLink(my_link);
    }

    // Create a large cube as a floor.
    std::shared_ptr<ChBodyEasyBox> floor(new ChBodyEasyBox(1, 0.2, 1, 1000));
    floor->SetPos(ChVector<>(0, -0.3, 0));
    floor->SetBodyFixed(true);
    floor->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.3f, 0.8f));
    sys.Add(floor);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Load a STEP model from file");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0.2, 0.2, -0.3));
    vis->AddTypicalLights();

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double time_step = 0.01;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(time_step);
        realtime_timer.Spin(time_step);
    }

    return 0;
}
