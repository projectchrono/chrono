// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Another demo for the POV-Ray exporter
//
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_postprocess/ChPovRay.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace postprocess;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(1e-3);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(1e-3);

    // Create the ChParticleClones, populate it with some random particles, and add it to physical system
    auto particles = chrono_types::make_shared<ChParticleCloud>();
    particles->SetMass(0.01);
    particles->SetInertiaXX(ChVector<>((2.0 / 5) * (0.005 * 0.005) * 0.01));

    auto particle_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    particles->GetCollisionModel()->ClearModel();
    particles->GetCollisionModel()->AddSphere(particle_mat, 0.005);
    particles->GetCollisionModel()->BuildModel();
    particles->SetCollide(true);

    for (int ix = 0; ix < 5; ix++)
        for (int iy = 0; iy < 5; iy++)
            for (int iz = 0; iz < 3; iz++)
                particles->AddParticle(ChCoordsys<>(ChVector<>(ix / 100.0, 0.1 + iy / 100.0, iz / 100.0)));

    auto particle_shape = chrono_types::make_shared<ChSphereShape>();
    particle_shape->GetSphereGeometry().rad = 0.005;
    particles->AddVisualShape(particle_shape);

    sys.Add(particles);

    // Create the floor body
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);

    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    floor->GetCollisionModel()->ClearModel();
    floor->GetCollisionModel()->AddBox(floor_mat, 0.1, 0.02, 0.1);
    floor->GetCollisionModel()->BuildModel();
    floor->SetCollide(true);

    auto floor_shape = chrono_types::make_shared<ChBoxShape>();
    floor_shape->GetBoxGeometry().Size = ChVector<>(0.1, 0.02, 0.1);
    floor_shape->SetColor(ChColor(0.5f, 0.5f, 0.5f));
    floor->AddVisualShape(floor_shape);

    sys.Add(floor);

    // Create falling boxes
    for (int ix = 0; ix < 2; ix++) {
        for (int iz = 0; iz < 4; iz++) {
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos(ChVector<>(0.05 + ix * 0.021, 0.04, 0 + iz * 0.021));
            body->SetMass(0.02);
            body->SetInertiaXX(ChVector<>((2.0 / 5.0) * (0.01 * 0.01) * 0.02));

            body->GetCollisionModel()->ClearModel();
            body->GetCollisionModel()->AddBox(floor_mat, 0.01, 0.01, 0.01);
            body->GetCollisionModel()->BuildModel();
            body->SetCollide(true);

            auto body_shape = chrono_types::make_shared<ChBoxShape>();
            body_shape->GetBoxGeometry().Size = ChVector<>(0.01, 0.01, 0.01);
            body->AddVisualShape(body_shape);

            sys.Add(body);
        }
    }

    // Create an exporter to POVray
    ChPovRay pov_exporter = ChPovRay(&sys);
    pov_exporter.SetTemplateFile(GetChronoDataFile("POVRay_chrono_template.pov"));
    pov_exporter.SetBasePath(GetChronoOutputPath() + "POVRAY_2");

    pov_exporter.SetCamera(ChVector<>(0.2, 0.3, 0.5), ChVector<>(0, 0, 0), 35);
    pov_exporter.SetLight(ChVector<>(-2, 2, -1), ChColor(1.0f, 1.0f, 1.0f), true);
    pov_exporter.SetPictureSize(640, 480);
    pov_exporter.SetAmbientLight(ChColor(0.8f, 0.8f, 0.8f));

    pov_exporter.SetCustomPOVcommandsScript(
        "light_source { <1,3,1.5> color rgb<1.1,1.1,1.1> } \
	    Grid(0.05, 0.04, rgb<0.7,0.7,0.7>, rgbt<1,1,1,1>)");

    pov_exporter.AddAll();

    pov_exporter.SetShowContacts(true,                                         //
                                 ChPovRay::ContactSymbol::VECTOR_SCALELENGTH,  //
                                 0.2,                                          // scale
                                 0.0007,                                       // width
                                 0.1,                                          // max size
                                 true, 0, 0.5                                  // colormap on, blue at 0, red at 0.5
    );

    pov_exporter.ExportScript();

    sys.SetSolverMaxIterations(50);

    while (sys.GetChTime() < 0.7) {
        sys.DoStepDynamics(0.005);
        std::cout << "time = " << sys.GetChTime() << std::endl;
        pov_exporter.ExportData();
    }

    return 0;
}
