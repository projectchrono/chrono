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
//  Demo code about splitting a system into domains using the MULTIDOMAIN module
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/serialization/ChArchiveBinary.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_multidomain/ChDomainManager.h"


using namespace chrono;
using namespace irrlicht;
using namespace postprocess;
using namespace multidomain;


void AddFallingItems(ChSystemSMC& sys) {
    // Shared contact material for falling objects
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();

    for (int ix = -2; ix < 3; ix++) {
        for (int iz = -2; iz < 3; iz++) {
            // Spheres
            {
                double mass = 1;
                double radius = 1.1;
                auto body = chrono_types::make_shared<ChBody>();
                body->SetInertiaXX((2.0 / 5.0) * mass * pow(radius, 2) * ChVector3d(1, 1, 1));
                body->SetMass(mass);
                body->SetPos(ChVector3d(4.0 * ix + 0.1, 4.0, 4.0 * iz));

                auto shape = chrono_types::make_shared<ChCollisionShapeSphere>(mat, radius);
                body->AddCollisionShape(shape);
                body->EnableCollision(true);

                auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(radius);
                sphere->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
                body->AddVisualShape(sphere);

                sys.AddBody(body);
            }

            // Boxes
            {
                double mass = 1;
                ChVector3d size(1.5, 1.5, 1.5);
                auto body = chrono_types::make_shared<ChBody>();

                body->SetMass(mass);
                body->SetPos(ChVector3d(4.0 * ix, 6.0, 4.0 * iz));

                auto shape = chrono_types::make_shared<ChCollisionShapeBox>(mat, size.x(), size.y(), size.z());
                body->AddCollisionShape(shape);
                body->EnableCollision(true);

                auto box = chrono_types::make_shared<ChVisualShapeBox>(size);
                box->SetTexture(GetChronoDataFile("textures/cubetexture_pinkwhite.png"));
                body->AddVisualShape(box);

                sys.AddBody(body);
            }
        }
    }
}

void AddContainerWall(std::shared_ptr<ChBody> body,
                      std::shared_ptr<ChContactMaterial> mat,
                      std::shared_ptr<ChVisualMaterial> vis_mat,
                      const ChVector3d& size,
                      const ChVector3d& pos,
                      bool visible = true) {
    auto coll_shape = chrono_types::make_shared<ChCollisionShapeBox>(mat, size.x(), size.y(), size.z());
    body->AddCollisionShape(coll_shape, ChFrame<>(pos, QUNIT));

    if (visible) {
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(size);
        vis_shape->SetMaterial(0, vis_mat);
        body->AddVisualShape(vis_shape, ChFrame<>(pos, QUNIT));
    }
}

std::shared_ptr<ChBody> AddContainer(ChSystemSMC& sys) {
    // The fixed body (5 walls)
    auto fixedBody = chrono_types::make_shared<ChBody>();

    fixedBody->SetMass(1.0);
    fixedBody->SetFixed(true);
    fixedBody->SetPos(ChVector3d());
    fixedBody->EnableCollision(true);

    // Contact material for container
    auto fixed_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    auto fixed_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    fixed_mat_vis->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));

    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector3d(20, 1, 20), ChVector3d(0, -5, 0));
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector3d(1, 10, 20.99), ChVector3d(-10, 0, 0));
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector3d(1, 10, 20.99), ChVector3d(10, 0, 0));
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector3d(20.99, 10, 1), ChVector3d(0, 0, -10), false);
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector3d(20.99, 10, 1), ChVector3d(0, 0, 10));

    sys.AddBody(fixedBody);

    // The rotating mixer body
    auto rotatingBody = chrono_types::make_shared<ChBody>();

    rotatingBody->SetMass(10.0);
    rotatingBody->SetInertiaXX(ChVector3d(50, 50, 50));
    rotatingBody->SetPos(ChVector3d(0, -1.6, 0));
    rotatingBody->EnableCollision(true);

    // Contact material for mixer body
    auto rot_mat = chrono_types::make_shared<ChContactMaterialSMC>();

    ChVector3d size(10, 5.5, 1.0);

    auto coll_shape = chrono_types::make_shared<ChCollisionShapeBox>(rot_mat, size.x(), size.y(), size.z());
    rotatingBody->AddCollisionShape(coll_shape);

    auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(size);
    vis_shape->SetTexture(GetChronoDataFile("textures/blue.png"));
    rotatingBody->AddVisualShape(vis_shape);

    sys.AddBody(rotatingBody);

    // A motor between the two
    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

    my_motor->Initialize(rotatingBody, fixedBody, ChFrame<>(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2)));
    auto mfun = chrono_types::make_shared<ChFunctionConst>(CH_PI / 2.0);  // speed w=90 deg/s
    my_motor->SetSpeedFunction(mfun);

    sys.AddLink(my_motor);

    return rotatingBody;
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::stringstream mbuffer;
    ChArchiveOutBinary archive_out(mbuffer);


    ///////////
    // TEST

    ChDomainManagerSharedmemory domain_manager;
    ChDomainBuilderSlices       domain_builder(
                                        2,              // how many domains 
                                        -20, 20,        // interval to slice
                                        ChAxis::X);     // axis about whom one needs the space slicing


    ChSystemSMC sys_0;
    sys_0.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    domain_manager.AddDomain(domain_builder.BuildDomain(
                                        &sys_0, // physical system of this domain
                                        0       // rank of this domain 
                                       ));

   
    ChSystemSMC sys_1;
    sys_1.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    domain_manager.AddDomain(domain_builder.BuildDomain(
                                        &sys_1, // physical system of this domain
                                        1       // rank of this domain 
                                       ));

    domain_manager.DoUpdateSharedLeaving();
    domain_manager.DoDomainsSendReceive();
    domain_manager.DoUpdateSharedReceived();


    system("pause");
    return 0;






    // Simulation and rendering time-step
    double time_step = 1e-4;


    // Create the physical system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Add fixed and moving bodies
    auto mixer = AddContainer(sys);
    AddFallingItems(sys);

    // Create the run-time visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Multidomain demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 18, -20));
    vis->AddTypicalLights();

    // Simulation loop
    double out_time = 0;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(time_step);
    }

    return 0;
}
