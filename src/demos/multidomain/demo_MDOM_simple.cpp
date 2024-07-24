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
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_multidomain/ChDomainManager.h"


using namespace chrono;
using namespace irrlicht;
using namespace postprocess;
using namespace multidomain;

// For multi domain simulations, each item (body, link, fea element or node, etc.) must have
// an unique ID, to be set via SetTag(). Here we use a static counter to help with the generation
// of unique IDs.
static int unique_ID = 1;


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
                body->SetTag(unique_ID); unique_ID++;
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
                body->SetTag(unique_ID); unique_ID++;

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
    body->SetTag(unique_ID); unique_ID++;

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

void PrintDebugDomainInfo(std::shared_ptr<ChDomain> domain) {
    std::cout <<     "DOMAIN ---- rank: " << domain->GetRank()  << "-----------------------------\n";

    for (auto body : domain->GetSystem()->GetBodies()) {
        std::cout << "  ChBody " << body->GetTag() << std::endl;
    }
    for (auto link : domain->GetSystem()->GetLinks()) {
        std::cout << "  ChLink " << link->GetTag() << std::endl;
        if (auto blink = std::dynamic_pointer_cast<ChLink>(link)) {
            auto mbo1 = dynamic_cast<ChBody*>(blink->GetBody1());
            auto mbo2 = dynamic_cast<ChBody*>(blink->GetBody2());
            if (mbo1 && mbo2) {
                std::cout << "       ChBody*  " << mbo1->GetTag() << std::endl;
                std::cout << "       ChBody*  " << mbo2->GetTag() << std::endl;
            }
            auto mnod1 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody1());
            auto mnod2 = dynamic_cast<fea::ChNodeFEAxyzrot*>(blink->GetBody2());
            if (mnod1 && mnod2) {
                std::cout << "       ChNodeFEAxyzrot*  " << mnod1->GetTag() << std::endl;
                std::cout << "       ChNodeFEAxyzrot*  " << mnod2->GetTag() << std::endl;
            }
        }
    }
    for (auto item : domain->GetSystem()->GetOtherPhysicsItems()) {
        if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
            std::cout << "  ChMesh " << mesh->GetTag() << std::endl;
            for (auto node : mesh->GetNodes()) {
                std::cout << "      ChNodeFEAbase " << node->GetTag() << std::endl;
            }
            for (auto el : mesh->GetElements()) {
                std::cout << "      ChElementBase " << std::endl;
                for (int i = 0; i < el->GetNumNodes(); ++i) {
                    std::cout << "          ChNodeFEABase " << el->GetNode(i)->GetTag() << std::endl;
                }
            }
        }
    }
    for (auto& interf : domain->GetInterfaces()) {
        std::cout << " interface to domain rank " << interf.second.side_OUT->GetRank() << " ...... \n";
        for (auto& item : interf.second.shared_items)
            std::cout << "  shared item tag " << item.first << std::endl;
        for (auto& node : interf.second.shared_nodes)
            std::cout << "  shared node tag " << node.first << std::endl;
    }
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::stringstream mbuffer;
    ChArchiveOutBinary archive_out(mbuffer);


    ///////////
    // TEST

    ChDomainManagerSharedmemory domain_manager;
    ChDomainBuilderSlices       domain_builder(
                                        std::vector<double>{0},  // positions of cuts along axis to slice, ex {-1,0,2} generates 5 domains
                                        ChAxis::X);     // axis about whom one needs the space slicing
    
    // Now one can know how many domains are expected to build, using domain_builder.GetTotRanks();
    // In this case we already know we split into 2 domains. So we build them as:

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

    // Now we populate the n domains with bodies, links, meshes, nodes, etc. Each item must be 
    // added to the ChSystem of the domain that the item overlaps with. In case of body shapes that overlap
    // with multiple domains, add to all of them.

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.1);
 
    auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 2,  // x,y,z size
        100,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    mrigidBody->SetPos(ChVector3d(-3,0,0));
    mrigidBody->SetTag(unique_ID); unique_ID++; // for multidomain, each item must have an unique tag!
    sys_0.AddBody(mrigidBody);
    sys_0.GetCollisionSystem()->BindItem(mrigidBody); 

    try {
        for (int i = 0; i < 10; ++i) {
            mrigidBody->SetPos(ChVector3d(-2 + i * 0.55, 0, 0)); // change pos of body as in simulation
            sys_0.ComputeCollisions(); // to force the AABB of bodies
            sys_1.ComputeCollisions(); // to force the AABB of bodies

            domain_manager.DoUpdateSharedLeaving();
            domain_manager.DoDomainsSendReceive();
            domain_manager.DoUpdateSharedReceived();

            for (auto& mdomain : domain_manager.domains)
                PrintDebugDomainInfo(mdomain.second);
            std::cout << std::endl;
            system("pause");
        }
    }
    catch (std::exception ex) {
        std::cout << ex.what() << std::endl;
    }
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
