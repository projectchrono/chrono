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
//  using shared memory (just OpenMP multithreading, no MPI).
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include "chrono_multidomain/ChDomainManagerSharedmemory.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace multidomain;
using namespace chrono::irrlicht;

// For multi domain simulations, each item (body, link, fea element or node, etc.) must have
// an unique ID, to be set via SetTag(). Here we use a static counter to help with the generation
// of unique IDs.
static int unique_ID = 1;




int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // This source generates a simple executable with basic std::cout output on the command
    // line console, enough to show basic functionality of the multidomain module.
    // 
    // All the examples herein are based on the ChDomainManagerSharedmemory  domain manager, 
    // that is, no need for MPI because the domains are split in the single executable using
    // OpenMP multithreading. This OpenMP multithreading is enough for didactical reasons,
    // then more advanced stuff must run usnig the MPI domain manager, shown in other demos.


    // EXAMPLE A
    //
    // Basic functionality of the multidomain module: a body traveling through two domains.

    // 1- first you need a domain manager. This will use the OpenMP multithreading 
    // as a method for parallelism:

    ChDomainManagerSharedmemory domain_manager;

    // For debugging/logging:
    domain_manager.verbose_partition = false; // will print partitioning in std::cout ?
    domain_manager.verbose_serialization = false; // will print serialization buffers in std::cout ?
    domain_manager.verbose_variable_updates = true; // will print all messages in std::cout ?
    domain_manager.serializer_type = DomainSerializerFormat::JSON;

    // 2- the domain builder.
    // You must define how the 3D space is divided in domains. 
    // ChdomainBuilder classes help you to do this. 
    // Here we split it using parallel planes like in sliced bread:

    ChDomainBuilderSlices       domain_builder(
                                        std::vector<double>{0},  // positions of cuts along axis to slice, ex {-1,0,2} generates 5 domains
                                        ChAxis::X);     // axis about whom one needs the space slicing
    

    // 3- create the ChDomain objects and their distinct ChSystem physical systems.
    // Now one can know how many domains are expected to build, using domain_builder.GetTotRanks();
    // But in this case we already know we split into 2 domains. So we build them as:

    ChSystemNSC sys_0;
    sys_0.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys_0.GetSolver()->AsIterative()->SetMaxIterations(15);

    domain_manager.AddDomain(domain_builder.BuildDomain(
                                        &sys_0, // physical system of this domain
                                        0       // rank of this domain 
                                       ));

    ChSystemNSC sys_1;
    sys_1.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys_1.GetSolver()->AsIterative()->SetMaxIterations(15);

    domain_manager.AddDomain(domain_builder.BuildDomain(
                                        &sys_1, // physical system of this domain
                                        1       // rank of this domain 
                                       ));

    // 4- we populate the n domains with bodies, links, meshes, nodes, etc. 
    // - Each "node" item (ChBody, ChNode stuff) must be added to the ChSystem of the domain that 
    //   the AABB of item overlaps with. Even added into multiple domains, then, becoming shared. 
    // - Each "edge" item (ChLink constraints, ChElement finite elements) must be added to the 
    //   domain where the reference point (1st node of fea element, master body in links) is inside.
    //   Hence edges are always into a single domain, and never shared among domains.
    // - If an edge is in the domain and references a node that is not overlapping with domain, 
    //   then such node(s) must be added too to the domain anyway, becoming shared with surrounding domain(s).

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.1);
 
    auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 2,  // x,y,z size
        100,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    sys_0.AddBody(mrigidBody);
    mrigidBody->SetPos(ChVector3d(-1.5,0,0));
    mrigidBody->SetPosDt(ChVector3d(20, 0, 0));
    // 5- a very important thing: for multidomain, each item (body, mesh, link, node, FEA element)
    // must have an unique tag! This SetTag() is needed because items might be shared between neighbouring domains. 
    mrigidBody->SetTag(unique_ID); unique_ID++; 

    auto mrigidBodyb = chrono_types::make_shared<ChBodyEasyBox>(0.7, 0.7, 0.7,  // x,y,z size
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    sys_1.AddBody(mrigidBodyb);
    mrigidBodyb->SetPos(ChVector3d(1.5, 0, 0));
    mrigidBodyb->SetTag(unique_ID); unique_ID++;

    auto mrigidBodyc = chrono_types::make_shared<ChBodyEasyBox>(5, 0.5, 5,  // x,y,z size
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    sys_0.AddBody(mrigidBodyc);
    mrigidBodyc->SetPos(ChVector3d(0, -1, 0));
    mrigidBodyc->SetFixed(true);
    mrigidBodyc->SetTag(unique_ID); unique_ID++;

    auto mrigidBodyd = chrono_types::make_shared<ChBodyEasySphere>(0.6,  // rad
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    sys_0.AddBody(mrigidBodyd);
    mrigidBodyd->SetPos(ChVector3d(-1.5, 2, 0));
    mrigidBodyd->SetPosDt(ChVector3d(20, 0, 0));
    mrigidBodyd->SetTag(unique_ID); unique_ID++;

    auto linkdistance = chrono_types::make_shared<ChLinkDistance>();
    sys_0.Add(linkdistance);
    linkdistance->Initialize(mrigidBody, mrigidBodyd, true, ChVector3d(0,1,0), ChVector3d(0,0,0));
    linkdistance->SetTag(unique_ID); unique_ID++;


    // HACK force update of AABB of bodies, already in StepDynamics() but here for first DoAllDomainPartitionUpdate()
    sys_0.GetCollisionSystem()->BindItem(mrigidBody); 
    sys_1.GetCollisionSystem()->BindItem(mrigidBodyb);
    sys_0.GetCollisionSystem()->BindItem(mrigidBodyc);

    // For debugging: open two 3D realtime view windows, each per domain:
    /*
    auto vis_irr_0 = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr_0->AttachSystem(&sys_0);
    vis_irr_0->SetWindowTitle("Domain 0");
    vis_irr_0->Initialize();
    vis_irr_0->AddSkyBox();
    vis_irr_0->AddCamera(ChVector3d(1, 2, 6), ChVector3d(0, 2, 0));
    vis_irr_0->AddTypicalLights();
    //vis_irr_0->BindAll();
    auto vis_irr_1 = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr_1->AttachSystem(&sys_1);
    vis_irr_1->SetWindowTitle("Domain 1");
    vis_irr_1->Initialize();
    vis_irr_1->AddSkyBox();
    vis_irr_1->AddCamera(ChVector3d(1, 2, 6), ChVector3d(0, 2, 0));
    vis_irr_1->AddTypicalLights();
    //vis_irr_1->BindAll();
    */
    system("pause");

    for (int i = 0; i < 25; ++i) {
        std::cout << "\n\n\n============= Time step " << i << std::endl << std::endl;
       
        // For debugging: open two 3D realtime view windows, each per domain:
        /*
        vis_irr_0->RemoveAllIrrNodes();
        vis_irr_0->BindAll();
        vis_irr_0->Run();
        vis_irr_0->BeginScene();
        vis_irr_0->Render();
        vis_irr_0->EndScene();
        vis_irr_0->RemoveAllIrrNodes();
        vis_irr_1->BindAll();
        vis_irr_1->Run();
        vis_irr_1->BeginScene();
        vis_irr_1->Render();
        vis_irr_1->EndScene();
        */
        // MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
        domain_manager.DoAllDomainPartitionUpdate();
        //system("pause");
        // MULTIDOMAIN TIME INTEGRATION
        domain_manager.DoAllStepDynamics(0.01);

        system("pause");
    }
   
   




    return 0;
}
