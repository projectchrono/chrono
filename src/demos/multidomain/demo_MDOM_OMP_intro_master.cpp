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
//  This demo shows basic functionality of the module: a body traveling through 
//  two domains with a sphere connected to its top via a link, and hitting another 
//  domain.
// 
//  We add all objects into a "master domain" that wraps all the scene, then we
//  let that  DoAllDomainInitialize() will split all the items automatically into
//  the corresponding domains. (This is easy and almost error-proof, but if you
//  need to avoid a master domain for efficiency/space reasons, look at the
//  alternative model creation mode in ...lowlevel.cpp demos.)
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveUtils.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_multidomain/ChDomainManagerSharedmemory.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace multidomain;
using namespace chrono::irrlicht;
using namespace chrono::fea;



int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // This source generates a simple executable with basic std::cout output on the command
    // line console, enough to show basic functionality of the multidomain module.
    // 
    // All the examples herein are based on the ChDomainManagerSharedmemory  domain manager, 
    // that is, no need for MPI because the domains are split in the single executable using
    // OpenMP multithreading. This OpenMP multithreading is enough for didactical reasons,
    // then more advanced stuff must run usnig the MPI domain manager, shown in other demos.


    // 1- First you need a ChDomainManagerSharedmemory. This will use OpenMP multithreading
    //    as a method for parallelism (spreading processes on multiple computing nodes)

    ChDomainManagerSharedmemory domain_manager;

    // For debugging/logging:
    domain_manager.verbose_partition = true; // will print partitioning in std::cout ?
    domain_manager.verbose_serialization = true; // will print serialization buffers in std::cout ?
    domain_manager.verbose_variable_updates = false; // will print all messages in std::cout ?
    domain_manager.serializer_type = DomainSerializerFormat::JSON;  // default BINARY, use JSON or XML for readable verbose

    // 2- Now you need a domain builder.
    //    You must define how the 3D space is divided in domains. 
    //    ChdomainBuilder classes help you to do this. 
    //    Here we split it using parallel planes like in sliced bread.
    //    Since we use a helper master domain, [n.of threads] = [n.of slices] + 1

    ChDomainBuilderSlices       domain_builder(
                                        std::vector<double>{0},  // positions of cuts along axis to slice, ex {-1,0,2} generates 5 domains
                                        ChAxis::X,  // axis about whom one needs the space slicing
                                        true);      // build also master domain, interfacing to all slices, for initial injection of objects
    

    // 3- Create the ChDomain objects and their distinct ChSystem physical systems.
    //    Now one can know how many domains are expected to build, using domain_builder.GetTotSlices();
    
    std::vector<ChSystemNSC*> sys_slices(domain_builder.GetTotSlices());

    for (int i = 0; i < domain_builder.GetTotSlices(); ++i) {
        sys_slices[i] = new ChSystemNSC;
        sys_slices[i]->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

        domain_manager.AddDomain(domain_builder.BuildDomain(
            sys_slices[i], // physical system of this domain
            i       // rank of this domain, must be unique and starting from 0! 
        ));

        // Set solver, timestepper, etc. that can work in multidomain mode. Do this after AddDomain(). 
        // (The solver has been defaulted to ChSolverPSORmultidomain when we did domain_manager.AddDomain().)
        sys_slices[i]->GetSolver()->AsIterative()->SetMaxIterations(12);
        sys_slices[i]->GetSolver()->AsIterative()->SetTolerance(1e-6);
    }



    // 4- Create and populate the MASTER domain with bodies, links, meshes, nodes, etc. 
    //    At the beginning of the simulation, the master domain will break into 
    //    multiple data structures and will serialize them into the proper subdomains.
    //    (Note that there is also a "low level" version of this demo that shows how
    //    to bypass the use of the master domain, in case of extremely large systems
    //    where you might want to create objects directly split in the n-th computing node.)

    // First we create the master domain and add it to the domain manager:
    ChSystemNSC sys_master;
    sys_master.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    domain_manager.AddDomain(domain_builder.BuildMasterDomain(
        &sys_master // physical system of the master domain (its rank automatically set to last domain+1)
    ));
    
    sys_master.GetSolver()->AsIterative()->SetMaxIterations(12);
    sys_master.GetSolver()->AsIterative()->SetTolerance(1e-6);


    // Ok, now we proceed as usual in Chrono, adding items into the system :-)

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.1);
 
    auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 2,  // x,y,z size
        100,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    sys_master.AddBody(mrigidBody);
    mrigidBody->SetPos(ChVector3d(-1.5,0,0));
    mrigidBody->SetPosDt(ChVector3d(20, 0, 0));

    // A very important thing: for multidomain, each item (body, mesh, link, node, FEA element)
    // must have an unique tag! This SetTag() is needed because items might be shared between neighbouring domains. 
    // One way to do this: for each created item, do    
    //    mrigidBody->SetTag(unique_ID); unique_ID++; 
    // However an easier alternative to using SetTag() one by one is that at the end you use the helper 
    // ChArchiveSetUniqueTags (see snippet later)

    auto mrigidBodyb = chrono_types::make_shared<ChBodyEasyBox>(0.7, 0.7, 0.7,  // x,y,z size
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    sys_master.AddBody(mrigidBodyb);
    mrigidBodyb->SetPos(ChVector3d(1.5, 0, 0));
    mrigidBodyb->SetFixed(true);

    auto mrigidBodyc = chrono_types::make_shared<ChBodyEasyBox>(5, 0.5, 5,  // x,y,z size
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    sys_master.AddBody(mrigidBodyc);
    mrigidBodyc->SetPos(ChVector3d(0, -1.15, 0));
    mrigidBodyc->SetFixed(true);
    
    auto mrigidBodyd = chrono_types::make_shared<ChBodyEasySphere>(0.6,  // rad
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat);        // contact material
    sys_master.AddBody(mrigidBodyd);
    mrigidBodyd->SetPos(ChVector3d(-1.5, 2, 0));
    mrigidBodyd->SetPosDt(ChVector3d(20, 0, 0));

    auto linkdistance = chrono_types::make_shared<ChLinkDistance>();
    sys_master.Add(linkdistance);
    linkdistance->Initialize(mrigidBody, mrigidBodyd, true, ChVector3d(0,1,0), ChVector3d(0,0,0));



    // 5- Set the tag IDs for all nodes, bodies, etc.
    //    To do this, use the helper ChArchiveSetUniqueTags, that traverses all the 
    //    hierarchies, sees if there is a SetTag() function in sub objects, and sets 
    //    the ID incrementally. More hassle-free than setting all IDs one by one by 
    //    hand as in ...lowlevel.cpp demos
    //    Call this after you finished adding items to systems.
    ChArchiveSetUniqueTags tagger;
    tagger.skip_already_tagged = false;
    tagger << CHNVP(sys_master);


    // For debugging: open two 3D realtime view windows, each per domain:
    
    auto vis_irr_0 = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr_0->AttachSystem(sys_slices[0]);
    vis_irr_0->SetWindowTitle("Domain 0");
    vis_irr_0->Initialize();
    vis_irr_0->AddSkyBox();
    vis_irr_0->AddCamera(ChVector3d(1, 2, 6), ChVector3d(0, 2, 0));
    vis_irr_0->AddTypicalLights();
    //vis_irr_0->BindAll();
    auto vis_irr_1 = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr_1->AttachSystem(sys_slices[1]);
    vis_irr_1->SetWindowTitle("Domain 1");
    vis_irr_1->Initialize();
    vis_irr_1->AddSkyBox();
    vis_irr_1->AddCamera(ChVector3d(1, 2, 6), ChVector3d(0, 2, 0));
    vis_irr_1->AddTypicalLights();
    //vis_irr_1->BindAll();
    
    system("pause");

    // 6 - INITIAL SETUP AND OBJECT INITIAL MIGRATION!
    //     Moves all the objects in master domain to all domains, slicing the system.
    //     Also does some initializations, like collision detection AABBs.
    domain_manager.DoAllDomainInitialize();

    // The master domain does not need to communicate anymore with the domains so do:
    domain_manager.master_domain_enabled = false;

    for (int i = 0; i < 25; ++i) {
        std::cout << "\n\n\n============= Time step " << i << std::endl << std::endl;
       
        // For debugging: open two 3D realtime view windows, each per domain:
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
        
        // MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
        domain_manager.DoAllDomainPartitionUpdate();

        // MULTIDOMAIN TIME INTEGRATION
        domain_manager.DoAllStepDynamics(0.01);

        system("pause");
    }
   

    for (auto sys_i : sys_slices) {
        delete sys_i;
    }


    return 0;
}
