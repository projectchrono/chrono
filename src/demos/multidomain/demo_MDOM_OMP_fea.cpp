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
//  domain. A FEA element is added too.
// 
//  Because of the presence of FEA, the timestepper is switched to explicit 
//  integration with diagonal lumped mass solver, ChSolverLumpedMultidomain 
//  (the easiest scenario for a distributed memory solver).
// 
//  This also means that this solver is not capable of solving nonsmooth DVI, 
//  so here  contacts if any are handled using smooth contact, via  ChSystemSMC
//  and ChContactMaterialSMC.
// 
//  It shows "low level" initializiation: in fact here we do not use a master 
//  domain that takes care of splitting the system at the beginning: it is up to 
//  the user to create objects in the corresponding system in already split way.
//  This low-level initialization can be more dangerous if one does not know
//  the rules about how to split/share bodies,nodes,elements etc., but the positive
//  aspect is that for extremely large systems one does not need to create a huge
//  master system at the beginning, that could not fit into a single node memory.
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
#include "chrono_multidomain/ChSolverLumpedMultidomain.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace multidomain;
using namespace chrono::irrlicht;
using namespace chrono::fea;

// For multi domain simulations, each item (body, link, fea element or node, etc.) must have
// an unique ID, to be set via SetTag(). Here we use a static counter to help with the generation
// of unique IDs, starting from 3 because 1,2 are reserved for all ChSystem and ChAssembly in all domains.
static int unique_ID = 3;




int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // This source generates a simple executable with basic std::cout output on the command
    // line console, enough to show basic functionality of the multidomain module.
    // 
    // All the examples herein are based on the ChDomainManagerSharedmemory  domain manager, 
    // that is, no need for MPI because the domains are split in the single executable using
    // OpenMP multithreading. This OpenMP multithreading is enough for didactical reasons,
    // then more advanced stuff must run usnig the MPI domain manager, shown in other demos.


    // 1- first you need a domain manager. This will use the OpenMP multithreading 
    // as a method for parallelism:

    ChDomainManagerSharedmemory domain_manager;

    // For debugging/logging:
    domain_manager.verbose_partition = true; // will print partitioning in std::cout ?
    domain_manager.verbose_serialization = true; // will print serialization buffers in std::cout ?
    domain_manager.verbose_variable_updates = false; // will print all messages in std::cout ?
    domain_manager.serializer_type = DomainSerializerFormat::JSON; // default BINARY, use JSON or XML for readable verbose

    // 2- the domain builder.
    // You must define how the 3D space is divided in domains. 
    // ChdomainBuilder classes help you to do this. We do NOT need the master domain.
    // Here we split it using parallel planes like in sliced bread:

    ChDomainBuilderSlices       domain_builder(
                                        std::vector<double>{0},  // positions of cuts along axis to slice, ex {-1,0,2} generates 5 domains
                                        ChAxis::X,  // axis about whom one needs the space slicing
                                        false);     // false: we do not create a master domain for initial injection  
    

    // 3- create the ChDomain objects and their distinct ChSystem physical systems.
    // Now one can know how many domains are expected to build, using domain_builder.GetTotRanks();
    // But in this case we already know we split into 2 domains. So we build them as:

    ChSystemSMC sys_0;
    sys_0.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    domain_manager.AddDomain(domain_builder.BuildDomain(
                                        &sys_0, // physical system of this domain
                                        0       // rank of this domain 
                                       ));
    // Set the time stepper: we have FEA, use an explicit time stepper. 
    auto explicit_timestepper0 = chrono_types::make_shared<ChTimestepperEulerExplIIorder>(&sys_0);
    explicit_timestepper0->SetConstraintsAsPenaltyON(2e6); // use penalty for constraints, skip linear systems completely
    sys_0.SetTimestepper(explicit_timestepper0);
    // Set the solver: efficient ChSolverLumpedMultidomain (needs explicit timestepper with constraint penalty)
    auto lumped_solver0 = chrono_types::make_shared<ChSolverLumpedMultidomain>();
    sys_0.SetSolver(lumped_solver0);

    ChSystemSMC sys_1;
    sys_1.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    domain_manager.AddDomain(domain_builder.BuildDomain(
                                        &sys_1, // physical system of this domain
                                        1       // rank of this domain 
                                       ));
    // Set the time stepper: we have FEA, use an explicit time stepper.
    auto explicit_timestepper1 = chrono_types::make_shared<ChTimestepperEulerExplIIorder>(&sys_1);
    explicit_timestepper1->SetConstraintsAsPenaltyON(2e6); // use penalty for constraints, skip linear systems completely
    sys_1.SetTimestepper(explicit_timestepper1);
    // Set the solver: efficient ChSolverLumpedMultidomain (needs explicit timestepper with constraint penalty)
    auto lumped_solver1 = chrono_types::make_shared<ChSolverLumpedMultidomain>();
    sys_1.SetSolver(lumped_solver1);

    // 4- we populate the n domains with bodies, links, meshes, nodes, etc. 
    // Each item must be added to the ChSystem of the domain that the item overlaps with. 
    // The rules are:
    // - Each "vertex" item (ChBody, ChNode stuff) must be added to the ChSystem of the domain that 
    //   its 3d position is into. If its AABB is overlapping with multiple domains, do not
    //   worry: the first time DoAllDomainInitialize() is called, it will spread copies to surrounding domains.
    // - Each "edge" item (ChLink constraints, ChElement finite elements) must be added to the 
    //   domain where the reference point (1st node of fea element, master body in links) is into.
    //   Note edges are always into a single domain, and never shared among domains.
    // - IMPORTANT: if an edge is in the domain and references a vertex that is not overlapping with domain, 
    //   then such vertex(s) must be added too to the domain anyway, becoming shared with surrounding domain(s).
    //   For this reason when you use SetTag() you MUST use the same ID of those shared vertex across domains,
    //   otherwise DoAllDomainInitialize() won't recognize this fact and will copy them n times as disconnected.

    // Create two collision materials, one for each domain. 
    // Note that they have the same tag ID so that they are not transported through domain interfaces, and just referenced.
    auto mat0 = chrono_types::make_shared<ChContactMaterialSMC>();
    mat0->SetFriction(0.1);
    mat0->SetYoungModulus(2e7);
    mat0->SetTag(unique_ID);
    auto mat1 = chrono_types::make_shared<ChContactMaterialSMC>();
    mat1->SetFriction(0.1);
    mat1->SetYoungModulus(2e7);
    mat1->SetTag(unique_ID); unique_ID++;

    // A moving box, initially contained in domain 0 
    auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 2,  // x,y,z size
        100,         // density
        true,        // visualization?
        true,        // collision?
        mat0);       // contact material
    sys_0.AddBody(mrigidBody);              // note "sys_0", cause it starts in domain 0
    mrigidBody->SetPos(ChVector3d(-1.5,0,0));
    mrigidBody->SetPosDt(ChVector3d(20, 0, 0));
    // 5- a very important thing: for multidomain, each item (body, mesh, link, node, FEA element)
    // must have an unique tag! This SetTag() is needed because items might be shared between neighbouring domains. 
    // Note: an alternative to using SetTag() one by one, at the end you can use the helper 
    // ChArchiveSetUniqueTags (see snippet later)
    mrigidBody->SetTag(unique_ID); unique_ID++; 

    // A box, initially contained in domain 1 
    auto mrigidBodyb = chrono_types::make_shared<ChBodyEasyBox>(0.7, 0.7, 0.7,  // x,y,z size
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat1);       // contact material
    sys_1.AddBody(mrigidBodyb);             // note "sys_1", cause it starts in domain 1
    mrigidBodyb->SetPos(ChVector3d(1.5, 0, 0));
    mrigidBodyb->SetFixed(true);
    mrigidBodyb->SetTag(unique_ID); unique_ID++;

    // A floor
    // Note, the floor starts overlapped betwen domains 0 and 1, but here we add only to domain 0
    // because for the first step the DoAllDomainInitialize()  would copy the floor from 0 to 1 anyway
    auto mrigidBody_floor_0 = chrono_types::make_shared<ChBodyEasyBox>(5, 0.5, 5,  // x,y,z size
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat0);       // contact material
    sys_0.AddBody(mrigidBody_floor_0);
    mrigidBody_floor_0->SetPos(ChVector3d(0, -1.15, 0));
    mrigidBody_floor_0->SetFixed(true);
    mrigidBody_floor_0->SetTag(unique_ID);  unique_ID++; 
    
    // A small sphere to connect via a constraint
    auto mrigidBodyd = chrono_types::make_shared<ChBodyEasySphere>(0.6,  // rad
        400,         // density
        true,        // visualization?
        true,        // collision?
        mat0);       // contact material
    sys_0.AddBody(mrigidBodyd);
    mrigidBodyd->SetPos(ChVector3d(-1.5, 2, 0));
    mrigidBodyd->SetPosDt(ChVector3d(20, 0, 0));
    mrigidBodyd->SetTag(unique_ID); unique_ID++;

    // A constraint. Add to domain 0 because the link reference is mrigidBodyd, now in domain 0
    auto linkdistance = chrono_types::make_shared<ChLinkDistance>();
    sys_0.Add(linkdistance);
    linkdistance->Initialize(mrigidBody, mrigidBodyd, true, ChVector3d(0,1,0), ChVector3d(0,0,0));
    linkdistance->SetTag(unique_ID); unique_ID++;

    // Add a FEA beam to test migration of meshes too
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    sys_0.Add(my_mesh);
    my_mesh->SetTag(unique_ID); unique_ID++;

    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    double beam_wy = 0.012;
    double beam_wz = 0.025;
    msection->SetAsRectangularSection(beam_wy, beam_wz);
    msection->SetYoungModulus(0.01e9);
    msection->SetShearModulus(0.01e9 * 0.3);
    
    auto hnode1 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(-1.2, 1.3, 0)));
    hnode1->SetMass(0.1); hnode1->GetInertia().fillDiagonal(0.1);
    hnode1->SetPosDt(ChVector3d(10, 0, 0));
    hnode1->SetTag(unique_ID); unique_ID++;
    my_mesh->AddNode(hnode1);
    auto hnode2 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d( -0.2, 1.3, 0)));
    hnode2->SetMass(0.1); hnode2->GetInertia().fillDiagonal(0.1);
    hnode2->SetPosDt(ChVector3d(10, 0, 0));
    hnode2->SetTag(unique_ID); unique_ID++;
    my_mesh->AddNode(hnode2);
    auto belement1 = chrono_types::make_shared<ChElementBeamEuler>();
    belement1->SetNodes(hnode1, hnode2);
    belement1->SetSection(msection);
    belement1->SetRestLength(1.0);
    my_mesh->AddElement(belement1);
    my_mesh->SetAutomaticGravity(false);

    // Alternative of manually setting SetTag() for all nodes, bodies, etc., is to use a
    // helper ChArchiveSetUniqueTags, that traverses all the hierarchies, sees if there is 
    // a SetTag function in sub objects, ans sets the ID incrementally. More hassle-free, but
    // at the cost that you are not setting the tag values as you like, ex for postprocessing needs.
    // Call this after you finished adding items to systems.
    //   ChArchiveSetUniqueTags tagger;
    //   tagger.skip_already_tagged = false;
    //   tagger << CHNVP(sys_0);
    //   tagger << CHNVP(sys_1);


    // For debugging: open two 3D realtime view windows, each per domain:
    
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
    
    system("pause");

    // INITIAL SETUP AND OBJECT INITIAL MIGRATION!
    // Moves all the objects in master domain to all domains, slicing the system.
    // Also does some initializations, like collision detection AABBs.
    domain_manager.DoAllDomainInitialize();

    // The master domain does not need to communicate anymore with the domains so do:
    domain_manager.master_domain_enabled = false;

    for (int i = 0; i < 200; ++i) {
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
        domain_manager.DoAllStepDynamics(0.002);

        system("pause");
    }
   
   




    return 0;
}