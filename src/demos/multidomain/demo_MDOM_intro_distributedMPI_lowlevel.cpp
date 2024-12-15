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
// Demo code about using the MULTIDOMAIN module on MPI.
// 
// This demo shows basic functionality of the module: a body traveling through 
// two domains with a sphere connected to its top via a link, and hitting another 
// domain. 
//     
// It shows "low level" initializiation: in fact here we do not use a master 
// domain that takes care of splitting the system at the beginning: it is up to 
// the user to create objects in the corresponding system in already split way.
// This low-level initialization can be more dangerous if one does not know
// the rules about how to split/share bodies,nodes,elements etc., but the positive
// aspect is that for extremely large systems one does not need to create a huge
// master system at the beginning, that could not fit into a single node memory.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include "chrono_multidomain/ChDomainManagerMPI.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"
#include "chrono_postprocess/ChBlender.h"

using namespace chrono;
using namespace multidomain;
using namespace postprocess;

// For multi domain simulations, each item (body, link, fea element or node, etc.) must have
// an unique ID, to be set via SetTag(). Here we use a static counter to help with the generation
// of unique IDs.
static int unique_ID = 1;



int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // This source generates a simple executable with basic std::cout output on the command
    // line console, enough to show basic functionality of the multidomain module.
    // 
    // All the examples herein are based on the ChDomainManagerMPI domain manager, that
    // allows using MPI to communicate between processes distributed on a computer cluster
    // with many nodes connected by ethernet, or Mellanox, or similar supercomputing architectures.
    // 
    // NOTE! in MPI architectures, the executable cannot be started in a single copy as all other
    // chrono demos: now you must open a shell where you can access "mpiexec" (or other commands
    // depending on the MPI version that you installed, it could be "mpirun" or such) and type
    // the command to start parallel computation, for example spawning 2 processes:
    // 
    //   mpiexec -n 2 demo_MDOM_intro_distributedMPI_lowlevel.exe  



    // 1- first you need a domain manager. This will use MPI distributed computing
    // as a method for parallelism (spreading processes on multiple computing nodes)

    ChDomainManagerMPI domain_manager(&argc,&argv);

    // This demo assumes 2 domains, i.e. "mpiexec -n 2 ..." etc.
    assert(domain_manager.GetMPItotranks() == 2);

    // For debugging/logging:
    domain_manager.verbose_partition = true; // will print  partitioning in std::cout?
    domain_manager.verbose_serialization = false; // will print interdomain serialization in std::cout?
    domain_manager.verbose_variable_updates = false; // will print interdomain variable updates in std::cout?
    domain_manager.serializer_type = DomainSerializerFormat::JSON;  // default BINARY, use JSON or XML for readable verbose


    // 2- the domain builder.
    // You must define how the 3D space is divided in domains. 
    // ChdomainBuilder classes help you to do this. 
    // Here we split it using parallel planes like in sliced bread.
    

    ChDomainBuilderSlices       domain_builder(
                                        std::vector<double>{0},  // positions of cuts along axis to slice, ex {-1,0,2} generates 5 domains
                                        ChAxis::X);     // axis about whom one needs the space slicing
    

    // 3- create the ChDomain object and its ChSystem physical system.
    // Only a single system is created, because this is already one of n 
    // parallel processes.

    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    domain_manager.SetDomain(domain_builder.BuildDomain(
                                        &sys, // physical system of this domain
                                        domain_manager.GetMPIrank()     // rank of this domain 
                                       ));
    sys.GetSolver()->AsIterative()->SetMaxIterations(20);
    sys.GetSolver()->AsIterative()->SetTolerance(1e-6);
 

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
 
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.1);

    // domain slice up to  x<=0
    if (domain_manager.GetMPIrank() == 0) {
        
        auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 2,  // x,y,z size
            100,         // density
            true,        // visualization?
            true,        // collision?
            mat);        // contact material
        sys.AddBody(mrigidBody);
        mrigidBody->SetPos(ChVector3d(-1.5, 0, 0));
        mrigidBody->SetPosDt(ChVector3d(2, 0, 0));
        // A very important thing: for multidomain, each item (body, mesh, link, node, FEA element)
        // must have an unique tag! This SetTag() is needed because items might be shared between neighbouring domains. 
        mrigidBody->SetTag(unique_ID); unique_ID++;

        auto mrigidBodyx = chrono_types::make_shared<ChBodyEasyBox>(1.4, 0.2,1.4,  // x,y,z size
            100,         // density
            true,        // visualization?
            true,        // collision?
            mat);        // contact material
        sys.AddBody(mrigidBodyx);
        mrigidBodyx->SetPos(ChVector3d(-1.5, 1.1, 0));
        mrigidBodyx->SetPosDt(ChVector3d(2, 0, 0));
        mrigidBodyx->SetTag(unique_ID); unique_ID++;

        // A floor
        // Note, the floor starts overlapped betwen domains 0 and 1, but here we add only to domain 0
        // because for the first step the DoAllDomainInitialize()  would copy the floor from 0 to 1 anyway
        // making it shared. 
        auto mrigidBody_floor_1 = chrono_types::make_shared<ChBodyEasyBox>(5, 0.5, 5,  // x,y,z size
            400,         // density
            true,        // visualization?
            true,        // collision?
            mat);        // contact material
        sys.AddBody(mrigidBody_floor_1);
        mrigidBody_floor_1->SetPos(ChVector3d(0, -1.15, 0));
        mrigidBody_floor_1->SetFixed(true);
        mrigidBody_floor_1->SetTag(20000);

        // A small sphere to connect via linkdistance constraint
        auto mrigidBodyd = chrono_types::make_shared<ChBodyEasySphere>(0.6,  // rad
            400,         // density
            true,        // visualization?
            true,        // collision?
            mat);        // contact material
        sys.AddBody(mrigidBodyd);
        mrigidBodyd->SetPos(ChVector3d(-1.5, 2, 0));
        mrigidBodyd->SetPosDt(ChVector3d(2, 0, 0));
        mrigidBodyd->SetTag(unique_ID); unique_ID++;

        // A constraint. Add to domain 0 because the link reference is mrigidBodyd, now in domain 0
        auto linkdistance = chrono_types::make_shared<ChLinkDistance>();
        sys.Add(linkdistance);
        linkdistance->Initialize(mrigidBody, mrigidBodyd, true, ChVector3d(0, 1, 0), ChVector3d(0, 0, 0));
        linkdistance->SetTag(unique_ID); unique_ID++;
    }

    // domain slice from x>0 
    if (domain_manager.GetMPIrank() == 1) {
        
        unique_ID = 10000; // quick tip: id of all objects starting in domain n.1 start from 10000 offset
        
        auto mrigidBodyb = chrono_types::make_shared<ChBodyEasyBox>(0.7, 0.7, 0.7,  // x,y,z size
            400,         // density
            true,        // visualization?
            true,        // collision?
            mat);        // contact material
        sys.AddBody(mrigidBodyb); 
        mrigidBodyb->SetPos(ChVector3d(1.5, 0, 0));
        mrigidBodyb->SetFixed(true);
        mrigidBodyb->SetTag(unique_ID); unique_ID++;
    }


    // Ok, we completed the definition of the multibody system.



    // OPTIONAL: POSTPROCESSING VIA BLENDER3D
   
    // Create an exporter to Blender
    ChBlender blender_exporter = ChBlender(&sys);

    // Set the path where it will save all files, a directory in DEMO_OUTPUT/.. will be created if not existing. 
    // The directories will have a name depending on the rank: MDOM_MPI_0, MDOM_MPI_1, MDOM_MPI_2, etc.
    blender_exporter.SetBasePath(GetChronoOutputPath() + "MDOM_MPI_" + std::to_string(domain_manager.GetMPIrank()));

    // Initial script, save once at the beginning
    blender_exporter.ExportScript();


    // INITIAL SETUP OF COLLISION AABBs 
    domain_manager.DoDomainInitialize(domain_manager.GetMPIrank());

    for (int i = 0; i < 150; ++i) {

        if (domain_manager.GetMPIrank()==0) 
            std::cout << "\n\n\n============= Time step " << i << std::endl << std::endl;

        // MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
        domain_manager.DoDomainPartitionUpdate(domain_manager.GetMPIrank());

        // MULTIDOMAIN TIME INTEGRATION
        // Just call the regular DoStepDynamics() as always done in Chrono.
        // When a regular DoStepDynamics is called here, it will execute some solver. The solver 
        // has been defaulted to ChSolverPSORmultidomain when we did domain_manager.SetDomain(),
        // and this type of solver is aware that MPI intercommunication must be done while doing
        // its iterations.
        sys.DoStepDynamics(0.01);

        // OPTIONAL POSTPROCESSING FOR 3D RENDERING
        // Before ExportData(), we must do a remove-add trick as an easy way to handle the fact
        // that objects are constantly added and removed from the i-th domain when crossing boundaries.
        blender_exporter.RemoveAll();  
        blender_exporter.AddAll();
        blender_exporter.ExportData();

    }
   
   




    return 0;
}
