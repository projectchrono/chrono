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
//  using distributed memory. This needs MPI, as many executables will be spawn
//  on different computing nodes. 
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include "chrono_multidomain/ChDomainManagerMPI.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"

using namespace chrono;
using namespace multidomain;

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
    //   mpiexec -n 2 demo_MDOM_intro_distributedMPI.exe  


    // EXAMPLE A
    //
    // Basic functionality of the multidomain module: a body traveling through two domains.

    // 1- first you need a domain manager. This will use MPI distributed computing
    // as a method for parallelism (spreading processes on multiple computing nodes)

    ChDomainManagerMPI domain_manager(&argc,&argv);

    // For debugging/logging:
    domain_manager.verbose_partition = false; // will print  partitioning in std::cout?
    domain_manager.verbose_serialization = true; // will print serialization buffers in std::cout?
    domain_manager.verbose_variable_updates = true; // will print all messages in std::cout?

    // 2- the domain builder.
    // You must define how the 3D space is divided in domains. 
    // ChdomainBuilder classes help you to do this. 
    // Here we split it using parallel planes like in sliced bread:

    ChDomainBuilderSlices       domain_builder(
                                        std::vector<double>{0},  // positions of cuts along axis to slice, ex {-1,0,2} generates 5 domains
                                        ChAxis::X);     // axis about whom one needs the space slicing
    

    // 3- create the ChDomain object and its ChSystem physical system.
    // Only a single system is created, because this is already one of n 
    // parallel processes.

    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.GetSolver()->AsIterative()->SetMaxIterations(5);

    domain_manager.SetDomain(domain_builder.BuildDomain(
                                        &sys, // physical system of this domain
                                        domain_manager.GetMPIrank()     // rank of this domain 
                                       ));

 

    // 4- we populate the n domains with bodies, links, meshes, nodes, etc. 
    // Each item must be added to the ChSystem of the domain that the item overlaps with. 

    // In this example, the body initially is in the first domain, rank=0, not in the second, rank=1.
    // So check the rank of this process and see which item to create. 
    if (domain_manager.GetMPIrank() == 0) {
        
        auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
        mat->SetFriction(0.1);

        auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 2,  // x,y,z size
            100,         // density
            true,        // visualization?
            true,        // collision?
            mat);        // contact material
        sys.AddBody(mrigidBody);
        mrigidBody->SetPos(ChVector3d(-1.5, 0, 0));
        mrigidBody->SetPosDt(ChVector3d(20, 0, 0));
        // 5- a very important thing: for multidomain, each item (body, mesh, link, node, FEA element)
        // must have an unique tag! This SetTag() is needed because items might be shared between neighbouring domains. 
        mrigidBody->SetTag(unique_ID); unique_ID++;
    }

    // INITIAL SETUP OF COLLISION AABBs 
    domain_manager.DoDomainInitialize(domain_manager.GetMPIrank());

    for (int i = 0; i < 12; ++i) {

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

    }
   
   




    return 0;
}
