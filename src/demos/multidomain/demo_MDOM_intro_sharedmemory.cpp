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
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include "chrono_multidomain/ChDomainManager.h"
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
    domain_manager.verbose_partition = true; // will print  partitioning in std::cout

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

    // 4- we populate the n domains with bodies, links, meshes, nodes, etc. 
    // Each item must be added to the ChSystem of the domain that the item overlaps with. 

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
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
    

    sys_0.GetCollisionSystem()->BindItem(mrigidBody); // HACK force update of AABB of bodies, already in StepDynamics() but here for first DoAllDomainPartitionUpdate()



    for (int i = 0; i < 12; ++i) {
        std::cout << "============= Time step " << i << std::endl << std::endl;

        // MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
        domain_manager.DoAllDomainPartitionUpdate();

        // MULTIDOMAIN TIME INTEGRATION
        domain_manager.DoAllStepDynamics(0.01);

        system("pause");
    }
   
   




    return 0;
}
