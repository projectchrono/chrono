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
// Demo code about splitting a system into domains using the MULTIDOMAIN module
// using distributed memory. This needs MPI, as many executables will be spawn
// on different computing nodes. 
//
// This demo shows basic functionality for FEA. Some finite elements are 
// used here.
// 
// Because of the presence of FEA, the default PSOR solver for non-smooth 
// dynamics cannot be used. The timestepper is switched to explicit 
// integration with diagonal lumped mass solver, ChSolverLumpedMultidomain 
// 
// We add all objects into a "master domain" that wraps all the scene, then we
// let that  DoAllDomainInitialize() will split all the items automatically into
// the corresponding domains. (This is easy and almost error-proof, but if you
// need to avoid a master domain for efficiency/space reasons, look at the
// alternative model creation mode in ...lowlevel.cpp demos.)
// 
// NOTE! in MPI architectures, the executable cannot be started in a single copy as all other
// chrono demos: now you must open a shell where you can access "mpiexec" (or other commands
// depending on the MPI version that you installed, it could be "mpirun" or such) and type
// the command to start parallel computation, for example spawning 3 processes:
// 
//   mpiexec -n 3 demo_MDOM_MPI_fea.exe  
//
// This demo can use -n X processes, with X=(ndomains+1master), ex -n 5 builds 4 domain slices, plus
// one process that is dedicated to the master domain. Ex:
//   mpiexec -n 2 demo_MDOM_MPI_fea.exe   -> reference case of 1 domain
//   mpiexec -n 3 demo_MDOM_MPI_fea.exe   -> simplest multidomain: 2 domains
// 
// After running, it produces postprocessing files ready for rendering in Blender3D
// To see the result in Blender3D, you must install the add-in for loading Chrono files,
// that is  chrono\src\importer_blender\chrono_import.py. So you can do:
// - menu  file/Import/Chrono import... 
// - check the "Automatic merge" in the top right panel
// - browse to  chrono\bin\Release\DEMO_OUTPUT\MDOM_MPI_0  and press the 
//   "Import Chrono simulation" button.
// Thanks to the automatic merge, also outputs in ..\MDOM_MPI_2 ..\MDOM_MPI_3 etc
// will be load automatically; the system will recognize the incremental numbering. 
// (Alternatively you can load the single domain results one by one, turning on the "Merge" 
// option while you call the  Chrono import...  menu multiple times.)
// 
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveUtils.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/fea/ChElementShellBST.h"

#include "chrono_multidomain/ChSolverLumpedMultidomain.h"
#include "chrono_multidomain/ChDomainManagerMPI.h"
#include "chrono_multidomain/ChDomainBuilder.h"

#include "chrono_postprocess/ChBlender.h"
#include <Windows.h>

using namespace chrono;
using namespace multidomain;
using namespace postprocess;
using namespace fea;


int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // This source generates a simple executable with basic std::cout output on the command
    // line console, enough to show basic functionality of the multidomain module.
    // 
    // All the examples herein are based on the ChDomainManagerMPI domain manager, that
    // allows using MPI to communicate between processes distributed on a computer cluster
    // with many nodes connected by ethernet, or Mellanox, or similar supercomputing architectures.


    // 1- First you need a ChDomainManagerMPI. This will use MPI distributed computing
    //    as a method for parallelism (spreading processes on multiple computing nodes)

    ChDomainManagerMPI domain_manager(&argc,&argv);

    // This demo assumes 1..7 processes (ex. 3 processes =2 slices + 1 master domain, i.e. "mpiexec -n 3 ..." etc.)
    assert(domain_manager.GetMPItotranks() >= 1);
    assert(domain_manager.GetMPItotranks() < 8);

    // For debugging/logging:
    domain_manager.verbose_partition = false; // will print  partitioning in std::cout?
    domain_manager.verbose_serialization = false; // will print interdomain serialization in std::cout?
    domain_manager.verbose_variable_updates = false; // will print interdomain variable updates in std::cout?
    domain_manager.serializer_type = DomainSerializerFormat::BINARY;  // default BINARY, use JSON or XML for readable verbose

    // 2- Now you need a domain builder.
    //    You must define how the 3D space is divided in domains. 
    //    ChdomainBuilder classes help you to do this. 
    //    Since we use a helper master domain, [n.of MPI ranks] = [n.of slices] + 1
    //    Here we split it using parallel planes like in sliced bread:

    ChDomainBuilderSlices       domain_builder(
                                        domain_manager.GetMPItotranks()-1,  // number of slices
                                        //-2e307, 2e307,      // min and max along axis to split in slices
                                        -30,30,
                                        ChAxis::X,          // axis about whom one needs the space slicing
                                        true);              // build also master domain, interfacing to all slices, for initial injection of objects   
    

    // 3- Create the ChDomain object and its ChSystem physical system.
    //    Only a single system is created, because this is already one of n 
    //    parallel processes. One must be the master domain.

    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    if (domain_manager.GetMPIrank() == domain_builder.GetMasterRank()) {
        domain_manager.SetDomain(domain_builder.BuildMasterDomain(
            &sys // physical system of this domain
        ));
    }
    else {
        domain_manager.SetDomain(domain_builder.BuildDomain(
            &sys,  // physical system of this domain
            domain_manager.GetMPIrank()  // rank of this domain, must be unique and starting from 0 
        ));
    }

    // 4- Set solver, timestepper, etc. that can work in multidomain mode. Do this after SetDomain(). 
    //    Set the time stepper: we have FEA, use an explicit time stepper. 
    auto explicit_timestepper = chrono_types::make_shared<ChTimestepperEulerExplIIorder>(&sys);
    explicit_timestepper->SetConstraintsAsPenaltyON(2e6); // use penalty for constraints, skip linear systems completely
    sys.SetTimestepper(explicit_timestepper);
    //    Set the solver: efficient ChSolverLumpedMultidomain (needs explicit timestepper with constraint penalty)
    auto lumped_solver = chrono_types::make_shared<ChSolverLumpedMultidomain>();
    sys.SetSolver(lumped_solver);
 

    // 5- Now populate ONLY THE MASTER domain with bodies, links, meshes, nodes, etc. 
    //    At the beginning of the simulation, the master domain will break into 
    //    multiple data structures and will serialize them into the proper subdomains.
    //    (Note that there is a "low level" version of this demo that shows how
    //    to bypass the use of the master domain, in case of extremely large systems
    //    where you might want to create objects directly split in the n-th computing node.)


    if (domain_manager.GetMPIrank() == domain_builder.GetMasterRank()) {
        
        // Create a mesh, that is a container for groups
        // of elements and their referenced nodes.
        auto mesh = chrono_types::make_shared<ChMesh>();
        sys.Add(mesh);


        // Create a material
        double density = 100;
        double E = 6e4;
        double nu = 0.0;
        double thickness = 0.01;

        auto elasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
        auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(elasticity);
        material->SetDensity(density);

        double corner_x = -0.5;
        double corner_z = -0.5;
        double size_x = 1;
        double size_z = 1;
        size_t nsections_x = 10;
        size_t nsections_z = 10;
        size_t fixed_x = 3;
        size_t fixed_z = 3;

        // Create nodes
        std::vector<std::shared_ptr<ChNodeFEAxyz>> nodes;  // for future loop when adding elements
        for (size_t iz = 0; iz <= nsections_z; ++iz) {
            for (size_t ix = 0; ix <= nsections_x; ++ix) {
                ChVector3d p(corner_x + ix * (size_x / nsections_x), 0, corner_z + iz * (size_z / nsections_z));

                auto node = chrono_types::make_shared<ChNodeFEAxyz>(p);

                mesh->AddNode(node);

                nodes.push_back(node);
            }
        }
        // Create elements
        for (size_t iz = 0; iz < nsections_z; ++iz) {
            for (size_t ix = 0; ix < nsections_x; ++ix) {
                auto elementA = chrono_types::make_shared<ChElementShellBST>();
                mesh->AddElement(elementA);

                std::shared_ptr<ChNodeFEAxyz> boundary_1;
                std::shared_ptr<ChNodeFEAxyz> boundary_2;
                std::shared_ptr<ChNodeFEAxyz> boundary_3;

                boundary_1 = nodes[(iz + 1) * (nsections_x + 1) + ix + 1];
                if (ix > 0)
                    boundary_2 = nodes[(iz + 1) * (nsections_x + 1) + ix - 1];
                else
                    boundary_2 = nullptr;
                if (iz > 0)
                    boundary_3 = nodes[(iz - 1) * (nsections_x + 1) + ix + 1];
                else
                    boundary_3 = nullptr;

                elementA->SetNodes(nodes[(iz) * (nsections_x + 1) + ix], nodes[(iz) * (nsections_x + 1) + ix + 1],
                    nodes[(iz + 1) * (nsections_x + 1) + ix], boundary_1, boundary_2, boundary_3);

                elementA->AddLayer(thickness, 0 * CH_DEG_TO_RAD, material);

                auto melementB = chrono_types::make_shared<ChElementShellBST>();
                mesh->AddElement(melementB);

                boundary_1 = nodes[(iz) * (nsections_x + 1) + ix];
                if (ix < nsections_x - 1)
                    boundary_2 = nodes[(iz) * (nsections_x + 1) + ix + 2];
                else
                    boundary_2 = nullptr;
                if (iz < nsections_z - 1)
                    boundary_3 = nodes[(iz + 2) * (nsections_x + 1) + ix];
                else
                    boundary_3 = nullptr;

                melementB->SetNodes(nodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                    nodes[(iz + 1) * (nsections_x + 1) + ix], nodes[(iz) * (nsections_x + 1) + ix + 1],
                    boundary_1, boundary_2, boundary_3);

                melementB->AddLayer(thickness, 0 * CH_DEG_TO_RAD, material);
            }
        }

        // fix some nodes
        for (int j = 0; j < fixed_x; ++j) {
            for (int k = 0; k < fixed_z; ++k) {
                nodes[j * (nsections_x + 1) + k]->SetFixed(true);
            }
        }


        // Visualization of the FEM mesh. Also this object will migrate from master domain to sliced domains.
        auto vis_shell_mesh = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
        vis_shell_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        vis_shell_mesh->SetShellResolution(2);
        vis_shell_mesh->SetBackfaceCull(true);
        mesh->AddVisualShapeFEA(vis_shell_mesh);

        // Visualization of the FEM nodes. Also this object will migrate from master domain to sliced domains.
        auto vis_shell_nodes = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
        vis_shell_nodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        vis_shell_nodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        vis_shell_nodes->SetSymbolsThickness(0.03);
        mesh->AddVisualShapeFEA(vis_shell_nodes);

        // 6- Set the tag IDs for all nodes, bodies, etc.
        //    To do this, use the helper ChArchiveSetUniqueTags, that traverses all the 
        //    hierarchies, sees if there is a SetTag() function in sub objects, ans sets 
        //    the ID incrementally. More hassle-free than setting all IDs one by one by 
        //    hand as in ...lowlevel.cpp demos
        //    Call this after you finished adding items to systems.
        ChArchiveSetUniqueTags tagger;
        tagger.skip_already_tagged = false;
        tagger << CHNVP(sys);
    }


    // OPTIONAL: POSTPROCESSING VIA BLENDER3D
   
    // Create an exporter to Blender
    ChBlender blender_exporter = ChBlender(&sys);
    // Set the path where it will save all files, a directory will be created if not existing. 
    // The directories will have a name depending on the rank: MDOM_MPI_0, MDOM_MPI_1, MDOM_MPI_2, etc.
    blender_exporter.SetBasePath(GetChronoOutputPath() + "MDOM_MPI_" + std::to_string(domain_manager.GetMPIrank()));
    // Trick: apply thin domain-specific jitter to all Blender scene, to overcome Blender issue of black artifacts
    // if rendering engine is Cycles and two surfaces are exactly complanar - as happens with objects shared between domains.
    blender_exporter.SetBlenderFrame(ChFramed(domain_manager.GetMPIrank() * 2e-5 * ChVector3d(1, 1, 1), Q_ROTATE_Y_TO_Z));
    // Initial script, save once at the beginning
    blender_exporter.ExportScript();


    // 7 - INITIAL SETUP AND OBJECT INITIAL MIGRATION!
    //     Moves all the objects in master domain to all domains, slicing the system.
    //     Also does some initializations, like collision detection AABBs.
    domain_manager.DoDomainInitialize(domain_manager.GetMPIrank());

    // The master domain does not need to communicate anymore with the domains so do:
    domain_manager.master_domain_enabled = false;

    for (int i = 0; i < 1000; ++i) {

        if (domain_manager.GetMPIrank()==0) 
            std::cout << "\n\n\n============= Time step (explicit) " << i << std::endl << std::endl;

        // MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
        domain_manager.DoDomainPartitionUpdate(domain_manager.GetMPIrank());

        // MULTIDOMAIN TIME INTEGRATION
        sys.DoStepDynamics(0.0001);

        // OPTIONAL POSTPROCESSING FOR 3D RENDERING
        // Before ExportData(), we must do a remove-add trick as an easy way to handle the fact
        // that objects are constantly added and removed from the i-th domain when crossing boundaries.
        // Since timestep is small for explicit integration, save only every n-th time steps.
        if ((i % 20) == 0) {
            blender_exporter.RemoveAll();
            blender_exporter.AddAll();
            blender_exporter.ExportData();
        }

    }
   
   




    return 0;
}
