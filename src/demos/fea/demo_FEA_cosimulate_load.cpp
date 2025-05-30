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
// FEA force-displacement co-simulation
//   - loading an Abaqus tetrahedron mesh
//   - apply a load to the mesh using an external tool, e.g. CFD.
//     (here simulated as a function in this .cpp file)
//
// =============================================================================

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLoadBodyMesh.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChLoadContactSurfaceMesh.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

// This function simulates the effect of an external program that
// gets a triangle mesh and outputs the forces acting on the nodes.
// In a real cosimulation scenario, this procedure could even reside
// on a different computing node and manage inputs/outputs via MPI or such.

void PerformExternalCosimulation(const std::vector<ChVector3d>& input_vert_pos,
                                 const std::vector<ChVector3d>& input_vert_vel,
                                 const std::vector<ChVector3i>& input_triangles,
                                 std::vector<ChVector3d>& vert_output_forces,
                                 std::vector<int>& vert_output_indexes) {
    vert_output_forces.clear();
    vert_output_indexes.clear();
    double ky = 10000;  // upward stiffness
    double ry = 20;     // upward damping
    // simple example: scan through all vertexes in the mesh, see if they sink below zero,
    // apply a penalty upward spring force if so.
    for (int iv = 0; iv < input_vert_pos.size(); ++iv) {
        if (input_vert_pos[iv].y() < 0) {
            double yforce = -ky * input_vert_pos[iv].y() - ry * input_vert_vel[iv].y();
            if (yforce > 0) {
                vert_output_forces.push_back(ChVector3d(0, yforce, 0));
                vert_output_indexes.push_back(iv);
            }
        }
    }
    // note that:
    // - we avoided adding forces to  vert_output_forces  when force was zero.
    // - vert_output_forces has the same size of vert_output_indexes, maybe smaller than input_vert_pos
}

// Utility to draw some triangles that are affected by cosimulation.
// Also plot forces as vectors.
// Mostly for debugging.
void draw_affected_triangles(ChVisualSystemIrrlicht& vis,
                             std::vector<ChVector3d>& vert_pos,
                             std::vector<ChVector3i>& triangles,
                             std::vector<int>& vert_indexes,
                             std::vector<ChVector3d>& vert_forces,
                             double forcescale = 0.01) {
    for (int it = 0; it < triangles.size(); ++it) {
        bool vert_hit = false;
        for (int io = 0; io < vert_indexes.size(); ++io) {
            if (triangles[it].x() == vert_indexes[io] || triangles[it].y() == vert_indexes[io] ||
                triangles[it].z() == vert_indexes[io])
                vert_hit = true;
        }
        if (vert_hit == true) {
            std::vector<chrono::ChVector3d> fourpoints = {vert_pos[triangles[it].x()], vert_pos[triangles[it].y()],
                                                          vert_pos[triangles[it].z()], vert_pos[triangles[it].x()]};
            tools::drawPolyline(&vis, fourpoints, ChColor(0.94f, 0.78f, 0.00f), true);
        }
    }
    if (forcescale > 0)
        for (int io = 0; io < vert_indexes.size(); ++io) {
            std::vector<chrono::ChVector3d> forceline = {vert_pos[vert_indexes[io]],
                                                         vert_pos[vert_indexes[io]] + vert_forces[io] * forcescale};
            tools::drawPolyline(&vis, forceline, ChColor(0.94f, 0.00f, 0.00f), true);
        }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Global parameter for tire:
    double tire_rad = 0.8;
    ChVector3d tire_center(0, 0.02 + tire_rad, 0.5);
    ChMatrix33<> tire_alignment(QuatFromAngleY(CH_PI));  // create rotated 180 deg on y

    // Create a Chrono physical system and the associated collision system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    // CREATE A FINITE ELEMENT MESH

    // Create the surface material
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(10e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // Create a mesh, that is a container for groups of FEA elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(my_mesh);

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->SetYoungModulus(0.003e9);  // rubber 0.01e9, steel 200e9
    mmaterial->SetPoissonRatio(0.4);
    mmaterial->SetRayleighDampingBeta(0.004);
    mmaterial->SetDensity(1000);

    // Load an ABAQUS .INP tetrahedron mesh file from disk, defining a tetrahedron mesh.
    // Note that not all features of INP files are supported. Also, quadratic tetrahedrons are promoted to linear.
    // This is much easier than creating all nodes and elements via C++ programming.
    // Ex. you can generate these .INP files using Abaqus or exporting from the SolidWorks simulation tool.
    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets;
    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh,
                                         GetChronoDataFile("models/tractor_wheel/tractor_wheel_coarse.INP").c_str(),
                                         mmaterial, node_sets, tire_center, tire_alignment);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 0;
    }

    // Create the contact surface(s).
    // Use the AddFacesFromBoundary() to select automatically the outer skin of the tetrahedron mesh.
    // Note that the contact material specified here is not used, as contacts will be emulated through cosimulation.
    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>(mysurfmaterial);
    mcontactsurf->AddFacesFromBoundary(*my_mesh);
    my_mesh->AddContactSurface(mcontactsurf);

    // Create a mesh load for cosimulation, acting on the contact surface above
    // (forces on nodes will be computed by an external procedure)
    auto mloadcontainer = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(mloadcontainer);

    auto mmeshload = chrono_types::make_shared<ChLoadContactSurfaceMesh>(mcontactsurf);
    mloadcontainer->Add(mmeshload);

    // Optional...  visualization

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColormapRange(0.0, 10);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    // CREATE A RIGID BODY WITH A MESH

    // Create also a rigid body with a rigid mesh that will be used for the cosimulation,
    // this time the ChLoadContactSurfaceMesh cannot be used as in the FEA case, so we
    // will use the ChLoadBodyMesh class:

    auto mrigidbody = chrono_types::make_shared<ChBody>();
    sys.Add(mrigidbody);
    mrigidbody->SetMass(200);
    mrigidbody->SetInertiaXX(ChVector3d(20, 20, 20));
    mrigidbody->SetPos(tire_center + ChVector3d(-1, 0, 0));

    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
        GetChronoDataFile("models/tractor_wheel/tractor_wheel_fine.obj"));
    mesh->Transform(VNULL, QuatFromAngleY(CH_PI));
    auto mesh_asset = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_asset->SetMesh(mesh);
    mrigidbody->AddVisualShape(mesh_asset);

    // this is used to use the mesh in cosimulation!
    auto mrigidmeshload = chrono_types::make_shared<ChLoadBodyMesh>(mrigidbody, *mesh);
    mloadcontainer->Add(mrigidmeshload);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1280, 720);
    vis->SetWindowTitle("demo_FEA_cosimulate_load");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddLightWithShadow(ChVector3d(1.5, 5.5, -2.5), ChVector3d(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                            ChColor(0.8f, 0.8f, 1.0f));
    vis->AddCamera(ChVector3d(1.0, 1.4, -1.2), ChVector3d(0, tire_rad, 0));

    vis->EnableShadows();

    // THE SOFT-REAL-TIME CYCLE

    // Change solver to embedded MINRES
    // NOTE! it is strongly advised that you compile the optional MKL module
    // if you need higher precision, and switch to its MKL solver - see demos for FEA & MKL.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // Enable for better convergence if using Euler implicit linearized

    // Change type of integrator:
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        sys.DoStepDynamics(0.005);

        // -------------------------------------------------------------------------
        // Here do the cosimulation
        // A <--> B
        // For example, A is this main program, and B can be an external program,
        // e.g. a CFD simulation tool.
        // The idea is that A --> B communicates the mesh position,
        // then A <-- B receives the computed forces to be applied at nodes.
        // In this example, to keep things simple, B is just a simple local function

        std::vector<ChVector3d> vert_pos;
        std::vector<ChVector3d> vert_vel;
        std::vector<ChVector3i> triangles;
        std::vector<ChVector3d> vert_forces;
        std::vector<int> vert_indexes;

        mmeshload->OutputSimpleMesh(vert_pos, vert_vel, triangles);

        PerformExternalCosimulation(vert_pos, vert_vel, triangles, vert_forces, vert_indexes);

        mmeshload->InputSimpleForces(vert_forces, vert_indexes);

        // now, just for debugging and some fun, draw some triangles
        // (only those that have a vertex that has a force applied):
        draw_affected_triangles(*vis, vert_pos, triangles, vert_indexes, vert_forces, 0.01);

        // Other example: call another cosimulation, this time for the rigid body
        // mesh (the second tire, the rigid one):
        vert_pos.clear();
        vert_vel.clear();
        triangles.clear();
        vert_forces.clear();
        vert_indexes.clear();

        mrigidmeshload->OutputSimpleMesh(vert_pos, vert_vel, triangles);

        PerformExternalCosimulation(vert_pos, vert_vel, triangles, vert_forces, vert_indexes);

        mrigidmeshload->InputSimpleForces(vert_forces, vert_indexes);

        // now, just for debugging and some fun, draw some triangles
        // (only those that have a vertex that has a force applied):
        draw_affected_triangles(*vis, vert_pos, triangles, vert_indexes, vert_forces, 0.01);

        // End of cosimulation block
        // -------------------------------------------------------------------------

        tools::drawGrid(vis.get(), 0.1, 0.1, 20, 20, ChCoordsys<>(VNULL, CH_PI_2, VECT_X), ChColor(0.40f, 0.40f, 0.40f),
                        true);

        vis->EndScene();
    }

    return 0;
}
