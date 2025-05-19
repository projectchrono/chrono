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
// FEA contacts
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/core/ChRandom.h"

#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChBuilderBeam.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemSMC sys;

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    // Create and set the collision system
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);
    ChCollisionModel::SetDefaultSuggestedMargin(0.006);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Use this value for an outward additional layer around meshes, that can improve
    // robustness of mesh-mesh collision detection (at the cost of having unnatural inflate effect)
    double sphere_swept_thickness = 0.002;

    // Create the surface material, containing information
    // about friction etc.
    // It is a SMC (penalty) material that we will assign to
    // all surfaces that might generate contacts.

    auto contact_material = chrono_types::make_shared<ChContactMaterialSMC>();
    contact_material->SetYoungModulus(1e5);
    contact_material->SetFriction(0.3f);
    contact_material->SetRestitution(0.2f);
    contact_material->SetAdhesion(0);

    // Create a floor:

    bool do_mesh_collision_floor = false;

    auto box_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/cube.obj"), true, true);

    if (do_mesh_collision_floor) {
        // floor as a triangle mesh surface:
        auto floor = chrono_types::make_shared<ChBody>();
        floor->SetPos(ChVector3d(0, -1, 0));
        floor->SetFixed(true);
        sys.Add(floor);

        auto floor_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(contact_material, box_mesh, false,
                                                                                   false, sphere_swept_thickness);
        floor->AddCollisionShape(floor_shape);
        floor->EnableCollision(true);

        auto box_mesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        box_mesh_shape->SetMesh(box_mesh);
        box_mesh_shape->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        floor->AddVisualShape(box_mesh_shape);
    } else {
        // floor as a simple collision primitive:
        auto floor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.1, 2, 2700, true, true, contact_material);
        floor->SetFixed(true);
        floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        sys.Add(floor);
    }

    // two falling objects:

    auto cube = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 2700, true, true, contact_material);
    cube->SetPos(ChVector3d(0.6, 0.5, 0.6));
    sys.Add(cube);

    auto sphere = chrono_types::make_shared<ChBodyEasySphere>(0.1, 2700, true, true, contact_material);
    sphere->SetPos(ChVector3d(0.8, 0.5, 0.6));
    sys.Add(sphere);

    // Example 1: tetrahedrons, with collisions

    // Create a mesh. We will use it for tetrahedrons.

    auto mesh = chrono_types::make_shared<ChMesh>();

    // 1) a FEA tetrahedron(s):

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    material->SetPoissonRatio(0.3);
    material->SetRayleighDampingBeta(0.003);
    material->SetDensity(1000);

    for (int i = 0; i < 4; ++i) {
        try {
            ChCoordsys<> cdown(ChVector3d(0, -0.4, 0));
            ChCoordsys<> crot(VNULL, QuatFromAngleY(CH_2PI * ChRandom::Get()) * QuatFromAngleX(CH_PI_2));
            ChCoordsys<> cydisp(ChVector3d(-0.3, 0.1 + i * 0.1, -0.3));
            ChCoordsys<> ctot = cdown >> crot >> cydisp;
            ChMatrix33<> rot(ctot.rot);
            ChMeshFileLoader::FromTetGenFile(mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                             GetChronoDataFile("fea/beam.ele").c_str(), material, ctot.pos, rot);
        } catch (std::exception err) {
            std::cerr << err.what();
            return 0;
        }
    }

    // Create the contact surface(s).
    // In this case it is a ChContactSurfaceMesh, that allows mesh-mesh collisions.

    auto contact_surface = chrono_types::make_shared<ChContactSurfaceMesh>(contact_material);
    contact_surface->AddFacesFromBoundary(*mesh, sphere_swept_thickness);
    mesh->AddContactSurface(contact_surface);

    // Remember to add the mesh to the system!
    sys.Add(mesh);

    // Example 2: beams, with collisions

    // Create a mesh. We will use it for beams only.

    auto mesh_beam = chrono_types::make_shared<ChMesh>();

    // 2) an ANCF cable:

    auto section_cable = chrono_types::make_shared<ChBeamSectionCable>();
    section_cable->SetDiameter(0.05);
    section_cable->SetYoungModulus(0.01e9);
    section_cable->SetRayleighDamping(0.05);

    ChBuilderCableANCF builder;

    builder.BuildBeam(mesh_beam,                     // containing mesh
                      section_cable,                 // section definition for the ChElementCableANCF elements
                      10,                            // number of ChElementCableANCF to create
                      ChVector3d(0, 0.1, -0.1),      // beginning of beam location
                      ChVector3d(0.5, 0.13, -0.1));  // end of beam location

    // Create the contact surface(s).
    // In this case it is a ChContactSurfaceNodeCloud, so just pass
    // all nodes to it.

    auto contact_nodecloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_material);
    contact_nodecloud->AddAllNodes(*mesh_beam, 0.025);  // use larger point size to match beam section radius
    mesh_beam->AddContactSurface(contact_nodecloud);

    // Remember to add the mesh to the system!
    sys.Add(mesh_beam);

    // Optional...  visualization

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChVisualShapeTriangleMesh).

    ChColormap::Type colormap_type = ChColormap::Type::FAST;
    ChVector2d colormap_range(0.0, 2.50);

    auto vis_mesh_A = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh_A->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh_A->SetColormap(colormap_type);
    vis_mesh_A->SetColormapRange(colormap_range);
    vis_mesh_A->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(vis_mesh_A);

    auto vis_mesh_B = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh_B->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    vis_mesh_B->SetWireframe(true);
    vis_mesh_B->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    mesh->AddVisualShapeFEA(vis_mesh_B);

    auto vis_beam_A = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_A->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_beam_A->SetColormap(colormap_type);
    vis_beam_A->SetColormapRange(colormap_range);
    vis_beam_A->SetSmoothFaces(true);
    mesh_beam->AddVisualShapeFEA(vis_beam_A);

    auto vis_beam_B = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_B->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_beam_B->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_beam_B->SetSymbolsThickness(0.008);
    mesh_beam->AddVisualShapeFEA(vis_beam_B);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "FEA contacts (SMC)",  //
                                         ChVector3d(0.0, 0.6, -1.0), VNULL,                          //
                                         true, "Node speed (m/s)", colormap_range, colormap_type);

    // SIMULATION LOOP

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // Enable for better convergence when using Euler implicit linearized

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.0005);
    }

    return 0;
}
