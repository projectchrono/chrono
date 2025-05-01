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

    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(1e5);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // Create a floor:

    bool do_mesh_collision_floor = false;

    auto mmeshbox = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/cube.obj"), true, true);

    if (do_mesh_collision_floor) {
        // floor as a triangle mesh surface:
        auto mfloor = chrono_types::make_shared<ChBody>();
        mfloor->SetPos(ChVector3d(0, -1, 0));
        mfloor->SetFixed(true);
        sys.Add(mfloor);

        auto floor_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mysurfmaterial, mmeshbox, false,
                                                                                   false, sphere_swept_thickness);
        mfloor->AddCollisionShape(floor_shape);
        mfloor->EnableCollision(true);

        auto masset_meshbox = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        masset_meshbox->SetMesh(mmeshbox);
        masset_meshbox->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        mfloor->AddVisualShape(masset_meshbox);
    } else {
        // floor as a simple collision primitive:
        auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.1, 2, 2700, true, true, mysurfmaterial);
        mfloor->SetFixed(true);
        mfloor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        sys.Add(mfloor);
    }

    // two falling objects:

    auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 2700, true, true, mysurfmaterial);
    mcube->SetPos(ChVector3d(0.6, 0.5, 0.6));
    sys.Add(mcube);

    auto msphere = chrono_types::make_shared<ChBodyEasySphere>(0.1, 2700, true, true, mysurfmaterial);
    msphere->SetPos(ChVector3d(0.8, 0.5, 0.6));
    sys.Add(msphere);

    // Example 1: tetrahedrons, with collisions

    // Create a mesh. We will use it for tetrahedrons.

    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // 1) a FEA tetrahedron(s):

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->SetYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->SetPoissonRatio(0.3);
    mmaterial->SetRayleighDampingBeta(0.003);
    mmaterial->SetDensity(1000);

    if (false) {
        for (int k = 0; k < 3; ++k)
            for (int j = 0; j < 3; ++j)
                for (int i = 0; i < 3; ++i) {
                    // Creates the nodes for the tetrahedron
                    ChVector3d offset(j * 0.21, i * 0.21, k * 0.21);
                    auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0.1, 0) + offset);
                    auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0.1, 0.2) + offset);
                    auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0.3, 0) + offset);
                    auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0.2, 0.1, 0) + offset);

                    my_mesh->AddNode(mnode1);
                    my_mesh->AddNode(mnode2);
                    my_mesh->AddNode(mnode3);
                    my_mesh->AddNode(mnode4);

                    auto melement1 = chrono_types::make_shared<ChElementTetraCorot_4>();
                    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);
                    melement1->SetMaterial(mmaterial);

                    my_mesh->AddElement(melement1);
                }
    }

    if (true) {
        double angles[4] = {0.24304, 0.774199, 0.963853, 0.47501};
        for (int i = 0; i < 4; ++i) {
            try {
                ChCoordsys<> cdown(ChVector3d(0, -0.4, 0));
                ChCoordsys<> crot(VNULL, QuatFromAngleY(CH_2PI * ChRandom::Get()) * QuatFromAngleX(CH_PI_2));
                ChCoordsys<> cydisp(ChVector3d(-0.3, 0.1 + i * 0.1, -0.3));
                ChCoordsys<> ctot = cdown >> crot >> cydisp;
                ChMatrix33<> mrot(ctot.rot);
                ChMeshFileLoader::FromTetGenFile(my_mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                                 GetChronoDataFile("fea/beam.ele").c_str(), mmaterial, ctot.pos, mrot);
            } catch (std::exception myerr) {
                std::cerr << myerr.what();
                return 0;
            }
        }
    }

    // Create the contact surface(s).
    // In this case it is a ChContactSurfaceMesh, that allows mesh-mesh collisions.

    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>(mysurfmaterial);
    mcontactsurf->AddFacesFromBoundary(*my_mesh, sphere_swept_thickness);
    my_mesh->AddContactSurface(mcontactsurf);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Example 2: beams, with collisions

    // Create a mesh. We will use it for beams only.

    auto my_mesh_beams = chrono_types::make_shared<ChMesh>();

    // 2) an ANCF cable:

    auto msection_cable2 = chrono_types::make_shared<ChBeamSectionCable>();
    msection_cable2->SetDiameter(0.05);
    msection_cable2->SetYoungModulus(0.01e9);
    msection_cable2->SetRayleighDamping(0.05);

    ChBuilderCableANCF builder;

    builder.BuildBeam(my_mesh_beams,             // the mesh where to put the created nodes and elements
                      msection_cable2,           // the ChBeamSectionCable to use for the ChElementCableANCF elements
                      10,                        // the number of ChElementCableANCF to create
                      ChVector3d(0, 0.1, -0.1),  // the 'A' point in space (beginning of beam)
                      ChVector3d(0.5, 0.13, -0.1));  // the 'B' point in space (end of beam)

    // Create the contact surface(s).
    // In this case it is a ChContactSurfaceNodeCloud, so just pass
    // all nodes to it.

    auto mcontactcloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mysurfmaterial);
    mcontactcloud->AddAllNodes(*my_mesh_beams, 0.025);  // use larger point size to match beam section radius
    my_mesh_beams->AddContactSurface(mcontactcloud);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh_beams);

    // Optional...  visualization

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChVisualShapeTriangleMesh).
    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemeshcoll->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddVisualShapeFEA(mvisualizemeshcoll);

    auto mvisualizemeshbeam = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemeshbeam->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemeshbeam->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemeshbeam->SetSmoothFaces(true);
    my_mesh_beams->AddVisualShapeFEA(mvisualizemeshbeam);

    auto mvisualizemeshbeamnodes = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizemeshbeamnodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshbeamnodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshbeamnodes->SetSymbolsThickness(0.008);
    my_mesh_beams->AddVisualShapeFEA(mvisualizemeshbeamnodes);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "FEA contacts (SMC)",
                                         ChVector3d(0.0, 0.6, -1.0));

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
