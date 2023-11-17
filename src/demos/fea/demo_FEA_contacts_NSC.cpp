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
// FEA contacts using non-smooth contacts.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/solver/ChSolverADMM.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::fea;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;
    sys.SetNumThreads(ChOMP::GetNumProcs(), 0, 1);

    // Create and set the collision system
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);
    ChCollisionModel::SetDefaultSuggestedMargin(0.006);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Use this value for an outward additional layer around meshes, that can improve
    // robustness of mesh-mesh collision detection (at the cost of having unnatural inflate effect)
    double sphere_swept_thickness = 0.002;

    // Create the surface material, containing information
    // about friction etc.
    // It is a NSC non-smooth contact material that we will assign to
    // all surfaces that might generate contacts.

    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0);

    // Create a floor:

    bool do_mesh_collision_floor = false;

    auto mmeshbox = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/cube.obj"), true, true);

    if (do_mesh_collision_floor) {
        // floor as a triangle mesh surface:
        auto mfloor = chrono_types::make_shared<ChBody>();
        mfloor->SetPos(ChVector<>(0, -1, 0));
        mfloor->SetBodyFixed(true);
        sys.Add(mfloor);

        auto floor_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mysurfmaterial, mmeshbox, false,
                                                                                   false, sphere_swept_thickness);
        mfloor->AddCollisionShape(floor_shape);
        mfloor->SetCollide(true);

        auto masset_meshbox = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        masset_meshbox->SetMesh(mmeshbox);
        masset_meshbox->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        mfloor->AddVisualShape(masset_meshbox);
    } else {
        // floor as a simple collision primitive:
        auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.1, 2, 2700, true, true, mysurfmaterial);
        mfloor->SetBodyFixed(true);
        mfloor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        sys.Add(mfloor);
    }

    // two falling objects:

    if (false) {
        auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 2700, true, true, mysurfmaterial);
        mcube->SetPos(ChVector<>(0.6, 0.5, 0.6));
        sys.Add(mcube);

        auto msphere = chrono_types::make_shared<ChBodyEasySphere>(0.1, 2700, true, true, mysurfmaterial);
        msphere->SetPos(ChVector<>(0.8, 0.5, 0.6));
        sys.Add(msphere);
    }

    // Create a mesh. We will use it for tetrahedrons.

    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a continuum material, that must be assigned to each solid element in the mesh,
    // and set its parameters

    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(2e5);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
    mmaterial->Set_RayleighDampingK(0.01);
    mmaterial->Set_density(1000);

    // Example: stack of tetrahedral meshes, with collision on the skin

    if (true) {
        for (int i = 0; i < 3; ++i) {
            ChCoordsys<> cdown1(VNULL, Q_from_AngAxis(CH_C_PI_2, VECT_X));
            ChCoordsys<> cdown2(ChVector<>(0, -0.4, 0));
            ChCoordsys<> crot(VNULL, Q_from_AngAxis(5 * CH_C_2PI * ChRandom(), VECT_Y));
            for (int j = -1; j < 2; ++j) {
                try {
                    ChCoordsys<> cydisp(ChVector<>(0.3 * j, 0.1 + i * 0.1, 0));
                    ChCoordsys<> ctot = cdown2 >> cdown1 >> cydisp >> crot;
                    ChMatrix33<> mrot(ctot.rot);
                    ChMeshFileLoader::FromTetGenFile(my_mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                                     GetChronoDataFile("fea/beam.ele").c_str(), mmaterial, ctot.pos,
                                                     mrot);
                } catch (ChException myerr) {
                    GetLog() << myerr.what();
                    return 0;
                }
            }
        }
    }

    // Example: a tetrahedral tire

    if (false) {
        std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets;
        ChCoordsys<> cpos(ChVector<>(0, 0.8, 0), Q_from_AngAxis(CH_C_PI_2, VECT_Y));
        ChMeshFileLoader::FromAbaqusFile(my_mesh,
                                         GetChronoDataFile("models/tractor_wheel/tractor_wheel_fine.INP").c_str(),
                                         mmaterial, node_sets, cpos.pos, cpos.rot);
    }

    // Create the contact surface(s).
    // In this case it is a ChContactSurfaceMesh, that allows mesh-mesh collsions.

    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>(mysurfmaterial);
    my_mesh->AddContactSurface(mcontactsurf);
    mcontactsurf->AddFacesFromBoundary(sphere_swept_thickness);  // do this after my_mesh->AddContactSurface

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Example: beams, with node collisions as point cloud

    // Create a mesh. We will use it for beams only.

    auto my_mesh_beams = chrono_types::make_shared<ChMesh>();

    // 2) an ANCF cable:

    for (int icable = 0; icable < 1; ++icable) {
        auto msection_cable2 = chrono_types::make_shared<ChBeamSectionCable>();
        msection_cable2->SetDiameter(0.05);
        msection_cable2->SetYoungModulus(0.01e9);
        msection_cable2->SetBeamRaleyghDamping(0.05);

        ChBuilderCableANCF builder;

        ChCoordsys<> cablepos(ChVector<>(0, icable * 0.11, 0), Q_from_AngAxis(ChRandom() * CH_C_2PI, VECT_Y));

        builder.BuildBeam(my_mesh_beams,    // the mesh where to put the created nodes and elements
                          msection_cable2,  // the ChBeamSectionCable to use for the ChElementCableANCF elements
                          12,               // the number of ChElementCableANCF to create
                          cablepos * ChVector<>(0, 0.1, -0.1),      // the 'A' point in space (beginning of beam)
                          cablepos * ChVector<>(0.5, 0.13, -0.1));  // the 'B' point in space (end of beam)

        // Create the contact surface(s).
        // In this case it is a ChContactSurfaceNodeCloud, so just pass
        // all nodes to it.

        auto mcontactcloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mysurfmaterial);
        my_mesh_beams->AddContactSurface(mcontactcloud);
        mcontactcloud->AddAllNodes(0.025);  // use larger point size to match beam section radius
    }

    // Remember to add the mesh to the system!
    sys.Add(my_mesh_beams);

    // Optional...  visualization

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChVisualShapeTriangleMesh).
    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);
    /*
        auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
        mvisualizemeshcoll->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
        mvisualizemeshcoll->SetWireframe(true);
        mvisualizemeshcoll->SetDefaultMeshColor(ChColor(0.2, 0.2, 0.2));
        my_mesh->AddVisualShapeFEA(mvisualizemeshcoll);
    */

    auto mvisualizemeshbeam = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_beams);
    mvisualizemeshbeam->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemeshbeam->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemeshbeam->SetSmoothFaces(true);
    my_mesh_beams->AddVisualShapeFEA(mvisualizemeshbeam);

    /*
        auto mvisualizemeshbeamnodes = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh_beams);
        mvisualizemeshbeamnodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        mvisualizemeshbeamnodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        mvisualizemeshbeamnodes->SetSymbolsThickness(0.008);
        my_mesh_beams->AddVisualShapeFEA(mvisualizemeshbeamnodes);
    */

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("FEA contacts");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0.0, 0.6, -1.0));
    vis->AddLightWithShadow(ChVector<>(1.5, 5.5, -2.5), ChVector<>(0, 0, 0), 3, 2.2, 7.2, 40, 512, ChColor(1, 1, 1));
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
    vis->EnableShadows();

    // SIMULATION LOOP

    // Use the ADMM solver: it has the capability of handling both FEA and NSC!
    auto solver = chrono_types::make_shared<ChSolverADMM>(chrono_types::make_shared<ChSolverPardisoMKL>());
    solver->EnableWarmStart(true);
    solver->SetMaxIterations(60);
    solver->SetToleranceDual(1e-4);
    solver->SetTolerancePrimal(1e-4);
    solver->SetRho(1);
    solver->SetStepAdjustPolicy(ChSolverADMM::AdmmStepType::BALANCED_UNSCALED);
    sys.SetSolver(solver);

    sys.SetSolverForceTolerance(1e-10);

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.005);
    }

    return 0;
}
