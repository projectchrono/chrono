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
#include "chrono/core/ChRandom.h"
#include "chrono/utils/ChUtils.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/solver/ChSolverADMM.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Select object types
    // -------------------

    std::string input;

    int floor_model = 1;
    std::cout << "Floor model:\n";
    std::cout << "  1. Box primitive [DEFAULT]" << std::endl;
    std::cout << "  2. Triangle mesh" << std::endl;
    std::cout << "\nSelect model: ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        std::istringstream stream(input);
        stream >> floor_model;
        ChClampValue(floor_model, 1, 2);
    }

    int mesh_model = 1;
    std::cout << "Object model:\n";
    std::cout << "  1. FEA beams [DEFAULT]" << std::endl;
    std::cout << "  2. FEA tire" << std::endl;
    std::cout << "\nSelect model: ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        std::istringstream stream(input);
        stream >> mesh_model;
        ChClampValue(mesh_model, 1, 2);
    }

    // Create the Chrono system
    // ------------------------

    ChSystemNSC sys;
    sys.SetGravityY();
    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    // Create and set the collision system
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);
    ChCollisionModel::SetDefaultSuggestedMargin(0.006);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Set an additional outward layer around meshes to improve robustness of mesh-mesh collision detection
    double sphere_swept_thickness = 0.002;

    // Create the contact surface material
    // -----------------------------------

    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    contact_mat->SetFriction(0.3f);
    contact_mat->SetRestitution(0);

    // Create the floor
    // ----------------

    switch (floor_model) {
        case 1: {
            // floor as a simple collision primitive:
            auto floor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.1, 2, 2700, true, true, contact_mat);
            floor->SetFixed(true);
            floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
            sys.Add(floor);

            break;
        }
        case 2: {
            // floor as a triangle mesh surface:
            auto floor = chrono_types::make_shared<ChBody>();
            floor->SetPos(ChVector3d(0, -1, 0));
            floor->SetFixed(true);
            sys.Add(floor);

            auto box_mesh =
                ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/cube.obj"), true, true);
            auto floor_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(contact_mat, box_mesh, false,
                                                                                       false, sphere_swept_thickness);
            floor->AddCollisionShape(floor_shape);
            floor->EnableCollision(true);

            auto box_asset = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            box_asset->SetMesh(box_mesh);
            box_asset->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
            floor->AddVisualShape(box_asset);

            break;
        }
    }

    // Create a continuum material
    // ---------------------------

    auto continuum_mat = chrono_types::make_shared<ChContinuumElastic>();
    continuum_mat->SetYoungModulus(2e5);  // rubber 0.01e9, steel 200e9
    continuum_mat->SetPoissonRatio(0.3);
    continuum_mat->SetRayleighDampingBeta(0.01);
    continuum_mat->SetDensity(1000);

    // FEA tetrahedral meshes with collision on the skin
    // -------------------------------------------------

    // Create an FEA mesh
    auto mesh_FEA = chrono_types::make_shared<ChMesh>();

    switch (mesh_model) {
        case 1: {
            // FEA beams
            for (int i = 0; i < 3; ++i) {
                ChCoordsys<> cdown1(VNULL, QuatFromAngleX(CH_PI_2));
                ChCoordsys<> cdown2(ChVector3d(0, -0.4, 0));
                ChCoordsys<> crot(VNULL, QuatFromAngleY(5 * CH_2PI * ChRandom::Get()));
                for (int j = -1; j < 2; ++j) {
                    try {
                        ChCoordsys<> cydisp(ChVector3d(0.3 * j, 0.1 + i * 0.1, 0));
                        ChCoordsys<> ctot = cdown2 >> cdown1 >> cydisp >> crot;
                        ChMatrix33<> mrot(ctot.rot);
                        ChMeshFileLoader::FromTetGenFile(mesh_FEA, GetChronoDataFile("fea/beam.node").c_str(),
                                                         GetChronoDataFile("fea/beam.ele").c_str(), continuum_mat,
                                                         ctot.pos, mrot);
                    } catch (std::exception myerr) {
                        std::cerr << myerr.what();
                        return 0;
                    }
                }
            }

            break;
        }

            // Example: a tetrahedral tire

        case 2: {
            // FEA tire
            std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets;
            ChCoordsys<> cpos(ChVector3d(0, 0.8, 0), QuatFromAngleY(CH_PI_2));
            ChMeshFileLoader::FromAbaqusFile(mesh_FEA,
                                             GetChronoDataFile("models/tractor_wheel/tractor_wheel_fine.INP").c_str(),
                                             continuum_mat, node_sets, cpos.pos, cpos.rot);
            break;
        }
    }

    // Create the contact surface
    auto contact_surface = chrono_types::make_shared<ChContactSurfaceMesh>(contact_mat);
    contact_surface->AddFacesFromBoundary(*mesh_FEA, sphere_swept_thickness);
    mesh_FEA->AddContactSurface(contact_surface);

    // Add mesh to system
    sys.Add(mesh_FEA);

    // ANCF beam with node collisions as point cloud
    // ---------------------------------------------

    // Create a mesh
    auto mesh_ANCF = chrono_types::make_shared<ChMesh>();

    // Create ANCF cable elements
    for (int icable = 0; icable < 1; ++icable) {
        auto section_cable = chrono_types::make_shared<ChBeamSectionCable>();
        section_cable->SetDiameter(0.05);
        section_cable->SetYoungModulus(0.01e9);
        section_cable->SetRayleighDamping(0.05);

        ChBuilderCableANCF builder;

        ChCoordsys<> cable_pos(ChVector3d(0, icable * 0.11, 0), QuatFromAngleY(ChRandom::Get() * CH_2PI));

        builder.BuildBeam(mesh_ANCF,                                 // containing mesh
                          section_cable,                             // section data for ANCF cable elements
                          12,                                        // number of ANCF cable elements
                          cable_pos * ChVector3d(0, 0.1, -0.1),      // beginning of beam
                          cable_pos * ChVector3d(0.5, 0.13, -0.1));  // end of beam

        // Create the contact surface
        auto contact_cloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_mat);
        contact_cloud->AddAllNodes(*mesh_ANCF, 0.025);  // use larger point size to match beam section radius
        mesh_ANCF->AddContactSurface(contact_cloud);
    }

    // Add mesh to system
    sys.Add(mesh_ANCF);

    // FEA mesh visualization
    // ----------------------

    // Visualization of the FEM mesh
    auto visualization_FEA = chrono_types::make_shared<ChVisualShapeFEA>();
    visualization_FEA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    visualization_FEA->SetColormapRange(0.0, 5.50);
    visualization_FEA->SetSmoothFaces(true);
    mesh_FEA->AddVisualShapeFEA(visualization_FEA);

    auto visualization_ANCF = chrono_types::make_shared<ChVisualShapeFEA>();
    visualization_ANCF->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    visualization_ANCF->SetColormapRange(0.0, 5.50);
    visualization_ANCF->SetSmoothFaces(true);
    mesh_ANCF->AddVisualShapeFEA(visualization_ANCF);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "FEA contacts (NSC)",
                                         ChVector3d(0.0, 1.2, -2.0));

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

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.005);
    }

    return 0;
}
