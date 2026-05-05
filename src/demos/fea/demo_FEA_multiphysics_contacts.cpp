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
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/multiphysics/ChDomainDeformation.h"
#include "chrono/fea/multiphysics/ChMaterial3DStressNeoHookean.h"
#include "chrono/fea/multiphysics/ChDrawer.h"
#include "chrono/fea/multiphysics/ChSurfaceOfDomain.h"
#include "chrono/fea/multiphysics/ChFieldElementHexahedron8.h"
#include "chrono/fea/multiphysics/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/multiphysics/ChFieldElementTetrahedron4.h"
#include "chrono/fea/multiphysics/ChFieldElementTetrahedron4Face.h"
#include "chrono/fea/multiphysics/ChFieldElementLoadableVolume.h"
#include "chrono/fea/multiphysics/ChFieldElementLoadableSurface.h"
#include "chrono/fea/multiphysics/ChBuilderVolume.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemSMC sys;
    sys.SetGravityY();

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    // Create and set the collision system
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);
    ChCollisionModel::SetDefaultSuggestedMargin(0.006);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Use this value for an outward additional layer around meshes, that can improve
    // robustness of mesh-mesh collision detection (at the cost of having unnatural inflate effect)
    double sphere_swept_thickness = 0.02;

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


    // Example 1: tetrahedron mesh, with collisions

    // field
    auto displacement_field = chrono_types::make_shared<ChFieldDisplacement3D>();
    sys.Add(displacement_field);

    // domain
    auto elastic_domain = chrono_types::make_shared<ChDomainDeformation>(displacement_field);
    sys.Add(elastic_domain);
    elastic_domain->SetAutomaticGravity(true);

    // material
    auto elastic_material = chrono_types::make_shared<ChMaterial3DStressNeoHookean>();
    elastic_material->SetDensity(5000);
    elastic_material->SetYoungModulus(3e6);
    elastic_material->SetPoissonRatio(0.35);
    elastic_domain->material = elastic_material;  // set the material in domain

    // mesh with elements, as box shape

    // Build a test volume discretized with a regular grid of finite elements.
    ChBuilderVolumeBoxTetra builder;
    builder.BuildVolume(ChFrame<>(ChVector3d(-0.5, 0.2, 0)),  // inital position of the box
                        8, 2, 3,                // N of elements in x,y,z direction
                        1, 0.2, 0.3);          // width in x,y,z direction
    builder.AddToDomain(elastic_domain);
    
    builder.BuildVolume(ChFrame<>(ChVector3d(0, 0.5, 0.5), QuatFromAngleAxis(1.56, ChVector3d(0, 1, 0))),  // inital position of the box
                        8, 2, 3,                              // N of elements in x,y,z direction
                        1, 0.2, 0.3);                         // width in x,y,z direction
    builder.AddToDomain(elastic_domain);
    
    // Create the contact surface(s).
    // In this case it is a ChContactSurfaceMesh, that allows mesh-mesh collisions.

    auto contact_surface = chrono_types::make_shared<ChContactSurfaceMesh>(contact_material);
    contact_surface->AddFacesFromBoundary(elastic_domain, sphere_swept_thickness);

    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);
    mesh->AddContactSurface(contact_surface); // workaround, the AddContactSurface mthd should be available in chDomain in future 

    // POSTPROCESSING & VISUALIZATION (optional)

    // show mesh painted with jet colormap proportional to Euler-Almansi strain intensity
    auto visual_mesh2 = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
    visual_mesh2->AddPositionExtractor(ExtractPos());
    visual_mesh2->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain().VonMises(), -0.1, 0.1, "Stretch");
    visual_mesh2->SetColormap(ChColormap(ChColormap::Type::JET));
    visual_mesh2->SetShrinkElements(true, 0.95);
    elastic_domain->AddVisualShape(visual_mesh2);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Multiphysics example: contacts (smooth, with penalty)");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 2, -4));
    vis->AddTypicalLights();

    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sys.SetSolver(mkl_solver);

    // Simulation loop
    double timestep = 0.0005;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(timestep);
    }

    return 0;
}
