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
// Authors: Antonio Recuero, Radu Serban
// =============================================================================
//
// FEA contact of ANCF
//
// =============================================================================

#include <iostream>
#include <cmath>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChContactSurfaceSegmentSet.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChBuilderBeam.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// ------------------------------------------------------------

class PrintContactInfo : public ChContactContainer::AddContactCallback {
  public:
    virtual void OnAddContact(const ChCollisionInfo& cinfo, ChContactMaterialComposite* const material) override {
        auto contactableA = cinfo.modelA->GetContactable();
        auto contactableB = cinfo.modelB->GetContactable();

        std::cout << "  " << cinfo.vpA.z() << "  |  " << cinfo.vpB.z() << std::endl;
    }
};

// ------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::cout << "-----------------------------------------------------------\n";
    std::cout << " 		ANCF Contact\n";
    std::cout << "-----------------------------------------------------------\n\n";

    // --------------------
    // Create Chrono system
    // --------------------

    ChSystemSMC sys;

    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));
    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    ChCollisionModel::SetDefaultSuggestedMargin(0.001);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // --------------------------------------------------------
    // Create a contact mat_shells, shared among all contactables
    // --------------------------------------------------------

    auto contact_material = chrono_types::make_shared<ChContactMaterialSMC>();
    contact_material->SetYoungModulus(6e4f);
    contact_material->SetFriction(0.3f);
    contact_material->SetRestitution(0.5f);
    contact_material->SetAdhesion(0);

    // ---------------
    // Add ground body
    // ---------------

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(3, 3, 0.2, 8000, true, true, contact_material);
    ground->SetPos(ChVector3d(0, 0, -0.1));
    ground->SetFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
    sys.Add(ground);

    // ------------------------------------------
    // Create a mesh with ShellANCF_3423 elements
    // ------------------------------------------

    auto mesh_shells = chrono_types::make_shared<ChMesh>();
    auto mat_shells = chrono_types::make_shared<ChMaterialShellANCF>(1000, 1e8, 0.3);

    try {
        std::vector<int> BC_NODES;
        ChVector3d Center(-0.5, -0.5, 0.5);
        ChMatrix33<> rot_transform(0);
        rot_transform(0, 0) = 1;
        rot_transform(1, 1) = -1;
        rot_transform(2, 1) = -1;
        std::vector<double> NODE_AVE_AREA;

        ChMeshFileLoader::ANCFShellFromGMFFile(mesh_shells, GetChronoDataFile("fea/Plate.mesh").c_str(), mat_shells,
                                               NODE_AVE_AREA, BC_NODES, Center, rot_transform, 0.8, false, false);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 1;
    }

    double sphere_swept_thickness = 0.008;
    double dz = 0.01;

    ////auto contact_surf_shells = chrono_types::make_shared<ChContactSurfaceMesh>(contact_material);
    ////contact_surf_shells->AddFacesFromBoundary(*mesh_shells, sphere_swept_thickness);
    ////mesh_shells->AddContactSurface(contact_surf_shells);

    auto contact_surf_shells = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_material);
    contact_surf_shells->AddAllNodes(*mesh_shells, sphere_swept_thickness);
    mesh_shells->AddContactSurface(contact_surf_shells);

    for (auto& e : mesh_shells->GetElements()) {
        auto e_shell = std::dynamic_pointer_cast<ChElementShellANCF_3423>(e);
        e_shell->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat_shells);  // add single layer with 0 deg fiber angle
        e_shell->SetAlphaDamp(0.08);                           // structural damping for this element
    }

    // Switch on/off mesh class gravity
    mesh_shells->SetAutomaticGravity(true);

    // Add the mesh to the system
    sys.Add(mesh_shells);

    // FEA visualization
    {
        auto fea_vis = chrono_types::make_shared<ChVisualShapeFEA>();
        fea_vis->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        fea_vis->SetWireframe(true);
        mesh_shells->AddVisualShapeFEA(fea_vis);
    }

    // ------------------------------------------
    // Create a mesh with CableANCF elements
    // ------------------------------------------

    auto mesh_cables = chrono_types::make_shared<ChMesh>();

    auto section_cables = chrono_types::make_shared<ChBeamSectionCable>();
    section_cables->SetDiameter(0.02);
    section_cables->SetYoungModulus(0.01e9);
    section_cables->SetRayleighDamping(0.05);

    ChBuilderCableANCF builder;

    builder.BuildBeam(mesh_cables,                    // mesh containing nodes and elements
                      section_cables,                 // ChBeamSectionCable to use for the ChElementCableANCF elements
                      8,                              // number of ChElementCableANCF to create
                      ChVector3d(+0.15, -0.1, 0.2),   // beam start point
                      ChVector3d(-0.15, -0.1, 0.3));  // beam end point

    const auto& nodes_cables = builder.GetLastBeamNodes();

    ////auto contact_surf_cables = chrono_types::make_shared<ChContactSurfaceMesh>(contact_material);
    ////contact_surf_cables->AddFacesFromBoundary(*mesh_cables, 0.01);
    ////mesh_cables->AddContactSurface(contact_surf_cables);

    auto contact_surf_cables = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_material);
    contact_surf_cables->AddAllNodes(*mesh_cables, 0.01);
    mesh_cables->AddContactSurface(contact_surf_cables);

    ////auto contact_surf_cables = chrono_types::make_shared<ChContactSurfaceSegmentSet>(contact_material);
    ////contact_surf_cables->AddAllSegments(*mesh_cables, 0.01);
    ////mesh_cables->AddContactSurface(contact_surf_cables);

    // Add the mesh to the system
    sys.Add(mesh_cables);

    // FEA visualization
    {
        auto fea_vis = chrono_types::make_shared<ChVisualShapeFEA>();
        fea_vis->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
        fea_vis->SetColorscaleMinMax(0.0, 5.50);
        fea_vis->SetSmoothFaces(true);
        mesh_cables->AddVisualShapeFEA(fea_vis);
    }

    // Register callback for printing contact info
    auto contact_callback = chrono_types::make_shared<PrintContactInfo>();
    sys.GetContactContainer()->RegisterAddContactCallback(contact_callback);

    // ------------------------------------
    // Create run-time visualization system
    // ------------------------------------

    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Z, sys, "ANCF Contact", ChVector3d(0.4, 0.4, 0.4),
                                         ChVector3d(-0.25, -0.25, 0.0));

    // -------------------------
    // Set solver and integrator
    // -------------------------

    // Set solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(150);
    solver->SetTolerance(1e-8);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // Enable for better convergence if using Euler implicit linearized

    ////auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    ////sys.SetSolver(mkl_solver);
    ////mkl_solver->LockSparsityPattern(true);
    ////sys.Update();

    // Set integrator
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxIters(200);
    mystepper->SetAbsTolerances(1e-04);
    mystepper->SetVerbose(false);

    ////sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise

    // ---------------
    // Simulation loop
    // ---------------

    double time_step = 5e-4;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        std::cout << "Time t = " << sys.GetChTime() << "s \n";
        for (auto n : nodes_cables)
            std::cout << n->GetPos().z() << "  ";
        std::cout << std::endl;

        ////std::cout << "Time t = " << sys.GetChTime() << "s \t";
        ////std::cout << "n contacts: " << sys.GetNumContacts() << "\t";
        ////std::cout << "pos.y = " << sampleNode->pos.y - y0 << "vs. " << -0.5 * 9.8 * pow(sys.GetChTime(), 2)
        ////          << "\n";

        sys.DoStepDynamics(time_step);
    }

    return 0;
}
