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

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadBodyMesh.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLoadContactSurfaceMesh.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include <iostream>
#include <cmath>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional>

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::fea;
using namespace chrono::irrlicht;

bool addConstrain = false;
bool addForce = false;
bool addGravity = true;
bool addPressure = false;
bool showBone = true;
bool addFixed = false;
double time_step = 0.0005;
int scaleFactor = 35;
double dz = 0.01;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemSMC sys;

    sys.SetNumThreads(ChOMP::GetNumProcs(), 0, 1);

    // collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0); // not needed, already 0 when using
    collision::ChCollisionModel::SetDefaultSuggestedMargin(
        0.001);  // max inside penetration - if not enough stiffness in material: troubles
    // Use this value for an outward additional layer around meshes, that can improve
    // robustness of mesh-mesh collision detection (at the cost of having unnatural inflate effect)
    double sphere_swept_thickness = 0.008;

    // Create the surface material, containing information
    // about friction etc.
    // It is a SMC (penalty) material that we will assign to
    // all surfaces that might generate contacts.
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(6e4f);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.5f);
    mysurfmaterial->SetAdhesion(0);

    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << " 		ANCF Shell Contact   \n";
    GetLog() << "-----------------------------------------------------------\n\n";
    GetLog() << "-----------------------------------------------------------\n\n";

    // Adding the ground
    if (true) {
        auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(3, 3, 0.2, 8000, true, true, mysurfmaterial);
        mfloor->SetBodyFixed(true);
        mfloor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
        sys.Add(mfloor);
    }

    std::vector<int> BC_NODES;
    auto material = chrono_types::make_shared<ChMaterialShellANCF>(1000, 1e8, 0.3);
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    ChVector<> Center(-0.5, -0.5, 0.5);
    ChMatrix33<> rot_transform(0);
    rot_transform(0, 0) = 1;
    rot_transform(1, 1) = -1;
    rot_transform(2, 1) = -1;
    std::vector<double> NODE_AVE_AREA;

    // Import the mesh
    try {
        ChMeshFileLoader::ANCFShellFromGMFFile(my_mesh, GetChronoDataFile("fea/Plate.mesh").c_str(), material,
                                               NODE_AVE_AREA, BC_NODES, Center, rot_transform, 0.8, false, false);
    } catch (ChException myerr) {
        GetLog() << myerr.what();
        return 0;
    }

    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>(mysurfmaterial);
    my_mesh->AddContactSurface(mcontactsurf);
    mcontactsurf->AddFacesFromBoundary(sphere_swept_thickness);  // do this after my_mesh->AddContactSurface

    ////auto mcontactcloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mysurfmaterial);
    ////my_mesh->AddContactSurface(mcontactcloud);
    ////mcontactcloud->AddAllNodes(sphere_swept_thickness);

    ////auto TotalNumNodes = my_mesh->GetNnodes();
    auto TotalNumElements = my_mesh->GetNelements();

    for (unsigned int ele = 0; ele < TotalNumElements; ele++) {
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element = std::dynamic_pointer_cast<ChElementShellANCF_3423>(my_mesh->GetElement(ele));
        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, material);
        // Set other element properties
        element->SetAlphaDamp(0.08);  // Structural damping for this element
    }

    // Switch off mesh class gravity

    my_mesh->SetAutomaticGravity(addGravity);
    sys.Set_G_acc(ChVector<>(0, 0, -9.8));

    // Add the mesh to the system
    sys.Add(my_mesh);

    // Mark completion of system construction

    // FEA visualization
    auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_mesh->SetColorscaleMinMax(0.0, 3.0);
    vis_mesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(vis_mesh);

    ////auto vis_elem = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    ////vis_elem->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    ////vis_elem->SetWireframe(true);
    ////my_mesh->AddVisualShapeFEA(vis_elem);

    auto vis_ctct = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    vis_ctct->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    vis_ctct->SetWireframe(true);
    vis_ctct->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddVisualShapeFEA(vis_ctct);

    auto vis_nodes = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    vis_nodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_nodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_nodes->SetSymbolsThickness(0.008);
    my_mesh->AddVisualShapeFEA(vis_nodes);

    // Create the Irrlicht visualization system
    auto vsys = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vsys->AttachSystem(&sys);
    vsys->SetWindowSize(800, 600);
    vsys->SetWindowTitle("ANCF Contact");
    vsys->SetCameraVertical(CameraVerticalDir::Z);
    vsys->Initialize();
    vsys->AddLogo();
    vsys->AddSkyBox();
    vsys->AddTypicalLights();
    vsys->AddCamera(ChVector<>(0.25, -0.25, 0.5), ChVector<>(0, -0.5, 0.0));
    vsys->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
    vsys->EnableShadows();

    // ---------------
    // Simulation loop
    // ---------------

    ////auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    ////sys.SetSolver(mkl_solver);
    ////mkl_solver->LockSparsityPattern(true);
    ////sys.Update();

    // Setup solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(150);
    solver->SetTolerance(1e-8);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // Enable for better convergence if using Euler implicit linearized

    // HHT or EULER_IMPLICIT_LINEARIZED
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(200);
    mystepper->SetAbsTolerances(1e-06);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);
    mystepper->SetVerbose(false);
    ////sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise

    while (vsys->Run()) {
        vsys->BeginScene();
        vsys->Render();
        vsys->EndScene();

        ////std::cout << "Time t = " << sys.GetChTime() << "s \t";
        ////std::cout << "n contacts: " << sys.GetNcontacts() << "\t";
        ////std::cout << "pos.y = " << sampleNode->pos.y - y0 << "vs. " << -0.5 * 9.8 * pow(sys.GetChTime(), 2)
        ////          << "\n";
        sys.DoStepDynamics(time_step);
    }

    return 0;
}
