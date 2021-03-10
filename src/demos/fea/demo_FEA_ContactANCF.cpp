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
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLoadContactSurfaceMesh.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
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
using namespace irr;

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

    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"ANCF Contact", core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(+0.5f, 0.5f, 0.3f),  // camera location
                                 core::vector3df(0.0f, 0.f, 0.f));    // "look at" location
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_DISTANCES);

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
        my_system.Add(mfloor);
        auto masset_texture = chrono_types::make_shared<ChTexture>();
        masset_texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
        mfloor->AddAsset(masset_texture);
    }

    int TotalNumNodes, TotalNumElements;
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

    TotalNumNodes = my_mesh->GetNnodes();
    TotalNumElements = my_mesh->GetNelements();

    for (int ele = 0; ele < TotalNumElements; ele++) {
        auto element = chrono_types::make_shared<ChElementShellANCF>();
        element = std::dynamic_pointer_cast<ChElementShellANCF>(my_mesh->GetElement(ele));
        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, material);
        // Set other element properties
        element->SetAlphaDamp(0.08);   // Structural damping for this element
        element->SetGravityOn(false);  // gravitational forces
    }

    // Switch off mesh class gravity

    my_mesh->SetAutomaticGravity(addGravity);
    my_system.Set_G_acc(ChVector<>(0, 0, -9.8));

    // Add the mesh to the system
    my_system.Add(my_mesh);

    // Mark completion of system construction

    //    ////////////////////////////////////////
    //    // Options for visualization in irrlicht
    //    ////////////////////////////////////////
    auto mvisualizemesh = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddAsset(mvisualizemeshcoll);

    auto mvisualizemeshbeam = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshbeam->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemeshbeam->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemeshbeam->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemeshbeam);

    auto mvisualizemeshbeamnodes = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshbeamnodes->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshbeamnodes->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshbeamnodes->SetSymbolsThickness(0.0008);
    my_mesh->AddAsset(mvisualizemeshbeamnodes);
    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();

    // ---------------
    // Simulation loop
    // ---------------

    ////auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    ////my_system.SetSolver(mkl_solver);
    ////mkl_solver->LockSparsityPattern(true);
    ////my_system.Update();

    // Setup solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(150);
    solver->SetTolerance(1e-8);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // Enable for better convergence if using Euler implicit linearized

    // HHT or EULER_IMPLICIT_LINEARIZED
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(200);
    mystepper->SetAbsTolerances(1e-06);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);
    mystepper->SetVerbose(false);
    ////my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise

    application.SetTimestep(time_step);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        ////std::cout << "Time t = " << my_system.GetChTime() << "s \t";
        ////std::cout << "n contacts: " << my_system.GetNcontacts() << "\t";
        ////std::cout << "pos.y = " << sampleNode->pos.y - y0 << "vs. " << -0.5 * 9.8 * pow(my_system.GetChTime(), 2)
        ////          << "\n";
        double t_s = my_system.GetChTime();

        application.DoStep();
        application.EndScene();
    }

    return 0;
}
