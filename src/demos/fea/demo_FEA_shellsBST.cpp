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
// FEA for thin shells of Kirchhoff-Love type, with BST triangle finite elements
//
// =============================================================================

#include <vector>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"

#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "FEA_SHELLS";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create (if needed) output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    sys.Add(mesh);

    // sys.Set_G_acc(VNULL); // to remove gravity effect, or:
    // mesh->SetAutomaticGravity(false);

    std::shared_ptr<ChNodeFEAxyz> nodePlotA;
    std::shared_ptr<ChNodeFEAxyz> nodePlotB;
    std::vector<std::shared_ptr<ChNodeFEAxyz>> nodesLoad;

    ChFunction_Recorder ref_X;
    ChFunction_Recorder ref_Y;

    ChVector<> load_force;

    //
    // BENCHMARK 1
    //
    // Add a single BST element:
    //

    if (false)  // set as 'true' to execute this
    {
        // Create a material

        double density = 0.1;
        double E = 1.2e3;
        double nu = 0.3;
        double thickness = 0.001;

        auto elasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
        auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(elasticity);
        material->SetDensity(density);

        // Create nodes
        double L = 1.0;

        ChVector<> p0(0, 0, 0);
        ChVector<> p1(L, 0, 0);
        ChVector<> p2(0, L, 0);
        ChVector<> p3(L, L, 0);
        ChVector<> p4(-L, L, 0);
        ChVector<> p5(L, -L, 0);

        auto node0 = chrono_types::make_shared<ChNodeFEAxyz>(p0);
        auto node1 = chrono_types::make_shared<ChNodeFEAxyz>(p1);
        auto node2 = chrono_types::make_shared<ChNodeFEAxyz>(p2);
        auto node3 = chrono_types::make_shared<ChNodeFEAxyz>(p3);
        auto node4 = chrono_types::make_shared<ChNodeFEAxyz>(p4);
        auto node5 = chrono_types::make_shared<ChNodeFEAxyz>(p5);
        mesh->AddNode(node0);
        mesh->AddNode(node1);
        mesh->AddNode(node2);
        mesh->AddNode(node3);
        mesh->AddNode(node4);
        mesh->AddNode(node5);

        // Create element

        auto element = chrono_types::make_shared<ChElementShellBST>();
        mesh->AddElement(element);

        element->SetNodes(node0, node1, node2, nullptr, nullptr, nullptr);  // node3, node4, node5);

        element->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, material);

        // TEST
        sys.Setup();
        sys.Update();
        GetLog() << "BST initial: \n"
                 << "Area: " << element->area << "\n"
                 << "l0: " << element->l0 << "\n"
                 << "phi0: " << element->phi0 << "\n"
                 << "k0: " << element->k0 << "\n"
                 << "e0: " << element->e0 << "\n";

        node1->SetPos(node1->GetPos() + ChVector<>(0.1, 0, 0));

        sys.Update();
        ChVectorDynamic<double> Fi(element->GetNdofs());
        element->ComputeInternalForces(Fi);
        GetLog() << "BST updated: \n"
                 << "phi: " << element->phi << "\n"
                 << "k: " << element->k << "\n"
                 << "e: " << element->e << "\n"
                 << "m: " << element->m << "\n"
                 << "n: " << element->n << "\n";
        ChVector<> resultant = VNULL;
        GetLog() << "Fi= \n";
        for (int i = 0; i < Fi.size(); ++i) {
            if (i % 3 == 0) {
                GetLog() << "-------" << i / 3 << "\n";
                ChVector<> fi = Fi.segment(i, 3);
                resultant += fi;
            }
            GetLog() << Fi(i) << "\n";
        }
        GetLog() << "resultant: " << resultant << "\n";
        // system("pause");
    }

    //
    // BENCHMARK 2
    //
    // Add a rectangular mesh of BST elements:
    //

    std::shared_ptr<ChNodeFEAxyz> nodemonitor;
    std::shared_ptr<ChElementShellBST> elementmonitor;

    if (true)  // set as 'true' to execute this
    {
        // Create a material

        double density = 100;
        double E = 6e4;
        double nu = 0.0;
        double thickness = 0.01;

        auto elasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
        auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(elasticity);
        material->SetDensity(density);

        double L_x = 1;
        size_t nsections_x = 40;
        double L_z = 1;
        size_t nsections_z = 40;

        // Create nodes
        std::vector<std::shared_ptr<ChNodeFEAxyz>> nodes;  // for future loop when adding elements
        for (size_t iz = 0; iz <= nsections_z; ++iz) {
            for (size_t ix = 0; ix <= nsections_x; ++ix) {
                ChVector<> p(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z));

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

                if (iz == 0 && ix == 1)
                    elementmonitor = elementA;

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

                elementA->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, material);

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

                melementB->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, material);
            }
        }

        // fix some nodes
        for (int j = 0; j < 30; ++j) {
            for (int k = 0; k < 30; ++k) {
                nodes[j * (nsections_x + 1) + k]->SetFixed(true);
            }
        }
        /*
        nodes[0*(nsections_x+1)+1]->SetFixed(true);
        nodes[1*(nsections_x+1)+1]->SetFixed(true);

        nodes[0*(nsections_x+1)+nsections_x-1]->SetFixed(true);
        nodes[1*(nsections_x+1)+nsections_x-1]->SetFixed(true);

        nodes[0*(nsections_x+1)+nsections_x]->SetFixed(true);
        nodes[1*(nsections_x+1)+nsections_x]->SetFixed(true);
        */
    }

    //
    // BENCHMARK 3
    //
    // Load and create shells from a .obj file containing a triangle mesh surface
    //

    if (false) {
        double density = 100;
        double E = 6e5;
        double nu = 0.0;
        double thickness = 0.01;

        auto elasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
        auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(elasticity);
        material->SetDensity(density);

        ChMeshFileLoader::BSTShellFromObjFile(mesh, GetChronoDataFile("models/cube.obj").c_str(), material, thickness);
    }

    // Visualization of the FEM mesh.
    auto vis_shell_A = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_shell_A->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    vis_shell_A->SetWireframe(true);
    vis_shell_A->SetShellResolution(2);
    ////vis_shell_A->SetBackfaceCull(true);
    mesh->AddVisualShapeFEA(vis_shell_A);

    auto vis_shell_B = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_shell_B->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_shell_B->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_shell_B->SetSymbolsThickness(0.006);
    mesh->AddVisualShapeFEA(vis_shell_B);

    if (false) {
        // Create a contact material
        auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
        mat->SetYoungModulus(6e4f);
        mat->SetFriction(0.3f);
        mat->SetRestitution(0.5f);
        mat->SetAdhesion(0);

        // Add collision geometry to the FEA mesh
        // (a) contact surface
        auto contact_surf = chrono_types::make_shared<ChContactSurfaceMesh>(mat);
        mesh->AddContactSurface(contact_surf);
        contact_surf->AddFacesFromBoundary(0.01);
        // (b) contact points
        ////auto contact_cloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mat);
        ////mesh->AddContactSurface(contact_cloud);
        ////contact_cloud->AddAllNodes(0.01);

        // Create a fixed collision shape
        auto cylinder =
            chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, 0.1, 1.0, 1000, true, true, mat);
        cylinder->SetBodyFixed(true);
        cylinder->SetPos(ChVector<>(0.75, -0.25, 0.5));
        cylinder->SetRot(Q_from_AngZ(CH_C_PI_2));
        cylinder->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.4f, 0.4f));
        sys.AddBody(cylinder);
    }

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("Shells FEA test: triangle BST elements");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(1, 0.3, 1.3), ChVector<>(0.5, -0.3, 0.5));
    vis->AddLight(ChVector<>(2, 2, 0), 6, ChColor(0.6f, 0.6f, 0.6f));
    vis->AddLight(ChVector<>(0, -2, 2), 6, ChColor(0.6f, 0.6f, 0.6f));

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sys.SetSolver(mkl_solver);

    // auto gmres_solver = chrono_types::make_shared<ChSolverGMRES>();
    // gmres_solver->SetMaxIterations(50);
    // sys.SetSolver(gmres_solver);

    // Simulation loop
    double timestep = 0.005;
    sys.Setup();
    sys.Update();

    ChFunction_Recorder rec_X;
    ChFunction_Recorder rec_Y;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
    }

    return 0;
}
