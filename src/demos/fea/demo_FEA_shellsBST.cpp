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
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChMeshFileLoader.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_mkl/ChSolverMKL.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;
using namespace irr;

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
    ChSystemSMC my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // my_system.Set_G_acc(VNULL); // to remove gravity effect, or:
    // my_mesh->SetAutomaticGravity(false);

    std::shared_ptr<ChNodeFEAxyz> nodePlotA;
    std::shared_ptr<ChNodeFEAxyz> nodePlotB;
    std::vector<std::shared_ptr<ChNodeFEAxyz>> nodesLoad;

    ChFunction_Recorder ref_X;
    ChFunction_Recorder ref_Y;

    ChVector<> load_force;

    //
    // BENCHMARK n.1
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

        auto melasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
        auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(melasticity);
        material->SetDensity(density);

        // Create nodes
        double L = 1.0;

        ChVector<> p0(0, 0, 0);
        ChVector<> p1(L, 0, 0);
        ChVector<> p2(0, L, 0);
        ChVector<> p3(L, L, 0);
        ChVector<> p4(-L, L, 0);
        ChVector<> p5(L, -L, 0);

        auto mnode0 = chrono_types::make_shared<ChNodeFEAxyz>(p0);
        auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(p1);
        auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(p2);
        auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(p3);
        auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(p4);
        auto mnode5 = chrono_types::make_shared<ChNodeFEAxyz>(p5);
        my_mesh->AddNode(mnode0);
        my_mesh->AddNode(mnode1);
        my_mesh->AddNode(mnode2);
        my_mesh->AddNode(mnode3);
        my_mesh->AddNode(mnode4);
        my_mesh->AddNode(mnode5);

        // Create element

        auto melement = chrono_types::make_shared<ChElementShellBST>();
        my_mesh->AddElement(melement);

        melement->SetNodes(mnode0, mnode1, mnode2, nullptr, nullptr, nullptr);  // mnode3, mnode4, mnode5);

        melement->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, material);

        // TEST
        my_system.Setup();
        my_system.Update();
        GetLog() << "BST initial: \n"
                 << "Area: " << melement->area << "\n"
                 << "l0: " << melement->l0 << "\n"
                 << "phi0: " << melement->phi0 << "\n"
                 << "k0: " << melement->k0 << "\n"
                 << "e0: " << melement->e0 << "\n";

        mnode1->SetPos(mnode1->GetPos() + ChVector<>(0.1, 0, 0));

        my_system.Update();
        ChVectorDynamic<double> Fi(melement->GetNdofs());
        melement->ComputeInternalForces(Fi);
        GetLog() << "BST updated: \n"
                 << "phi: " << melement->phi << "\n"
                 << "k: " << melement->k << "\n"
                 << "e: " << melement->e << "\n"
                 << "m: " << melement->m << "\n"
                 << "n: " << melement->n << "\n";
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
    // BENCHMARK n.2
    //
    // Add a rectangular mesh of BST elements:
    //

    std::shared_ptr<ChNodeFEAxyz> mnodemonitor;
    std::shared_ptr<ChElementShellBST> melementmonitor;

    if (true)  // set as 'true' to execute this
    {
        // Create a material

        double density = 100;
        double E = 6e4;
        double nu = 0.0;
        double thickness = 0.01;

        auto melasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
        auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(melasticity);
        material->SetDensity(density);

        double L_x = 1;
        size_t nsections_x = 40;
        double L_z = 1;
        size_t nsections_z = 40;

        // Create nodes
        std::vector<std::shared_ptr<ChNodeFEAxyz>> mynodes;  // for future loop when adding elements
        for (size_t iz = 0; iz <= nsections_z; ++iz) {
            for (size_t ix = 0; ix <= nsections_x; ++ix) {
                ChVector<> p(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z));

                auto mnode = chrono_types::make_shared<ChNodeFEAxyz>(p);

                my_mesh->AddNode(mnode);

                mynodes.push_back(mnode);
            }
        }
        // Create elements
        for (size_t iz = 0; iz < nsections_z; ++iz) {
            for (size_t ix = 0; ix < nsections_x; ++ix) {
                auto melementA = chrono_types::make_shared<ChElementShellBST>();
                my_mesh->AddElement(melementA);

                if (iz == 0 && ix == 1)
                    melementmonitor = melementA;

                std::shared_ptr<ChNodeFEAxyz> boundary_1;
                std::shared_ptr<ChNodeFEAxyz> boundary_2;
                std::shared_ptr<ChNodeFEAxyz> boundary_3;

                boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1];
                if (ix > 0)
                    boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1];
                else
                    boundary_2 = nullptr;
                if (iz > 0)
                    boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1];
                else
                    boundary_3 = nullptr;

                melementA->SetNodes(mynodes[(iz) * (nsections_x + 1) + ix], mynodes[(iz) * (nsections_x + 1) + ix + 1],
                                    mynodes[(iz + 1) * (nsections_x + 1) + ix], boundary_1, boundary_2, boundary_3);

                melementA->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, material);

                auto melementB = chrono_types::make_shared<ChElementShellBST>();
                my_mesh->AddElement(melementB);

                boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix];
                if (ix < nsections_x - 1)
                    boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2];
                else
                    boundary_2 = nullptr;
                if (iz < nsections_z - 1)
                    boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix];
                else
                    boundary_3 = nullptr;

                melementB->SetNodes(mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                                    mynodes[(iz + 1) * (nsections_x + 1) + ix],
                                    mynodes[(iz) * (nsections_x + 1) + ix + 1], boundary_1, boundary_2, boundary_3);

                melementB->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, material);
            }
        }

        // fix some nodes
        for (int j = 0; j < 30; ++j) {
            for (int k = 0; k < 30; ++k) {
                mynodes[j * (nsections_x + 1) + k]->SetFixed(true);
            }
        }
        /*
        mynodes[0*(nsections_x+1)+1]->SetFixed(true);
        mynodes[1*(nsections_x+1)+1]->SetFixed(true);

        mynodes[0*(nsections_x+1)+nsections_x-1]->SetFixed(true);
        mynodes[1*(nsections_x+1)+nsections_x-1]->SetFixed(true);

        mynodes[0*(nsections_x+1)+nsections_x]->SetFixed(true);
        mynodes[1*(nsections_x+1)+nsections_x]->SetFixed(true);
        */
    }

    //
    // BENCHMARK n.3
    //
    // Load and create shells from a .obj file containing a triangle mesh surface
    //

    if (false) {
        double density = 100;
        double E = 6e5;
        double nu = 0.0;
        double thickness = 0.01;

        auto melasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
        auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(melasticity);
        material->SetDensity(density);

        ChMeshFileLoader::BSTShellFromObjFile(my_mesh, GetChronoDataFile("cube.obj").c_str(), material, thickness);
    }

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    auto mvisualizeshellA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizeshellA->SetSmoothFaces(true);
    // mvisualizeshellA->SetWireframe(true);
    mvisualizeshellA->SetShellResolution(2);
    // mvisualizeshellA->SetBackfaceCull(true);
    my_mesh->AddAsset(mvisualizeshellA);

    auto mvisualizeshellB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizeshellB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeshellB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizeshellB->SetSymbolsThickness(0.006);
    my_mesh->AddAsset(mvisualizeshellB);

    //
    // VISUALIZATION
    //

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Shells FEA test: triangle BST elements", core::dimension2d<u32>(1024, 768),
                         false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddLightWithShadow(irr::core::vector3df(2, 2, 2), irr::core::vector3df(0, 0, 0), 6, 0.2, 6, 50);
    application.AddLight(irr::core::vector3df(-2, -2, 0), 6, irr::video::SColorf(0.6f, 1.0f, 1.0f, 1.0f));
    application.AddLight(irr::core::vector3df(0, -2, -2), 6, irr::video::SColorf(0.6f, 1.0f, 1.0f, 1.0f));
    // application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(1.f, 0.3f, 1.3f), core::vector3df(0.5f, -0.3f, 0.5f));

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    application.AddShadowAll();
    //
    // THE SOFT-REAL-TIME CYCLE
    //

    // Change solver to MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
    mkl_solver->LockSparsityPattern(true);
    my_system.SetSolver(mkl_solver);

    // auto gmres_solver = chrono_types::make_shared<ChSolverGMRES>();
    // gmres_solver->SetMaxIterations(50);
    // my_system.SetSolver(gmres_solver);

    double timestep = 0.005;
    application.SetTimestep(timestep);
    my_system.Setup();
    my_system.Update();

    ChFunction_Recorder rec_X;
    ChFunction_Recorder rec_Y;

    double mtime = 0;

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
