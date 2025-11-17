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
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtils.h"

#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::postprocess;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Select demo
    std::cout << "Demo options:" << std::endl;
    std::cout << "1  : single BSTshell element" << std::endl;
    std::cout << "2  : rectangular mesh of BSTshell elements" << std::endl;
    std::cout << "3  : mesh of BSTshell elements intialized from OBJ file" << std::endl;
    std::cout << "\nSelect option (1, 2, or 3): ";

    int demo = 1;
    std::cin >> demo;
    std::cout << std::endl;
    ChClampValue(demo, 1, 3);

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "FEA_SHELLS";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Chrono physical system
    ChSystemSMC sys;

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    sys.Add(mesh);

    // sys.SetGravitationalAcceleration(VNULL); // to remove gravity effect, or:
    // mesh->SetAutomaticGravity(false);

    std::shared_ptr<ChNodeFEAxyz> nodePlotA;
    std::shared_ptr<ChNodeFEAxyz> nodePlotB;
    std::vector<std::shared_ptr<ChNodeFEAxyz>> nodesLoad;

    ChFunctionInterp ref_X;
    ChFunctionInterp ref_Y;

    ChVector3d load_force;

    // BENCHMARK 1
    // Add a single BST element:

    if (demo == 1) {
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

        ChVector3d p0(0, 0, 0);
        ChVector3d p1(L, 0, 0);
        ChVector3d p2(0, L, 0);
        ChVector3d p3(L, L, 0);
        ChVector3d p4(-L, L, 0);
        ChVector3d p5(L, -L, 0);

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

        element->AddLayer(thickness, 0 * CH_DEG_TO_RAD, material);

        // TEST
        sys.Setup();
        sys.Update(false);
        std::cout << "BST initial: \n"
                  << "Area: " << element->area << "\n"
                  << "l0: " << element->l0 << "\n"
                  << "phi0: " << element->phi0 << "\n"
                  << "k0: " << element->k0 << "\n"
                  << "e0: " << element->e0 << "\n";

        node1->SetPos(node1->GetPos() + ChVector3d(0.1, 0, 0));
        node1->SetFixed(true);

        sys.Update(false);
        ChVectorDynamic<double> Fi(element->GetNumCoordsPosLevel());
        element->ComputeInternalForces(Fi);
        std::cout << "BST updated: \n"
                  << "phi: " << element->phi << "\n"
                  << "k: " << element->k << "\n"
                  << "e: " << element->e << "\n"
                  << "m: " << element->m << "\n"
                  << "n: " << element->n << "\n";
        ChVector3d resultant = VNULL;
        std::cout << "Fi= \n";
        for (int i = 0; i < Fi.size(); ++i) {
            if (i % 3 == 0) {
                std::cout << "-------" << i / 3 << "\n";
                ChVector3d fi = Fi.segment(i, 3);
                resultant += fi;
            }
            std::cout << Fi(i) << "\n";
        }
        std::cout << "resultant: " << resultant << "\n";
        // system("pause");
    }

    // BENCHMARK 2
    // Add a rectangular mesh of BST elements:

    std::shared_ptr<ChNodeFEAxyz> nodemonitor;
    std::shared_ptr<ChElementShellBST> elementmonitor;

    if (demo == 2) {
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
                ChVector3d p(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z));

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

                elementA->AddLayer(thickness, 0 * CH_DEG_TO_RAD, material);

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

                melementB->AddLayer(thickness, 0 * CH_DEG_TO_RAD, material);
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

    // BENCHMARK 3
    // Load and create shells from a .obj file containing a triangle mesh surface

    if (demo == 3) {
        double density = 100;
        double E = 6e5;
        double nu = 0.0;
        double thickness = 0.01;

        auto elasticity = chrono_types::make_shared<ChElasticityKirchhoffIsothropic>(E, nu);
        auto material = chrono_types::make_shared<ChMaterialShellKirchhoff>(elasticity);
        material->SetDensity(density);

        ChMeshFileLoader::BSTShellFromObjFile(mesh, GetChronoDataFile("models/sphere.obj").c_str(), material,
                                              thickness);

        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(0)))
            mnode->SetFixed(true);
    }

    // Visualization of the FEM mesh.
    auto vis_shell_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_shell_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    vis_shell_mesh->SetWireframe(true);
    vis_shell_mesh->SetShellResolution(2);
    ////vis_shell_mesh->SetBackfaceCull(true);
    mesh->AddVisualShapeFEA(vis_shell_mesh);

    auto vis_shell_speed = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_shell_speed->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_shell_speed->SetColormapRange(0.0, 7.5);
    vis_shell_speed->SetWireframe(false);
    vis_shell_speed->SetShellResolution(3);
    mesh->AddVisualShapeFEA(vis_shell_speed);

    auto vis_shell_nodes = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_shell_nodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_shell_nodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_shell_nodes->SetSymbolsThickness(0.006);
    mesh->AddVisualShapeFEA(vis_shell_nodes);

    if (false) {
        // Create a contact material
        auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
        mat->SetYoungModulus(6e4f);
        mat->SetFriction(0.3f);
        mat->SetRestitution(0.5f);
        mat->SetAdhesion(0);

        // Add collision geometry to the FEA mesh
        // (a) contact surface
        auto contact_surf = chrono_types::make_shared<ChContactSurfaceMesh>(mat);
        contact_surf->AddFacesFromBoundary(*mesh, 0.01);
        mesh->AddContactSurface(contact_surf);
        // (b) contact points
        ////auto contact_cloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mat);
        ////contact_cloud->AddAllNodes(*mesh, 0.01);
        ////mesh->AddContactSurface(contact_cloud);

        // Create a fixed collision shape
        auto cylinder = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.1, 1.0, 1000, true, true, mat);
        cylinder->SetFixed(true);
        cylinder->SetPos(ChVector3d(0.75, -0.25, 0.5));
        cylinder->SetRot(QuatFromAngleZ(CH_PI_2));
        cylinder->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.4f, 0.4f));
        sys.AddBody(cylinder);
    }

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "BST triangle shell",
                                         ChVector3d(1, 0.3, 1.3), ChVector3d(0.5, -0.3, 0.5));

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
    sys.Update(false);

    ChFunctionInterp rec_X;
    ChFunctionInterp rec_Y;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
    }

    // EXPLICIT INTEGRATION
    //
    // Explicit integration is an alternative to implicit integration. If you want to test
    // the explicit integration, just delete the previous while(){...} loop and uncomment the following piece of code.
    //
    // As you may know, explicit integration does not have to solve systems with stiffness or damping matrices at each
    // time step, so each time step has less CPU overhead, but this comes at a cost: very short time steps must
    // be used otherwise the integration will diverge - especially if the system has high stiffness and/or low masses.
    // For the case of the falling cloth, we succesfully tested
    //   ChTimestepperEulerExplicitIIorder  timestep = 0.00002  (not suggested, better use higher order)
    //   ChTimestepperHeun              timestep = 0.0001   (Heun is like a 2nd order Runge Kutta)
    //   ChTimestepperRungeKutta    timestep = 0.0005   (the famous 4th order Runge Kutta, of course slower)
    //
    // You will see that the explicit integrator does not introduce numerical damping unlike implicit integrators,
    // so the motion will be more oscillatory and undamped, thus amplificating the risk of divergence (if you add �
    // structural damping, this might help with stability, by the way)
    //
    // In explicit integrators, you can optionally enable mass lumping via  SetDiagonalLumpingON() , just remember:
    // - this helps reducing CPU overhead because it does not lead to linear systems
    // - not all finite elements/bodies support this: nodes with non-diagonal inertias lead to approximation in lumping
    // - to avoid linear systems, this option also enables "constraints by penalty". Constraints, if any,
    //   will turn into penalty forces. Penalty factors can be set as optional parameters in SetDiagonalLumpingON(..)
    //   It is not the case of this demo, but if you add constraints, you'll see that they will be satisfied
    //   approximately with some oscillatory clearance. The higher the penalty, the smaller the amplitude of such
    //   clearances, but the higher the risk of divergence.
    //
    // Final tip: if the time step is extremely small, it is not worth while rendering all time steps, in some cases the
    // video rendering could become the real bottleneck.
    /*
    auto explicit_timestepper = chrono_types::make_shared<ChTimestepperHeun>(&sys);
    explicit_timestepper->SetDiagonalLumpingON(); // use diagonal mass lumping, skip linear systems completely
    sys.SetTimestepper(explicit_timestepper);
    timestep = 0.0001;

    while (vsys->Run()) {
        vsys->BeginScene();
        vsys->Render();
        vsys->EndScene();

        // Tip: draw scene only each N steps of the explicit integrator because
        // explicit integration requires many small timesteps, each with low CPU overhead, though.
        // So the video rendering could become the real bottleneck if redrawing each time step.
        for (int i = 0; i<5; ++i)
            sys.DoStepDynamics(timestep);
    }
    */

    return 0;
}
