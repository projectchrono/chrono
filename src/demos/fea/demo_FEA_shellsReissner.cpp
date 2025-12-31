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
// FEA for shells of Reissner 6-field type
//
// =============================================================================

#include <vector>
#include <cmath>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"

#include "chrono/utils/ChUtils.h"

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

    // Select experiment
    std::cout << "FEA nonlinear static analysis. ";
    std::cout << "Options:" << std::endl;
    std::cout << "1  : EANS shell cantilever" << std::endl;
    std::cout << "2  : Slip angular plate" << std::endl;
    std::cout << "3  : Clamped half-cylinder" << std::endl;
    std::cout << "\nSelect option (1, 2, or 3): ";

    int experiment = 1;
    std::cin >> experiment;
    std::cout << std::endl;
    ChClampValue(experiment, 1, 3);

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "FEA_SHELLS";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Chrono physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    sys.Add(mesh);

    // sys.SetGravitationalAcceleration(VNULL); or
    mesh->SetAutomaticGravity(false);

    std::shared_ptr<ChNodeFEAxyzrot> nodePlotA;
    std::shared_ptr<ChNodeFEAxyzrot> nodePlotB;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodesLoad;

    ChFunctionInterp ref_X;
    ChFunctionInterp ref_Y;

    ChVector3d load_torque;
    ChVector3d load_force;

    // Set up selected experiment
    std::string experiment_name = "";
    ChVector3d camera_location;

    switch (experiment) {
        case 1: {
            // ---------------------
            // EANS shell cantilever
            // ---------------------

            experiment_name = "EANS shell cantilever";
            camera_location = ChVector3d(0.0, 6.0, -6.0);

            double rect_thickness = 0.10;
            double rect_L = 10.0;
            double rect_W = 1;

            // Create a material
            double rho = 0.0;
            double E = 1.2e6;
            double nu = 0.0;

            auto melasticity = chrono_types::make_shared<ChElasticityReissnerIsothropic>(E, nu, 1.0, 0.01);
            auto mat = chrono_types::make_shared<ChMaterialShellReissner>(melasticity);
            // In case you need also damping it would add...
            // auto mdamping = chrono_types::make_shared<ChDampingReissnerRayleigh>(melasticity,0.01);
            // auto mat = chrono_types::make_shared<ChMaterialShellReissner>(melasticity, nullptr, mdamping);
            mat->SetDensity(rho);

            // Create the nodes

            int nels_L = 12;
            int nels_W = 1;
            std::vector<std::shared_ptr<ChElementShellReissner4>> elarray(nels_L * nels_W);
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodearray((nels_L + 1) * (nels_W + 1));
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes_start(nels_W + 1);
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes_end(nels_W + 1);

            for (int il = 0; il <= nels_L; ++il) {
                for (int iw = 0; iw <= nels_W; ++iw) {
                    // Make nodes
                    ChVector3d nodepos(rect_L * ((double)il / (double)nels_L), 0,
                                       rect_W * ((double)iw / (double)nels_W));
                    ChQuaternion<> noderot(QUNIT);

                    ChFrame<> nodeframe(nodepos, noderot);

                    auto node = chrono_types::make_shared<ChNodeFEAxyzrot>(nodeframe);
                    mesh->AddNode(node);

                    node->GetInertia().fillDiagonal(0);  // approx
                    node->SetMass(0);

                    nodearray[il * (nels_W + 1) + iw] = node;

                    if (il == 0)
                        nodes_start[iw] = node;
                    if (il == nels_L)
                        nodes_end[iw] = node;

                    // Make elements
                    if (il > 0 && iw > 0) {
                        auto element = chrono_types::make_shared<ChElementShellReissner4>();
                        mesh->AddElement(element);

                        element->SetNodes(
                            nodearray[(il - 1) * (nels_W + 1) + (iw - 1)], nodearray[(il) * (nels_W + 1) + (iw - 1)],
                            nodearray[(il) * (nels_W + 1) + (iw)], nodearray[(il - 1) * (nels_W + 1) + (iw)]);

                        element->AddLayer(rect_thickness, 0 * CH_DEG_TO_RAD, mat);

                        elarray[(il - 1) * (nels_W) + (iw - 1)] = element;
                    }
                }
            }
            nodesLoad = nodes_end;
            nodePlotA = nodes_end.front();
            nodePlotB = nodes_end.back();

            for (auto mstartnode : nodes_start) {
                mstartnode->SetFixed(true);
            }

            // applied load
            // load_force = ChVector3d(200000,0, 20000);
            load_force = ChVector3d(0, 4, 0);
            // load_torque = ChVector3d(0, 0, 50*CH_PI/3.0);

            // reference solution for (0, 4, 0) shear to plot
            ref_Y.AddPoint(0.10, 1.309);
            ref_X.AddPoint(0.40, 0.103);
            ref_Y.AddPoint(0.20, 2.493);
            ref_X.AddPoint(0.80, 0.381);
            ref_Y.AddPoint(0.30, 3.488);
            ref_X.AddPoint(1.20, 0.763);
            ref_Y.AddPoint(0.40, 4.292);
            ref_X.AddPoint(1.60, 1.184);
            ref_Y.AddPoint(0.50, 4.933);
            ref_X.AddPoint(2.00, 1.604);
            ref_Y.AddPoint(0.60, 5.444);
            ref_X.AddPoint(2.40, 2.002);
            ref_Y.AddPoint(0.70, 5.855);
            ref_X.AddPoint(2.80, 2.370);
            ref_Y.AddPoint(0.80, 6.190);
            ref_X.AddPoint(3.20, 2.705);
            ref_Y.AddPoint(0.90, 6.467);
            ref_X.AddPoint(3.60, 3.010);
            ref_Y.AddPoint(1.00, 6.698);
            ref_X.AddPoint(4.00, 3.286);

            break;
        }

        case 2: {
            // ------------------
            // Slit angular plate
            // ------------------

            experiment_name = "Slit angular plate";
            camera_location = ChVector3d(0.0, 12.0, -30.0);

            double plate_thickness = 0.03;
            double plate_Ri = 6;
            double plate_Ro = 10;

            // Create a material
            double rho = 0.0;
            double E = 21e6;
            double nu = 0.0;

            auto mat = chrono_types::make_shared<ChMaterialShellReissnerIsothropic>(rho, E, nu, 1.0, 0.01);

            // Create the nodes

            int nels_U = 60;
            int nels_W = 10;
            double arc = CH_2PI * 1;
            std::vector<std::shared_ptr<ChElementShellReissner4>> elarray(nels_U * nels_W);
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodearray((nels_U + 1) * (nels_W + 1));
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes_start(nels_W + 1);
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes_end(nels_W + 1);

            for (int iu = 0; iu <= nels_U; ++iu) {
                for (int iw = 0; iw <= nels_W; ++iw) {
                    // Make nodes
                    double u = ((double)iu / (double)nels_U);
                    double w = ((double)iw / (double)nels_W);
                    ChVector3d nodepos((plate_Ri + (plate_Ro - plate_Ri) * w) * std::cos(u * arc), 0,
                                       (plate_Ri + (plate_Ro - plate_Ri) * w) * std::sin(u * arc));
                    ChQuaternion<> noderot(QUNIT);
                    ChFrame<> nodeframe(nodepos, noderot);

                    auto node = chrono_types::make_shared<ChNodeFEAxyzrot>(nodeframe);
                    mesh->AddNode(node);

                    node->GetInertia().fillDiagonal(0.0);
                    node->SetMass(0);

                    nodearray[iu * (nels_W + 1) + iw] = node;

                    if (iu == 0)
                        nodes_start[iw] = node;
                    if (iu == nels_U)
                        nodes_end[iw] = node;

                    // Make elements
                    if (iu > 0 && iw > 0) {
                        auto element = chrono_types::make_shared<ChElementShellReissner4>();
                        mesh->AddElement(element);

                        element->SetNodes(
                            nodearray[(iu) * (nels_W + 1) + (iw)], nodearray[(iu - 1) * (nels_W + 1) + (iw)],
                            nodearray[(iu - 1) * (nels_W + 1) + (iw - 1)], nodearray[(iu) * (nels_W + 1) + (iw - 1)]);

                        element->AddLayer(plate_thickness, 0 * CH_DEG_TO_RAD, mat);

                        elarray[(iu - 1) * (nels_W) + (iw - 1)] = element;
                    }
                }
            }

            nodesLoad = nodes_end;
            nodePlotA = nodes_end.front();
            nodePlotB = nodes_end.back();

            for (auto mstartnode : nodes_start) {
                mstartnode->SetFixed(true);
            }

            load_force = ChVector3d(0, 0.8 * 4, 0);
            load_torque = VNULL;

            // reference solution to plot
            ref_X.AddPoint(0.025, 1.305);
            ref_Y.AddPoint(0.025, 1.789);
            ref_X.AddPoint(0.10, 4.277);
            ref_Y.AddPoint(0.10, 5.876);
            ref_X.AddPoint(0.20, 6.725);
            ref_Y.AddPoint(0.20, 9.160);
            ref_X.AddPoint(0.30, 8.340);
            ref_Y.AddPoint(0.30, 11.213);
            ref_X.AddPoint(0.40, 9.529);
            ref_Y.AddPoint(0.40, 12.661);
            ref_X.AddPoint(0.50, 10.468);
            ref_Y.AddPoint(0.50, 13.768);
            ref_X.AddPoint(0.60, 11.257);
            ref_Y.AddPoint(0.60, 14.674);
            ref_X.AddPoint(0.70, 11.970);
            ref_Y.AddPoint(0.70, 15.469);
            ref_X.AddPoint(0.80, 12.642);
            ref_Y.AddPoint(0.80, 16.202);
            ref_X.AddPoint(0.9, 13.282);
            ref_Y.AddPoint(0.90, 16.886);
            ref_X.AddPoint(1.00, 13.891);
            ref_Y.AddPoint(1.00, 17.528);

            break;
        }

        case 3: {
            // ---------------------
            // Clamped half-cylinder
            // ---------------------

            experiment_name = "Clamped half-cylinder";
            camera_location = ChVector3d(3, 3, 6.5);

            double plate_thickness = 0.03;
            double plate_R = 1.016;
            double plate_L = 3.048;

            // Create a material
            double rho = 0.0;
            double E = 2.0685e7;
            double nu = 0.3;

            auto mat = chrono_types::make_shared<ChMaterialShellReissnerIsothropic>(rho, E, nu, 1.0, 0.01);

            // In case you want to test laminated shells, use this:
            auto mat_ortho = chrono_types::make_shared<ChMaterialShellReissnerOrthotropic>(
                rho, 2.0685e7, 0.517e7, 0.3, 0.795e7, 0.795e7, 0.795e7, 1.0, 0.01);

            // Create the nodes

            int nels_U = 32;
            int nels_W = 32;
            double arc = CH_PI;
            std::vector<std::shared_ptr<ChElementShellReissner4>> elarray(nels_U * nels_W);
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodearray((nels_U + 1) * (nels_W + 1));
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes_start(nels_W + 1);
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes_end(nels_W + 1);
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes_left(nels_U + 1);
            std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes_right(nels_U + 1);

            for (int iu = 0; iu <= nels_U; ++iu) {
                for (int iw = 0; iw <= nels_W; ++iw) {
                    // Make nodes
                    double u = ((double)iu / (double)nels_U);
                    double w = ((double)iw / (double)nels_W);
                    ChVector3d nodepos((plate_R)*std::cos(w * arc), (plate_R)*std::sin(w * arc), u * plate_L);
                    ChQuaternion<> noderot(QUNIT);
                    ChFrame<> nodeframe(nodepos, noderot);

                    auto node = chrono_types::make_shared<ChNodeFEAxyzrot>(nodeframe);
                    mesh->AddNode(node);

                    node->GetInertia().fillDiagonal(0.0);
                    node->SetMass(0.0);

                    nodearray[iu * (nels_W + 1) + iw] = node;

                    if (iu == 0)
                        nodes_start[iw] = node;
                    if (iu == nels_U)
                        nodes_end[iw] = node;
                    if (iw == 0)
                        nodes_left[iu] = node;
                    if (iw == nels_W)
                        nodes_right[iu] = node;

                    // Make elements
                    if (iu > 0 && iw > 0) {
                        auto element = chrono_types::make_shared<ChElementShellReissner4>();
                        mesh->AddElement(element);

                        element->SetNodes(
                            nodearray[(iu) * (nels_W + 1) + (iw)], nodearray[(iu - 1) * (nels_W + 1) + (iw)],
                            nodearray[(iu - 1) * (nels_W + 1) + (iw - 1)], nodearray[(iu) * (nels_W + 1) + (iw - 1)]);

                        element->AddLayer(plate_thickness, 0 * CH_DEG_TO_RAD, mat);
                        // In case you want to test laminated shells, do instead:
                        //  element->AddLayer(plate_thickness/3, 0 * CH_DEG_TO_RAD, mat_ortho);
                        //  element->AddLayer(plate_thickness/3, 90 * CH_DEG_TO_RAD, mat_ortho);
                        //  element->AddLayer(plate_thickness/3, 0 * CH_DEG_TO_RAD, mat_ortho);

                        elarray[(iu - 1) * (nels_W) + (iw - 1)] = element;
                    }
                }
            }

            nodesLoad.push_back(nodes_end[nodes_end.size() / 2]);
            nodePlotA = nodes_end[nodes_end.size() / 2];
            nodePlotB = nodes_end[nodes_end.size() / 2];

            for (auto node : nodes_start) {
                node->SetFixed(true);
            }

            auto truss = chrono_types::make_shared<ChBody>();
            truss->SetFixed(true);
            sys.Add(truss);
            for (auto node : nodes_left) {
                auto mlink = chrono_types::make_shared<ChLinkMateGeneric>(false, true, false, true, false, true);
                mlink->Initialize(node, truss, false, node->Frame(), node->Frame());
                sys.Add(mlink);
            }
            for (auto node : nodes_right) {
                auto mlink = chrono_types::make_shared<ChLinkMateGeneric>(false, true, false, true, false, true);
                mlink->Initialize(node, truss, false, node->Frame(), node->Frame());
                sys.Add(mlink);
            }

            load_force = ChVector3d(0, -2000, 0);
            load_torque = VNULL;

            // reference solution to plot
            ref_X.AddPoint(0.10, 1 - 0.16);
            ref_X.AddPoint(0.20, 1 - 0.37);
            ref_X.AddPoint(0.30, 1 - 0.66);
            ref_X.AddPoint(0.40, 1 - 1.13);
            ref_X.AddPoint(0.50, 1 - 1.32);
            ref_X.AddPoint(0.60, 1 - 1.44);
            ref_X.AddPoint(0.70, 1 - 1.52);
            ref_X.AddPoint(0.80, 1 - 1.60);
            ref_X.AddPoint(0.90, 1 - 1.66);
            ref_X.AddPoint(1.00, 1 - 1.71);

            break;
        }
    }

    std::cout << experiment_name << std::endl;

    // Visualization of the FEM mesh
    auto vis_mesh_1 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh_1->SetSmoothFaces(true);
    vis_mesh_1->SetWireframe(true);
    mesh->AddVisualShapeFEA(vis_mesh_1);

    auto vis_mesh_2 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_mesh_2->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_mesh_2->SetSymbolsThickness(0.05);
    vis_mesh_2->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(vis_mesh_2);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Reissner Shells - " + experiment_name,
                                         camera_location);

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sys.SetSolver(mkl_solver);

    // Change type of integrator
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    // sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    // sys.SetTimestepperType(ChTimestepper::Type::HHT);

    if (auto mint = std::dynamic_pointer_cast<ChTimestepperImplicit>(sys.GetTimestepper())) {
        mint->SetMaxIters(5);
        mint->SetAbsTolerances(1e-12, 1e-12);
    }

    double timestep = 0.1;
    sys.Setup();
    sys.Update(false);

    ChFunctionInterp rec_X;
    ChFunctionInterp rec_Y;

    double time = 0;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        // update load at end nodes, as simple lumped nodal forces
        double load_scale = time * 0.1;
        for (auto node : nodesLoad) {
            node->SetForce(load_force * load_scale * (1. / (double)nodesLoad.size()));
            node->SetTorque(load_torque * load_scale * (1. / (double)nodesLoad.size()));
        }

        sys.DoStaticNonlinear(3);
        // sys.DoStaticLinear();

        time += timestep;

        if (nodePlotA && nodePlotB) {
            rec_Y.AddPoint(load_scale, nodePlotA->GetPos().y());
            rec_X.AddPoint(load_scale, nodePlotB->GetPos().y());
        }

        vis->EndScene();
    }

    // Outputs results in a GNUPLOT plot
    std::string gplfilename = out_dir + "/shell_benchmark.gpl";
    ChGnuPlot gpl(gplfilename);
    gpl.SetGrid(false, 1, ChColor(0.8f, 0.8f, 0.8f));
    gpl.SetLabelX("Torque T/T0");
    gpl.SetLabelY("Tip displacement [m]");
    gpl << "set key left top";
    gpl.Plot(rec_Y, "W", " with lines lt -1 lc rgb'#00AAEE' ");
    gpl.Plot(rec_X, "-U", " with lines lt -1 lc rgb'#AA00EE' ");
    gpl.Plot(ref_Y, "W ref.", "pt 4 ps 1.4");
    gpl.Plot(ref_X, "-U ref.", "pt 4 ps 1.4");

    return 0;
}
