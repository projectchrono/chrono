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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// FEA nonlinear static analysis of 3D beams.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    // Create a truss:
    auto my_body_A = chrono_types::make_shared<ChBody>();

    my_body_A->SetFixed(true);
    sys.AddBody(my_body_A);

    // Attach a 'box' shape asset for visualization.
    auto mboxtruss = chrono_types::make_shared<ChVisualShapeBox>(0.02, 0.5, 0.5);
    my_body_A->AddVisualShape(mboxtruss, ChFrame<>(ChVector3d(-0.01, -0.2, -0.25)));

    // Create a FEM mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    double rotstep = 15;
    double rotmax = 90;

    ChVectorN<double, 3> loads;
    loads(0) = -4.448;
    loads(1) = -8.896;
    loads(2) = -13.345;

    double z_spacing = -0.07;
    double y_spacing = -0.14;

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> endnodes[3];
    for (int nload = 0; nload < 3; ++nload) {
        int i = 0;
        for (double rot = 0; rot <= rotmax; rot += rotstep) {
            double rot_rad = rot * CH_DEG_TO_RAD;

            //
            // Add some EULER-BERNOULLI BEAMS:
            //

            // Create a section, i.e. thickness and material properties
            // for beams. This will be shared among some beams.
            auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

            double beam_wz = 0.0032024;  // 3.175;
            double beam_wy = 0.01237;    // 12.7;
            double beam_L = 0.508;
            msection->SetDensity(2700);
            msection->SetYoungModulus(71.7e9);
            msection->SetShearModulusFromPoisson(0.31);
            msection->SetRayleighDamping(0.0);
            msection->SetAsRectangularSection(beam_wy, beam_wz);

            // This helps creating sequences of nodes and ChElementBeamEuler elements:
            ChBuilderBeamEuler builder;

            builder.BuildBeam(
                my_mesh,   // the mesh where to put the created nodes and elements
                msection,  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                10,        // the number of ChElementBeamEuler to create
                ChVector3d(0, nload * y_spacing, i * z_spacing),       // the 'A' point in space (beginning of beam)
                ChVector3d(beam_L, nload * y_spacing, i * z_spacing),  // the 'B' point in space (end of beam)
                ChVector3d(0, 1, 0)
                // ChVector3d(0, std::cos(rot_rad), std::sin(rot_rad))
            );  // the 'Y' up direction of the section for the beam

            // After having used BuildBeam(), you can retrieve the nodes used for the beam,
            // For example say you want to fix the A end and apply a force to the B end:
            builder.GetLastBeamNodes().front()->SetFixed(true);

            // builder.GetLastBeamNodes().back()->SetForce(ChVector3d (0, load,0));
            builder.GetLastBeamNodes().back()->SetForce(
                ChVector3d(0, loads(nload) * std::cos(rot_rad), loads(nload) * std::sin(rot_rad)));

            endnodes[nload].push_back(builder.GetLastBeamNodes().back());

            ++i;
        }
    }

    //
    // Final touches..
    //

    // note, this benchmark not using gravity.. use sys.SetGravitationalAcceleration(VNULL); or..
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChVisualShapeTriangleMesh).
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MY);
    mvisualizebeamA->SetColorscaleMinMax(-0.001, 6);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.02);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Statics of beam");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0.0, 0.6, -1.0));
    vis->AttachSystem(&sys);

    // Use a solver that can handle stiffness matrices:
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // Perform nonlinear statics
    sys.DoStaticNonlinear(20, true);

    // Output data
    const std::string out_dir = GetChronoOutputPath() + "BEAM_STATICS";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::ofstream file_out1(out_dir + "/benchmark_CE_princeton_L1.dat");
    for (int i = 0; i < endnodes[0].size(); ++i) {
        double node_y = endnodes[0][i]->GetPos().y() - 0 * y_spacing;
        double node_z = endnodes[0][i]->GetPos().z() - i * z_spacing;
        double node_a =
            std::atan2(endnodes[0][i]->GetRotMat().GetAxisY().y(), endnodes[0][i]->GetRotMat().GetAxisY().z()) -
            CH_PI_2;
        std::cout << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]"
                  << std::endl;
        file_out1 << node_y << " " << node_z << " " << node_a << std::endl;
    }

    std::ofstream file_out2(out_dir + "/benchmark_CE_princeton_L2.dat");
    for (int i = 0; i < endnodes[1].size(); ++i) {
        double node_y = endnodes[1][i]->GetPos().y() - 1 * y_spacing;
        double node_z = endnodes[1][i]->GetPos().z() - i * z_spacing;
        double node_a =
            std::atan2(endnodes[1][i]->GetRotMat().GetAxisY().y(), endnodes[1][i]->GetRotMat().GetAxisY().z()) -
            CH_PI_2;
        std::cout << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]"
                  << std::endl;
        file_out2 << node_y << " " << node_z << " " << node_a << std::endl;
    }

    std::ofstream file_out3(out_dir + "/benchmark_CE_princeton_L3.dat");
    for (int i = 0; i < endnodes[2].size(); ++i) {
        double node_y = endnodes[2][i]->GetPos().y() - 2 * y_spacing;
        double node_z = endnodes[2][i]->GetPos().z() - i * z_spacing;
        double node_a =
            std::atan2(endnodes[2][i]->GetRotMat().GetAxisY().y(), endnodes[2][i]->GetRotMat().GetAxisY().z()) -
            CH_PI_2;
        std::cout << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]"
                  << std::endl;
        file_out3 << node_y << " " << node_z << " " << node_a << std::endl;
    }

    // 3D view

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        tools::drawGrid(vis.get(), 0.05, 0.05, 10, 10, ChCoordsys<>(ChVector3d(0.25, -0.20, 0), 0, VECT_Y),
                        ChColor(0.3f, 0.3f, 0.3f), true);

        tools::drawGrid(vis.get(), 0.05, 0.05, 10, 10, ChCoordsys<>(ChVector3d(0.25, -0.45, -0.25), CH_PI_2, VECT_X),
                        ChColor(0.3f, 0.3f, 0.3f), true);

        tools::drawGrid(vis.get(), 0.05, 0.05, 10, 10, ChCoordsys<>(ChVector3d(0.001, -0.20, -0.25), CH_PI_2, VECT_Y),
                        ChColor(0.3f, 0.3f, 0.3f), true);

        sys.DoStepDynamics(0.001);

        vis->EndScene();
    }

    return 0;
}
