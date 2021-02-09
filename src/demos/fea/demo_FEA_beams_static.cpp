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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "BEAM_STATICS";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Statics of beam", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    // application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));

    // Create a truss:
    auto my_body_A = chrono_types::make_shared<ChBody>();

    my_body_A->SetBodyFixed(true);
    my_system.AddBody(my_body_A);

    // Attach a 'box' shape asset for visualization.
    auto mboxtruss = chrono_types::make_shared<ChBoxShape>();
    mboxtruss->GetBoxGeometry().Pos = ChVector<>(-0.01, -0.2, -0.25);
    mboxtruss->GetBoxGeometry().SetLengths(ChVector<>(0.02, 0.5, 0.5));
    my_body_A->AddAsset(mboxtruss);

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
            double rot_rad = rot * CH_C_DEG_TO_RAD;

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
            msection->SetGwithPoissonRatio(0.31);
            msection->SetBeamRaleyghDamping(0.0);
            msection->SetAsRectangularSection(beam_wy, beam_wz);

            // This helps creating sequences of nodes and ChElementBeamEuler elements:
            ChBuilderBeamEuler builder;

            builder.BuildBeam(
                my_mesh,   // the mesh where to put the created nodes and elements
                msection,  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                10,        // the number of ChElementBeamEuler to create
                ChVector<>(0, nload * y_spacing, i * z_spacing),       // the 'A' point in space (beginning of beam)
                ChVector<>(beam_L, nload * y_spacing, i * z_spacing),  // the 'B' point in space (end of beam)
                ChVector<>(0, 1, 0)
                // ChVector<>(0, cos(rot_rad), sin(rot_rad))
                );  // the 'Y' up direction of the section for the beam

            // After having used BuildBeam(), you can retrieve the nodes used for the beam,
            // For example say you want to fix the A end and apply a force to the B end:
            builder.GetLastBeamNodes().front()->SetFixed(true);

            // builder.GetLastBeamNodes().back()->SetForce(ChVector<> (0, load,0));
            builder.GetLastBeamNodes().back()->SetForce(
                ChVector<>(0, loads(nload) * cos(rot_rad), loads(nload) * sin(rot_rad)));

            endnodes[nload].push_back(builder.GetLastBeamNodes().back());

            ++i;
        }
    }

    //
    // Final touches..
    //

    // note, this benchmark not using gravity.. use my_system.Set_G_acc(VNULL); or..
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MY);
    mvisualizebeamA->SetColorscaleMinMax(-0.001, 6);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.02);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // Use a solver that can handle stiffness matrices:
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    my_system.SetSolver(mkl_solver);

    application.SetTimestep(0.001);
    application.SetVideoframeSaveInterval(10);

    // Perform nonlinear statics
    my_system.DoStaticNonlinear(20, true);
    application.SetPaused(true);

    // Output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::string filename1 = out_dir + "/benchmark_CE_princeton_L1.dat";
    chrono::ChStreamOutAsciiFile file_out1(filename1.c_str());
    for (int i = 0; i < endnodes[0].size(); ++i) {
        double node_y = endnodes[0][i]->GetPos().y() - 0 * y_spacing;
        double node_z = endnodes[0][i]->GetPos().z() - i * z_spacing;
        double node_a =
            atan2(endnodes[0][i]->GetA().Get_A_Yaxis().y(), endnodes[0][i]->GetA().Get_A_Yaxis().z()) - CH_C_PI_2;
        GetLog() << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]\n";
        file_out1 << node_y << " " << node_z << " " << node_a << "\n";
    }

    std::string filename2 = out_dir + "/benchmark_CE_princeton_L2.dat";
    chrono::ChStreamOutAsciiFile file_out2(filename2.c_str());
    for (int i = 0; i < endnodes[1].size(); ++i) {
        double node_y = endnodes[1][i]->GetPos().y() - 1 * y_spacing;
        double node_z = endnodes[1][i]->GetPos().z() - i * z_spacing;
        double node_a =
            atan2(endnodes[1][i]->GetA().Get_A_Yaxis().y(), endnodes[1][i]->GetA().Get_A_Yaxis().z()) - CH_C_PI_2;
        GetLog() << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]\n";
        file_out2 << node_y << " " << node_z << " " << node_a << "\n";
    }

    std::string filename3 = out_dir + "/benchmark_CE_princeton_L3.dat";
    chrono::ChStreamOutAsciiFile file_out3(filename3.c_str());
    for (int i = 0; i < endnodes[2].size(); ++i) {
        double node_y = endnodes[2][i]->GetPos().y() - 2 * y_spacing;
        double node_z = endnodes[2][i]->GetPos().z() - i * z_spacing;
        double node_a =
            atan2(endnodes[2][i]->GetA().Get_A_Yaxis().y(), endnodes[2][i]->GetA().Get_A_Yaxis().z()) - CH_C_PI_2;
        GetLog() << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]\n";
        file_out3 << node_y << " " << node_z << " " << node_a << "\n";
    }

    // 3D view

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.05, 0.05, 10, 10,
                             ChCoordsys<>(ChVector<>(0.25, -0.20, 0), 0, VECT_Y), video::SColor(50, 120, 120, 120),
                             true);

        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.05, 0.05, 10, 10,
                             ChCoordsys<>(ChVector<>(0.25, -0.45, -0.25), CH_C_PI_2, VECT_X),
                             video::SColor(50, 120, 120, 120), true);

        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.05, 0.05, 10, 10,
                             ChCoordsys<>(ChVector<>(0.001, -0.20, -0.25), CH_C_PI_2, VECT_Y),
                             video::SColor(50, 160, 160, 160), true);

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
