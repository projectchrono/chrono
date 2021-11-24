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
// Show how to use the ChModalAssembly to do substructuring
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_modal/ChModalAssembly.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_postprocess/ChGnuPlot.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;
using namespace irr;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "MODAL_ASSEMBLY";

double beam_Young = 100.e6;
double beam_density = 500;
double beam_wz = 0.3;
double beam_wy = 0.05;
double beam_Rmax = 6.2;
double beam_Rmin = 0.2;
double rad_s = 0.2;


void BuildAssembly( std::shared_ptr<ChModalAssembly> my_assembly,  
                    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& nodes  ) // just for returning some info on the assembly
{
    // BODY: the base & tower:

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(5, 2, 5, 200);
    my_body_A->SetBodyFixed(true);
    my_body_A->SetPos(ChVector<>(0, -10, 0));
    my_assembly->Add(my_body_A);

    // Attach a 'cylinder' shape asset for visualization of the tower.
    auto mtower = chrono_types::make_shared<ChCylinderShape>();
    mtower->GetCylinderGeometry().p1 = ChVector<>(0, 1, 0);
    mtower->GetCylinderGeometry().p2 = ChVector<>(0, 10, 0);
    mtower->GetCylinderGeometry().rad = 0.2;
    my_body_A->AddAsset(mtower);

    // BODY: the rotating hub:

    auto my_body_hub = chrono_types::make_shared<ChBodyEasyCylinder>(0.2, 0.5, 1000);
    my_body_hub->SetPos(ChVector<>(0, 0, 1));
    my_body_hub->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    my_assembly->Add(my_body_hub);


    // CONSTRAINT: the hub of the motor.

    // Since we are going to ue the DoStaticNonlinearRheonomic analysis, we must use
    // a motor that imposes a speed (so, motor imposing torques are not fit). Hence:

    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_motor->Initialize(my_body_hub, my_body_A, ChFrame<>(ChVector<>(0, 0, 1)));
    auto my_angle = chrono_types::make_shared<ChFunction_Ramp>(0, rad_s); // alpha_0, dalpha/dt (in rad/s)
    my_motor->SetAngleFunction(my_angle);
    my_assembly->AddInternal(my_motor);

    // MESH:  Create a FEM mesh, that is a container for groups
    //        of elements and their referenced nodes.

    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_assembly->AddInternal(my_mesh);

    my_mesh->SetAutomaticGravity(false);
    

    // BEAMS:

    

    // Create a simplified section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.
    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    msection->SetDensity(beam_density);
    msection->SetYoungModulus(beam_Young);
    msection->SetGwithPoissonRatio(0.31);
    msection->SetBeamRaleyghDampingBeta(0.00001);
    msection->SetBeamRaleyghDampingAlpha(0.001);
    msection->SetAsRectangularSection(beam_wy, beam_wz);
    msection->compute_inertia_damping_matrix = true; //*** not much different
    msection->compute_inertia_stiffness_matrix = true; //*** not much differen

    // This helps creating sequences of nodes and ChElementBeamEuler elements:
    ChBuilderBeamEuler builder;

    builder.BuildBeam(
        my_mesh,   // the mesh where to put the created nodes and elements
        msection,  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
        6,         // the number of ChElementBeamEuler to create
        ChVector<>(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
        ChVector<>(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
        ChVector<>(0, 0, 1)     // the 'Y' up direction of the section for the beam
    );

    nodes = builder.GetLastBeamNodes();

    // CONSTRAINT: connect root of blade to the hub. Use a motor, but with zero speed.

    auto my_root = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_root->Initialize(nodes.front(), my_body_hub, ChFrame<>(ChVector<>(0, 0.5, 1), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto my_angle2 = chrono_types::make_shared<ChFunction_Const>(0); // rad 
    my_root->SetMotorFunction(my_angle2);
    my_assembly->AddInternal(my_root);

    //
    // VISUALIZATION ASSETS:
    //

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_TX);
    mvisualizebeamA->SetColorscaleMinMax(-0.001, 600);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.2);
    mvisualizebeamC->SetSymbolsScale(0.1);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

}


int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Directory for output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    //
    // CREATE THE MODEL
    //

    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;

    // no gravity used here
    my_system.Set_G_acc(VNULL);
    
    // just for passing data for testing and output
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;

   // CREATE THE ASSEMBLY HERE

    /*
    auto my_assembly1 = chrono_types::make_shared<ChModalAssembly>();
    my_system.Add(my_assembly1);

    BuildAssembly(my_assembly1, nodes);
    */

    auto my_assembly2 = chrono_types::make_shared<ChModalAssembly>();
    my_system.Add(my_assembly2);

    BuildAssembly(my_assembly2, nodes);
    
    my_assembly2->DumpSubassemblyMatrices(true, true, true, true, (out_dir+"/dump").c_str());

    my_assembly2->ComputeModes(15);
    
    for (int i = 0; i < my_assembly2->Get_modes_frequencies().rows(); ++i)
        GetLog() << "Mode n." << i
                 << "  frequency [Hz]: " << my_assembly2->Get_modes_frequencies()(i) 
                 << "  damping ratio:" << my_assembly2->Get_modes_damping_ratios()(i) 
                 << "    Re=" << my_assembly2->Get_modes_eig()(i).real() << "  Im=" <<   my_assembly2->Get_modes_eig()(i).imag()
                 << "\n";

    //
    // VISUALIZATION
    //

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Sub assembly", core::dimension2d<u32>(1024, 768));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddLightWithShadow(irr::core::vector3df(20, 20, 20), irr::core::vector3df(0, 0, 0), 50, 5, 50, 55);
    application.AddLight(irr::core::vector3df(-20, -20, 0), 6, irr::video::SColorf(0.6f, 1.0f, 1.0f, 1.0f));
    application.AddLight(irr::core::vector3df(0, -20, -20), 6, irr::video::SColorf(0.6f, 1.0f, 1.0f, 1.0f));
    // application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(1.f, 0.3f, 10.f), core::vector3df(0.f, 0.f, 0.f));

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
    // ANALYSIS
    //

    // Some general settings: 
    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    my_system.SetSolver(mkl_solver);

    application.SetTimestep(0.01);
   
    // use HHT second order integrator (but slower)
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        // mystepper->SetVerbose(true);
        mystepper->SetStepControl(false);
    }

    // Set paused Irrlicht visualization just to show the deformed structure in 3D, until spacebar pressed:
    application.SetPaused(true);

    /*
    // Some plots after the static analysis:
    {
        ChVectorDynamic<> plotx(nodes.size());
        ChVectorDynamic<> ploty(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            plotx(i) = nodes[i]->GetPos().y();
            ploty(i) = nodes[i]->GetPos().z();
        }
        ChGnuPlot mplot_flap_displ((out_dir + "/flapwise_displ.dat").c_str());
        mplot_flap_displ.SetGrid();
        mplot_flap_displ.Plot(plotx, ploty, "Flapwise displacement", " with lines lt -1 lc rgb'#00AAEE'");

        ChVectorDynamic<> ploty_analytic(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            ploty(i) = nodes[i]->GetPos_dt().x();
            ploty_analytic(i) = -nodes[i]->GetPos().y() * rad_s;
        }
        ChGnuPlot mplot_edge_speed((out_dir + "/flapwise_speed.dat").c_str());
        mplot_edge_speed.SetGrid();
        mplot_edge_speed.Plot(plotx, ploty, "Edgewise speed", " with lines lt -1 lc rgb'#00AAEE'");
        mplot_edge_speed.Plot(plotx, ploty_analytic, "Expected analytic edgewise speed", " with lines lt -1 lc rgb'#AA00EE'");

        for (int i = 0; i < nodes.size(); ++i) {
            ploty(i) = nodes[i]->GetPos_dtdt().y();
            ploty_analytic(i) = -nodes[i]->GetPos().y() * rad_s * rad_s;
        }
        ChGnuPlot mplot_centeripetal_accel((out_dir + "/centripetal_acc.dat").c_str());
        mplot_centeripetal_accel.SetGrid();
        mplot_centeripetal_accel.Plot(plotx, ploty, "Centripetal acceleration", " with lines lt -1 lc rgb'#00AAEE'");
        mplot_centeripetal_accel.Plot(plotx, ploty_analytic, "Expected centripetal acceleration", " with lines lt -1 lc rgb'#AA00EE'");
    }
    */

    //my_system.SetDumpSolverMatrices(false);


    // 2- DYNAMICS

    std::vector<double> rec_t; 
    std::vector<double> rec_tip_edge_d;
    std::vector<double> rec_tip_flap_d;
    int elapsed_frames_start_dynamics = 200;
    
    // Need to show modes in 3D? No changes in your while() loop! In fact if you set application.SetModalShow(true), 
    // the application.BeginScene() in the simulation loop will overimpose the nth mode shape, displayed in 3D as a 
    // continuous oscillation; at the same time application.DoStep() won't advance dynamics. 
    // Note, in order to have this modal visualization  working, a ChModalAssembly must have been added to the ChSystem, 
    // where some modes must have been already computed.
    application.SetModalShow(true);
    application.SetModalSpeed(15);
    application.SetModalAmplitude(0.8);
    application.SetModalModeNumber(0);

    // Optional: this opens the GUI for changing the N of the mode shape via a slider in the Irrlicht view:
    application.SetShowInfos(true);
    application.SetInfosTab(1);

    while (application.GetDevice()->run()) {

        application.BeginScene();

        application.DrawAll();

        tools::drawGrid(application.GetVideoDriver(), 1, 1, 12, 12,
                             ChCoordsys<>(ChVector<>(0, 0, 0), CH_C_PI_2, VECT_Z),
                             video::SColor(100, 120, 120, 120), true);

        application.DoStep();

        // for plotting the tip oscillations, in the blade root coordinate:
        if (!application.GetPaused()) {
            rec_t.push_back(my_system.GetChTime());
            rec_tip_edge_d.push_back(nodes.front()->TransformPointParentToLocal(nodes.back()->GetPos()).z());
            rec_tip_flap_d.push_back(nodes.front()->TransformPointParentToLocal(nodes.back()->GetPos()).y());
        }

        /*
        // for simplified testing of the tilting control of the blade, with sudden jump:
        if (my_system.GetChTime() > 2){
            if (auto myfunct = std::dynamic_pointer_cast<ChFunction_Const>(my_root->GetMotorFunction()))
                myfunct->Set_yconst(0.4);
        }
        */

        // for starting automatically the dynamic simulation after n frames of showing statics:
        elapsed_frames_start_dynamics--;
        if (elapsed_frames_start_dynamics <= 0)
            application.SetPaused(false);

        application.EndScene();
    }

    /*
    ChGnuPlot mplot_tip_edge_d((out_dir + "/tip_edge_d.dat").c_str());
    mplot_tip_edge_d.SetGrid();
    mplot_tip_edge_d.Plot(rec_t, rec_tip_edge_d, "Edgewise displacement (t)", " with lines lt -1 lc rgb'#00AAEE'");

    ChGnuPlot mplot_tip_flap_d((out_dir + "/tip_flap_d.dat").c_str());
    mplot_tip_flap_d.SetGrid();
    mplot_tip_flap_d.Plot(rec_t, rec_tip_flap_d, "Flapwise displacement (t)", " with lines lt -1 lc rgb'#00AAEE'");
    */

    return 0;
}
