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
// FEA nonlinear static analysis of 3D beams, including centrifugal effect.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_postprocess/ChGnuPlot.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;
using namespace irr;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "BEAM_ROTOR";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    double beam_Young = 100.e6;
    double beam_density = 400;
    double beam_wz = 0.3;
    double beam_wy = 0.02;
    double beam_Rmax = 6.2;
    double beam_Rmin = 0.2;
    double rad_s = 3;
    ChVector<> tip_abs_force(0,0,-36.4); // for uniform rotation use only z value
    bool   use_euler = true;
    bool   use_iga = false; 
    bool   use_timoshenko = false; 

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


    // BODY: the base & tower:

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(10,2,10,3000);
    my_body_A->SetBodyFixed(true);
    my_body_A->SetPos(ChVector<>(0, -10, 0));
    my_system.Add(my_body_A);

    // Attach a 'cylinder' shape asset for visualization of the tower.
    auto mtower = chrono_types::make_shared<ChCylinderShape>();
    mtower->GetCylinderGeometry().p1 = ChVector<>(0, 1, 0);
    mtower->GetCylinderGeometry().p2 = ChVector<>(0, 10, 0);
    mtower->GetCylinderGeometry().rad = 0.2;
    my_body_A->AddAsset(mtower);

    // BODY: the rotating hub:

    auto my_body_hub = chrono_types::make_shared<ChBodyEasyCylinder>(0.2,0.5,1000);
    my_body_hub->SetPos(ChVector<>(0, 0, 1));
    my_body_hub->SetRot(Q_from_AngAxis(CH_C_PI_2,VECT_X));
    my_system.Add(my_body_hub);


    // CONSTRAINT: the hub of the motor.
    
    // Since we are going to ue the DoStaticNonlinearRheonomic analysis, we must use
    // a motor that imposes a speed (so, motor imposing torques are not fit). Hence:

    if (false) {
        // WARNING! the ChLinkMotorRotationSpeed introduces an aux state that cannot be solved via static analysis functions!!!
        auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        my_motor->Initialize(my_body_hub, my_body_A, ChFrame<>(ChVector<>(0, 0, 1)));
        auto my_speed = chrono_types::make_shared<ChFunction_Const>(rad_s); // rad/s
        my_motor->SetSpeedFunction(my_speed);
        my_system.Add(my_motor);
    } 
    else {
        auto my_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        my_motor->Initialize(my_body_hub, my_body_A, ChFrame<>(ChVector<>(0, 0, 1)));
        auto my_angle = chrono_types::make_shared<ChFunction_Ramp>(0,rad_s); // alpha_0, dalpha/dt (in rad/s)
        my_motor->SetAngleFunction(my_angle);
        my_system.Add(my_motor);
    }

   
    // MESH:  Create a FEM mesh, that is a container for groups
    //        of elements and their referenced nodes.

    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_system.Add(my_mesh);

    // no gravity used here
    my_system.Set_G_acc(VNULL);
    my_mesh->SetAutomaticGravity(false);
    

    // BEAMS:
    

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;

    if (use_euler) {
        // Create a simplified section, i.e. thickness and material properties
        // for beams. This will be shared among some beams.
        auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

        
        msection->SetDensity(beam_density);
        msection->SetYoungModulus(beam_Young);
        msection->SetGwithPoissonRatio(0.31);
        msection->SetBeamRaleyghDampingBeta(0.0001);
        msection->SetBeamRaleyghDampingAlpha(0);
        msection->SetAsRectangularSection(beam_wy, beam_wz);
        msection->compute_inertia_damping_matrix = true; //*** not much different
        msection->compute_inertia_stiffness_matrix = true; //*** not much differen

        // This helps creating sequences of nodes and ChElementBeamEuler elements:
        ChBuilderBeamEuler builder;

        builder.BuildBeam(
            my_mesh,   // the mesh where to put the created nodes and elements
            msection,  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
            6,        // the number of ChElementBeamEuler to create
            ChVector<>(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
            ChVector<>(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
            ChVector<>(0, 0, 1)     // the 'Y' up direction of the section for the beam
        );
        
        for (auto el : builder.GetLastBeamElements())
           el->SetUseGeometricStiffness(true);  // default true, if false convergence is bad
        
        nodes = builder.GetLastBeamNodes();
    }
    if (use_iga) {
        auto msection = chrono_types::make_shared<ChBeamSectionCosseratEasyRectangular>(
            beam_wy,  // width of section in y direction
            beam_wz,  // width of section in z direction
            beam_Young,  // Young modulus
            beam_Young * 0.3,  // shear modulus
            beam_density            // density
            );

        ChBuilderBeamIGA builder;
        builder.BuildBeam(
            my_mesh,                   // the mesh to put the elements in
            msection,                  // section of the beam
            6,                 // number of sections (spans)
            ChVector<>(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
            ChVector<>(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
            ChVector<>(0, 0, 1),     // the 'Y' up direction of the section for the beam
            1);                    // order (3 = cubic, etc)

        nodes = builder.GetLastBeamNodes();
    }
    if (use_timoshenko) {
        double Izz = (1.0 / 12.0) * beam_wz * pow(beam_wy, 3);
        double Iyy = (1.0 / 12.0) * beam_wy * pow(beam_wz, 3);
        DampingCoefficients mcoeffs; mcoeffs.bt = mcoeffs.bx = mcoeffs.by = mcoeffs.bz = 0.001;
        auto msection = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>(
            beam_Young * beam_wy * beam_wz,
            (Izz + Iyy) * beam_Young * 0.3,
            Iyy * beam_Young,
            Izz * beam_Young,
            beam_Young * 0.3 * beam_wy * beam_wz,
            beam_Young * 0.3 * beam_wy * beam_wz,
            mcoeffs,
            0,
            0,
            0,
            0,
            0,
            beam_density * beam_wy * beam_wz,
            beam_density * Iyy,
            beam_density * Izz,
            0,
            0,
            0,
            0
            );
        // for visualization as rectangular section
        msection->SetDrawShape(chrono_types::make_shared<ChBeamSectionShapeRectangular>(beam_wy, beam_wz));

        auto mtaperedsection = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        mtaperedsection->SetSectionA(msection);
        mtaperedsection->SetSectionB(msection);

        ChBuilderBeamTaperedTimoshenko builder;
        builder.BuildBeam(
            my_mesh,                   // the mesh to put the elements in
            mtaperedsection,                  // section of the beam
            6,                 // number of sections (spans)
            ChVector<>(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
            ChVector<>(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
            ChVector<>(0, 0, 1)     // the 'Y' up direction of the section for the beam
            );                    // order (3 = cubic, etc)

        nodes = builder.GetLastBeamNodes();
    }


    // CONSTRAINT: connect root of blade to the hub. Use a motor, but with zero speed.

    auto my_root = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_root->Initialize(nodes.front(), my_body_hub, ChFrame<>(ChVector<>(0, 0.5, 1), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto my_angle = chrono_types::make_shared<ChFunction_Const>(0); // rad 
    my_root->SetMotorFunction(my_angle);
    my_system.Add(my_root);

    nodes.back()->SetForce(tip_abs_force);


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


    //
    // VISUALIZATION
    //

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Rotor with simplified blade: steady state statics & dynamics", core::dimension2d<u32>(1024, 768));

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


    // 1- STATICS

    //my_system.SetDumpSolverMatrices(true);

    // Perform nonlinear statics, with assigned speeds and accelerations (that generate inertial and gyroscopic loads)
    // as for a blade in steady-state rotation.
    // In order to provide speeds and accelerations, there are two ways:
    // - using a callback to update them at each iteration via    myanalysis->SetCallbackIterationBegin(mycallback);
    // - or letting the solver compute them from motors, via  myanalysis->SetAutomaticSpeedAndAccelerationComputation(true); 
    // The latter is limited in functionality, so for the moment let's use a callback that is called 
    // at each nonlinear statics iteration step:

    class MyCallback : public ChStaticNonLinearRheonomicAnalysis::IterationCallback {
    public:
        // Override this function of the callback to update speeds
        // and accelerations during the nonlinear static loop.
        void OnIterationBegin(const double load_scaling, const int iteration_n, ChStaticNonLinearRheonomicAnalysis* analysis) override {
            for (auto in : blade_nodes) {     
                // Set node speed and angular velocity, as moved by hub motor:
                in->SetPos_dt(ChVector<>(-in->GetPos().y() * blade_rad_s, 0, 0));
                in->SetWvel_par(ChVector<>(0, 0,  blade_rad_s));
                // Set also centripetal acceleration:
                in->SetPos_dtdt(ChVector<>(0, -in->GetPos().y() * blade_rad_s * blade_rad_s, 0));
            }
        }
        // some data used by the callback to make things simple
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> blade_nodes;
        double blade_rad_s;
    };

    auto mycallback = chrono_types::make_shared<MyCallback>();
    mycallback->blade_nodes = nodes;
    mycallback->blade_rad_s = rad_s;

    auto myanalysis = chrono_types::make_shared<ChStaticNonLinearRheonomicAnalysis>(my_system);
    myanalysis->SetMaxIterations(25);
    myanalysis->SetVerbose(true);
    myanalysis->SetCallbackIterationBegin(mycallback);
    // As an alternative to providing the callback, a much simpler option is to let the static solver
    // compute the speed and acceleration as inferred by the rheonomic joints, instead of the
    // previous line just use:
    //   myanalysis->SetAutomaticSpeedAndAccelerationComputation(true);
    // However this functionality is currently limited because it computes speeds/accelerations only at initial undeformed state.

    // EXECUTE NONLINEAR STATIC ANALYSIS HERE:
    my_system.DoStaticAnalysis(myanalysis);
    

    // Set paused Irrlicht visualization just to show the deformed structure in 3D, until spacebar pressed:
    application.SetPaused(true);

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

    /*
    // TRICK: force nodes to needed speed
    for (auto in : nodes) {
        in->SetPos_dt(ChVector<>(-in->GetPos().y() * rad_s, 0, 0));
        in->SetWvel_par(ChVector<>(0, 0,  rad_s));
    }
    */
    //my_system.SetDumpSolverMatrices(false);


    // 2- DYNAMICS

    std::vector<double> rec_t; 
    std::vector<double> rec_tip_edge_d;
    std::vector<double> rec_tip_flap_d;
    int elapsed_frames_start_dynamics = 200;

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

    ChGnuPlot mplot_tip_edge_d((out_dir + "/tip_edge_d.dat").c_str());
    mplot_tip_edge_d.SetGrid();
    mplot_tip_edge_d.Plot(rec_t, rec_tip_edge_d, "Edgewise displacement (t)", " with lines lt -1 lc rgb'#00AAEE'");

    ChGnuPlot mplot_tip_flap_d((out_dir + "/tip_flap_d.dat").c_str());
    mplot_tip_flap_d.SetGrid();
    mplot_tip_flap_d.Plot(rec_t, rec_tip_flap_d, "Flapwise displacement (t)", " with lines lt -1 lc rgb'#00AAEE'");


    return 0;
}
