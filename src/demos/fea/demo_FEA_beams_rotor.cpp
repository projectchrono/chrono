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
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_postprocess/ChGnuPlot.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    double beam_Young = 100.e6;
    double beam_density = 400;
    double beam_wz = 0.3;
    double beam_wy = 0.02;
    double beam_Rmax = 6.2;
    double beam_Rmin = 0.2;
    double rad_s = 3;
    ChVector3d tip_abs_force(0, 0, -36.4);  // for uniform rotation use only z value
    bool use_euler = true;
    bool use_iga = false;
    bool use_timoshenko = false;

    // Directory for output data
    const std::string out_dir = GetChronoOutputPath() + "BEAM_ROTOR";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    //
    // CREATE THE MODEL
    //

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    // BODY: the base & tower:

    auto my_body_A = chrono_types::make_shared<ChBodyEasyBox>(10, 2, 10, 3000);
    my_body_A->SetFixed(true);
    my_body_A->SetPos(ChVector3d(0, -10, 0));
    sys.Add(my_body_A);

    // Attach a 'cylinder' shape asset for visualization of the tower.
    auto mtower = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 9.0);
    my_body_A->AddVisualShape(mtower, ChFrame<>(ChVector3d(0, 5.5, 0), QuatFromAngleX(CH_PI_2)));

    // BODY: the rotating hub:

    auto my_body_hub = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.2, 0.5, 1000);
    my_body_hub->SetPos(ChVector3d(0, 0, 1));
    my_body_hub->SetRot(QuatFromAngleX(CH_PI_2));
    sys.Add(my_body_hub);

    // CONSTRAINT: the hub of the motor.

    // Since we are going to ue the DoStaticNonlinearRheonomic analysis, we must use
    // a motor that imposes a speed (so, motor imposing torques are not fit). Hence:

    if (false) {
        // WARNING! the ChLinkMotorRotationSpeed introduces an aux state that cannot be solved via static analysis
        // functions!!!
        auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        my_motor->Initialize(my_body_hub, my_body_A, ChFrame<>(ChVector3d(0, 0, 1)));
        auto my_speed = chrono_types::make_shared<ChFunctionConst>(rad_s);  // rad/s
        my_motor->SetSpeedFunction(my_speed);
        sys.Add(my_motor);
    } else {
        auto my_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        my_motor->Initialize(my_body_hub, my_body_A, ChFrame<>(ChVector3d(0, 0, 1)));
        auto my_angle = chrono_types::make_shared<ChFunctionRamp>(0, rad_s);  // alpha_0, dalpha/dt (in rad/s)
        my_motor->SetAngleFunction(my_angle);
        sys.Add(my_motor);
    }

    // MESH:  Create a FEM mesh, that is a container for groups
    //        of elements and their referenced nodes.

    auto my_mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(my_mesh);

    // no gravity used here
    sys.SetGravitationalAcceleration(VNULL);
    my_mesh->SetAutomaticGravity(false);

    // BEAMS:

    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;

    if (use_euler) {
        // Create a simplified section, i.e. thickness and material properties
        // for beams. This will be shared among some beams.
        auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

        msection->SetDensity(beam_density);
        msection->SetYoungModulus(beam_Young);
        msection->SetShearModulusFromPoisson(0.31);
        msection->SetRayleighDampingBeta(0 * 0.00001);
        msection->SetRayleighDampingAlpha(0 * 0.001);
        msection->SetAsRectangularSection(beam_wy, beam_wz);
        msection->compute_inertia_damping_matrix = true;    //// NOTE: not much different
        msection->compute_inertia_stiffness_matrix = true;  //// NOTE: not much different

        // This helps creating sequences of nodes and ChElementBeamEuler elements:
        ChBuilderBeamEuler builder;

        builder.BuildBeam(my_mesh,   // the mesh where to put the created nodes and elements
                          msection,  // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                          6,         // the number of ChElementBeamEuler to create
                          ChVector3d(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
                          ChVector3d(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
                          ChVector3d(0, 0, 1)           // the 'Y' up direction of the section for the beam
        );

        for (auto el : builder.GetLastBeamElements())
            el->SetUseGeometricStiffness(true);  // default true, if false convergence is bad

        sys.SetNumThreads(1);  //// TODO: fix race conditions in num diff
        // for (auto el : builder.GetLastBeamElements())
        //    el->use_numerical_diff_for_KR = true;

        nodes = builder.GetLastBeamNodes();
    }
    if (use_iga) {
        auto msection =
            chrono_types::make_shared<ChBeamSectionCosseratEasyRectangular>(beam_wy,  // width of section in y direction
                                                                            beam_wz,  // width of section in z direction
                                                                            beam_Young,        // Young modulus
                                                                            beam_Young * 0.3,  // shear modulus
                                                                            beam_density       // density
            );

        ChBuilderBeamIGA builder;
        builder.BuildBeam(my_mesh,                      // the mesh to put the elements in
                          msection,                     // section of the beam
                          6,                            // number of sections (spans)
                          ChVector3d(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
                          ChVector3d(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
                          ChVector3d(0, 0, 1),          // the 'Y' up direction of the section for the beam
                          1);                           // order (3 = cubic, etc)

        nodes = builder.GetLastBeamNodes();
    }
    if (use_timoshenko) {
        double Izz = (1.0 / 12.0) * beam_wz * std::pow(beam_wy, 3);
        double Iyy = (1.0 / 12.0) * beam_wy * std::pow(beam_wz, 3);
        DampingCoefficients mcoeffs;
        mcoeffs.bt = mcoeffs.bx = mcoeffs.by = mcoeffs.bz = 0.001;
        auto msection = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>(
            beam_Young * beam_wy * beam_wz, (Izz + Iyy) * beam_Young * 0.3, Iyy * beam_Young, Izz * beam_Young,
            beam_Young * 0.3 * beam_wy * beam_wz, beam_Young * 0.3 * beam_wy * beam_wz, mcoeffs, 0, 0, 0, 0, 0,
            beam_density * beam_wy * beam_wz, beam_density * Iyy, beam_density * Izz, 0, 0, 0, 0);
        // for visualization as rectangular section
        msection->SetDrawShape(chrono_types::make_shared<ChBeamSectionShapeRectangular>(beam_wy, beam_wz));

        auto mtaperedsection = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
        mtaperedsection->SetSectionA(msection);
        mtaperedsection->SetSectionB(msection);

        ChBuilderBeamTaperedTimoshenko builder;
        builder.BuildBeam(my_mesh,                      // the mesh to put the elements in
                          mtaperedsection,              // section of the beam
                          6,                            // number of sections (spans)
                          ChVector3d(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
                          ChVector3d(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
                          ChVector3d(0, 0, 1)           // the 'Y' up direction of the section for the beam
        );                                              // order (3 = cubic, etc)

        nodes = builder.GetLastBeamNodes();
    }

    // CONSTRAINT: connect root of blade to the hub. Use a motor, but with zero speed.

    auto my_root = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_root->Initialize(nodes.front(), my_body_hub, ChFrame<>(ChVector3d(0, 0.5, 1), QuatFromAngleX(CH_PI_2)));
    auto my_angle = chrono_types::make_shared<ChFunctionConst>(0);  // rad
    my_root->SetMotorFunction(my_angle);
    sys.Add(my_root);

    nodes.back()->SetForce(tip_abs_force);

    //
    // VISUALIZATION ASSETS:
    //

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_TX);
    mvisualizebeamA->SetColorscaleMinMax(-0.001, 600);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.2);
    mvisualizebeamC->SetSymbolsScale(0.1);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("Rotor with simplified blade: steady state statics & dynamics");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddLightWithShadow(ChVector3d(20, 20, 20), ChVector3d(0, 0, 0), 50, 5, 50, 55);
    vis->AddLight(ChVector3d(-20, -20, 0), 6, ChColor(0.6f, 1.0f, 1.0f));
    vis->AddLight(ChVector3d(0, -20, -20), 6, ChColor(0.6f, 1.0f, 1.0f));
    vis->AddCamera(ChVector3d(1.0, 0.3, 10.0), ChVector3d(0.0, 0.0, 0.0));
    vis->EnableShadows();

    // --TEST--

    /// Given the position of a point in local frame coords, and
    /// assuming it is sticky to frame, return the acceleration in parent coords.
    /// par_frame is the referenced frame expressed in inertia coordinate,
    /// local_pos is the local position of point P expressed in the par_frame,
    /// par_acc is the translational acceleration of point P but expressed in inertia coordiante.
    auto PointAccelerationLocalToParent = [&](const chrono::ChFrameMoving<double>& par_frame,
                                              const chrono::ChVector3d& local_pos, chrono::ChVector3d& par_acc) {
        chrono::ChVector3d par_pos = par_frame.TransformDirectionLocalToParent(local_pos);
        chrono::ChVector3d alpha = par_frame.GetAngAccParent();
        chrono::ChVector3d omega = par_frame.GetAngVelParent();
        par_acc = par_frame.GetPosDt2() + chrono::Vcross(par_frame.GetAngAccParent(), par_pos) +
                  chrono::Vcross(omega, chrono::Vcross(omega, par_pos));
    };

    // define a rotatiing frame, with rotational angular velocity 2.3rad/s about X axis
    chrono::ChFrameMoving<double> rot_frame;
    chrono::ChVector3d omega = chrono::ChVector3d(2.3, 0, 0);
    rot_frame.SetAngVelLocal(omega);
    rot_frame.SetAngAccLocal(VNULL);
    std::cout << "Frame w_loc =" << rot_frame.GetAngVelLocal() << "   w_abs =" << rot_frame.GetAngVelParent()
              << std::endl;

    // solve the velocites and accelerations of point P due to the rotation of rot_frame
    auto TestCase = [&](const chrono::ChFrameMoving<double> m_rot_frame, const chrono::ChVector3d& m_dpos_rel) {
        // elvauated by chrono method
        chrono::ChVector3d vel_par = rot_frame.PointSpeedLocalToParent(m_dpos_rel);
        chrono::ChVector3d acc_par = m_rot_frame.PointAccelerationLocalToParent(m_dpos_rel, VNULL, VNULL);

        // evaluated by new method
        chrono::ChVector3d acc_par_ref;  // accelerations calculated by new developed method
        PointAccelerationLocalToParent(m_rot_frame, m_dpos_rel, acc_par_ref);

        std::cout << "rot_frame.GetCoordsys():\t" << m_rot_frame.GetCoordsys() << std::endl;
        std::cout << "rot_frame.GetCoordsysDt():\t" << m_rot_frame.GetPosDt() << "\t" << m_rot_frame.GetAngVelLocal()
                  << std::endl;
        std::cout << "rot_frame.GetCoordsysDt2():\t" << m_rot_frame.GetPosDt2() << "\t" << m_rot_frame.GetAngAccLocal()
                  << std::endl;
        std::cout << "vel_par:\t" << vel_par << std::endl;
        std::cout << "acc_par from chrono method:\t" << acc_par << std::endl;
        std::cout << "acc_par_ref from new method:\t" << acc_par_ref << std::endl;
        std::cout << "acc_par_ref from r*w^2:\t" << -7.0 * omega.x() * omega.x() << std::endl;
        std::cout << std::endl << std::endl;
    };

    chrono::ChVector3d dpos_rel1 = chrono::ChVector3d(-7, 0, 0);
    std::cout << "Test case 1: a rotating point P along X axis, so its velocity and acceleration should be zero."
              << std::endl
              << "But the acceleration from chrono method is NOT zero!" << std::endl;
    TestCase(rot_frame, dpos_rel1);

    chrono::ChVector3d dpos_rel2 = chrono::ChVector3d(0, -7, 0);
    std::cout << "Test case 2: a rotating point P along Y axis, so its velocity and accelerations have some values."
              << std::endl
              << "But the acceleration from chrono methos is A HALF of new method." << std::endl;
    TestCase(rot_frame, dpos_rel2);

    system("pause");

    // Some general settings:
    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // use HHT second order integrator (but slower)
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        // mystepper->SetVerbose(true);
        mystepper->SetStepControl(false);
    }

    // 1- STATICS

    // sys.EnableSolverMatrixWrite(true);

    // Perform nonlinear statics, with assigned speeds and accelerations (that generate inertial and gyroscopic loads)
    // as for a blade in steady-state rotation.
    // In order to provide speeds and accelerations, there are two ways:
    // - using a callback to update them at each iteration via    myanalysis->SetCallbackIterationBegin(mycallback);
    // - or letting the solver compute them from motors, via
    // myanalysis->SetAutomaticSpeedAndAccelerationComputation(true); The latter is limited in functionality, so for the
    // moment let's use a callback that is called at each nonlinear statics iteration step:

    class MyCallback : public ChStaticNonLinearRheonomicAnalysis::IterationCallback {
      public:
        // Override this function of the callback to update speeds
        // and accelerations during the nonlinear static loop.
        void OnIterationBegin(const double load_scaling,
                              const int iteration_n,
                              ChStaticNonLinearRheonomicAnalysis* analysis) override {
            for (auto in : blade_nodes) {
                // Set node speed and angular velocity, as moved by hub motor:
                in->SetPosDt(ChVector3d(-in->GetPos().y() * blade_rad_s, 0, 0));
                in->SetAngVelParent(ChVector3d(0, 0, blade_rad_s));
                // Set also centripetal acceleration:
                in->SetPosDt2(ChVector3d(0, -in->GetPos().y() * blade_rad_s * blade_rad_s, 0));
            }
        }
        // some data used by the callback to make things simple
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> blade_nodes;
        double blade_rad_s;
    };

    auto mycallback = chrono_types::make_shared<MyCallback>();
    mycallback->blade_nodes = nodes;
    mycallback->blade_rad_s = rad_s;

    ChStaticNonLinearRheonomicAnalysis analysis;
    analysis.SetMaxIterations(25);
    analysis.SetVerbose(true);
    analysis.SetCallbackIterationBegin(mycallback);

    // As an alternative to providing the callback, a much simpler option is to let the static solver
    // compute the speed and acceleration as inferred by the rheonomic joints, instead of the
    // previous line just use:
    //   myanalysis->SetAutomaticSpeedAndAccelerationComputation(true);
    // However this functionality is currently limited because it computes speeds/accelerations only at initial
    // undeformed state.

    // EXECUTE NONLINEAR STATIC ANALYSIS HERE:
    sys.DoStaticAnalysis(analysis);

    // Some plots after the static analysis:
    {
        ChVectorDynamic<> plotx(nodes.size());
        ChVectorDynamic<> ploty(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            plotx(i) = nodes[i]->GetPos().y();
            ploty(i) = nodes[i]->GetPos().z();
        }
        ChGnuPlot mplot_flap_displ(out_dir + "/flapwise_displ.dat");
        mplot_flap_displ.SetGrid();
        mplot_flap_displ.Plot(plotx, ploty, "Flapwise displacement", " with lines lt -1 lc rgb'#00AAEE'");

        ChVectorDynamic<> ploty_analytic(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            ploty(i) = nodes[i]->GetPosDt().x();
            ploty_analytic(i) = -nodes[i]->GetPos().y() * rad_s;
        }
        ChGnuPlot mplot_edge_speed(out_dir + "/flapwise_speed.dat");
        mplot_edge_speed.SetGrid();
        mplot_edge_speed.Plot(plotx, ploty, "Edgewise speed", " with lines lt -1 lc rgb'#00AAEE'");
        mplot_edge_speed.Plot(plotx, ploty_analytic, "Expected analytic edgewise speed",
                              " with lines lt -1 lc rgb'#AA00EE'");

        for (int i = 0; i < nodes.size(); ++i) {
            ploty(i) = nodes[i]->GetPosDt2().y();
            ploty_analytic(i) = -nodes[i]->GetPos().y() * rad_s * rad_s;
        }
        ChGnuPlot mplot_centeripetal_accel(out_dir + "/centripetal_acc.dat");
        mplot_centeripetal_accel.SetGrid();
        mplot_centeripetal_accel.Plot(plotx, ploty, "Centripetal acceleration", " with lines lt -1 lc rgb'#00AAEE'");
        mplot_centeripetal_accel.Plot(plotx, ploty_analytic, "Expected centripetal acceleration",
                                      " with lines lt -1 lc rgb'#AA00EE'");
    }

    /*
    // TRICK: force nodes to needed speed
    for (auto in : nodes) {
        in->SetPosDt(ChVector3d(-in->GetPos().y() * rad_s, 0, 0));
        in->SetAngVelParent(ChVector3d(0, 0,  rad_s));
    }
    */
    // sys.EnableSolverMatrixWrite(false);

    // 2- DYNAMICS

    std::vector<double> rec_t;
    std::vector<double> rec_tip_edge_d;
    std::vector<double> rec_tip_flap_d;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawGrid(vis.get(), 1, 1, 12, 12, ChCoordsys<>(ChVector3d(0, 0, 0), CH_PI_2, VECT_Z),
                        ChColor(0.4f, 0.4f, 0.4f), true);

        sys.DoStepDynamics(0.01);

        // for plotting the tip oscillations, in the blade root coordinate:
        rec_t.push_back(sys.GetChTime());
        rec_tip_edge_d.push_back(nodes.front()->TransformPointParentToLocal(nodes.back()->GetPos()).z());
        rec_tip_flap_d.push_back(nodes.front()->TransformPointParentToLocal(nodes.back()->GetPos()).y());

        /*
        // for simplified testing of the tilting control of the blade, with sudden jump:
        if (sys.GetChTime() > 2){
            if (auto myfunct = std::dynamic_pointer_cast<ChFunctionConst>(my_root->GetMotorFunction()))
                myfunct->SetConstant(0.4);
        }
        */

        vis->EndScene();
    }

    ChGnuPlot mplot_tip_edge_d(out_dir + "/tip_edge_d.dat");
    mplot_tip_edge_d.SetGrid();
    mplot_tip_edge_d.Plot(rec_t, rec_tip_edge_d, "Edgewise displacement (t)", " with lines lt -1 lc rgb'#00AAEE'");

    ChGnuPlot mplot_tip_flap_d(out_dir + "/tip_flap_d.dat");
    mplot_tip_flap_d.SetGrid();
    mplot_tip_flap_d.Plot(rec_t, rec_tip_flap_d, "Flapwise displacement (t)", " with lines lt -1 lc rgb'#00AAEE'");

    return 0;
}
