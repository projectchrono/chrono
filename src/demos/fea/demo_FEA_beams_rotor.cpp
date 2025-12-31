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
// FEA nonlinear static static_analysis of 3D beams, including centrifugal effect
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_postprocess/ChGnuPlot.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::postprocess;

// -----------------------------------------------------------------------------

enum class BeamElementType { EULER, IGA, TIMOSHENKO };
BeamElementType beam_type = BeamElementType::TIMOSHENKO;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

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

    // Directory for output data
    const std::string out_dir = GetChronoOutputPath() + "BEAM_ROTOR";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // --------------------------
    // CREATE THE MODEL
    // --------------------------

    // Create a Chrono physical system
    ChSystemNSC sys;

    // Use MKL Pardiso solver
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // Use HHT integrator
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        // mystepper->SetVerbose(true);
        mystepper->SetStepControl(false);
    }

    // Base and tower body
    auto tower = chrono_types::make_shared<ChBodyEasyBox>(10, 2, 10, 3000);
    tower->SetFixed(true);
    tower->SetPos(ChVector3d(0, -10, 0));
    sys.Add(tower);

    // Attach a cylinder shape asset for visualization of the tower
    auto tower_shape = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 9.0);
    tower->AddVisualShape(tower_shape, ChFrame<>(ChVector3d(0, 5.5, 0), QuatFromAngleX(CH_PI_2)));

    // Rotating hub
    auto hub = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.2, 0.5, 1000);
    hub->SetPos(ChVector3d(0, 0, 1));
    hub->SetRot(QuatFromAngleX(CH_PI_2));
    sys.Add(hub);

    // Hub motor
    // Since we are going to ue the DoStaticNonlinearRheonomic static_analysis, we must use
    // a motor that imposes a speed (so, motor imposing torques are not fit). Hence:
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(hub, tower, ChFrame<>(ChVector3d(0, 0, 1)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, rad_s));
    sys.Add(motor);

    // FEA mesh
    auto fea_mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(fea_mesh);

    // no gravity used here
    sys.SetGravitationalAcceleration(VNULL);
    fea_mesh->SetAutomaticGravity(false);

    // FEA beams
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;
    switch (beam_type) {
        case BeamElementType::EULER: {
            // Create a simplified section, i.e. thickness and material properties
            // for beams. This will be shared among some beams.
            auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

            section->SetDensity(beam_density);
            section->SetYoungModulus(beam_Young);
            section->SetShearModulusFromPoisson(0.31);
            section->SetRayleighDampingBeta(0 * 0.00001);
            section->SetRayleighDampingAlpha(0 * 0.001);
            section->SetAsRectangularSection(beam_wy, beam_wz);
            section->compute_inertia_damping_matrix = true;    //// NOTE: not much different
            section->compute_inertia_stiffness_matrix = true;  //// NOTE: not much different

            // This helps creating sequences of nodes and ChElementBeamEuler elements:
            ChBuilderBeamEuler builder;

            builder.BuildBeam(fea_mesh,  // the mesh where to put the created nodes and elements
                              section,   // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
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

            break;
        }
        case BeamElementType::IGA: {
            auto section = chrono_types::make_shared<ChBeamSectionCosseratEasyRectangular>(
                beam_wy,           // width of section in y direction
                beam_wz,           // width of section in z direction
                beam_Young,        // Young modulus
                beam_Young * 0.3,  // shear modulus
                beam_density       // density
            );

            ChBuilderBeamIGA builder;
            builder.BuildBeam(fea_mesh,                     // the mesh to put the elements in
                              section,                      // section of the beam
                              6,                            // number of sections (spans)
                              ChVector3d(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
                              ChVector3d(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
                              ChVector3d(0, 0, 1),          // the 'Y' up direction of the section for the beam
                              1);                           // order (3 = cubic, etc)

            nodes = builder.GetLastBeamNodes();

            break;
        }
        case BeamElementType::TIMOSHENKO: {
            double Izz = (1.0 / 12.0) * beam_wz * std::pow(beam_wy, 3);
            double Iyy = (1.0 / 12.0) * beam_wy * std::pow(beam_wz, 3);
            DampingCoefficients damping_coeffs;
            damping_coeffs.bt = damping_coeffs.bx = damping_coeffs.by = damping_coeffs.bz = 0.001;
            auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGeneric>(
                beam_Young * beam_wy * beam_wz, (Izz + Iyy) * beam_Young * 0.3, Iyy * beam_Young, Izz * beam_Young,
                beam_Young * 0.3 * beam_wy * beam_wz, beam_Young * 0.3 * beam_wy * beam_wz, damping_coeffs, 0, 0, 0, 0,
                0, beam_density * beam_wy * beam_wz, beam_density * Iyy, beam_density * Izz, 0, 0, 0, 0);
            // for visualization as rectangular section
            section->SetDrawShape(chrono_types::make_shared<ChBeamSectionShapeRectangular>(beam_wy, beam_wz));

            auto tapered_section = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGeneric>();
            tapered_section->SetSectionA(section);
            tapered_section->SetSectionB(section);

            ChBuilderBeamTaperedTimoshenko builder;
            builder.BuildBeam(fea_mesh,                     // the mesh to put the elements in
                              tapered_section,              // section of the beam
                              6,                            // number of sections (spans)
                              ChVector3d(0, beam_Rmin, 1),  // the 'A' point in space (beginning of beam)
                              ChVector3d(0, beam_Rmax, 1),  // the 'B' point in space (end of beam)
                              ChVector3d(0, 0, 1)           // the 'Y' up direction of the section for the beam
            );                                              // order (3 = cubic, etc)

            nodes = builder.GetLastBeamNodes();

            break;
        }
    }

    // Connect root of blade to the hub (use a motor, but with zero speed)
    auto root_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    root_motor->Initialize(nodes.front(), hub, ChFrame<>(ChVector3d(0, 0.5, 1), QuatFromAngleX(CH_PI_2)));
    root_motor->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(0));
    sys.Add(root_motor);

    // Apply tip force (in absolute frame)
    nodes.back()->SetForce(tip_abs_force);

    // FEA mesh visualization
    auto vis_beam_1 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_1->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_TX);
    vis_beam_1->SetColormapRange(-0.001, 600);
    vis_beam_1->SetSmoothFaces(true);
    vis_beam_1->SetWireframe(false);
    fea_mesh->AddVisualShapeFEA(vis_beam_1);

    auto vis_beam_2 = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_2->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    vis_beam_2->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_beam_2->SetSymbolsThickness(0.2);
    vis_beam_2->SetSymbolsScale(0.1);
    vis_beam_2->SetZbufferHide(false);
    fea_mesh->AddVisualShapeFEA(vis_beam_2);

    // Run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys,
                                         "Rotor with simplified blade: steady state statics & dynamics",
                                         ChVector3d(2.0, 0.6, 20.0));

    // --------------------------
    // TEST
    // --------------------------

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
              << "But the acceleration from chrono method is A HALF of new method." << std::endl;
    TestCase(rot_frame, dpos_rel2);

    // --------------------------
    // STATICS
    // --------------------------

    // Perform nonlinear statics, with assigned speeds and accelerations (that generate inertial and gyroscopic loads)
    // as for a blade in steady-state rotation.
    // There are two ways to provide speeds and accelerations:
    //   1. using a callback to update them at each iteration via SetCallbackIterationBegin(IterationCallback), or
    //   2. letting the solver compute them from motors, via SetAutomaticSpeedAndAccelerationComputation(true).
    // The latter is limited in functionality, so use option 1.
    class StaticsIterationCallback : public ChStaticNonLinearRheonomicAnalysis::IterationCallback {
      public:
        StaticsIterationCallback(const std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes, double blade_rad)
            : nodes(nodes), radius(blade_rad) {}

        void OnIterationBegin(const double load_scaling,
                              const int iteration_n,
                              ChStaticNonLinearRheonomicAnalysis* static_analysis) override {
            for (auto in : nodes) {
                // Set node speed and angular velocity, as moved by hub motor
                in->SetPosDt(ChVector3d(-in->GetPos().y() * radius, 0, 0));
                in->SetAngVelParent(ChVector3d(0, 0, radius));
                // Set node centripetal acceleration
                in->SetPosDt2(ChVector3d(0, -in->GetPos().y() * radius * radius, 0));
            }
        }

      private:
        std::vector<std::shared_ptr<ChNodeFEAxyzrot>> nodes;
        double radius;
    };

    auto statics_callback = chrono_types::make_shared<StaticsIterationCallback>(nodes, rad_s);

    ChStaticNonLinearRheonomicAnalysis static_analysis;
    static_analysis.SetMaxIterations(25);
    static_analysis.SetVerbose(true);
    static_analysis.SetCallbackIterationBegin(statics_callback);

    // As an alternative to providing the callback, a much simpler option is to let the static solver
    // compute the speed and acceleration as inferred by the rheonomic joints, instead of the
    // previous line just use:
    //   myanalysis->SetAutomaticSpeedAndAccelerationComputation(true);
    // However this functionality is currently limited because it computes speeds/accelerations only at initial
    // undeformed state.

    // Perform nonlinear static static_analysis
    sys.DoStaticAnalysis(static_analysis);

    // Some plots after the static static_analysis
    {
        ChVectorDynamic<> plotx(nodes.size());
        ChVectorDynamic<> ploty(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            plotx(i) = nodes[i]->GetPos().y();
            ploty(i) = nodes[i]->GetPos().z();
        }
        ChGnuPlot plot_flap_displ(out_dir + "/flapwise_displ.dat");
        plot_flap_displ.SetGrid();
        plot_flap_displ.Plot(plotx, ploty, "Flapwise displacement", " with lines lt -1 lc rgb'#00AAEE'");

        ChVectorDynamic<> ploty_analytic(nodes.size());
        for (int i = 0; i < nodes.size(); ++i) {
            ploty(i) = nodes[i]->GetPosDt().x();
            ploty_analytic(i) = -nodes[i]->GetPos().y() * rad_s;
        }
        ChGnuPlot plot_edge_speed(out_dir + "/flapwise_speed.dat");
        plot_edge_speed.SetGrid();
        plot_edge_speed.Plot(plotx, ploty, "Edgewise speed", " with lines lt -1 lc rgb'#00AAEE'");
        plot_edge_speed.Plot(plotx, ploty_analytic, "Expected analytic edgewise speed",
                             " with lines lt -1 lc rgb'#AA00EE'");

        for (int i = 0; i < nodes.size(); ++i) {
            ploty(i) = nodes[i]->GetPosDt2().y();
            ploty_analytic(i) = -nodes[i]->GetPos().y() * rad_s * rad_s;
        }
        ChGnuPlot plot_centeripetal_accel(out_dir + "/centripetal_acc.dat");
        plot_centeripetal_accel.SetGrid();
        plot_centeripetal_accel.Plot(plotx, ploty, "Centripetal acceleration", " with lines lt -1 lc rgb'#00AAEE'");
        plot_centeripetal_accel.Plot(plotx, ploty_analytic, "Expected centripetal acceleration",
                                     " with lines lt -1 lc rgb'#AA00EE'");
    }

    /*
    // TRICK: force nodes to needed speed
    for (auto in : nodes) {
        in->SetPosDt(ChVector3d(-in->GetPos().y() * rad_s, 0, 0));
        in->SetAngVelParent(ChVector3d(0, 0,  rad_s));
    }
    */

    // --------------------------
    // DYNAMICS
    // --------------------------

    std::vector<double> rec_t;
    std::vector<double> rec_tip_edge_d;
    std::vector<double> rec_tip_flap_d;

    ChRealtimeStepTimer realtime_timer;
    double time_step = 0.01;

    while (vis->Run()) {
        // for plotting the tip oscillations, in the blade root coordinate:
        rec_t.push_back(sys.GetChTime());
        rec_tip_edge_d.push_back(nodes.front()->TransformPointParentToLocal(nodes.back()->GetPos()).z());
        rec_tip_flap_d.push_back(nodes.front()->TransformPointParentToLocal(nodes.back()->GetPos()).y());

        /*
        // simplified testing of the tilting control of the blade, with sudden jump
        if (sys.GetChTime() > 2){
            if (auto myfunct = std::dynamic_pointer_cast<ChFunctionConst>(root_motor->GetMotorFunction()))
                myfunct->SetConstant(0.4);
        }
        */

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(time_step);
        realtime_timer.Spin(time_step);
    }

    ChGnuPlot plot_tip_edge_d(out_dir + "/tip_edge_d.dat");
    plot_tip_edge_d.SetGrid();
    plot_tip_edge_d.Plot(rec_t, rec_tip_edge_d, "Edgewise displacement (t)", " with lines lt -1 lc rgb'#00AAEE'");

    ChGnuPlot plot_tip_flap_d(out_dir + "/tip_flap_d.dat");
    plot_tip_flap_d.SetGrid();
    plot_tip_flap_d.Plot(rec_t, rec_tip_flap_d, "Flapwise displacement (t)", " with lines lt -1 lc rgb'#00AAEE'");

    return 0;
}
