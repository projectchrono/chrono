// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo code for using a hydraulic actuator model exchange FMU in conjunction
// with a simple crane multibody mechanical system.
//
// =============================================================================

#include <cmath>
#include <iomanip>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_fmi/ChConfigFMI.h"
#include "chrono_fmi/ChExternalFmu.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

// -----------------------------------------------------------------------------

ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

double t_end = 20;

double t_step = 5e-4;

bool output = true;
double output_fps = 1000;

bool render = true;

// -----------------------------------------------------------------------------
// External actuator FMU wrapper.
// This class can accommodate an arbitrary model exchange FMU that provides inputs and outputs common to an actuator
// meant to be connected between two Chrono rigid bodies. Such an actuator FMU must provide the following variables:
// [parameters]
//   "init_s" - initial actuator length (real, parameter, fixed)
//   "init_F" - initial actuator load   (real, parameter, fixed)
// [inputs]
//   "s"      - actuator length         (real, input, continuous)
//   "sd"     - actuator length rate    (real, input, continuous)
//   "Uref"   - input signal            (real, input, continuous)
// [outputs]
//   "F"      - actuator force          (real, outpuyt, continuous)
// -----------------------------------------------------------------------------
class ChExternalActuatorFmu : public ChExternalFmu {
  public:
    ChExternalActuatorFmu(const std::string& fmu_filename) : m_fmu_filename(fmu_filename), is_attached(false) {}

    void SetUnpackDirectory(const std::string& unpack_dir) { m_unpack_dir = unpack_dir; }
    void SetInitialLoad(double F) { m_initial_F = F; }
    void SetActuationFunction(std::shared_ptr<ChFunction> actuation) { m_actuation = actuation; }

    double GetForce() const { return GetRealVariable("F"); }
    ChVector3d GetPoint1Abs() const { return m_aloc1; }
    ChVector3d GetPoint2Abs() const { return m_aloc2; }

    void Initialize(std::shared_ptr<ChBody> body1,  ///< first connected body
                    std::shared_ptr<ChBody> body2,  ///< second connected body
                    ChVector3d loc1,                ///< location of connection point on body 1
                    ChVector3d loc2                 ///< location of connection point on body 2
    ) {
        // Cache connected bodies and body local connection points
        m_body1 = body1.get();
        m_body2 = body2.get();

        m_loc1 = loc1;
        m_loc2 = loc2;
        m_aloc1 = body1->TransformPointLocalToParent(loc1);
        m_aloc2 = body2->TransformPointLocalToParent(loc2);

        // Resize temporary vector of generalized body forces
        m_Qforce.resize(12);

        // Load the FMU
        try {
            Load("actuator_fmu", m_fmu_filename, m_unpack_dir);
        } catch (std::exception& e) {
            std::cerr << "ERROR loading FMU: " << e.what() << std::endl;
            throw e;
        }

        if (m_verbose)
            PrintInfo();

        double init_s = (m_aloc1 - m_aloc2).Length();

        // Set FMU parameters (initial length and initial load)
        SetRealParameterValue("init_F", m_initial_F);
        SetRealParameterValue("init_s", init_s);

        // Set FMU actuation function
        SetRealInputChFunction("Uref", m_actuation);

        // Set FMU initial state conditions
        // NOTE: This is FMU-dependent
        //// TODO: how can we generalize this?
        SetInitialCondition("U", 0.0);
        SetInitialCondition("p1", 4.163e6);
        SetInitialCondition("p2", 3.461e6);

        // Initialize the base class
        ChExternalFmu::Initialize();

        is_attached = true;
    }

    virtual bool IsStiff() const override { return true; }

    virtual void Update(double time, bool update_assets) override {
        ChExternalFmu::Update(time, update_assets);

        if (is_attached) {
            // Calculate length and length rate from attached bodies
            m_aloc1 = m_body1->TransformPointLocalToParent(m_loc1);
            m_aloc2 = m_body2->TransformPointLocalToParent(m_loc2);

            auto avel1 = m_body1->PointSpeedLocalToParent(m_loc1);
            auto avel2 = m_body2->PointSpeedLocalToParent(m_loc2);

            ChVector3d dir = (m_aloc1 - m_aloc2).GetNormalized();
            double s = (m_aloc1 - m_aloc2).Length();
            double sd = Vdot(dir, avel1 - avel2);

            // Set FMU inputs (actuator length and length rate)
            SetRealVariable("s", s);
            SetRealVariable("sd", sd);

            ////std::cout << "time = " << time << "    s=" << s << "   sd=" << sd << std::endl;

            // Get FMU actuator force
            double F = GetRealVariable("F");

            // Set force acting on bodies 1 and 2
            ChVector3d force = F * dir;

            auto atorque1 = Vcross(m_aloc1 - m_body1->GetPos(), force);          // applied torque (absolute frame)
            auto ltorque1 = m_body1->TransformDirectionParentToLocal(atorque1);  // applied torque (local frame)
            m_Qforce.segment(0, 3) = force.eigen();
            m_Qforce.segment(3, 3) = ltorque1.eigen();

            auto atorque2 = Vcross(m_aloc2 - m_body2->GetPos(), -force);         // applied torque (absolute frame)
            auto ltorque2 = m_body2->TransformDirectionParentToLocal(atorque2);  // applied torque (local frame)
            m_Qforce.segment(6, 3) = -force.eigen();
            m_Qforce.segment(9, 3) = ltorque2.eigen();
        }
    }

    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override {
        ChExternalFmu::IntLoadResidual_F(off, R, c);

        if (is_attached) {
            // Add forces to connected bodies (calculated in Update)
            if (m_body1->Variables().IsActive()) {
                R.segment(m_body1->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(0, 3);
                R.segment(m_body1->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(3, 3);
            }
            if (m_body2->Variables().IsActive()) {
                R.segment(m_body2->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(6, 3);
                R.segment(m_body2->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(9, 3);
            }
        }
    }

    void PrintInfo() {
        // Print all FMU variables
        PrintFmuVariables();

        // Print names of all FMU states
        auto s_list = GetStatesList();
        std::cout << "\nFMU states:  ";
        for (const auto& v : s_list)
            std::cout << v << "  ";
        std::cout << std::endl;

        // Print names of all real FMU parameters
        auto rp_list = GetRealParametersList();
        std::cout << "FMU real parameters:  ";
        for (const auto& v : rp_list)
            std::cout << v << "  ";
        std::cout << std::endl;

        // Print names of all real FMU parameters
        auto ip_list = GetIntParametersList();
        std::cout << "FMU integer parameters:  ";
        for (const auto& v : ip_list)
            std::cout << v << "  ";
        std::cout << std::endl;

        // Print names of all real FMU inputs
        auto ri_list = GetRealInputsList();
        std::cout << "FMU real inputs:  ";
        for (const auto& v : ri_list)
            std::cout << v << "  ";
        std::cout << "\n" << std::endl;
    }

  private:
    std::string m_fmu_filename;               ///< name of the FMU file
    std::string m_unpack_dir;                 ///< name of directory where FMU file is unpacked
    bool is_attached;                         ///< true if actuator attached to bodies
    ChBody* m_body1;                          ///< first conected body
    ChBody* m_body2;                          ///< second connected body
    ChVector3d m_loc1;                        ///< point on body 1 (local frame)
    ChVector3d m_loc2;                        ///< point on body 2 (local frame)
    ChVector3d m_aloc1;                       ///< point on body 1 (global frame)
    ChVector3d m_aloc2;                       ///< point on body 2 (global frame)
    double m_initial_F;                       ///< initial load
    std::shared_ptr<ChFunction> m_actuation;  ///< actuation function
    ChVectorDynamic<> m_Qforce;               ///< generalized forcing terms
};

// -----------------------------------------------------------------------------

void GetActuatorLength(std::shared_ptr<ChBody> crane,
                       const ChVector3d& point_ground,
                       const ChVector3d& point_crane,
                       double& s,
                       double& sd) {
    const auto& P1 = point_ground;
    const auto& V1 = VNULL;

    auto P2 = crane->TransformPointLocalToParent(point_crane);
    auto V2 = crane->PointSpeedLocalToParent(point_crane);

    ChVector3d dir = (P2 - P1).GetNormalized();

    s = (P2 - P1).Length();
    sd = Vdot(dir, V2 - V1);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -----------------------
    // Load model exchange FMU
    // -----------------------

    // Specify FMU and unpack directory
#ifdef FMU_EXPORT_SUPPORT
    std::string fmu_filename = ACTUATOR_FMU2_FILENAME;  // FMU generated in current build

    if (argc > 1) {
        render = false;
        output = false;
    }
#else
    if (argc != 2) {
        std::cout << "Usage: ./demo_FMI_hydraulic_crane_modex [FMU_filename]" << std::endl;
        return 1;
    }
    std::string fmu_filename = argv[1];  // FMU (fully qualified filename) specified as program argument
#endif

    // FMU unpack directory
    std::string unpack_dir = DEMO_FMU_MAIN_DIR + std::string("/tmp_unpack_actuator_fmu_me/");

    // --------------------
    // Create Chrono system
    // --------------------

    ChSystemSMC sys;
    ChVector3d Gacc(0, 0, -9.8);
    sys.SetGravitationalAcceleration(Gacc);

    ChVector3d attachment_ground(std::sqrt(3.0) / 2, 0, 0);
    ChVector3d attachment_crane(0, 0, 0);

    double crane_mass = 500;
    double crane_length = 1.0;
    double crane_angle = CH_PI / 6;
    ChVector3d crane_pos(0.5 * crane_length * std::cos(crane_angle), 0, 0.5 * crane_length * std::sin(crane_angle));

    double pend_length = 0.3;
    double pend_mass = 100;
    ChVector3d pend_pos = 2.0 * crane_pos + ChVector3d(0, 0, -pend_length);

    auto connection_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
    connection_sph->SetColor(ChColor(0.7f, 0.3f, 0.3f));

    // Estimate initial required force (moment balance about crane pivot)
    auto Gtorque = Vcross(crane_mass * Gacc, crane_pos) + Vcross(pend_mass * Gacc, pend_pos);
    auto dir = (crane_pos - attachment_ground).GetNormalized();
    auto F0 = Gtorque.Length() / Vcross(dir, crane_pos).Length();

    // --------------------------
    // Create the crane mechanism
    // --------------------------

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->AddVisualShape(connection_sph, ChFrame<>());
    ground->AddVisualShape(connection_sph, ChFrame<>(attachment_ground, QUNIT));
    sys.AddBody(ground);

    auto crane = chrono_types::make_shared<ChBody>();
    crane->SetMass(crane_mass);
    crane->SetPos(crane_pos);
    crane->SetRot(QuatFromAngleY(-crane_angle));
    crane->AddVisualShape(connection_sph, ChFrame<>(attachment_crane, QUNIT));
    crane->AddVisualShape(connection_sph, ChFrame<>(ChVector3d(crane_length / 2, 0, 0), QUNIT));
    auto crane_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.015, crane_length);
    crane->AddVisualShape(crane_cyl, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));
    sys.AddBody(crane);

    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetMass(pend_mass);
    ball->SetPos(pend_pos);
    auto ball_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
    ball->AddVisualShape(ball_sph);
    auto ball_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.005, pend_length);
    ball->AddVisualShape(ball_cyl, ChFrame<>(ChVector3d(0, 0, pend_length / 2), QUNIT));
    sys.AddBody(ball);

    auto rev_joint = chrono_types::make_shared<ChLinkRevolute>();
    rev_joint->Initialize(ground, crane, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
    sys.AddLink(rev_joint);

    auto sph_joint = chrono_types::make_shared<ChLinkLockSpherical>();
    sph_joint->Initialize(crane, ball, ChFrame<>(2.0 * crane_pos, QUNIT));
    sys.AddLink(sph_joint);

    // Create a dummy TSDA (zero force) to visualize the actuator
    auto dummy_tsda = chrono_types::make_shared<ChLinkTSDA>();
    dummy_tsda->Initialize(ground, crane, true, attachment_ground, attachment_crane);
    dummy_tsda->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    sys.AddLink(dummy_tsda);

    // --------------------------------------
    // Set up the actuator model exchange FMU
    // --------------------------------------

    // Hydraulic actuation
    auto f_segment = chrono_types::make_shared<ChFunctionSequence>();
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0), 0.5);         // 0.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0, 0.4), 1.5);     // 0.0 -> 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0.6), 5.0);       // 0.6
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0.6, -0.8), 2.0);  // 0.6 -> -1.0
    f_segment->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(-1.0, 1.0), 1.0);  // -1.0 -> 0.0
    auto actuation = chrono_types::make_shared<ChFunctionRepeat>(f_segment, 0, 10, 10);

    // Create the actuator FMU wrapper
    auto fmu_wrapper = chrono_types::make_shared<ChExternalActuatorFmu>(fmu_filename);
    fmu_wrapper->SetVerbose(true);
    fmu_wrapper->SetUnpackDirectory(unpack_dir);
    fmu_wrapper->SetActuationFunction(actuation);
    fmu_wrapper->SetInitialLoad(F0);

    // Load and initialize the actuator FMU wrapper
    fmu_wrapper->Initialize(ground, crane, attachment_ground, attachment_crane);

    // Add actuator FMU wrapper as a physics item
    sys.Add(fmu_wrapper);

    // ----------------------------------------
    // Create the run-time visualization system
    // ----------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
                vis_irr->SetWindowSize(800, 600);
                vis_irr->SetWindowTitle("Hydraulic actuator demo");
                vis_irr->SetCameraVertical(CameraVerticalDir::Z);
                vis_irr->SetBackgroundColor(ChColor(0.37f, 0.50f, 0.60f));
                vis_irr->Initialize();
                vis_irr->AddCamera(ChVector3d(0.5, -1, 0.5), ChVector3d(0.5, 0, 0.5));
                vis_irr->AddLogo();
                vis_irr->AddTypicalLights();
                vis_irr->AttachSystem(&sys);

                vis = vis_irr;
#endif
                break;
            }
            default:
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
                vis_vsg->AttachSystem(&sys);
                vis_vsg->SetWindowTitle("Hydraulic actuator demo");
                vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
                vis_vsg->SetBackgroundColor(ChColor(0.37f, 0.50f, 0.60f));
                vis_vsg->AddCamera(ChVector3d(0.3, -2, 0.5), ChVector3d(0.3, 0, 0.5));
                vis_vsg->SetWindowSize(1280, 800);
                vis_vsg->SetWindowPosition(100, 100);
                vis_vsg->SetCameraAngleDeg(40.0);
                vis_vsg->SetLightIntensity(1.0f);
                vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
                vis_vsg->EnableShadows();
                vis_vsg->Initialize();

                vis = vis_vsg;
#endif
                break;
            }
        }
    }

    // -------------------------
    // Create output directories
    // -------------------------

    std::string out_dir = GetChronoOutputPath() + "DEMO_FMI_HYDRAULIC_CRANE_MODEX";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Solver settings
    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(sys.GetTimestepper());
    integrator->SetMaxIters(50);
    integrator->SetAbsTolerances(1e-4, 1e2);

    // Initialize output file
    Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", " ", " ", " ", " ");
    utils::ChWriterCSV csv(" ");

    double t = 0;
    double Uref = actuation->GetVal(t);
    double F = 0;
    ChVectorDynamic<> y(fmu_wrapper->GetNumStates());
    y = fmu_wrapper->GetInitialStates();
    double s, sd;
    GetActuatorLength(crane, attachment_ground, attachment_crane, s, sd);

    csv << t << s << sd << Uref << y.format(rowFmt) << F << std::endl;

    // Simulation loop
    int output_frame = 1;

    ChTimer timer;
    timer.start();
    while (t <= t_end) {
        if (render) {
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        sys.DoStepDynamics(t_step);
        t += t_step;

        if (output && t >= output_frame / output_fps) {
            GetActuatorLength(crane, attachment_ground, attachment_crane, s, sd);
            Uref = actuation->GetVal(t);
            y = fmu_wrapper->GetStates();
            F = fmu_wrapper->GetForce();
            csv << t << s << sd << Uref << y.format(rowFmt) << F << std::endl;
            output_frame++;
        }
    }
    timer.stop();
    auto RTF = timer() / t_end;
    std::cout << "sim time: " << t_end << "  RTF: " << RTF;

    // -------------------
    // Postprocess results
    // -------------------

    if (output) {
        std::string out_file = out_dir + "/hydraulic_crane.out";
        csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
        {
            postprocess::ChGnuPlot gplot(out_dir + "/displ.gpl");
            gplot.SetOutputWindowTitle("Actuator length");
            gplot.SetCanvasSize(800, 640);
            gplot.SetGrid();
            gplot.SetLegend("left bottom");
            gplot.SetLabelX("time [s]");
            gplot.SetLabelY("s [m] , sd [m/s]");
            gplot.SetRangeX(0, t_end);
            gplot.Plot(out_file, 1, 2, "s", " with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 3, "sd", " with lines lt 2 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_input.gpl");
            gplot.SetOutputWindowTitle("Hydraulic Input");
            gplot.SetCanvasSize(800, 640);
            gplot.SetGrid();
            gplot.SetLabelX("time [s]");
            gplot.SetLabelY("U");
            gplot.SetRangeX(0, t_end);
            gplot.Plot(out_file, 1, 4, "", " with lines lt -1 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_pressure.gpl");
            gplot.SetOutputWindowTitle("Hydraulic Pressures");
            gplot.SetCanvasSize(800, 640);
            gplot.SetGrid();
            gplot.SetLegend("left bottom");
            gplot.SetLabelX("time [s]");
            gplot.SetLabelY("p [N/m2]");
            gplot.SetRangeX(0, t_end);
            gplot.Plot(out_file, 1, 6, "p0", " with lines lt 1 lw 2");
            gplot.Plot(out_file, 1, 7, "p1", " with lines lt 2 lw 2");
        }
        {
            postprocess::ChGnuPlot gplot(out_dir + "/hydro_force.gpl");
            gplot.SetOutputWindowTitle("Hydraulic Force");
            gplot.SetCanvasSize(800, 640);
            gplot.SetGrid();
            gplot.SetLabelX("time [s]");
            gplot.SetLabelY("F [N]");
            gplot.SetRangeX(0, t_end);
            gplot.SetRangeY(1000, 9000);
            gplot.Plot(out_file, 1, 8, "", " with lines lt -1 lw 2");
        }
#endif
    }

    return 0;
}
