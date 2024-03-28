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
// Authors: Alessandro Tasora, Chao Peng
// =============================================================================
//
// Show how to use the ChAssembly to manage the hierarchy of the entire system
// containing substructures.
//
// =============================================================================

#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono_modal/ChModalAssembly.h"

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;
using namespace filesystem;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "SLIDER_CRANK";

constexpr double time_step = 0.001;
constexpr double time_length = 10;

constexpr bool RUN_ORIGIN = false;
constexpr bool RUN_MODAL = true;
constexpr bool USE_HERTING = true;
constexpr bool USE_LINEAR_INERTIAL_TERM = true;

void MakeAndRunDemo_SliderCrank(bool do_modal_reduction, ChMatrixDynamic<>& mdeflection) {
    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double crank_L = 10.0;
    double rod_L = 20.0;
    int crank_parts = 2;
    int rod_parts = 4;
    int crank_totalelements = crank_parts * 8;
    int rod_totalelements = rod_parts * 8;

    double E = 7.0e10;
    double rho = 2700;
    double miu = 0.31;
    double G = E / (2.0 * (1 + miu));

    double b = 0.4;
    double h = 0.4;
    double Area = b * h;  //=0.16
    double Iyy = 1 / 12.0 * b * h * h * h;
    double Izz = 1 / 12.0 * h * b * b * b;
    double Iyz = 0;
    double Ip = Iyy + Izz;

    double EA = E * Area;
    double EIyy = E * Iyy;
    double EIzz = E * Izz;
    double GIp = G * Ip;

    double mass_per_unit_length = rho * Area;
    double Jyy = rho * Iyy;
    double Jzz = rho * Izz;
    double Jxx = rho * Ip;
    double Jyz = rho * Iyz;

    // parent system
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 10);
    my_ground->SetPos(VNULL);
    my_ground->SetFixed(true);
    sys.AddBody(my_ground);

    // slider
    auto my_slider = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.2, 0.2, 10);
    my_slider->SetPos(ChVector3d(crank_L + rod_L, 0, 0));
    my_slider->SetMass(1.0);
    my_slider->SetInertiaXX(ChVector3d(1, 1, 1));
    sys.AddBody(my_slider);

    auto my_link_slider = chrono_types::make_shared<ChLinkMatePrismatic>();
    my_link_slider->Initialize(my_slider, my_ground, ChFrame<>(my_ground->GetCoordsys()));
    sys.AddLink(my_link_slider);

    // Prepare for mesh: Beam section:
    auto section = chrono_types::make_shared<ChBeamSectionRayleighAdvancedGeneric>();
    section->SetAxialRigidity(EA);
    section->SetTorsionRigidityX(GIp);
    section->SetBendingRigidityY(EIyy);
    section->SetBendingRigidityZ(EIzz);
    section->SetMassPerUnitLength(mass_per_unit_length);
    section->SetInertiaJxxPerUnitLength(Jxx);
    section->SetInertiasPerUnitLength(Jyy, Jzz, Jyz);
    section->SetArtificialJyyJzzFactor(0.0 / 500.0);
    section->SetSectionRotation(0);
    section->SetCenterOfMass(0, 0);
    section->SetCentroidY(0);
    section->SetCentroidZ(0);
    section->SetShearCenterY(0);
    section->SetShearCenterZ(0);
    section->SetRayleighDampingBeta(0.002);
    section->SetRayleighDampingAlpha(0.000);

    // A function to make a single modal assembly
    auto MakeSingleModalAssembly = [&](std::shared_ptr<ChModalAssembly> mmodal_assembly, int mn_ele, double mstart_x,
                                       double mend_x) {
        // Settings
        mmodal_assembly->SetUseLinearInertialTerm(USE_LINEAR_INERTIAL_TERM);
        if (USE_HERTING)
            mmodal_assembly->SetReductionType(chrono::modal::ChModalAssembly::ReductionType::HERTING);
        else
            mmodal_assembly->SetReductionType(chrono::modal::ChModalAssembly::ReductionType::CRAIG_BAMPTON);

        sys.Add(mmodal_assembly);

        // Initialize mesh
        auto mesh_internal = chrono_types::make_shared<ChMesh>();
        mmodal_assembly->AddInternalMesh(mesh_internal);
        auto mesh_boundary = chrono_types::make_shared<ChMesh>();
        mmodal_assembly->AddMesh(mesh_boundary);
        mesh_internal->SetAutomaticGravity(false);
        mesh_boundary->SetAutomaticGravity(false);

        // Build boundary nodes
        auto my_node_L = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(mstart_x, 0, 0), QUNIT));
        my_node_L->SetMass(0);
        my_node_L->GetInertia().setZero();
        mesh_boundary->AddNode(my_node_L);

        auto my_node_R = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(mend_x, 0, 0), QUNIT));
        my_node_R->SetMass(0);
        my_node_R->GetInertia().setZero();
        mesh_boundary->AddNode(my_node_R);

        ChBuilderBeamEuler builder;
        // The other nodes are internal nodes: let the builder.BuildBeam add them to mesh_internal
        builder.BuildBeam(mesh_internal,       // the mesh where to put the created nodes and elements
                          section,             // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                          mn_ele,              // the number of ChElementBeamEuler to create
                          my_node_L,           // the 'A' point in space (beginning of beam)
                          my_node_R,           // the 'B' point in space (end of beam)
                          ChVector3d(0, 1, 0)  // the 'Y' up direction of the section for the beam
        );
    };

    // A function to divide a beam structure into several modal assemblies
    auto BuildBeamStructure = [&](std::vector<std::shared_ptr<ChModalAssembly>>& mmodal_assembly_list, double mstart_x,
                                  double mend_x, int mnparts, int mntotalele) {
        // Mesh the beam structure with several seperate modal assemblies
        double delta_x = (mend_x - mstart_x) / mnparts;
        for (int i_part = 0; i_part < mnparts; i_part++) {
            auto my_assembly = chrono_types::make_shared<ChModalAssembly>();
            mmodal_assembly_list.push_back(my_assembly);
            MakeSingleModalAssembly(my_assembly, mntotalele / mnparts, mstart_x + delta_x * i_part,
                                    mstart_x + delta_x * (i_part + 1));
        }

        // Build the fix links to connect modal assemblies
        if (mnparts > 1)
            for (int i_link = 0; i_link < mnparts - 1; i_link++) {
                auto node_L = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
                    mmodal_assembly_list.at(i_link)->GetMeshes().front()->GetNodes().back());
                auto node_R = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
                    mmodal_assembly_list.at(i_link + 1)->GetMeshes().front()->GetNodes().front());
                auto my_link = chrono_types::make_shared<ChLinkMateFix>();
                my_link->Initialize(node_L, node_R);
                sys.AddLink(my_link);
            }
    };

    // Modelling of crank
    std::vector<std::shared_ptr<ChModalAssembly>> modal_assembly_crank;
    BuildBeamStructure(modal_assembly_crank, 0, crank_L, crank_parts, crank_totalelements);

    // Root driving link of crank
    auto root_node_crank = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_crank.front()->GetMeshes().front()->GetNodes().front());
    auto my_drivinglink_crank = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_drivinglink_crank->Initialize(root_node_crank, my_ground, ChFrame<>(my_ground->GetPos(), QUNIT));
    auto driving_fun = chrono_types::make_shared<ChFunctionSetpoint>();
    driving_fun->SetSetpoint(0, 0);
    my_drivinglink_crank->SetAngleFunction(driving_fun);
    sys.AddLink(my_drivinglink_crank);

    // Retrieve the tip node of crank
    auto tip_node_crank = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_crank.back()->GetMeshes().front()->GetNodes().back());

    // Modelling of rod
    std::vector<std::shared_ptr<ChModalAssembly>> modal_assembly_rod;
    BuildBeamStructure(modal_assembly_rod, crank_L, crank_L + rod_L, rod_parts, rod_totalelements);

    // Root link of rod
    auto root_node_rod = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_rod.front()->GetMeshes().front()->GetNodes().front());
    auto my_midlink = chrono_types::make_shared<ChLinkMateRevolute>();
    my_midlink->Initialize(root_node_rod, tip_node_crank, tip_node_crank->Frame());
    sys.AddLink(my_midlink);

    // Retrieve the tip node of rod
    auto tip_node_rod = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_rod.back()->GetMeshes().front()->GetNodes().back());
    auto my_tiplink = chrono_types::make_shared<ChLinkMateRevolute>();
    my_tiplink->Initialize(tip_node_rod, my_slider, tip_node_rod->Frame());
    sys.AddLink(my_tiplink);

    // Retrieve the middle node of rod
    std::shared_ptr<ChNodeFEAxyzrot> mid_node_rod;
    if (rod_parts % 2) {  // rod_parts is odd: 1, 3, 5, 7, ...
        auto beam_nodes = modal_assembly_rod.at(rod_parts / 2)->GetMeshesInternal().front()->GetNodes();
        mid_node_rod = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(beam_nodes.at(beam_nodes.size() / 2));
    } else {  // rod_parts is even: 2, 4, 6, 8, ...
        auto beam_nodes = modal_assembly_rod.at(rod_parts / 2)->GetMeshes().front()->GetNodes();
        mid_node_rod = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(beam_nodes.front());
    }

    // gravity is zero
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Set linear solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
#else
    auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(qr_solver);
#endif

    sys.Setup();
    sys.Update();

    if (do_modal_reduction) {
        // ChGeneralizedEigenvalueSolverLanczos eigen_solver;
        ChGeneralizedEigenvalueSolverKrylovSchur eigen_solver;

        auto modes_settings = ChModalSolveUndamped(12, 1e-5, 500, 1e-10, false, eigen_solver);

        for (int i_part = 0; i_part < crank_parts; i_part++) {
            auto damping_beam = ChModalDampingReductionR(*modal_assembly_crank.at(i_part));
            // modal_assembly_crank.at(i_part)->verbose = true;
            modal_assembly_crank.at(i_part)->DoModalReduction(modes_settings, damping_beam);
            modal_assembly_crank.at(i_part)->WriteSubassemblyMatrices(
                true, true, true, true, (out_dir + "/crank_modal_assembly_" + std::to_string(i_part)).c_str());
        }

        for (int i_part = 0; i_part < rod_parts; i_part++) {
            auto damping_beam = ChModalDampingReductionR(*modal_assembly_rod.at(i_part));
            // modal_assembly_rod.at(i_part)->verbose = true;
            modal_assembly_rod.at(i_part)->DoModalReduction(modes_settings, damping_beam);
            modal_assembly_rod.at(i_part)->WriteSubassemblyMatrices(
                true, true, true, true, (out_dir + "/rod_modal_assembly_" + std::to_string(i_part)).c_str());
        }
    }

    // Do dynamics simulation
    {
        // use HHT second order integrator (but slower)
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        if (hht_stepper != nullptr) {
            // hht_stepper->SetVerbose(false);
            hht_stepper->SetStepControl(false);
            // hht_stepper->SetRelTolerance(1e-4);
            // hht_stepper->SetAbsTolerances(1e-8);
            // hht_stepper->SetAlpha(-0.2);
            // hht_stepper->SetModifiedNewton(false);
            // hht_stepper->SetMaxiters(20);
        }

        int Nframes = (int)(time_length / time_step);
        int frame = 0;
        double t0 = 5.0;  // the time instant of rotation stop

        mdeflection.resize(Nframes, 14);
        mdeflection.setZero();
        while (frame < Nframes) {
            double t = sys.GetChTime();
            if (t < t0)
                driving_fun->SetSetpoint(3.0 * CH_PI * t * t / 50.0, t);
            else {
                driving_fun->SetSetpoint(3.0 * CH_PI * t0 * t0 / 50.0, t);
            }

            sys.DoStepDynamics(time_step);

            // The middle node of connecting rod
            ChFrameMoving<> relative_frame_mid = root_node_rod->TransformParentToLocal(mid_node_rod->Frame());

            mdeflection(frame, 0) = sys.GetChTime();
            mdeflection(frame, 1) = root_node_crank->GetRot().GetRotVec().z() * CH_RAD_TO_DEG;
            // transverse displacement of middle point of the connecting rod
            mdeflection(frame, 2) = relative_frame_mid.GetPos().x();
            mdeflection(frame, 3) = relative_frame_mid.GetPos().y();
            mdeflection(frame, 4) = relative_frame_mid.GetPos().z();
            mdeflection(frame, 5) = relative_frame_mid.GetRot().GetRotVec().x();
            mdeflection(frame, 6) = relative_frame_mid.GetRot().GetRotVec().y();
            mdeflection(frame, 7) = relative_frame_mid.GetRot().GetRotVec().z();
            if (do_modal_reduction) {
                ChFrameMoving<> relative_frame_F = root_node_rod->TransformParentToLocal(modal_assembly_rod.back()->GetFloatingFrameOfReference());
                mdeflection(frame, 8) = relative_frame_F.GetPos().x();
                mdeflection(frame, 9) = relative_frame_F.GetPos().y();
                mdeflection(frame, 10) = relative_frame_F.GetPos().z();
                mdeflection(frame, 11) = relative_frame_F.GetRot().GetRotVec().x();
                mdeflection(frame, 12) = relative_frame_F.GetRot().GetRotVec().y();
                mdeflection(frame, 13) = relative_frame_F.GetRot().GetRotVec().z();
            }

            if (frame % 100 == 0) {
                std::cout << "t: " << sys.GetChTime() << "\t";
                std::cout << "Rot Angle (deg): " << mdeflection(frame, 1) << "\t";
                std::cout << "Rel. Def.:\t" << relative_frame_mid.GetPos().x() << "\t" << relative_frame_mid.GetPos().y()
                         << "\t" << relative_frame_mid.GetPos().z() << "\n";
            }
            frame++;
        }
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Directory for output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (create_directory(path(out_dir))) {
        ChTimer m_timer_computation;
        double time_corot = 0;
        double time_modal = 0;

        if (RUN_ORIGIN) {
            std::cout << "1. Run corotational beam model:\n";

            m_timer_computation.start();
            ChMatrixDynamic<> rel_deflection_corot;
            MakeAndRunDemo_SliderCrank(false, rel_deflection_corot);
            m_timer_computation.stop();

            time_corot = m_timer_computation.GetTimeSeconds();
            std::cout << "Computation time of corotational beam model: t_corot = \t" << time_corot << " seconds\n";
            m_timer_computation.reset();

            std::ofstream file_defl_corot((out_dir + "/deflection_corot.dat").c_str());
            file_defl_corot << std::setprecision(12) << std::scientific;
            StreamOut(rel_deflection_corot, file_defl_corot);
        }

        if (RUN_MODAL) {
            std::cout << "\n\n2. Run modal reduction model:\n";

            m_timer_computation.start();
            ChMatrixDynamic<> rel_deflection_modal;
            MakeAndRunDemo_SliderCrank(true, rel_deflection_modal);
            m_timer_computation.stop();

            time_modal = m_timer_computation.GetTimeSeconds();
            std::cout << "Computation time of modal reduction model: t_modal = \t" << time_modal << " seconds\n";
            m_timer_computation.reset();

            std::ofstream file_defl_modal((out_dir + "/deflection_modal.dat").c_str());
            file_defl_modal << std::setprecision(12) << std::scientific;
            StreamOut(rel_deflection_modal, file_defl_modal);
        }

        if (time_corot && time_modal)
            std::cout << "\n\n3. Ratio of computation time: t_modal / t_corot = \t" << time_modal / time_corot << "\n";
    } else {
        std::cout << "  ...Error creating subdirectories\n";
    }

    return 0;
}
