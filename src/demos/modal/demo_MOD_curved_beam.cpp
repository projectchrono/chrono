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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"

#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
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
const std::string out_dir = GetChronoOutputPath() + "CURVED_BEAM";

constexpr bool RUN_ORIGIN = true;
constexpr bool RUN_MODAL = true;
constexpr bool USE_HERTING = false;
constexpr bool USE_LINEAR_INERTIAL_TERM = true;

void MakeAndRunDemo_CurvedBeam(bool do_modal_reduction) {
    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double radius = 100.0;
    double rotangle = 45.0 * CH_DEG_TO_RAD;
    int n_parts = 4;
    int n_totalelements = n_parts * 8;

    double b = 1.0;
    double h = 1.0;
    double Area = b * h;
    double Iyy = 1 / 12.0 * b * h * h * h;
    double Izz = 1 / 12.0 * h * b * b * b;
    double Iyz = 0;
    //double Ip = Iyy + Izz;

    double rho = 7800 * 2.2046226218 / pow(39.37007874, 3);
    double mass_per_unit_length = rho * Area;
    double Jyy = rho * Iyy;
    double Jzz = rho * Izz;
    //double Jxx = rho * Ip;
    double Jyz = rho * Iyz;

    // Parent system: ground
    auto my_ground = chrono_types::make_shared<ChBody>();
    my_ground->SetMass(1.0);
    my_ground->SetPos(VNULL);
    my_ground->SetFixed(true);
    sys.AddBody(my_ground);

    // Prepare for mesh: Beam section
    auto section = chrono_types::make_shared<ChBeamSectionTimoshenkoAdvancedGenericFPM>();
    double Qy = 0;
    double Qz = 0;
    section->SetMassMatrixFPM(mass_per_unit_length, Jyy, Jzz, Jyz, Qy, Qz);
    double k11 = 1e7;
    double k22 = 4.166667e6;
    double k33 = 4.166667e6;
    double k44 = 7.03e5;
    double k55 = 8.333333e5;
    double k66 = 8.333333e5;
    double k16 = -8.333333e3;
    ChMatrixNM<double, 6, 6> Klaw;
    Klaw.setZero();
    Klaw(0, 0) = k11;
    Klaw(1, 1) = k22;
    Klaw(2, 2) = k33;
    Klaw(3, 3) = k44;
    Klaw(4, 4) = k55;
    Klaw(5, 5) = k66;
    Klaw(0, 5) = k16;
    Klaw(5, 0) = k16;
    section->SetStiffnessMatrixFPM(Klaw);
    section->SetRayleighDampingBeta(0.002);
    section->SetRayleighDampingAlpha(0.000);

    auto tapered_section = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
    tapered_section->SetSectionA(section);
    tapered_section->SetSectionB(section);

    // A function to make a single modal assembly
    auto MakeSingleModalAssembly = [&](std::shared_ptr<ChModalAssembly> mmodal_assembly, int mn_ele,
                                       double mstart_angle, double mend_angle) {
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

        // Build nodes
        auto mbeam_nodes = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>(mn_ele + 1);
        double mdelta_angle = (mend_angle - mstart_angle) / mn_ele;
        for (int i_node = 0; i_node < mn_ele + 1; i_node++) {
            double loc_angle = mstart_angle + mdelta_angle * i_node;
            ChQuaternion qrot;
            qrot.SetFromAngleZ(-loc_angle);
            mbeam_nodes.at(i_node) = chrono_types::make_shared<ChNodeFEAxyzrot>(
                ChFrame<>({radius * sin(loc_angle), -radius * (1.0 - cos(loc_angle)), 0}, qrot));

            if (i_node == 0 || i_node == mn_ele)
                mesh_boundary->AddNode(mbeam_nodes.at(i_node));
            else
                mesh_internal->AddNode(mbeam_nodes.at(i_node));
        }

        // Build elements
        auto mbeam_elements = std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenkoFPM>>(mn_ele);
        for (int i_ele = 0; i_ele < mn_ele; i_ele++) {
            mbeam_elements.at(i_ele) = chrono_types::make_shared<ChElementBeamTaperedTimoshenkoFPM>();

            mbeam_elements.at(i_ele)->SetNodes(mbeam_nodes.at(i_ele), mbeam_nodes.at(i_ele + 1));

            ChMatrix33<> mrot;
            mrot.SetFromAxisX(mbeam_nodes.at(i_ele + 1)->Frame().GetPos() - mbeam_nodes.at(i_ele)->Frame().GetPos(),
                            VECT_Y);
            ChQuaternion<> elrot = mrot.GetQuaternion();
            mbeam_elements.at(i_ele)->SetNodeAreferenceRot(elrot.GetConjugate() *
                                                           mbeam_elements.at(i_ele)->GetNodeA()->Frame().GetRot());
            mbeam_elements.at(i_ele)->SetNodeBreferenceRot(elrot.GetConjugate() *
                                                           mbeam_elements.at(i_ele)->GetNodeB()->Frame().GetRot());

            mbeam_elements.at(i_ele)->SetTaperedSection(tapered_section);

            mesh_internal->AddElement(mbeam_elements.at(i_ele));
        }
    };

    // Mesh the curved beam with several seperate modal assemblies
    std::vector<std::shared_ptr<ChModalAssembly>> modal_assembly_list;
    double delta_angle = rotangle / n_parts;
    for (int i_part = 0; i_part < n_parts; i_part++) {
        auto my_assembly = chrono_types::make_shared<ChModalAssembly>();
        modal_assembly_list.push_back(my_assembly);
        MakeSingleModalAssembly(my_assembly, n_totalelements / n_parts, delta_angle * i_part,
                                delta_angle * (i_part + 1));
    }

    // Build the fix links to connect modal assemblies
    if (n_parts > 1)
        for (int i_link = 0; i_link < n_parts - 1; i_link++) {
            auto node_L = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
                modal_assembly_list.at(i_link)->GetMeshes().front()->GetNodes().back());
            auto node_R = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
                modal_assembly_list.at(i_link + 1)->GetMeshes().front()->GetNodes().front());
            auto my_link = chrono_types::make_shared<ChLinkMateFix>();
            my_link->Initialize(node_L, node_R);
            sys.AddLink(my_link);
        }

    // Root link
    auto root_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_list.front()->GetMeshes().front()->GetNodes().front());
    auto my_rootlink = chrono_types::make_shared<ChLinkMateFix>();
    my_rootlink->Initialize(root_node, my_ground);
    sys.AddLink(my_rootlink);

    // Retrieve the tip node
    auto tip_node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(
        modal_assembly_list.back()->GetMeshes().front()->GetNodes().back());
    ChVector3d tip_pos_x0 = tip_node->GetPos();

    // Gravity is zero
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

    // Do modal reduction for all modal assemblies
    if (do_modal_reduction) {
        // ChGeneralizedEigenvalueSolverLanczos eigen_solver;
        ChGeneralizedEigenvalueSolverKrylovSchur eigen_solver;

        auto modes_settings = ChModalSolveUndamped(12, 1e-5, 500, 1e-10, false, eigen_solver);

        for (int i_part = 0; i_part < n_parts; i_part++) {
            auto damping_beam = ChModalDampingReductionR(*modal_assembly_list.at(i_part));
            // modal_assembly_list.at(i_part)->verbose = true;
            modal_assembly_list.at(i_part)->DoModalReduction(modes_settings, damping_beam);
            modal_assembly_list.at(i_part)->WriteSubassemblyMatrices(
                true, true, true, true, (out_dir + "/modal_assembly_" + std::to_string(i_part)).c_str());
        }
    }

    // Apply the external load
    double Pz = 600;
    tip_node->SetForce(ChVector3d(0, 0, Pz));
    std::cout << "The applied force is: " << Pz << " N\n";

    // Static analysis
    sys.DoStaticNonlinear(100);

    // Postprocess: print the tip displacement
    ChVector3d tip_displ = tip_node->GetPos() - tip_pos_x0;
    std::cout << "Tip displacement:\t" << tip_displ.x() << "\t" << tip_displ.y() << "\t" << tip_displ.z() << "\n";
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Directory for output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (create_directory(path(out_dir))) {

        ChTimer m_timer_computation;
        double time_corot = 0;
        double time_modal = 0;

        if (RUN_ORIGIN) {
            std::cout << "1. Run corotational beam model:\n";

            m_timer_computation.start();
            MakeAndRunDemo_CurvedBeam(false);
            m_timer_computation.stop();

            time_corot = m_timer_computation.GetTimeSeconds();
            std::cout << "Computation time of corotational beam model: t_corot = \t" << time_corot << " seconds\n";
            m_timer_computation.reset();
        }

        if (RUN_MODAL) {
            std::cout << "\n\n2. Run modal reduction model:\n";

            m_timer_computation.start();
            MakeAndRunDemo_CurvedBeam(true);
            m_timer_computation.stop();

            time_modal = m_timer_computation.GetTimeSeconds();
            std::cout << "Computation time of modal reduction model: t_modal = \t" << time_modal << " seconds\n";
            m_timer_computation.reset();
        }

        if (time_corot && time_modal)
            std::cout << "\n\n3. Ratio of computation time: t_modal / t_corot = \t" << time_modal / time_corot << "\n";
    } else {
        std::cout << "  ...Error creating subdirectories\n";
    }

    return 0;
}
