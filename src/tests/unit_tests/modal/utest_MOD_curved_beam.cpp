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
// Authors: Chao Peng, Alessandro Tasora
// =============================================================================
//
// Unit test for modal assembly using the "curved beam" model.
//
// This benchmark model "curved beam" is originally proposed in:
// 1. Bathe, K.J., Bolourchi, S.: Large displacement analysis of three-dimensional
// beam structures. Int. J. Numer. Methods Eng.14(7), 961–986(1979).
// and then used in the research paper:
// 2. Sonneville, V., Scapolan, M., Shan, M. et al. Modal reduction procedures
// for flexible multibody dynamics. Multibody Syst Dyn 51, 377–418 (2021).

// The simulation results from modal redution are compared against the results from
// corotational formulation in chrono::fea module.
//
// Successful execution of this unit test may validate: the material stiffness
// matrix, the geometric stiffness matrix, and the gravity load
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"

#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChMesh.h"
#include "chrono_modal/ChModalAssembly.h"

#include "chrono/solver/ChDirectSolverLS.h"
#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;

void RunCurvedBeam(bool do_modal_reduction, bool use_herting, ChVector3d& res) {
    // Create a Chrono physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // Parameters
    double radius = 100.0;
    double rotangle = 45.0 * CH_DEG_TO_RAD;
    int n_parts = 5;
    int n_totalelements = n_parts * 8;

    // damping coefficients
    double damping_alpha = 0;
    double damping_beta = 0.002;

    double b = 1.0;
    double h = 1.0;
    double Area = b * h;
    double Iyy = 1 / 12.0 * b * h * h * h;
    double Izz = 1 / 12.0 * h * b * b * b;
    double Iyz = 0;

    double rho = 7800 * 2.2046226218 / std::pow(39.37007874, 3);
    double mass_per_unit_length = rho * Area;
    double Jyy = rho * Iyy;
    double Jzz = rho * Izz;
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
    section->SetRayleighDampingBeta(damping_beta);
    section->SetRayleighDampingAlpha(damping_alpha);

    auto tapered_section = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
    tapered_section->SetSectionA(section);
    tapered_section->SetSectionB(section);

    // A function to make a single modal assembly
    auto MakeSingleModalAssembly = [&](std::shared_ptr<ChModalAssembly> mod_assem, int mn_ele, double mstart_angle,
                                       double mend_angle) {
        // Settings
        mod_assem->SetInternalNodesUpdate(true);
        mod_assem->SetUseLinearInertialTerm(true);
        mod_assem->SetUseStaticCorrection(false);
        mod_assem->SetModalAutomaticGravity(true);  // with gravity
        if (use_herting)
            mod_assem->SetReductionType(chrono::modal::ChModalAssembly::ReductionType::HERTING);
        else
            mod_assem->SetReductionType(chrono::modal::ChModalAssembly::ReductionType::CRAIG_BAMPTON);
        sys.Add(mod_assem);

        // Initialize mesh
        auto mesh_internal = chrono_types::make_shared<ChMesh>();
        mod_assem->AddInternalMesh(mesh_internal);
        auto mesh_boundary = chrono_types::make_shared<ChMesh>();
        mod_assem->AddMesh(mesh_boundary);
        mesh_internal->SetAutomaticGravity(true);  // with gravity
        mesh_boundary->SetAutomaticGravity(true);  // with gravity

        // Build nodes
        auto mbeam_nodes = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>(mn_ele + 1);
        double mdelta_angle = (mend_angle - mstart_angle) / mn_ele;
        for (int i_node = 0; i_node < mn_ele + 1; i_node++) {
            double loc_angle = mstart_angle + mdelta_angle * i_node;
            ChQuaternion qrot;
            qrot.SetFromAngleZ(-loc_angle);
            mbeam_nodes.at(i_node) = chrono_types::make_shared<ChNodeFEAxyzrot>(
                ChFrame<>({radius * std::sin(loc_angle), -radius * (1.0 - std::cos(loc_angle)), 0}, qrot));

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

    // Mesh the curved beam with several seperate modal assemblies to deal with the significant geometrical nonliearity
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
    auto tip_node =
        std::dynamic_pointer_cast<ChNodeFEAxyzrot>(modal_assembly_list.back()->GetMeshes().front()->GetNodes().back());
    // Store the initial position of tip node
    ChVector3d tip_pos_x0 = tip_node->GetPos();

    // Set gravity
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));  // -Z axis

    // Set linear solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
#else
    auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(qr_solver);
#endif

    sys.Setup();
    sys.Update(false);

    // Do modal reduction for all modal assemblies
    if (do_modal_reduction) {
        auto eigen_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();

        // The success of eigen solve is sensitive to the frequency shift (1e-4). If the eigen solver fails, try to
        // tune the shift value.
        ChModalSolverUndamped<ChUnsymGenEigenvalueSolverKrylovSchur> modal_solver(12, 1e-4, true, false, eigen_solver);
        auto damping_beam = ChModalDampingRayleigh(damping_alpha, damping_beta);

        for (int i_part = 0; i_part < n_parts; i_part++) {
            modal_assembly_list.at(i_part)->DoModalReduction(modal_solver, damping_beam);
        }
    }

    // Apply the external load
    double Pz = 600;
    tip_node->SetForce(ChVector3d(0, 0, Pz));

    // Static analysis
    sys.DoStaticNonlinear(100);

    // Print the tip displacement
    res = tip_node->GetPos() - tip_pos_x0;
    std::cout << "Tip displacement is:\t" << res.x() << "\t" << res.y() << "\t" << res.z() << "\n";
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    double tol = 1.0;

    std::cout << "1. Run corotational beam model not reduced:\n";
    // ChModalAssembly should be able to run successfully in the full state
    ChVector3d res_corot;
    RunCurvedBeam(false, false, res_corot);

    std::cout << "\n\n2. Run modal reduction model with Craig Bampton method:\n";
    ChVector3d res_modal_CraigBampton;
    RunCurvedBeam(true, false, res_modal_CraigBampton);
    bool check_CraigBampton = (res_modal_CraigBampton - res_corot).eigen().norm() < tol;

    std::cout << "\n\n3. Run modal reduction model with Herting method:\n";
    ChVector3d res_modal_Herting;
    RunCurvedBeam(true, true, res_modal_Herting);
    bool check_Herting = (res_modal_Herting - res_corot).eigen().norm() < tol;

    bool is_passed = check_CraigBampton && check_Herting;
    std::cout << "\nUNIT TEST of modal assembly with curved beam: " << (is_passed ? "PASSED" : "FAILED") << std::endl;

    return !is_passed;
}
