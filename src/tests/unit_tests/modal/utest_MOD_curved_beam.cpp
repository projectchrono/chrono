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
// WARNING: examples are using imperial units!!!
// WARNING: in the paper all units that should refer to forces are actually
//          adimensionalized with respect to gravity!!!
//          So forces of 600lb are actually 600lb*in/s^2
//          So Young mod of 1e7 psi is actually 1e7 lb*in/s^2/in^2
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

constexpr double meters_to_inches = 39.37007874;
constexpr double kilograms_to_libres = 2.2046226218;

class LoadScaling : public ChStaticNonLinearAnalysisIncremental::LoadIncrementCallback {
  public:
    LoadScaling(std::shared_ptr<ChNodeFEAxyzrot> node, double nominal_load)
        : m_node(node), m_nominal_load(nominal_load) {}

    virtual void OnLoadScaling(const double load_scaling,
                               const int iteration_n,
                               ChStaticNonLinearAnalysisIncremental* analysis) override {
        m_node->SetForce(ChVector3d(0, 0, load_scaling * m_nominal_load));
    }

  private:
    std::shared_ptr<ChNodeFEAxyzrot> m_node;
    double m_nominal_load;
};

// -----------------------------------------------------------------------------

ChVector3d RunCurvedBeam(bool do_modal_reduction, bool use_herting, bool use_gravity, bool verbose) {
    // Create a Chrono physical system
    ChSystemNSC sys;

    sys.Clear();
    sys.SetChTime(0);

    // WARNING: using IMPERIAL UNITS

    // Parameters
    double radius = 100.0;  // inches
    double rotangle = 45.0 * CH_DEG_TO_RAD;
    int n_parts = 5;
    int n_totalelements = n_parts * 8;

    // damping coefficients
    double damping_alpha = 0;
    double damping_beta = 0.002;

    double b = 1.0;  // inches
    double h = 1.0;  // inches
    double Area = b * h;
    double Iyy = 1 / 12.0 * b * h * h * h;
    double Izz = 1 / 12.0 * h * b * b * b;
    double Iyz = 0;

    double rho = 7800 * kilograms_to_libres / std::pow(meters_to_inches, 3);  // CONVERTING DENSITY IN IMPERIAL UNITS
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
    const double E = 1e7;
    double k11 = E;
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
        mod_assem->SetModalAutomaticGravity(use_gravity);
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
        mesh_internal->SetAutomaticGravity(use_gravity);
        mesh_boundary->SetAutomaticGravity(use_gravity);

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

    // Mesh the curved beam with several separate modal assemblies to deal with the significant geometrical nonlinearity
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
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, use_gravity ? -5.0 : 0.0));

    // double grav_acc_inches = 9.81 * meters_to_inches;
    //  sys.SetGravitationalAcceleration(ChVector3d(0, 0, USE_GRAVITY ? -grav_acc_inches : 0.0));  // -Z axis
    //   sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0.0));  // -Z axis

    // Set linear solver for both system simulation and modal reduction
    // #undef CHRONO_PARDISO_MKL
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
    mkl_solver->LockSparsityPattern(false);

    for (int i_part = 0; i_part < n_parts; i_part++) {
        auto mkl_modal_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        modal_assembly_list.at(i_part)->SetModalSolver(mkl_modal_solver);
    }
#else
    auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(qr_solver);

    for (int i_part = 0; i_part < n_parts; i_part++) {
        auto qr_solver = chrono_types::make_shared<ChSolverSparseQR>();
        modal_assembly_list.at(i_part)->SetModalSolver(qr_solver);
    }
    qr_solver->LockSparsityPattern(false);
#endif

    sys.Setup();
    sys.Update(UpdateFlags::UPDATE_ALL);

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

    // Create a load scaling callback object
    // Bathe paper talks about a load parameter k = P*R^2/E/I
    double Pz = 600;
    // double load_parameter = Pz * radius * radius / E / Izz;

    // double load_parameter = 6.0;
    // double Pz = load_parameter * E * Izz / radius / radius;
    auto load_scaling = chrono_types::make_shared<LoadScaling>(tip_node, Pz);

    // Static analysis (incremental)
    ChStaticNonLinearAnalysisIncremental static_analysis;
    static_analysis.SetLoadIncrementCallback(load_scaling);
    static_analysis.SetMaxIterationsNewton(100);
    static_analysis.SetCorrectionTolerance(1e-4, 1e-8);
    static_analysis.SetVerbose(verbose);

    sys.DoStaticAnalysis(static_analysis);

    // Print the tip displacement
    auto res = (tip_node->GetPos() - tip_pos_x0);
    // std::cout << "Tip rest position:\t" << tip_pos_x0.x() << "\t" << tip_pos_x0.y() << "\t" << tip_pos_x0.z() <<
    // "\n"; std::cout << "Tip original position:\t" << tip_node->GetPos().x() << "\t" << tip_node->GetPos().y() << "\t"
    //           << tip_node->GetPos().z() << "\n";
    std::cout << "Tip displacement:\t" << res.x() << "\t" << res.y() << "\t" << res.z();
    // std::cout << "Load parameter 'k': " << load_parameter << std::endl;
    // std::cout << "Adimensional tip displacement:\t" << res.x() / radius << "\t" << res.y() / radius << "\t" <<
    // res.z() / radius << "\n";

    return res;
}

int main(int argc, char* argv[]) {
    bool verbose = false;
    double tol = 0.05;

    //// WARNING: consider that are inches
    const ChVector3d reference_solution_Sonneville = {-23.821, 13.732, 53.610};  // 16 elements
    // const ChVector3d reference_solution_Bathe = {-23.5, 13.4, 53.604};               // 8 elements
    // const ChVector3d reference_solution_Ibrahimbegovic = {-23.814, -13.729, 53.605};  // 8 elements

    // Bathe reference values for K = 6.0, taken from Fig. 9
    // const ChVector3d ref_values = {-0.20807, 0.1222, 0.50532};

    const ChVector3d ref_values_no_grav = reference_solution_Sonneville;

    std::cout << "############### WITH GRAVITY DISABLED ###############\n";
    bool use_gravity = false;
    std::cout << "Original model (not reduced):\n";
    ChVector3d res_full = RunCurvedBeam(false, false, use_gravity, verbose);
    bool check_full = (res_full - ref_values_no_grav).eigen().norm() / ref_values_no_grav.Length() < tol;
    std::cout << "\nSTATUS: " << (check_full ? "PASSED" : "FAILED");

    std::cout << "\n\nReduced Model - Craig Bampton:\n";
    ChVector3d res_modal_CraigBampton_nograv = RunCurvedBeam(true, false, use_gravity, verbose);
    bool check_CraigBampton_nograv =
        (res_modal_CraigBampton_nograv - ref_values_no_grav).eigen().norm() / ref_values_no_grav.Length() < tol;
    std::cout << "\nSTATUS: " << (check_CraigBampton_nograv ? "PASSED" : "FAILED");

    std::cout << "\n\nReduced Model - Herting:\n";
    ChVector3d res_modal_Herting_nograv = RunCurvedBeam(true, true, use_gravity, verbose);
    bool check_Herting_nograv =
        (res_modal_Herting_nograv - ref_values_no_grav).eigen().norm() / ref_values_no_grav.Length() < tol;
    std::cout << "\nSTATUS: " << (check_Herting_nograv ? "PASSED" : "FAILED");

    std::cout << "\n\n############### WITH GRAVITY ENABLED ###############\n";

    use_gravity = true;
    std::cout << "\n\nOriginal model (not reduced) - WITH GRAVITY - USED AS REFERENCE:\n";
    ChVector3d res_full_grav = RunCurvedBeam(false, false, use_gravity, verbose);

    std::cout << "\n\nReduced Model - Craig Bampton - WITH GRAVITY:\n";
    ChVector3d res_modal_CraigBampton_gravity = RunCurvedBeam(true, false, use_gravity, verbose);
    bool check_CraigBampton_gravity =
        (res_modal_CraigBampton_gravity - res_full_grav).eigen().norm() / res_full_grav.Length() < tol;
    std::cout << "\nSTATUS: " << (check_CraigBampton_gravity ? "PASSED" : "FAILED");

    std::cout << "\n\nReduced Model - Herting - WITH GRAVITY:\n";
    ChVector3d res_modal_Herting_gravity = RunCurvedBeam(true, true, use_gravity, verbose);
    bool check_Herting_gravity =
        (res_modal_Herting_gravity - res_full_grav).eigen().norm() / res_full_grav.Length() < tol;
    std::cout << "\nSTATUS: " << (check_Herting_gravity ? "PASSED" : "FAILED");

    bool passed =
        check_CraigBampton_nograv && check_Herting_nograv && check_CraigBampton_gravity && check_Herting_gravity;
    std::cout << "\nUNIT TEST of modal assembly with curved beam: " << (passed ? "PASSED" : "FAILED");

    return !passed;
}
