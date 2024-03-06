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
    double rotangle = 45.0 * CH_C_DEG_TO_RAD;
    int n_elements = 16;

    double b = 1.0;
    double h = 1.0;
    double Area = b * h;
    double Iyy = 1 / 12.0 * b * h * h * h;
    double Izz = 1 / 12.0 * h * b * b * b;
    double Iyz = 0;
    double Ip = Iyy + Izz;

    double rho = 7800 * 2.2046226218 / pow(39.37007874, 3);
    double mass_per_unit_length = rho * Area;
    double Jyy = rho * Iyy;
    double Jzz = rho * Izz;
    double Jxx = rho * Ip;
    double Jyz = rho * Iyz;

    // parent system
    auto my_ground = chrono_types::make_shared<ChBody>();
    my_ground->SetMass(1.0);
    my_ground->SetPos(VNULL);
    my_ground->SetBodyFixed(true);
    sys.AddBody(my_ground);

    // Prepare for mesh: Beam section:
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
    section->SetBeamRaleyghDampingBeta(0.002);
    section->SetBeamRaleyghDampingAlpha(0.000);

    auto tapered_section = chrono_types::make_shared<ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM>();
    tapered_section->SetSectionA(section);
    tapered_section->SetSectionB(section);

    // for crank
    auto assembly_beam = chrono_types::make_shared<ChModalAssembly>();
    assembly_beam->use_linear_inertial_term = USE_LINEAR_INERTIAL_TERM;
    if (USE_HERTING)
        assembly_beam->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Herting;
    else
        assembly_beam->modal_reduction_type = chrono::modal::ChModalAssembly::Reduction_Type::Craig_Bampton;
    sys.Add(assembly_beam);

    auto mesh_internal_beam = chrono_types::make_shared<ChMesh>();
    assembly_beam->AddInternal(mesh_internal_beam);  // NOTE: MESH FOR INTERNAL NODES: USE assembly->AddInternal()
    auto mesh_boundary_beam = chrono_types::make_shared<ChMesh>();
    assembly_beam->Add(mesh_boundary_beam);  // NOTE: MESH FOR BOUNDARY NODES: USE assembly->Add()
    mesh_internal_beam->SetAutomaticGravity(false);
    mesh_boundary_beam->SetAutomaticGravity(false);

    auto beam_nodes = std::vector<std::shared_ptr<ChNodeFEAxyzrot>>(n_elements + 1);
    for (int i_node = 0; i_node < n_elements + 1; i_node++) {
        double loc_angle = rotangle / n_elements * i_node;
        ChQuaternion qrot;
        qrot.Q_from_AngZ(-loc_angle);
        beam_nodes.at(i_node) = chrono_types::make_shared<ChNodeFEAxyzrot>(
            ChFrame<>({radius * sin(loc_angle), -radius * (1.0 - cos(loc_angle)), 0}, qrot));

        if (i_node == 0 || i_node == n_elements)
            mesh_boundary_beam->AddNode(beam_nodes.at(i_node));
        else
            mesh_internal_beam->AddNode(beam_nodes.at(i_node));
    }
    ChVector<> tip_pos_x0 = beam_nodes.back()->GetPos();

    auto my_rootlink = chrono_types::make_shared<ChLinkMateFix>();
    my_rootlink->Initialize(beam_nodes[0], my_ground, ChFrame<>(my_ground->GetPos(), QUNIT));
    sys.AddLink(my_rootlink);

    auto beam_elements = std::vector<std::shared_ptr<ChElementBeamTaperedTimoshenkoFPM>>(n_elements);
    for (int i_ele = 0; i_ele < n_elements; i_ele++) {
        beam_elements.at(i_ele) = chrono_types::make_shared<ChElementBeamTaperedTimoshenkoFPM>();

        beam_elements.at(i_ele)->SetNodes(beam_nodes.at(i_ele), beam_nodes.at(i_ele + 1));

        ChMatrix33<> mrot;
        mrot.Set_A_Xdir(beam_nodes.at(i_ele + 1)->Frame().GetPos() - beam_nodes.at(i_ele)->Frame().GetPos(), VECT_Y);
        ChQuaternion<> elrot = mrot.Get_A_quaternion();
        beam_elements.at(i_ele)->SetNodeAreferenceRot(elrot.GetConjugate() %
                                                      beam_elements.at(i_ele)->GetNodeA()->Frame().GetRot());
        beam_elements.at(i_ele)->SetNodeBreferenceRot(elrot.GetConjugate() %
                                                      beam_elements.at(i_ele)->GetNodeB()->Frame().GetRot());

        beam_elements.at(i_ele)->SetTaperedSection(tapered_section);

        mesh_internal_beam->AddElement(beam_elements.at(i_ele));
    }

    // gravity is zero
    sys.Set_G_acc(ChVector<>(0, 0, 0));

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

        auto damping_beam = ChModalDampingReductionR(*assembly_beam);
        assembly_beam->verbose = true;
        assembly_beam->SwitchModalReductionON(modes_settings, damping_beam);
        assembly_beam->DumpSubassemblyMatrices(true, true, true, true, (out_dir + "/assembly_curved_reduced").c_str());
    }

    double Pz = 600;
    beam_nodes.back()->SetForce(ChVector<>(0, 0, Pz));
    GetLog() << "The applied force is: " << Pz << " N\n";
    sys.DoStaticNonlinear(100);

    ChVector<> tip_pos = beam_nodes.back()->GetPos();
    ChVector<> tip_displ = tip_pos - tip_pos_x0;
    GetLog() << "Tip displacement:\t" << tip_displ.x() << "\t" << tip_displ.y() << "\t" << tip_displ.z() << "\n";
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2021 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Directory for output data
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (create_directory(path(out_dir))) {
        if (RUN_ORIGIN) {
            GetLog() << "Run corotational beam model:\n";
            MakeAndRunDemo_CurvedBeam(false);
        }

        if (RUN_MODAL) {
            GetLog() << "\n\nRun modal reduction model:\n";
            MakeAndRunDemo_CurvedBeam(true);
        }
    } else {
        GetLog() << "  ...Error creating subdirectories\n";
    }

    return 0;
}
