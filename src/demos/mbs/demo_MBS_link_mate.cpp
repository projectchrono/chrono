// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Chao Peng, Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demonstration of ChLinkMateGeneric() and the importance of its tangent
// stiffness matrix in the static and eigenvalue analysis.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace filesystem;

// Used to sort the Eigen matrix
namespace Eigen {
template <class T>
void swap(T&& a, T&& b) {
    a.swap(b);
}
}  // namespace Eigen

class eig_vect_and_val {
  public:
    Eigen::VectorXcd eigen_vect;
    std::complex<double> eigen_val;

    // sort by imaginary part
    bool operator<(const eig_vect_and_val& str) const { return (eigen_val.imag() < str.eigen_val.imag()); }
};

class EigenSolver {
  public:
    EigenSolver() {}
    EigenSolver(ChSystemNSC& msys) {
        ChSparseMatrix M_sp;
        ChSparseMatrix K_sp;
        ChSparseMatrix R_sp;
        ChSparseMatrix Cq_sp;
        msys.GetMassMatrix(&M_sp);
        msys.GetStiffnessMatrix(&K_sp);
        msys.GetDampingMatrix(&R_sp);
        msys.GetConstraintJacobianMatrix(&Cq_sp);

        int n_v = Cq_sp.cols();
        int n_c = Cq_sp.rows();

        ChMatrixDynamic<double> Ad(2 * n_v + n_c, 2 * n_v + n_c);
        Ad.setZero();
        ChMatrixDynamic<double> Ed(2 * n_v + n_c, 2 * n_v + n_c);
        Ed.setZero();
        // A  =  [  0     I      0   ]
        //       [ -K    -R     -Cq' ]
        //       [ -Cq    0      0   ]
        Ad << Eigen::MatrixXd::Zero(n_v, n_v), Eigen::MatrixXd::Identity(n_v, n_v), Eigen::MatrixXd::Zero(n_v, n_c),
            -K_sp.toDense(), -R_sp.toDense(), -Cq_sp.toDense().transpose(), -Cq_sp.toDense(),
            Eigen::MatrixXd::Zero(n_c, n_v), Eigen::MatrixXd::Zero(n_c, n_c);

        // E  =  [  I     0     0 ]
        //       [  0     M     0 ]
        //       [  0     0     0 ]
        Ed << Eigen::MatrixXd::Identity(n_v, n_v), Eigen::MatrixXd::Zero(n_v, n_v), Eigen::MatrixXd::Zero(n_v, n_c),
            Eigen::MatrixXd::Zero(n_v, n_v), M_sp.toDense(), Eigen::MatrixXd::Zero(n_v, n_c),
            Eigen::MatrixXd::Zero(n_c, n_v), Eigen::MatrixXd::Zero(n_c, n_v), Eigen::MatrixXd::Zero(n_c, n_c);

        Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges;
        ges.compute(Ad, Ed);

        std::vector<std::complex<double>> eigen_values;
        std::vector<Eigen::VectorXcd> eigen_vectors;

        for (int i = 0; i < ges.betas().size(); i++) {
            if (ges.betas()(i)) {
                std::complex<double> e_temp = ges.alphas()(i) / ges.betas()(i);
                if (std::abs(e_temp) < 1e6) {
                    eig_vect_and_val vec_and_val{ges.eigenvectors().col(i).head(n_v), e_temp};
                    all_eigen_vectors_and_values.push_back(vec_and_val);
                }
            }
        }

        // sort
        std::sort(all_eigen_vectors_and_values.begin(), all_eigen_vectors_and_values.end());
    }

    void ShowAllEigenvalues() {
        if (all_eigen_vectors_and_values.size() == 0)
            GetLog() << "There is not any eigenvalue found for the system.\n";
        else {
            GetLog() << "The eigenvalues of the system are:\n";
            for (int i = 0; i < all_eigen_vectors_and_values.size(); i++) {
                GetLog() << "Eigenvalue " << std::to_string(i + 1).c_str()
                         << ":\tReal: " << all_eigen_vectors_and_values.at(i).eigen_val.real()
                         << "\tImag: " << all_eigen_vectors_and_values.at(i).eigen_val.imag() << "\n";
            }
        }
    }

    eig_vect_and_val GetMode(int imode) {
        eig_vect_and_val eig_o;

        int nmodes = (int)(all_eigen_vectors_and_values.size() / 2);
        // int imode = 1;  // Which order of mode are you interested in? starting from 1
        eig_o.eigen_val = all_eigen_vectors_and_values.at(nmodes + imode - 1).eigen_val;
        eig_o.eigen_vect = all_eigen_vectors_and_values.at(nmodes + imode - 1).eigen_vect;
        eig_o.eigen_vect = eig_o.eigen_vect / eig_o.eigen_vect.cwiseAbs().maxCoeff();  // normalized to unit

        GetLog() << "\nThe eigenvalue of mode " << imode << " is: "
                 << ":\tReal: " << eig_o.eigen_val.real() << ":\tImag: " << eig_o.eigen_val.imag() << "\n";

        return eig_o;
    }

    std::vector<eig_vect_and_val> all_eigen_vectors_and_values;
};

//====================================
// Test 1
// First example: Pendulum
//====================================
void test_pendulum() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: static and eigenvalue analysis of a pendulum\n\n";

    // Some variables to parameterize the model
    // The length of the pendulum, m
    double length = 4.0;
    // The mass of the end point, kg
    double tip_mass = 15.0;
    // The offset angle of the pendulum at the initial configuration, rad
    double offset_angle = 30.0 * CH_C_DEG_TO_RAD;

    // Gravity acceleration, m/s^2
    double gacc = 9.81;

    // Whether the tangent stiffness matrix of constraint (Kc) is used?
    // The Kc matrix is mandotary for the static and eigenvalue analysis of a pendulum,
    // otherwise it cannot find the static equilibrium status, and the eigenvalues are also zero
    // because there is no stiffness matrix in the system.
    bool use_Kc = true;

    // ====================================
    // Build the system
    // ====================================
    // The physical system: it contains all physical objects.
    ChSystemNSC sys;

    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(load_container);

    auto my_root = chrono_types::make_shared<ChBody>();
    my_root->SetCoord(VNULL, QUNIT);
    my_root->SetMass(1e6);
    my_root->SetInertiaXX(ChVector<>(1, 1, 1));
    my_root->SetNameString("base");
    my_root->SetBodyFixed(true);
    my_root->SetCollide(false);
    sys.AddBody(my_root);

    auto cyl_rev = chrono_types::make_shared<ChCylinderShape>(0.1, 0.4);
    my_root->AddVisualShape(cyl_rev, ChFrame<>(VNULL, Q_from_AngY(CH_C_PI_2)));

    auto my_mass = chrono_types::make_shared<ChBody>();
    ChVector<> mass_pos = ChVector<>(length * std::sin(offset_angle), 0, -length * std::cos(offset_angle));
    ChVector<> Zdir = (my_root->GetPos() - mass_pos).GetNormalized();
    ChVector<> Ydir = my_root->GetRot().GetYaxis().GetNormalized();
    ChVector<> Xdir = Ydir.Cross(Zdir).GetNormalized();
    ChVector<> mX;
    ChVector<> mY;
    ChVector<> mZ;
    Xdir.DirToDxDyDz(mX, mY, mZ, Ydir);
    ChQuaternion<> mass_rot = ChMatrix33<>(mX, mY, mZ).Get_A_quaternion();
    my_mass->SetCoord(mass_pos, mass_rot);
    my_mass->SetMass(tip_mass);
    my_mass->SetInertiaXX(ChVector<>(0, 0, 0));
    my_mass->SetNameString("mass");
    my_mass->SetCollide(false);
    sys.AddBody(my_mass);

    auto sph = chrono_types::make_shared<ChSphereShape>(0.3);
    sph->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    my_mass->AddVisualShape(sph);

    ChFrameMoving<> rel_frame;
    my_mass->TransformParentToLocal(my_root->GetFrame_COG_to_abs(), rel_frame);
    geometry::ChLineSegment seg(VNULL, rel_frame.GetPos());
    auto cyl = chrono_types::make_shared<ChCylinderShape>(0.05, seg.GetLength());
    cyl->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    my_mass->AddVisualShape(cyl, seg.GetFrame());

    // Revolute joint at the root
    auto my_joint = chrono_types::make_shared<ChLinkMateGeneric>();
    // RotY is free
    my_joint->SetConstrainedCoords(true, true, true, true, false, true);
    my_joint->SetNameString("revolute_joint");
    my_joint->Initialize(my_mass, my_root, my_root->GetFrame_COG_to_abs());
    my_joint->SetUseTangentStiffness(use_Kc);
    sys.Add(my_joint);

    // 3D visualization
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(800, 800);
    vis->SetWindowTitle("Test demo: Pendulum");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(3, 1, 3));
    vis->AddTypicalLights();
    vis->EnableBodyFrameDrawing(false);
    vis->EnableLinkFrameDrawing(false);

    // gravity
    sys.Set_G_acc({0, 0, -gacc});

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // The pendulum has one rigid-motion degree of freedom. We need to use the
    // static solver: ChStaticNonLinearRigidMotion().
    ChStaticNonLinearRigidMotion rigid_static_analysis;
    rigid_static_analysis.SetIncrementalSteps(10);
    rigid_static_analysis.SetMaxIterations(100);
    rigid_static_analysis.SetResidualTolerance(1e-16);
    rigid_static_analysis.SetVerbose(false);

    sys.Setup();

    // ====================================
    // Static analysis
    // ====================================
    GetLog() << "\nThe initial position of the end mass is:\n"
             << "\tx:  " << my_mass->GetPos().x() << "\ty:  " << my_mass->GetPos().y()
             << "\tz:  " << my_mass->GetPos().z() << "\n";

    // Firstly, we call DoFullAssembly() to calculate the reaction forces/torques at the initial configuration of
    // system.
    sys.DoFullAssembly();
    // Secondly, we perform the static analysis using the solver ChStaticNonLinearRigidMotion().
    sys.DoStaticAnalysis(rigid_static_analysis);

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawCoordsys(vis.get(), ChCoordsys<>(VNULL, QUNIT), 1.0);
        vis->EndScene();
    }

    GetLog() << "\nAfter doing the nonlinear static analysis:\n";
    GetLog() << "\tThe final position of the end mass is:\n"
             << "\t\tx:  " << my_mass->GetPos().x() << "\ty:  " << my_mass->GetPos().y()
             << "\tz:  " << my_mass->GetPos().z() << "\n";
    GetLog() << "\tThe reaction forces at the root are:\n"
             << "\t\tfx:  " << my_joint->Get_react_force().x() << "\tfy:  " << my_joint->Get_react_force().y()
             << "\tfz:  " << my_joint->Get_react_force().z() << "\n";
    GetLog() << "\tThe reaction torques at the root are:\n"
             << "\t\tmx:  " << my_joint->Get_react_torque().x() << "\tmy:  " << my_joint->Get_react_torque().y()
             << "\tmz:  " << my_joint->Get_react_torque().z() << "\n";

    // ====================================
    // Eigenvalue analysis
    // ====================================
    EigenSolver eig_solver(sys);
    eig_solver.ShowAllEigenvalues();

    GetLog() << "\nThe theoretical oscillation frequency is:\t" << std::sqrt(gacc / length) << " rad/s\n";
}

// ====================================
// Test 2
// Second example: Anchor chain
// ====================================
void test_anchorchain() {
    GetLog() << "\n-------------------------------------------------\n";
    GetLog() << "TEST: static and eigenvalue analysis of an anchor chain\n\n";

    // The mass and inertia properties of every link in the anchor chain
    double mass = 10;
    double Jxx = 10;
    double Jyy = 10;
    double Jzz = 10;

    // We have three anchors to determine the initial configuration of the anchor chain:
    // A: located at {0,0,0} as the left anchor point.
    // B: located at {xB,0,0} as the right anchor point.
    // C: located at {xC,0,zC} as the bottom anchor point.
    // A series of links are built between A and B, B and C via rigid bodies and joints
    // to form the shape 'V' at the initial configuration of the anchor chain.
    double xA = 0.0;
    double xB = 10.0;
    double xC = xB / 2.0;
    double zC = -6.0;

    // Gravity
    double gacc = 9.81;

    // Nrig rigid bodies are built between A,B and B,C.
    int Nrig = 20;

    // Whether the tangent stiffness matrix of constraint (Kc) is used?
    // The Kc matrix is mandotary for the static and eigenvalue analysis of the anchor chain,
    // otherwise it cannot find the static equilibrium status, and the eigenvalues are also zero
    // because there is no stiffness matrix in the system.
    bool use_Kc = true;

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.AddOtherPhysicsItem(load_container);

    auto box = chrono_types::make_shared<ChBoxShape>(0.2, 0.2, 0.2);
    box->SetColor(ChColor(1, 0, 0));

    auto wallA = chrono_types::make_shared<ChBody>();
    wallA->SetNameString("wallA");
    wallA->SetPos({xA, 0, 0});
    wallA->SetBodyFixed(true);
    wallA->AddVisualShape(box);
    sys.AddBody(wallA);

    auto anchorA = chrono_types::make_shared<ChBody>();
    anchorA->SetNameString("anchorA");
    anchorA->SetPos({xA, 0, 0});
    anchorA->SetRot(Q_from_AngY(CH_C_PI_4));
    anchorA->SetMass(mass);
    anchorA->SetInertiaXX({Jxx, Jyy, Jzz});
    anchorA->AddVisualShape(box);
    sys.AddBody(anchorA);

    auto jointA = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);
    jointA->Initialize(anchorA, wallA, wallA->GetFrame_COG_to_abs());
    jointA->SetUseTangentStiffness(use_Kc);
    sys.AddLink(jointA);

    auto wallB = chrono_types::make_shared<ChBody>();
    wallB->SetNameString("wallB");
    wallB->SetPos({xB, 0, 0});
    wallB->SetBodyFixed(true);
    wallB->AddVisualShape(box);
    sys.AddBody(wallB);

    auto anchorB = chrono_types::make_shared<ChBody>();
    anchorB->SetNameString("anchorB");
    anchorB->SetPos({xB, 0, 0});
    anchorB->SetRot(Q_from_AngY(-CH_C_PI_4));
    anchorB->SetMass(mass);
    anchorB->SetInertiaXX({Jxx, Jyy, Jzz});
    anchorB->AddVisualShape(box);
    sys.AddBody(anchorB);

    auto jointB = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);
    jointB->Initialize(anchorB, wallB, wallB->GetFrame_COG_to_abs());
    jointB->SetUseTangentStiffness(use_Kc);
    sys.AddLink(jointB);

    auto anchorC = chrono_types::make_shared<ChBody>();
    anchorC->SetNameString("anchorC");
    anchorC->SetPos({xC, 0.0, zC});
    anchorC->SetMass(mass);
    anchorC->SetInertiaXX({Jxx, Jyy, Jzz});
    anchorC->AddVisualShape(box);
    sys.AddBody(anchorC);

    std::vector<std::shared_ptr<ChBody>> knot_list;
    // Build N rigid bodies between pA and pB
    auto BuildAnchorChain_body = [&](std::shared_ptr<ChBody> pA, std::shared_ptr<ChBody> pB, int mN) {
        double len_tot = (pA->GetPos() - pB->GetPos()).Length();
        double len = len_tot / (double)(mN + 1);

        ChVector<> Xdir = (pB->GetPos() - pA->GetPos()).GetNormalized();
        ChVector<> Ydir = {0, 1, 0};
        ChVector<> Zdir = Xdir.Cross(Ydir).GetNormalized();
        ChVector<> mX;
        ChVector<> mY;
        ChVector<> mZ;
        Xdir.DirToDxDyDz(mX, mY, mZ, Ydir);
        ChQuaternion<> knot_rot = ChMatrix33<>(mX, mY, mZ).Get_A_quaternion();

        for (int i_body = 0; i_body < mN; i_body++) {
            auto knot = chrono_types::make_shared<ChBody>();
            ChVector<> deltaP = (pB->GetPos() - pA->GetPos()) / (double)(mN + 1);
            knot->SetPos(pA->GetPos() + deltaP * (i_body + 1));
            knot->SetRot(knot_rot);
            knot->SetMass(mass);
            knot->SetInertiaXX({Jxx, Jyy, Jzz});

            auto cyl_rev = chrono_types::make_shared<ChCylinderShape>(0.1, len * 0.8);
            cyl_rev->SetColor(ChColor(0, 0, 1));
            knot->AddVisualShape(cyl_rev, ChFrame<>(VNULL, Q_from_AngY(CH_C_PI_2)));

            knot_list.push_back(knot);
            sys.AddBody(knot);
        }
    };

    knot_list.push_back(anchorA);
    BuildAnchorChain_body(anchorA, anchorC, Nrig);
    knot_list.push_back(anchorC);
    BuildAnchorChain_body(anchorC, anchorB, Nrig);
    knot_list.push_back(anchorB);

    std::vector<std::shared_ptr<ChLinkMateGeneric>> link_list;
    for (int i_link = 0; i_link < knot_list.size() - 1; i_link++) {
        auto link_joint = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, false, false);

        double halfdis = 0.5 * (knot_list.at(i_link + 1)->GetPos() - knot_list.at(i_link)->GetPos()).Length();
        ChFrame<> ref_frame1({halfdis, 0, 0}, QUNIT);
        ChFrame<> ref_frame2({-halfdis, 0, 0}, QUNIT);
        link_joint->Initialize(knot_list.at(i_link), knot_list.at(i_link + 1), true, ref_frame1, ref_frame2);

        link_joint->SetUseTangentStiffness(use_Kc);

        link_list.push_back(link_joint);
        sys.AddLink(link_joint);
    }

    // Gravity
    sys.Set_G_acc({0, 0, -gacc});
    sys.Setup();

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // Create output directory and output file
    std::string out_dir = GetChronoOutputPath() + "ANCHOR_CHAIN";
    GetLog() << "out_dir is:\n" << out_dir << "\n";

    if (true) {  // static analysis
        GetLog() << "\n\n ******** Static analysis ******** \n\n";

        // Set solver for static analysis
        ChStaticNonLinearRigidMotion rigid_static_analysis;
        rigid_static_analysis.SetCorrectionTolerance(1e-16, 1e-16);
        rigid_static_analysis.SetIncrementalSteps(10);
        rigid_static_analysis.SetMaxIterations(100);
        rigid_static_analysis.SetVerbose(false);

        // There are (2*Nrig+1) mass points along the anchor chain.(Excluding the two ends A,B)
        // anchorA, anchorB could contribute a half mass onto the total mass of anchor chain.
        double total_mass = (2 * Nrig + 2) * mass;
        // The total length of anchor chain is:
        double total_length =
            (anchorA->GetPos() - anchorC->GetPos()).Length() + (anchorB->GetPos() - anchorC->GetPos()).Length();
        // The mass per unit length of the anchor chain
        double mass_per_unit_length = total_mass / total_length;

        GetLog() << "\nAt the initial configuration:\n";
        GetLog() << "anchorC position:\t" << anchorC->GetPos().x() << "\t" << anchorC->GetPos().y() << "\t"
                 << anchorC->GetPos().z() << "\n";

        // Firstly, we call DoFullAssembly() to calculate the reaction forces/torques at the initial configuration of
        // system.
        sys.DoFullAssembly();
        // Secondly, we perform the static analysis using the solver ChStaticNonLinearRigidMotion().
        sys.DoStaticAnalysis(rigid_static_analysis);

        GetLog() << "\nAfter doing the static nonlinear analysis:\n";
        GetLog() << "anchorC position:\t" << anchorC->GetPos().x() << "\t" << anchorC->GetPos().y() << "\t"
                 << anchorC->GetPos().z() << "\n";

        // The coordiantes of the rigid bodies in the equilibrium configuration
        ChMatrixDynamic<> coords;
        coords.resize(sys.Get_bodylist().size(), 3);
        for (int i_body = 0; i_body < sys.Get_bodylist().size(); i_body++) {
            coords(i_body, 0) = sys.Get_bodylist().at(i_body)->GetPos().x();
            coords(i_body, 1) = sys.Get_bodylist().at(i_body)->GetPos().y();
            coords(i_body, 2) = sys.Get_bodylist().at(i_body)->GetPos().z();
        }
        // sort according to the X coordinate (horizontal coordinate)
        std::sort(coords.rowwise().begin(), coords.rowwise().end(),
                  [](auto const& r1, auto const& r2) { return r1(0) < r2(0); });

        // The reaction forces and torques of the joints in the equilibrium configuration
        ChMatrixDynamic<> reactions;
        reactions.resize(sys.Get_linklist().size(), 7);
        for (int i_link = 0; i_link < sys.Get_linklist().size(); i_link++) {
            reactions(i_link, 0) = sys.Get_linklist().at(i_link)->GetLinkAbsoluteCoords().pos.x();

            ChVector<> f_loc = sys.Get_linklist().at(i_link)->Get_react_force();
            ChVector<> m_loc = sys.Get_linklist().at(i_link)->Get_react_torque();
            ChQuaternion<> q_link = sys.Get_linklist().at(i_link)->GetLinkAbsoluteCoords().rot;
            // Transform the reaction forces and torques of the joints from local frame to the absolute frame.
            // The horizontal reaction forces (along X direction) should be equal among all joints for the catenary
            // curve.
            ChVector<> f_out = q_link.Rotate(f_loc);
            ChVector<> m_out = q_link.Rotate(m_loc);

            reactions(i_link, 1) = f_out.x();  // it should be a constant value
            reactions(i_link, 2) = f_out.y();
            reactions(i_link, 3) = f_out.z();
            reactions(i_link, 4) = m_out.x();
            reactions(i_link, 5) = m_out.y();
            reactions(i_link, 6) = m_out.z();
        }
        // sort according to the X coordinate (horizontal coordinate)
        std::sort(reactions.rowwise().begin(), reactions.rowwise().end(),
                  [](auto const& r1, auto const& r2) { return r1(0) < r2(0); });

        // The horizontal reaction force of the catenary curve
        double T0 = reactions.col(1).cwiseAbs().mean();
        // The coefficient of the catenary curve
        double a_coeff = T0 / (gacc * mass_per_unit_length);
        // Offset of the theoretical catenary curve
        double z_offset = anchorC->GetPos().z() - a_coeff;
        double x_offset = xC;
        // catenary curve comparison with the analytical formula
        ChMatrixDynamic<> catenary_cmp;
        catenary_cmp.resize(sys.Get_bodylist().size(), 3);
        for (int i_body = 0; i_body < sys.Get_bodylist().size(); i_body++) {
            catenary_cmp(i_body, 0) = coords(i_body, 0);  // X coordinate
            catenary_cmp(i_body, 1) = coords(i_body, 2);  // Z coordiante from simulation
            // Z coordiante from the theoretical catinary curve
            catenary_cmp(i_body, 2) = a_coeff * std::cosh((coords(i_body, 0) - x_offset) / a_coeff) + z_offset;
        }

        if (create_directory(path(out_dir))) {
            // coordinates of rigid bodies
            ChStreamOutAsciiFile file_coords((out_dir + "/equilibrium_coords.dat").c_str());
            file_coords.SetNumFormat("%.12g");
            StreamOUTdenseMatlabFormat(coords, file_coords);

            // catinary curve for comparison with the analytical formula
            ChStreamOutAsciiFile file_catenary((out_dir + "/catenary_cmp.dat").c_str());
            file_catenary.SetNumFormat("%.12g");
            StreamOUTdenseMatlabFormat(catenary_cmp, file_catenary);

            // reaction forces and torques of all joints
            ChStreamOutAsciiFile file_reactions((out_dir + "/equilibrium_reactions.dat").c_str());
            file_reactions.SetNumFormat("%.12g");
            StreamOUTdenseMatlabFormat(reactions, file_reactions);
        } else {
            GetLog() << "  ...Error creating subdirectories\n";
        }
    }

    if (true) {  // eigenvalue analysis
        GetLog() << "\n\n ******** Eigenvalue analysis ******** \n\n";

        // solve the eigenvalues at the equilibrium status
        EigenSolver eig_solver(sys);
        eig_solver.ShowAllEigenvalues();

        int nmodes = 5;  // how many modes are you going to extract?
        ChMatrixDynamic<> modal_freq(nmodes, 3);
        for (int imode = 1; imode <= nmodes; imode++) {
            eig_vect_and_val eig_i = eig_solver.GetMode(imode);
            modal_freq(imode - 1, 0) = eig_i.eigen_val.real();
            modal_freq(imode - 1, 1) = eig_i.eigen_val.imag();
            modal_freq(imode - 1, 2) = eig_i.eigen_val.imag() / CH_C_2PI;

            ChMatrixDynamic<> modal_shape_i(sys.GetNbodies(), 10);
            int r = 0;
            for (auto ibody : sys.Get_bodylist()) {
                if (ibody->IsActive()) {
                    modal_shape_i(r, 0) = r + 1;  // index of rigid body in the system
                    modal_shape_i(r, 1) = ibody->GetPos().x();
                    modal_shape_i(r, 2) = ibody->GetPos().y();
                    modal_shape_i(r, 3) = ibody->GetPos().z();
                    for (int c = 0; c < 6; c++) {
                        // No damping in the system, the real part is always zero.
                        modal_shape_i(r, c + 4) = eig_i.eigen_vect(6 * r + c).imag();
                    }
                    r++;
                }
            }
            // sort according to the X coordinate (horizontal coordinate)
            std::sort(modal_shape_i.rowwise().begin(), modal_shape_i.rowwise().end(),
                      [](auto const& r1, auto const& r2) { return r1(1) < r2(1); });

            if (create_directory(path(out_dir))) {
                ChStreamOutAsciiFile file_shape((out_dir + "/modal_shape_" + std::to_string(imode) + ".dat").c_str());
                file_shape.SetNumFormat("%.12g");
                StreamOUTdenseMatlabFormat(modal_shape_i, file_shape);
            } else {
                GetLog() << "  ...Error creating subdirectories\n";
            }
        }

        if (create_directory(path(out_dir))) {
            ChStreamOutAsciiFile file_freq((out_dir + "/modal_freq.dat").c_str());
            file_freq.SetNumFormat("%.12g");
            StreamOUTdenseMatlabFormat(modal_freq, file_freq);
        } else {
            GetLog() << "  ...Error creating subdirectories\n";
        }
    }

    if (true) {  // dynamic analysis
        GetLog() << "\n\n ******** Dynamic analysis ******** \n\n";

        // use HHT second order integrator (but slower)
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        if (hht_stepper != nullptr) {
            hht_stepper->SetVerbose(false);
            hht_stepper->SetStepControl(false);
            hht_stepper->SetAlpha(-0.2);
            hht_stepper->SetModifiedNewton(false);
        }

        auto DoDynamicsUnderImpulse = [&](const ChVector<>& vec_f, const std::string& filename) {
            // Add the excitation at the middle point C
            auto push_force =
                chrono_types::make_shared<ChLoadBodyForce>(knot_list.at(Nrig + 1), vec_f, false, VNULL, true);
            load_container->Add(push_force);

            double time_step = 0.01;
            double time_length = 100.0;
            int Nframes = (int)(time_length / time_step);
            int frame = 0;

            ChMatrixDynamic<> vibration;
            vibration.resize(Nframes, 4);

            // 3D visualization
            auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis->AttachSystem(&sys);
            vis->SetCameraVertical(CameraVerticalDir::Z);
            vis->SetWindowSize(1000, 800);
            vis->SetWindowTitle("Test demo: AnchorChain");
            vis->Initialize();
            vis->AddLogo();
            vis->AddSkyBox();
            vis->AddCamera(anchorC->GetPos() + ChVector<>(1, -10, 5), anchorC->GetPos() + ChVector<>(1, 0, 3));
            vis->AddTypicalLights();
            vis->EnableBodyFrameDrawing(false);
            vis->EnableLinkFrameDrawing(false);

            while (vis->Run() && frame < Nframes) {
                vis->BeginScene();
                vis->Render();
                tools::drawCoordsys(vis.get(), ChCoordsys<>({xC, 0, 0}, QUNIT), 1.0);
                vis->EndScene();

                // A short period later, the excitation force is removed to allow free vibration
                if (sys.GetChTime() > 5.0) {
                    push_force->SetForce({0, 0, 0}, false);
                }

                sys.DoStepDynamics(time_step);

                vibration(frame, 0) = sys.GetChTime();
                vibration(frame, 1) = anchorC->GetPos().x();
                vibration(frame, 2) = anchorC->GetPos().y();
                vibration(frame, 3) = anchorC->GetPos().z();

                if (frame % 20 == 0) {
                    GetLog() << "t: " << sys.GetChTime() << "\t";
                    GetLog() << "anchorC position:\t" << anchorC->GetPos().x() << "\t" << anchorC->GetPos().y() << "\t"
                             << anchorC->GetPos().z() << "\n";
                }
                frame++;
            }

            if (create_directory(path(out_dir))) {
                ChStreamOutAsciiFile file_vibration((out_dir + "/" + filename + ".dat").c_str());
                file_vibration.SetNumFormat("%.12g");
                StreamOUTdenseMatlabFormat(vibration, file_vibration);
            } else {
                GetLog() << "  ...Error creating subdirectories\n";
            }
        };

        // store the equilibrium status
        double T0;
        ChState X0;
        ChStateDelta V0;
        ChStateDelta A0;
        ChVectorDynamic<> L0;
        sys.StateSetup(X0, V0, A0);
        sys.StateGather(X0, V0, T0);
        sys.StateGatherAcceleration(A0);
        L0.resize(sys.GetNconstr());
        sys.StateGatherReactions(L0);

        // excitation in X direction (In-plane horizontal motion is expected)
        GetLog() << "\n\nExcitation in +X direction\n\n";
        ChVector<> vec_fx = ChVector<>(mass * gacc * 5, 0, 0);
        DoDynamicsUnderImpulse(vec_fx, "vibration_x");

        // recover the system to the exactly same equilibrium status
        sys.StateScatter(X0, V0, T0, true);
        sys.StateScatterAcceleration(A0);
        sys.StateScatterReactions(L0);
        // excitation in Y direction (Out-of-plane motion is expected)
        GetLog() << "\n\nExcitation in +Y direction\n\n";
        ChVector<> vec_fy = ChVector<>(0, mass * gacc * 5, 0);
        DoDynamicsUnderImpulse(vec_fy, "vibration_y");

        // recover the system to the exactly same equilibrium status
        sys.StateScatter(X0, V0, T0, true);
        sys.StateScatterAcceleration(A0);
        sys.StateScatterReactions(L0);
        // excitation in Z direction (In-plane vertical motion is expected)
        GetLog() << "\n\nExcitation in -Z direction\n\n";
        ChVector<> vec_fz = ChVector<>(0, 0, -mass * gacc * 5);
        DoDynamicsUnderImpulse(vec_fz, "vibration_z");
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // test_pendulum();

    test_anchorchain();

    return 0;
}
