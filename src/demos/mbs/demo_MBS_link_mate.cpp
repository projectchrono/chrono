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

#include <iomanip>

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
        msys.GetMassMatrix(M_sp);
        msys.GetStiffnessMatrix(K_sp);
        msys.GetDampingMatrix(R_sp);
        msys.GetConstraintJacobianMatrix(Cq_sp);

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
            std::cerr << "There is not any eigenvalue found for the system." << std::endl;
        else {
            std::cout << "The eigenvalues of the system are:" << std::endl;
            for (int i = 0; i < all_eigen_vectors_and_values.size(); i++) {
                std::cout << "Eigenvalue " << std::to_string(i + 1)
                          << ":\tReal: " << all_eigen_vectors_and_values.at(i).eigen_val.real()
                          << "\tImag: " << all_eigen_vectors_and_values.at(i).eigen_val.imag() << std::endl;
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

        std::cout << std::endl
                  << "The eigenvalue of mode " << imode << " is: "
                  << ":\tReal: " << eig_o.eigen_val.real() << ":\tImag: " << eig_o.eigen_val.imag() << std::endl;

        return eig_o;
    }

    std::vector<eig_vect_and_val> all_eigen_vectors_and_values;
};

//====================================
// Test 1
// First example: Pendulum
//====================================
void test_pendulum() {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: static and eigenvalue analysis of a pendulum\n" << std::endl;

    // Some variables to parameterize the model
    // The length of the pendulum, m
    double length = 4.0;
    // The mass of the end point, kg
    double tip_mass = 15.0;
    // The offset angle of the pendulum at the initial configuration, rad
    double offset_angle = 30.0 * CH_DEG_TO_RAD;

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
    my_root->SetCoordsys(VNULL, QUNIT);
    my_root->SetMass(1e6);
    my_root->SetInertiaXX(ChVector3d(1, 1, 1));
    my_root->SetName("base");
    my_root->SetFixed(true);
    my_root->EnableCollision(false);
    sys.AddBody(my_root);

    auto cyl_rev = chrono_types::make_shared<ChVisualShapeCylinder>(0.1, 0.4);
    my_root->AddVisualShape(cyl_rev, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

    auto my_mass = chrono_types::make_shared<ChBody>();
    ChVector3d mass_pos = ChVector3d(length * std::sin(offset_angle), 0, -length * std::cos(offset_angle));
    ChVector3d Zdir = (my_root->GetPos() - mass_pos).GetNormalized();
    ChVector3d Ydir = my_root->GetRot().GetAxisY().GetNormalized();
    ChVector3d Xdir = Ydir.Cross(Zdir).GetNormalized();
    ChVector3d mX;
    ChVector3d mY;
    ChVector3d mZ;
    Xdir.GetDirectionAxesAsX(mX, mY, mZ, Ydir);
    ChQuaternion<> mass_rot = ChMatrix33<>(mX, mY, mZ).GetQuaternion();
    my_mass->SetCoordsys(mass_pos, mass_rot);
    my_mass->SetMass(tip_mass);
    my_mass->SetInertiaXX(ChVector3d(0, 0, 0));
    my_mass->SetName("mass");
    my_mass->EnableCollision(false);
    sys.AddBody(my_mass);

    auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.3);
    sph->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    my_mass->AddVisualShape(sph);

    ChFrameMoving<> rel_frame = my_mass->TransformParentToLocal(my_root->GetFrameCOMToAbs());
    ChLineSegment seg(VNULL, rel_frame.GetPos());
    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.05, seg.GetLength());
    cyl->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    my_mass->AddVisualShape(cyl, seg.GetFrame());

    // Revolute joint at the root
    auto my_joint = chrono_types::make_shared<ChLinkMateGeneric>();
    // RotY is free
    my_joint->SetConstrainedCoords(true, true, true, true, false, true);
    my_joint->SetName("revolute_joint");
    my_joint->Initialize(my_mass, my_root, my_root->GetFrameCOMToAbs());
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
    vis->AddCamera(ChVector3d(3, 1, 3));
    vis->AddTypicalLights();
    vis->EnableBodyFrameDrawing(false);
    vis->EnableLinkFrameDrawing(false);

    // gravity
    sys.SetGravitationalAcceleration({0, 0, -gacc});

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // The pendulum has one rigid-motion degree of freedom. We need to use the
    // static solver: ChStaticNonLinearAnalysis().
    ChStaticNonLinearAnalysis rigid_static_analysis;
    rigid_static_analysis.SetIncrementalSteps(10);
    rigid_static_analysis.SetMaxIterations(100);
    rigid_static_analysis.SetResidualTolerance(1e-16);
    rigid_static_analysis.SetVerbose(false);

    sys.Setup();

    // ====================================
    // Static analysis
    // ====================================
    std::cout << std::endl
              << "The initial position of the end mass is:\n"
              << "\tx:  " << my_mass->GetPos().x() << "\ty:  " << my_mass->GetPos().y()
              << "\tz:  " << my_mass->GetPos().z() << std::endl;

    // First, perform a full assembly to calculate the reaction forces/torques at the initial system configuration
    sys.DoAssembly(AssemblyAnalysis::Level::FULL);

    // Second, perform the static analysis using the solver ChStaticNonLinearAnalysis()
    sys.DoStaticAnalysis(rigid_static_analysis);

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawCoordsys(vis.get(), ChCoordsys<>(VNULL, QUNIT), 1.0);
        vis->EndScene();
    }

    const auto& pos = my_mass->GetPos();
    const auto& reaction = my_joint->GetReaction2();
    const auto& rforce = reaction.force;
    const auto& rtorque = reaction.torque;

    std::cout << "\nAfter doing the nonlinear static analysis:" << std::endl;
    std::cout << "\tThe final position of the end mass is:\n"
              << "\t\tx:  " << pos.x() << "\ty:  " << pos.y() << "\tz:  " << pos.z() << std::endl;
    std::cout << "\tThe reaction forces at the root are:\n"
              << "\t\tfx:  " << rforce.x() << "\tfy:  " << rforce.y() << "\tfz:  " << rforce << std::endl;
    std::cout << "\tThe reaction torques at the root are:\n"
              << "\t\tmx:  " << rtorque.x() << "\tmy:  " << rtorque.y() << "\tmz:  " << rtorque.z() << std::endl;

    // ====================================
    // Eigenvalue analysis
    // ====================================
    EigenSolver eig_solver(sys);
    eig_solver.ShowAllEigenvalues();

    std::cout << std::endl
              << "The theoretical oscillation frequency is:\t" << std::sqrt(gacc / length) << " rad/s" << std::endl;
}

// ====================================
// Test 2
// Second example: Anchor chain
// ====================================
void test_anchorchain() {
    std::cout << "\n-------------------------------------------------" << std::endl;
    std::cout << "TEST: static and eigenvalue analysis of an anchor chain\n" << std::endl;

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

    auto box = chrono_types::make_shared<ChVisualShapeBox>(0.2, 0.2, 0.2);
    box->SetColor(ChColor(1, 0, 0));

    auto wallA = chrono_types::make_shared<ChBody>();
    wallA->SetName("wallA");
    wallA->SetPos({xA, 0, 0});
    wallA->SetFixed(true);
    wallA->AddVisualShape(box);
    sys.AddBody(wallA);

    auto anchorA = chrono_types::make_shared<ChBody>();
    anchorA->SetName("anchorA");
    anchorA->SetPos({xA, 0, 0});
    anchorA->SetRot(QuatFromAngleY(CH_PI_4));
    anchorA->SetMass(mass);
    anchorA->SetInertiaXX({Jxx, Jyy, Jzz});
    anchorA->AddVisualShape(box);
    sys.AddBody(anchorA);

    auto jointA = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);
    jointA->Initialize(anchorA, wallA, wallA->GetFrameCOMToAbs());
    jointA->SetUseTangentStiffness(use_Kc);
    sys.AddLink(jointA);

    auto wallB = chrono_types::make_shared<ChBody>();
    wallB->SetName("wallB");
    wallB->SetPos({xB, 0, 0});
    wallB->SetFixed(true);
    wallB->AddVisualShape(box);
    sys.AddBody(wallB);

    auto anchorB = chrono_types::make_shared<ChBody>();
    anchorB->SetName("anchorB");
    anchorB->SetPos({xB, 0, 0});
    anchorB->SetRot(QuatFromAngleY(-CH_PI_4));
    anchorB->SetMass(mass);
    anchorB->SetInertiaXX({Jxx, Jyy, Jzz});
    anchorB->AddVisualShape(box);
    sys.AddBody(anchorB);

    auto jointB = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);
    jointB->Initialize(anchorB, wallB, wallB->GetFrameCOMToAbs());
    jointB->SetUseTangentStiffness(use_Kc);
    sys.AddLink(jointB);

    auto anchorC = chrono_types::make_shared<ChBody>();
    anchorC->SetName("anchorC");
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

        ChVector3d Xdir = (pB->GetPos() - pA->GetPos()).GetNormalized();
        ChVector3d Ydir = {0, 1, 0};
        ChVector3d Zdir = Xdir.Cross(Ydir).GetNormalized();
        ChVector3d mX;
        ChVector3d mY;
        ChVector3d mZ;
        Xdir.GetDirectionAxesAsX(mX, mY, mZ, Ydir);
        ChQuaternion<> knot_rot = ChMatrix33<>(mX, mY, mZ).GetQuaternion();

        for (int i_body = 0; i_body < mN; i_body++) {
            auto knot = chrono_types::make_shared<ChBody>();
            ChVector3d deltaP = (pB->GetPos() - pA->GetPos()) / (double)(mN + 1);
            knot->SetPos(pA->GetPos() + deltaP * (i_body + 1));
            knot->SetRot(knot_rot);
            knot->SetMass(mass);
            knot->SetInertiaXX({Jxx, Jyy, Jzz});

            auto cyl_rev = chrono_types::make_shared<ChVisualShapeCylinder>(0.1, len * 0.8);
            cyl_rev->SetColor(ChColor(0, 0, 1));
            knot->AddVisualShape(cyl_rev, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

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
    sys.SetGravitationalAcceleration({0, 0, -gacc});
    sys.Setup();

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // Create output directory and output file
    std::string out_dir = GetChronoOutputPath() + "ANCHOR_CHAIN";
    std::cout << "out_dir is:\n" << out_dir << std::endl;

    if (true) {  // static analysis
        std::cout << "\n\n******** Static analysis ******** \n" << std::endl;

        // Set solver for static analysis
        ChStaticNonLinearAnalysis rigid_static_analysis;
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

        std::cout << "\nAt the initial configuration:" << std::endl;
        std::cout << "anchorC position:\t" << anchorC->GetPos().x() << "\t" << anchorC->GetPos().y() << "\t"
                  << anchorC->GetPos().z() << std::endl;

        // First, perform a full assembly to calculate the reaction forces/torques at the initial system configuration
        sys.DoAssembly(AssemblyAnalysis::Level::FULL);

        // Second, perform the static analysis using the solver ChStaticNonLinearAnalysis()
        sys.DoStaticAnalysis(rigid_static_analysis);

        std::cout << "\nAfter doing the static nonlinear analysis:" << std::endl;
        std::cout << "anchorC position:\t" << anchorC->GetPos().x() << "\t" << anchorC->GetPos().y() << "\t"
                  << anchorC->GetPos().z() << std::endl;

        // The coordiantes of the rigid bodies in the equilibrium configuration
        ChMatrixDynamic<> coords;
        coords.resize(sys.GetBodies().size(), 3);
        for (int i_body = 0; i_body < sys.GetBodies().size(); i_body++) {
            coords(i_body, 0) = sys.GetBodies().at(i_body)->GetPos().x();
            coords(i_body, 1) = sys.GetBodies().at(i_body)->GetPos().y();
            coords(i_body, 2) = sys.GetBodies().at(i_body)->GetPos().z();
        }
        // sort according to the X coordinate (horizontal coordinate)
        std::sort(coords.rowwise().begin(), coords.rowwise().end(),
                  [](auto const& r1, auto const& r2) { return r1(0) < r2(0); });

        // The reaction forces and torques of the joints in the equilibrium configuration
        ChMatrixDynamic<> reactions;
        reactions.resize(sys.GetLinks().size(), 7);
        for (int i_link = 0; i_link < sys.GetLinks().size(); i_link++) {
            reactions(i_link, 0) = sys.GetLinks().at(i_link)->GetFrame2Abs().GetCoordsys().pos.x();

            const auto& reaction = sys.GetLinks().at(i_link)->GetReaction2();
            const auto& f_loc = reaction.force;
            const auto& m_loc = reaction.torque;
            ChQuaternion<> q_link = sys.GetLinks().at(i_link)->GetFrame2Abs().GetCoordsys().rot;
            // Transform the reaction forces and torques of the joints from local frame to the absolute frame.
            // The horizontal reaction forces (along X direction) should be equal among all joints for the catenary
            // curve.
            ChVector3d f_out = q_link.Rotate(f_loc);
            ChVector3d m_out = q_link.Rotate(m_loc);

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
        catenary_cmp.resize(sys.GetBodies().size(), 3);
        for (int i_body = 0; i_body < sys.GetBodies().size(); i_body++) {
            catenary_cmp(i_body, 0) = coords(i_body, 0);  // X coordinate
            catenary_cmp(i_body, 1) = coords(i_body, 2);  // Z coordiante from simulation
            // Z coordiante from the theoretical catinary curve
            catenary_cmp(i_body, 2) = a_coeff * std::cosh((coords(i_body, 0) - x_offset) / a_coeff) + z_offset;
        }

        if (create_directory(path(out_dir))) {
            // coordinates of rigid bodies
            std::ofstream file_coords(out_dir + "/equilibrium_coords.dat");
            file_coords << std::setprecision(12) << std::scientific;
            StreamOut(coords, file_coords);

            // catinary curve for comparison with the analytical formula
            std::ofstream file_catenary(out_dir + "/catenary_cmp.dat");
            file_catenary << std::setprecision(12) << std::scientific;
            StreamOut(catenary_cmp, file_catenary);

            // reaction forces and torques of all joints
            std::ofstream file_reactions(out_dir + "/equilibrium_reactions.dat");
            file_reactions << std::setprecision(12) << std::scientific;
            StreamOut(reactions, file_reactions);
        } else {
            std::cerr << "  ...Error creating subdirectories" << std::endl;
        }
    }

    if (true) {  // eigenvalue analysis
        std::cout << "\n\n******** Eigenvalue analysis ******** \n" << std::endl;

        // solve the eigenvalues at the equilibrium status
        EigenSolver eig_solver(sys);
        eig_solver.ShowAllEigenvalues();

        int nmodes = 5;  // how many modes are you going to extract?
        ChMatrixDynamic<> modal_freq(nmodes, 3);
        for (int imode = 1; imode <= nmodes; imode++) {
            eig_vect_and_val eig_i = eig_solver.GetMode(imode);
            modal_freq(imode - 1, 0) = eig_i.eigen_val.real();
            modal_freq(imode - 1, 1) = eig_i.eigen_val.imag();
            modal_freq(imode - 1, 2) = eig_i.eigen_val.imag() / CH_2PI;

            ChMatrixDynamic<> modal_shape_i(sys.GetNumBodiesActive(), 10);
            int r = 0;
            for (auto ibody : sys.GetBodies()) {
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
                std::ofstream file_shape(out_dir + "/modal_shape_" + std::to_string(imode) + ".dat");
                file_shape << std::setprecision(12) << std::scientific;
                StreamOut(modal_shape_i, file_shape);
            } else {
                std::cerr << "  ...Error creating subdirectories" << std::endl;
            }
        }

        if (create_directory(path(out_dir))) {
            std::ofstream file_freq(out_dir + "/modal_freq.dat");
            file_freq << std::setprecision(12) << std::scientific;
            StreamOut(modal_freq, file_freq);
        } else {
            std::cerr << "  ...Error creating subdirectories" << std::endl;
        }
    }

    if (true) {  // dynamic analysis
        std::cout << "\n\n******** Dynamic analysis ******** \n" << std::endl;

        // use HHT second order integrator (but slower)
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        if (hht_stepper != nullptr) {
            hht_stepper->SetVerbose(false);
            hht_stepper->SetStepControl(false);
            hht_stepper->SetAlpha(-0.2);
            hht_stepper->SetModifiedNewton(false);
        }

        auto DoDynamicsUnderImpulse = [&](const ChVector3d& vec_f, const std::string& filename) {
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
            vis->AddCamera(anchorC->GetPos() + ChVector3d(1, -10, 5), anchorC->GetPos() + ChVector3d(1, 0, 3));
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
                    std::cout << "t: " << sys.GetChTime() << "\t";
                    std::cout << "anchorC position:\t" << anchorC->GetPos().x() << "\t" << anchorC->GetPos().y() << "\t"
                              << anchorC->GetPos().z() << std::endl;
                }
                frame++;
            }

            if (create_directory(path(out_dir))) {
                std::ofstream file_vibration(out_dir + "/" + filename + ".dat");
                file_vibration << std::setprecision(12) << std::scientific;
                StreamOut(vibration, file_vibration);
            } else {
                std::cout << "  ...Error creating subdirectories" << std::endl;
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
        L0.resize(sys.GetNumConstraints());
        sys.StateGatherReactions(L0);

        // excitation in X direction (In-plane horizontal motion is expected)
        std::cout << "\nExcitation in +X direction\n" << std::endl;
        ChVector3d vec_fx = ChVector3d(mass * gacc * 5, 0, 0);
        DoDynamicsUnderImpulse(vec_fx, "vibration_x");

        // recover the system to the exactly same equilibrium status
        sys.StateScatter(X0, V0, T0, true);
        sys.StateScatterAcceleration(A0);
        sys.StateScatterReactions(L0);
        // excitation in Y direction (Out-of-plane motion is expected)
        std::cout << "\nExcitation in +Y direction\n" << std::endl;
        ChVector3d vec_fy = ChVector3d(0, mass * gacc * 5, 0);
        DoDynamicsUnderImpulse(vec_fy, "vibration_y");

        // recover the system to the exactly same equilibrium status
        sys.StateScatter(X0, V0, T0, true);
        sys.StateScatterAcceleration(A0);
        sys.StateScatterReactions(L0);
        // excitation in Z direction (In-plane vertical motion is expected)
        std::cout << "\nExcitation in -Z direction\n" << std::endl;
        ChVector3d vec_fz = ChVector3d(0, 0, -mass * gacc * 5);
        DoDynamicsUnderImpulse(vec_fz, "vibration_z");
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\n"
              << "Chrono version: " << CHRONO_VERSION << std::endl;

    // test_pendulum();

    test_anchorchain();

    return 0;
}
