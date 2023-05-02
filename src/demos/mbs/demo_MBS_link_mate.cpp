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

class PauseEventReceiver : public irr::IEventReceiver {
  public:
    PauseEventReceiver(bool start_paused = false) : is_paused(start_paused) {}
    virtual ~PauseEventReceiver() {}
    bool OnEvent(const irr::SEvent& event) {
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown &&
            event.KeyInput.Key == irr::KEY_SPACE) {
            is_paused = !is_paused;
            return true;
        }
        return false;
    }

    bool IsNotPaused() const { return !is_paused; }

  private:
    bool is_paused = false;
};

void EigenSolver(ChSystemNSC& msys) {
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
    //// A  =  [  0     I      0   ]
    ////       [ -K    -R     -Cq' ]
    ////       [ -Cq    0      0   ]
    Ad << Eigen::MatrixXd::Zero(n_v, n_v), Eigen::MatrixXd::Identity(n_v, n_v), Eigen::MatrixXd::Zero(n_v, n_c),
        -K_sp.toDense(), -R_sp.toDense(), -Cq_sp.toDense().transpose(), -Cq_sp.toDense(),
        Eigen::MatrixXd::Zero(n_c, n_v), Eigen::MatrixXd::Zero(n_c, n_c);

    //// E  =  [  I     0     0 ]
    ////       [  0     M     0 ]
    ////       [  0     0     0 ]
    Ed << Eigen::MatrixXd::Identity(n_v, n_v), Eigen::MatrixXd::Zero(n_v, n_v), Eigen::MatrixXd::Zero(n_v, n_c),
        Eigen::MatrixXd::Zero(n_v, n_v), M_sp.toDense(), Eigen::MatrixXd::Zero(n_v, n_c),
        Eigen::MatrixXd::Zero(n_c, n_v), Eigen::MatrixXd::Zero(n_c, n_v), Eigen::MatrixXd::Zero(n_c, n_c);

    Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges;
    ges.compute(Ad, Ed);

    std::vector<std::complex<double>> eigenvalues;

    for (int i = 0; i < ges.betas().size(); i++) {
        if (ges.betas()(i)) {
            std::complex<double> e_temp = ges.alphas()(i) / ges.betas()(i);
            if (std::abs(e_temp) < 1e6)
                eigenvalues.push_back(e_temp);
        }
    }

    if (eigenvalues.size() == 0)
        GetLog() << "There is not any eigenvalue found for the system.\n";
    else {
        Eigen::MatrixXd eig_D(eigenvalues.size(), 2);
        for (int i = 0; i < eigenvalues.size(); i++) {
            eig_D(i, 0) = eigenvalues.at(i).real();
            eig_D(i, 1) = eigenvalues.at(i).imag();
        }

        std::sort(eig_D.rowwise().begin(), eig_D.rowwise().end(),
                  [](auto const& r1, auto const& r2) { return r1(1) < r2(1); });

        GetLog() << "The eigenvalues of the system are:\n";
        for (int i = 0; i < eig_D.rows(); i++) {
            GetLog() << "Eigenvalue " << std::to_string(i + 1).c_str() << ":\tReal: " << eig_D(i, 0)
                     << "\tImag: " << eig_D(i, 1) << "\n";
        }
    }
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
    double length = 2.0;
    // The mass of the end point, kg
    double tip_mass = 13.5;
    // The offset angle of the pendulum at the initial configuration, rad
    double offset_angle = 30.0 * CH_C_DEG_TO_RAD;

    // Gravity acceleration, m/s^2
    double gacc = 10.0;

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

    auto cyl_rev = chrono_types::make_shared<ChCylinderShape>();
    cyl_rev->GetCylinderGeometry().p1 = ChVector<>(-0.2, 0, 0);
    cyl_rev->GetCylinderGeometry().p2 = ChVector<>(0.2, 0, 0);
    cyl_rev->GetCylinderGeometry().rad = 0.1;
    my_root->AddVisualShape(cyl_rev);

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

    auto sph = chrono_types::make_shared<ChSphereShape>();
    sph->GetSphereGeometry().rad = 0.3;
    sph->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    my_mass->AddVisualShape(sph);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = VNULL;
    ChFrameMoving<> rel_frame;
    my_mass->TransformParentToLocal(my_root->GetFrame_COG_to_abs(), rel_frame);
    cyl->GetCylinderGeometry().p2 = rel_frame.GetPos();
    cyl->GetCylinderGeometry().rad = 0.05;
    cyl->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    my_mass->AddVisualShape(cyl);

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
    vis->SetWindowSize(800, 800);
    vis->SetWindowTitle("Test demo: Pendulum");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(3, 1, 3));
    vis->AddTypicalLights();
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);

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
    EigenSolver(sys);

    GetLog() << "\nThe theoretical oscillation frequency is:\t" << std::sqrt(gacc / length) << " rad/s\n";
}

// ====================================
// Test 2
// Second example: Mooring line
// ====================================
void test_mooringline() {
    // The density is temporarily used to facilitate the code implementation
    double density = 7800;

    // The mass and inertia properties of every link in the mooring line
    double mass = 10;
    double Jxx = 10;
    double Jyy = 10;
    double Jzz = 10;

    // We have three anchors to determine the initial configuration of the mooring line:
    // A: located at {0,0,0} as the left anchor point.
    // B: located at {xB,0,0} as the right anchor point.
    // C: located at {xC,0,zC} as the bottom anchor point.
    // A series of links are built between A and B, B and C via rigid bodies and joints
    // to form the shape 'V' at the initial configuration of the mooring line.
    double xB = 10.0;
    double xC = xB / 2.0;
    double zC = -6.0;

    // Gravity
    double gacc = 10.0;

    // Whether the tangent stiffness matrix of constraint (Kc) is used?
    // The Kc matrix is mandotary for the static and eigenvalue analysis of the mooring line,
    // otherwise it cannot find the static equilibrium status, and the eigenvalues are also zero
    // because there is no stiffness matrix in the system.
    bool use_Kc = true;

    // The physical system: it contains all physical objects.
    ChSystemNSC sys;
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.AddOtherPhysicsItem(load_container);

    auto wallA = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.2, 0.2, density, true, false);
    wallA->SetNameString("wallA");
    wallA->SetPos({0, 0, 0});
    wallA->SetBodyFixed(true);
    sys.AddBody(wallA);

    auto anchorA = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.2, 0.2, density, true, false);
    anchorA->SetNameString("anchorA");
    anchorA->SetPos({0, 0, 0});
    anchorA->SetRot(Q_from_AngY(CH_C_PI_4));
    anchorA->SetMass(mass);
    anchorA->SetInertiaXX({Jxx, Jyy, Jzz});
    sys.AddBody(anchorA);

    auto jointA = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);
    jointA->Initialize(anchorA, wallA, wallA->GetFrame_COG_to_abs());
    jointA->SetUseTangentStiffness(use_Kc);
    sys.AddLink(jointA);

    auto wallB = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.2, 0.2, density, true, false);
    wallB->SetNameString("wallB");
    wallB->SetPos({xB, 0, 0});
    wallB->SetBodyFixed(true);
    sys.AddBody(wallB);

    auto anchorB = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.2, 0.2, density, true, false);
    anchorB->SetNameString("anchorB");
    anchorB->SetPos({xB, 0, 0});
    anchorB->SetRot(Q_from_AngY(-CH_C_PI_4));
    anchorB->SetMass(mass);
    anchorB->SetInertiaXX({Jxx, Jyy, Jzz});
    sys.AddBody(anchorB);

    auto jointB = chrono_types::make_shared<ChLinkMateGeneric>(true, true, true, true, false, true);
    jointB->Initialize(anchorB, wallB, wallB->GetFrame_COG_to_abs());
    jointB->SetUseTangentStiffness(use_Kc);
    sys.AddLink(jointB);

    auto anchorC = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.2, 0.2, density, true, false);
    anchorC->SetNameString("anchorC");
    anchorC->SetPos({xC, 0.0, zC});
    anchorC->SetMass(mass);
    anchorC->SetInertiaXX({Jxx, Jyy, Jzz});
    sys.AddBody(anchorC);

    std::vector<std::shared_ptr<ChBody>> knot_list;
    // Build N rigid bodies between A and B
    auto BuildMooringLine_body = [&](std::shared_ptr<ChBody> pA, std::shared_ptr<ChBody> pB, int N) {
        double len_tot = (pA->GetPos() - pB->GetPos()).Length();
        double len = len_tot / (double)(N + 1);

        ChVector<> Xdir = (pB->GetPos() - pA->GetPos()).GetNormalized();
        ChVector<> Ydir = {0, 1, 0};
        ChVector<> Zdir = Xdir.Cross(Ydir).GetNormalized();
        ChVector<> mX;
        ChVector<> mY;
        ChVector<> mZ;
        Xdir.DirToDxDyDz(mX, mY, mZ, Ydir);
        ChQuaternion<> knot_rot = ChMatrix33<>(mX, mY, mZ).Get_A_quaternion();

        for (int i_body = 0; i_body < N; i_body++) {
            auto knot = chrono_types::make_shared<ChBody>();
            ChVector<> deltaP = (pB->GetPos() - pA->GetPos()) / (double)(N + 1);
            knot->SetPos(pA->GetPos() + deltaP * (i_body + 1));
            knot->SetRot(knot_rot);
            knot->SetMass(mass);
            knot->SetInertiaXX({Jxx, Jyy, Jzz});

            auto cyl_rev = chrono_types::make_shared<ChCylinderShape>();
            ChVector<> deltaV = {0.5 * len * 0.6, 0, 0};
            cyl_rev->GetCylinderGeometry().p1 = deltaV;
            cyl_rev->GetCylinderGeometry().p2 = -deltaV;
            cyl_rev->GetCylinderGeometry().rad = 0.1;
            knot->AddVisualShape(cyl_rev);

            knot_list.push_back(knot);
            sys.AddBody(knot);
        }
    };

    int N = 20;
    knot_list.push_back(anchorA);
    BuildMooringLine_body(anchorA, anchorC, N);
    knot_list.push_back(anchorC);
    BuildMooringLine_body(anchorC, anchorB, N);
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

    // Change solver to PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);

    // use HHT second order integrator (but slower)
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht_stepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    if (hht_stepper != nullptr) {
        hht_stepper->SetVerbose(false);
        hht_stepper->SetStepControl(false);
        hht_stepper->SetAlpha(-0.2);
        hht_stepper->SetModifiedNewton(true);

        hht_stepper->SetQcDoClamp(true);
        hht_stepper->SetQcClamping(1e30);
    }

    // sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    // auto euler_implicit = std::dynamic_pointer_cast<ChTimestepperEulerImplicit>(sys.GetTimestepper());
    // if (euler_implicit != nullptr) {
    //     euler_implicit->SetVerbose(false);
    // }

    ChStaticNonLinearRigidMotion rigid_static_analysis;
    rigid_static_analysis.SetCorrectionTolerance(1e-16, 1e-16);
    rigid_static_analysis.SetIncrementalSteps(10);
    rigid_static_analysis.SetMaxIterations(100);
    rigid_static_analysis.SetVerbose(true);

    sys.Setup();

    if (true) {  // static analysis
        GetLog() << "At the initial configuration:\n";
        GetLog() << "anchorC->GetPos():\t" << anchorC->GetPos();

        // Firstly, we call DoFullAssembly() to calculate the reaction forces/torques at the initial configuration of
        // system.
        sys.DoFullAssembly();
        // Secondly, we perform the static analysis using the solver ChStaticNonLinearRigidMotion().
        sys.DoStaticAnalysis(rigid_static_analysis);

        GetLog() << "After doing the static nonlinear analysis:\n";
        GetLog() << "anchorC->GetPos():\t" << anchorC->GetPos();

        ChMatrixDynamic<> coords;
        coords.resize(sys.Get_bodylist().size(), 3);
        for (int i_body = 0; i_body < sys.Get_bodylist().size(); i_body++) {
            coords(i_body, 0) = sys.Get_bodylist().at(i_body)->GetPos().x();
            coords(i_body, 1) = sys.Get_bodylist().at(i_body)->GetPos().y();
            coords(i_body, 2) = sys.Get_bodylist().at(i_body)->GetPos().z();
        }

        std::sort(coords.rowwise().begin(), coords.rowwise().end(),
                  [](auto const& r1, auto const& r2) { return r1(0) < r2(0); });

        ChMatrixDynamic<> reactions;
        reactions.resize(sys.Get_linklist().size(), 7);
        for (int i_link = 0; i_link < sys.Get_linklist().size(); i_link++) {
            reactions(i_link, 0) = sys.Get_linklist().at(i_link)->GetLinkAbsoluteCoords().pos.x();
            reactions(i_link, 1) = sys.Get_linklist().at(i_link)->Get_react_force().x();
            reactions(i_link, 2) = sys.Get_linklist().at(i_link)->Get_react_force().y();
            reactions(i_link, 3) = sys.Get_linklist().at(i_link)->Get_react_force().z();
            reactions(i_link, 4) = sys.Get_linklist().at(i_link)->Get_react_torque().x();
            reactions(i_link, 5) = sys.Get_linklist().at(i_link)->Get_react_torque().y();
            reactions(i_link, 6) = sys.Get_linklist().at(i_link)->Get_react_torque().z();
        }
        std::sort(reactions.rowwise().begin(), reactions.rowwise().end(),
                  [](auto const& r1, auto const& r2) { return r1(0) < r2(0); });

        // Create output directory and output file
        std::string out_dir = GetChronoOutputPath() + "MOORING_LINE";
        GetLog() << "out_dir is:\n" << out_dir << "\n";

        if (create_directory(path(out_dir))) {
            // coordinates of bodies
            ChStreamOutAsciiFile file_coords((out_dir + "/coords.dat").c_str());
            file_coords.SetNumFormat("%.12g");
            StreamOUTdenseMatlabFormat(coords, file_coords);

            // reactions of links
            ChStreamOutAsciiFile file_reactions((out_dir + "/reactions.dat").c_str());
            file_reactions.SetNumFormat("%.12g");
            StreamOUTdenseMatlabFormat(reactions, file_reactions);
        } else {
            GetLog() << "  ...Error creating subdirectories\n";
        }
    }

    if (true) {
        EigenSolver(sys);
    }

    if (true) {  // dynamic analysis
        ChVector<> vec_f = ChVector<>(0, 0, -mass * gacc * N);
        auto pull_force1 = chrono_types::make_shared<ChLoadBodyForce>(knot_list.at(N), vec_f, false, VNULL, true);
        load_container->Add(pull_force1);
        auto pull_force2 = chrono_types::make_shared<ChLoadBodyForce>(knot_list.at(N + 2), vec_f, false, VNULL, true);
        load_container->Add(pull_force2);

        double time_step = 0.01;
        double time_length = 100.0;
        int frame = 0;

        // 3D visualization
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(1000, 1000);
        vis->SetWindowTitle("Test demo: Mooringline");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(anchorC->GetPos() + ChVector<>(1, -10, 2), anchorC->GetPos() + ChVector<>(1, 0, 2));
        vis->AddTypicalLights();
        vis->EnableBodyFrameDrawing(true);
        vis->EnableLinkFrameDrawing(true);

        // Pause event receiver
        PauseEventReceiver pause_ER(true);
        vis->AddUserEventReceiver(&pause_ER);

        while (vis->Run() && sys.GetChTime() < time_length) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (pause_ER.IsNotPaused()) {
                sys.DoStepDynamics(time_step);
            }

            frame++;

            if (frame % 10 == 0) {
                GetLog() << "t: " << sys.GetChTime() << "\t";
                GetLog() << "anchorC->GetPos():\t" << anchorC->GetPos().x() << "  " << anchorC->GetPos().y() << "  "
                         << anchorC->GetPos().z() << "\n";
            }
        }
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // test_pendulum();

    test_mooringline();

    return 0;
}
