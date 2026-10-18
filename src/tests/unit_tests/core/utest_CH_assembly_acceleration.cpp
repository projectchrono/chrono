// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut
// =============================================================================
//
// Tests for DoAssembly at the ACCELERATION level: the acceleration and reaction it
// reports must be the constrained ones, including the quadratic-velocity term.
//
// A point mass on a ChLinkDistance (pendulum, pivot at the origin, length L) has
// the closed-form constrained acceleration
//     a = g - (g . phat + |v|^2 / L) phat
// and constraint force on the bob  F = -m (g . phat + |v|^2 / L) phat  (toward the
// pivot), where phat is the unit vector from pivot to bob and v is the (tangential)
// velocity. The |v|^2 / L term is what a velocity-increment "acceleration" omits
// (issue #846), so the tests run at several angular speeds; omega = 0 is the case
// that always passed. The quadratic-velocity term is obtained by central
// differencing of the constraint with step Delta = 1e-6, so roundoff in C (of
// order eps * L for a distance constraint) is amplified by 1/Delta^2: the floor on
// the reported accelerations and reactions scales with L and is about 1e-4 * L in
// SI units. The tolerance below follows that scaling and is checked at L = 1 and
// L = 100, with the Barzilai-Borwein iterative solver and with a direct sparse solver.
//
// =============================================================================

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/functions/ChFunctionSine.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"

using namespace chrono;

namespace {

const double G = 9.81;
const double M = 1.0;
const double theta = 35.0 * 3.14159265358979323846 / 180.0;

struct Pendulum {
    ChSystemNSC sys;
    std::shared_ptr<ChBody> bob;
    std::shared_ptr<ChLinkDistance> link;
    ChVector3d phat;  // pivot -> bob
    double L;
    double omega;

    Pendulum(double omega_, double L_, ChSolver::Type solver) : L(L_), omega(omega_) {
        sys.SetGravitationalAcceleration(ChVector3d(0, -G, 0));
        if (solver != ChSolver::Type::PSOR)
            sys.SetSolverType(solver);

        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);
        sys.Add(ground);

        bob = chrono_types::make_shared<ChBody>();
        bob->SetMass(M);
        bob->SetInertiaXX(ChVector3d(1e-2, 1e-2, 1e-2));
        phat = ChVector3d(std::sin(theta), -std::cos(theta), 0);
        ChVector3d that(std::cos(theta), std::sin(theta), 0);  // tangent
        bob->SetPos(phat * L);
        bob->SetPosDt(that * (omega * L));
        sys.Add(bob);

        link = chrono_types::make_shared<ChLinkDistance>();
        link->Initialize(ground, bob, false, ChVector3d(0, 0, 0), phat * L, true);
        sys.AddLink(link);
    }

    double Lambda() const { return Vdot(ChVector3d(0, -G, 0), phat) + (omega * L) * (omega * L) / L; }
    ChVector3d ExactAcc() const { return ChVector3d(0, -G, 0) - phat * Lambda(); }
    ChVector3d ExactForceOnBob() const { return phat * (-M * Lambda()); }

    // Reaction on the bob (body 2 of the link) in the absolute frame.
    ChVector3d ReactionOnBobAbs() const { return link->GetFrame2Abs().TransformDirectionLocalToParent(link->GetReaction2().force); }
};

struct PendulumResult {
    double acc_err;         // |a_chrono - a_exact|
    double acc_err_radial;  // (a_chrono - a_exact) . phat
    double react_err;       // |F_chrono - F_exact| (vectors, absolute frame)
    double angacc;          // |alpha| (must stay ~0: the constraint force acts at the COM)
    double vel_drift;       // |v_after - v_before|
};

PendulumResult RunPendulum(double omega, double L, ChSolver::Type solver, int action) {
    Pendulum p(omega, L, solver);
    ChVector3d v_before = p.bob->GetPosDt();

    auto flag = p.sys.DoAssembly(action);
    EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED);
    EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::ACCELERATION_INACCURATE);

    ChVector3d d = p.bob->GetPosDt2() - p.ExactAcc();
    PendulumResult r;
    r.acc_err = d.Length();
    r.acc_err_radial = Vdot(d, p.phat);
    r.react_err = (p.ReactionOnBobAbs() - p.ExactForceOnBob()).Length();
    r.angacc = p.bob->GetAngAccParent().Length();
    r.vel_drift = (p.bob->GetPosDt() - v_before).Length();
    return r;
}

// Minimal integrable whose solver fails on the acceleration-level solve (the second call to
// StateSolveCorrection) and records whether accelerations were nevertheless published.
class FailingIntegrable : public ChIntegrableIIorder {
  public:
    int num_solves = 0;
    bool acc_scattered = false;
    virtual unsigned int GetNumCoordsPosLevel() override { return 1; }
    virtual unsigned int GetNumCoordsVelLevel() override { return 1; }
    virtual void StateGather(ChState& x, ChStateDelta& v, double& T) override {
        x(0) = 0;
        v(0) = 0;
        T = 0;
    }
    virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T, UpdateFlags update_flags) override {}
    virtual void StateScatterAcceleration(const ChStateDelta& a) override { acc_scattered = true; }
    virtual void LoadResidual_F(ChVectorDynamic<>& R, const double c) override {}
    virtual void LoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override {}
    virtual void LoadConstraint_C(ChVectorDynamic<>& Qc, const double c, const double c_vel, const bool do_clamp = false, const double mclam = 1e30) override {}
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc, const double c, const double c_vel) override {}
    virtual bool StateSolveCorrection(ChStateDelta& Dv,
                                      ChVectorDynamic<>& L,
                                      const ChVectorDynamic<>& R,
                                      const ChVectorDynamic<>& Qc,
                                      const double c_a,
                                      const double c_v,
                                      const double c_x,
                                      const ChState& x,
                                      const ChStateDelta& v,
                                      const double T,
                                      bool force_state_scatter,
                                      UpdateFlags update_flags,
                                      bool call_setup,
                                      bool call_analyze) override {
        num_solves++;
        Dv.setConstant(0.0);
        return num_solves < 2;  // velocity-level step succeeds, acceleration-level step fails
    }
};

// One-degree-of-freedom particle of mass m under a velocity-dependent force F = f0 - b v (so dF/dv = -b
// is nonzero), solving its own linear system exactly with whatever factors it is given. The published
// acceleration must equal F(v)/m at the published velocity, which only holds if the acceleration-level
// solve carries no dF/dv or dF/dx factors (they belong to an integration step, not to an evaluation of
// the acceleration at the current state).
class DampedParticle : public ChIntegrableIIorder {
  public:
    double m = 2.0, b = 3.0, f0 = 5.0;
    double x = 0, v = 1.5, a = 0, T = 0;
    bool a_published = false;
    virtual unsigned int GetNumCoordsPosLevel() override { return 1; }
    virtual unsigned int GetNumCoordsVelLevel() override { return 1; }
    virtual void StateGather(ChState& xs, ChStateDelta& vs, double& Ts) override {
        xs(0) = x;
        vs(0) = v;
        Ts = T;
    }
    virtual void StateScatter(const ChState& xs, const ChStateDelta& vs, const double Ts, UpdateFlags update_flags) override {
        x = xs(0);
        v = vs(0);
        T = Ts;
    }
    virtual void StateScatterAcceleration(const ChStateDelta& as) override {
        a = as(0);
        a_published = true;
    }
    virtual void LoadResidual_F(ChVectorDynamic<>& R, const double c) override { R(0) += c * (f0 - b * v); }
    virtual void LoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override { R(0) += c * m * w(0); }
    virtual void LoadConstraint_C(ChVectorDynamic<>& Qc, const double c, const double c_vel, const bool do_clamp = false, const double mclam = 1e30) override {}
    virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc, const double c, const double c_vel) override {}
    virtual bool StateSolveCorrection(ChStateDelta& Dv,
                                      ChVectorDynamic<>& L,
                                      const ChVectorDynamic<>& R,
                                      const ChVectorDynamic<>& Qc,
                                      const double c_a,
                                      const double c_v,
                                      const double c_x,
                                      const ChState& xs,
                                      const ChStateDelta& vs,
                                      const double Ts,
                                      bool force_state_scatter,
                                      UpdateFlags update_flags,
                                      bool call_setup,
                                      bool call_analyze) override {
        Dv(0) = R(0) / (c_a * m + c_v * (-b));  // [c_a M + c_v dF/dv + c_x dF/dx] Dv = R, with dF/dx = 0
        return true;
    }
};

}  // namespace

TEST(AssemblyAnalysis, acceleration_level_includes_quadratic_velocity_term) {
    // The old velocity-increment path was off by omega^2 L along the rod (up to 9 m/s^2 at L = 1).
    for (double L : {1.0, 100.0}) {
        for (ChSolver::Type solver : {ChSolver::Type::BARZILAIBORWEIN, ChSolver::Type::SPARSE_QR}) {
            const double tol = 1e-3 * L;  // central-difference floor scales with L (see file header)
            for (double omega : {0.0, 1.0, 2.0, 3.0}) {
                PendulumResult r = RunPendulum(omega, L, solver, AssemblyAnalysis::Level::FULL);
                std::cout << "L = " << L << "  solver = " << (solver == ChSolver::Type::SPARSE_QR ? "SPARSE_QR" : "BB       ") << "  omega = " << omega << "  |a err| = " << r.acc_err
                          << "  radial = " << r.acc_err_radial << "  |F err| = " << r.react_err << "  |alpha| = " << r.angacc << std::endl;
                EXPECT_LT(r.acc_err, tol) << "L = " << L << " omega = " << omega;
                EXPECT_LT(r.react_err, tol) << "L = " << L << " omega = " << omega;
                EXPECT_LT(r.angacc, 1e-9) << "L = " << L << " omega = " << omega;
            }
        }
    }
}

TEST(AssemblyAnalysis, acceleration_only_action_does_not_drift_velocity) {
    // Level::ACCELERATION alone still runs the velocity-level step; it may change a consistent
    // velocity only by O(dt * a) with dt = 1e-6.
    PendulumResult r = RunPendulum(2.0, 1.0, ChSolver::Type::BARZILAIBORWEIN, AssemblyAnalysis::Level::ACCELERATION);
    std::cout << "ACCELERATION only: |a err| = " << r.acc_err << "  |v drift| = " << r.vel_drift << std::endl;
    EXPECT_LT(r.acc_err, 1e-3);
    EXPECT_LT(r.vel_drift, 1e-4);
}

TEST(AssemblyAnalysis, free_body_gets_gravity_and_no_reactions) {
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -G, 0));
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(M);
    body->SetInertiaXX(ChVector3d(1e-2, 1e-2, 1e-2));
    body->SetPos(ChVector3d(0.3, 0.2, 0.1));
    body->SetPosDt(ChVector3d(1, 2, 3));
    sys.Add(body);

    auto flag = sys.DoAssembly(AssemblyAnalysis::Level::FULL);
    EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED);
    EXPECT_EQ(sys.GetNumConstraints(), 0u);
    EXPECT_LT((body->GetPosDt2() - ChVector3d(0, -G, 0)).Length(), 1e-9);
    EXPECT_LT(body->GetAngAccParent().Length(), 1e-9);
}

TEST(AssemblyAnalysis, frame_kinematics_keeps_the_requested_step_and_consistent_accelerations) {
    // DoFrameKinematics() advances time by the system step after each DoAssembly(FULL); the assembly
    // must not leave its internal 1e-6 step behind in the system.
    Pendulum p(2.0, 1.0, ChSolver::Type::BARZILAIBORWEIN);
    auto flag = p.sys.DoFrameKinematics(0.002, 0.001);
    EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED);
    EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::ACCELERATION_INACCURATE);
    EXPECT_DOUBLE_EQ(p.sys.GetStep(), 0.001);
    EXPECT_NEAR(p.sys.GetChTime(), 0.002, 1e-9);
    EXPECT_LT((p.bob->GetPosDt2() - p.ExactAcc()).Length(), 1e-3);
}

TEST(AssemblyAnalysis, frame_kinematics_ends_assembled_at_frame_time) {
    // A body driven by a ChLinkMotorRotationAngle with angle(t) = t. After DoFrameKinematics(), the
    // configuration must correspond to the frame end time, not to the start of the last step (issue #851).
    // The second frame ends with a shortened last step.
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    sys.SetSolverType(ChSolver::Type::SPARSE_QR);
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);
    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(ChVector3d(1, 0, 0));
    sys.Add(body);
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(body, ground, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, 1));
    sys.AddLink(motor);

    for (double frame_time : {0.5, 1.05}) {
        auto flag = sys.DoFrameKinematics(frame_time, 0.1);
        EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED);
        EXPECT_NEAR(sys.GetChTime(), frame_time, 1e-12);
        EXPECT_NEAR(motor->GetMotorAngle(), frame_time, 1e-9);
        EXPECT_LT((body->GetPos() - ChVector3d(std::cos(frame_time), std::sin(frame_time), 0)).Length(), 1e-9);
        EXPECT_NEAR(body->GetAngVelParent().z(), 1.0, 1e-6);
    }
}

TEST(AssemblyAnalysis, velocity_dependent_force_acceleration_is_F_over_m) {
    DampedParticle p;
    ChAssemblyAnalysis analysis(p);
    auto flag = analysis.AssemblyAnalysis(AssemblyAnalysis::Level::VELOCITY | AssemblyAnalysis::Level::ACCELERATION, 1e-6);
    EXPECT_EQ(flag, AssemblyAnalysis::ExitFlag::SUCCESS);
    EXPECT_TRUE(p.a_published);
    double a_exact = (p.f0 - p.b * p.v) / p.m;  // at the published velocity
    std::cout << "damped particle: a = " << p.a << "  exact F(v)/m = " << a_exact << std::endl;
    EXPECT_NEAR(p.a, a_exact, 1e-8);
}

TEST(AssemblyAnalysis, motor_driven_body_matches_prescribed_motion) {
    // A body whose center of mass sits at radius r from a motor axis (z through the origin), driven by a
    // ChLinkMotorRotationAngle with a time-dependent angle whose first and second derivatives are both
    // nonzero at the assembly time. The rheonomic constraint exercises the Ct part of the velocity level
    // and the time part of the quadratic-velocity term. Exact: angular velocity thetad and angular
    // acceleration thetadd about z; a_COM = alpha x r_vec - thetad^2 r_vec. Gravity off to isolate it.
    //
    // Six coupled constraints (revolute lock + angle): the reported acceleration is a velocity increment
    // divided by dt = 1e-6, so an iterative solver's residual in the velocity is amplified by 1e6. The
    // accuracy assertions therefore use the direct sparse solver and the Barzilai-Borwein solver. With the
    // default PSOR settings the velocity residual (about 1e-2 here) becomes an O(1e4) acceleration error,
    // which DoAssembly() must report as ACCELERATION_INACCURATE, unless the tolerance of that check is raised.
    // This amplification is a property of the velocity-increment definition that the assembly has always
    // used, not of the quadratic-velocity term.
    const double r = 0.8;
    const double pi = 3.14159265358979323846;
    struct Case {
        ChSolver::Type solver;
        double acc_tol;  // tolerance of the acceleration accuracy check (0: default)
    };
    for (Case c : {Case{ChSolver::Type::SPARSE_QR, 0}, Case{ChSolver::Type::BARZILAIBORWEIN, 0}, Case{ChSolver::Type::PSOR, 0},
                   Case{ChSolver::Type::PSOR, 1e30}}) {
        ChSystemNSC sys;
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
        if (c.solver != ChSolver::Type::PSOR)
            sys.SetSolverType(c.solver);
        EXPECT_DOUBLE_EQ(sys.GetAssemblyAccelerationTolerance(), 1e-2);
        if (c.acc_tol > 0)
            sys.SetAssemblyAccelerationTolerance(c.acc_tol);
        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);
        sys.Add(ground);
        auto body = chrono_types::make_shared<ChBody>();
        body->SetMass(1.5);
        body->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
        body->SetPos(ChVector3d(r, 0, 0));
        sys.Add(body);
        auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        motor->Initialize(body, ground, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
        // angle(t) = shift + A sin(2 pi f t + pi/4) with shift = -A sin(pi/4): angle(0) = 0, angle'(0) != 0, angle''(0) != 0
        auto fun = chrono_types::make_shared<ChFunctionSine>(0.3, 2.0, pi / 4, -0.3 * std::sin(pi / 4));
        motor->SetAngleFunction(fun);
        sys.AddLink(motor);
        ASSERT_NEAR(fun->GetVal(0.0), 0.0, 1e-12);
        const double thd = fun->GetDer(0.0);
        const double thdd = fun->GetDer2(0.0);

        auto flag = sys.DoAssembly(AssemblyAnalysis::Level::FULL);
        EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED);

        ChVector3d rvec = body->GetPos();
        ChVector3d w_exact(0, 0, thd), al_exact(0, 0, thdd);
        ChVector3d v_exact = Vcross(w_exact, rvec);
        ChVector3d a_exact = Vcross(al_exact, rvec) + Vcross(w_exact, Vcross(w_exact, rvec));
        double v_err = (body->GetPosDt() - v_exact).Length();
        double a_err = (body->GetPosDt2() - a_exact).Length();
        const char* name = c.solver == ChSolver::Type::SPARSE_QR ? "SPARSE_QR" : (c.solver == ChSolver::Type::PSOR ? "PSOR     " : "BB       ");
        std::cout << "motor, solver = " << name << "  thd = " << thd << " thdd = " << thdd << "  w.z = " << body->GetAngVelParent().z()
                  << "  alpha.z = " << body->GetAngAccParent().z() << "  |v err| = " << v_err << "  |a err| = " << a_err << std::endl;
        if (c.solver == ChSolver::Type::PSOR) {
            // inaccurate accelerations, reported unless the check is effectively disabled
            if (c.acc_tol > 0)
                EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::ACCELERATION_INACCURATE);
            else
                EXPECT_EQ(flag, AssemblyAnalysis::ExitFlag::ACCELERATION_INACCURATE);
            continue;
        }
        EXPECT_NE(flag, AssemblyAnalysis::ExitFlag::ACCELERATION_INACCURATE);
        EXPECT_LT((rvec - ChVector3d(r, 0, 0)).Length(), 1e-9);  // position-level assembly left the consistent configuration alone
        EXPECT_LT(v_err, 1e-6);
        EXPECT_LT((body->GetAngVelParent() - w_exact).Length(), 1e-6);
        EXPECT_LT(a_err, 1e-3);
        EXPECT_LT((body->GetAngAccParent() - al_exact).Length(), 1e-3);
    }
}

TEST(AssemblyAnalysis, acceleration_level_solve_failure_is_reported_and_not_published) {
    FailingIntegrable integrable;
    ChAssemblyAnalysis analysis(integrable);
    auto flag = analysis.AssemblyAnalysis(AssemblyAnalysis::Level::VELOCITY | AssemblyAnalysis::Level::ACCELERATION, 1e-6);
    EXPECT_EQ(flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED);
    EXPECT_EQ(integrable.num_solves, 2);
    EXPECT_FALSE(integrable.acc_scattered);
}
