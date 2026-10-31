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
//
// Unit tests for the Jacobians of stiff spring-damper-actuators (ChLinkTSDA and ChLinkRSDA).
// The stiffness (K = -dQ/dq) and damping (R = -dQ/dv) matrices that a stiff link loads into the system matrices are
// compared with:
// - analytical Jacobians, for linear springs in configurations where they are known in closed form;
// - central finite-difference approximations of the generalized forces of the link, for nonlinear springs in general
//   configurations (attachment offsets, arbitrary body orientations, nonzero velocities, preload).
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"

#include "gtest/gtest.h"

using namespace chrono;

// -----------------------------------------------------------------------------

struct Jacobians {
    ChMatrixDynamic<> K;  // stiffness matrix, -dQ/dq
    ChMatrixDynamic<> R;  // damping matrix, -dQ/dv
};

// Create a system with two free bodies (no gravity).
static void CreateBodies(ChSystem& sys, std::shared_ptr<ChBody>& body1, std::shared_ptr<ChBody>& body2) {
    sys.SetGravitationalAcceleration(VNULL);

    body1 = chrono_types::make_shared<ChBody>();
    body1->SetMass(2.0);
    body1->SetInertiaXX(ChVector3d(0.1, 0.2, 0.3));
    sys.AddBody(body1);

    body2 = chrono_types::make_shared<ChBody>();
    body2->SetMass(3.0);
    body2->SetInertiaXX(ChVector3d(0.2, 0.3, 0.4));
    sys.AddBody(body2);
}

// Jacobians loaded by the stiff links of the system into the system stiffness and damping matrices.
static Jacobians LinkJacobians(ChSystem& sys) {
    sys.Setup();
    sys.Update(UpdateFlags::UPDATE_ALL);
    sys.DescriptorPrepareInject();

    ChSparseMatrix K;
    ChSparseMatrix R;
    sys.GetStiffnessMatrix(K);
    sys.GetDampingMatrix(R);

    return {ChMatrixDynamic<>(K), ChMatrixDynamic<>(R)};
}

// Generalized forces of the link at the given system state.
static ChVectorDynamic<> LinkForces(ChSystem& sys, ChLinkBase& link, const ChState& x, const ChStateDelta& v, double time) {
    sys.StateScatter(x, v, time, UpdateFlags::UPDATE_ALL);
    ChVectorDynamic<> Q(sys.GetNumCoordsVelLevel());
    Q.setZero();
    static_cast<ChPhysicsItem&>(link).IntLoadResidual_F(0, Q, 1.0);
    return Q;
}

// Central finite-difference approximation of the Jacobians of the link generalized forces.
// Positions are perturbed at the velocity level (rotations through the body orientation increment).
static Jacobians FiniteDifferenceJacobians(ChSystem& sys, ChLinkBase& link) {
    const double h = 1e-6;

    int nq = sys.GetNumCoordsPosLevel();
    int nv = sys.GetNumCoordsVelLevel();
    ChState x(nq, &sys);
    ChStateDelta v(nv, &sys);
    double time;
    sys.StateGather(x, v, time);

    Jacobians jac;
    jac.K.resize(nv, nv);
    jac.R.resize(nv, nv);

    ChState x_pert(nq, &sys);
    ChStateDelta dx(nv, &sys);
    for (int j = 0; j < nv; j++) {
        dx.setZero(nv, &sys);
        dx(j) = h;
        sys.StateIncrementX(x_pert, x, dx);
        auto Qp = LinkForces(sys, link, x_pert, v, time);
        dx(j) = -h;
        sys.StateIncrementX(x_pert, x, dx);
        auto Qm = LinkForces(sys, link, x_pert, v, time);
        jac.K.col(j) = -(Qp - Qm) / (2 * h);
    }

    ChStateDelta v_pert(nv, &sys);
    for (int j = 0; j < nv; j++) {
        v_pert = v;
        v_pert(j) += h;
        auto Qp = LinkForces(sys, link, x, v_pert, time);
        v_pert(j) -= 2 * h;
        auto Qm = LinkForces(sys, link, x, v_pert, time);
        jac.R.col(j) = -(Qp - Qm) / (2 * h);
    }

    // Restore the system state
    sys.StateScatter(x, v, time, UpdateFlags::UPDATE_ALL);

    return jac;
}

// Check that two matrices agree, relative to the largest entry of the reference matrix.
static void CheckMatrix(const ChMatrixDynamic<>& A, const ChMatrixDynamic<>& ref, double rel_tol, const std::string& label) {
    ASSERT_EQ(A.rows(), ref.rows()) << label;
    ASSERT_EQ(A.cols(), ref.cols()) << label;
    double scale = std::max(1.0, ref.cwiseAbs().maxCoeff());
    double error = (A - ref).cwiseAbs().maxCoeff();
    EXPECT_LE(error, rel_tol * scale) << label << ": max error " << error << ", max entry " << scale;
}

// Nonlinear TSDA force: cubic spring, quadratic damping, and actuation.
class TSDAForce : public ChLinkTSDA::ForceFunctor {
  public:
    virtual double evaluate(double time, double rest_length, double length, double vel, const ChLinkTSDA& link) override {
        double d = length - rest_length;
        return 50 - 2e3 * d - 5e4 * d * d * d - 40 * vel - 15 * vel * std::abs(vel);
    }
};

// Nonlinear RSDA torque: cubic spring, regularized Coulomb friction, and viscous damping.
class RSDATorque : public ChLinkRSDA::TorqueFunctor {
  public:
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override {
        double d = angle - rest_angle;
        return -300 * d - 800 * d * d * d - 1.2 * std::tanh(vel / 2.0) - 5 * vel;
    }
};

// -----------------------------------------------------------------------------

// Linear TSDA between the body centers, preloaded (current length different from the rest length), at zero relative
// velocity. The forces on the bodies are +/- f n, with n the unit vector from body 2 to body 1 and
// f = -k (l - l0) - c dl/dt, and do not depend on the body orientations. Then, with A = k n n' + k (l - l0) / l (I - n n')
// and B = c n n', the blocks of K and R corresponding to the body translations are [A -A; -A A] and [B -B; -B B].
TEST(ChLinkTSDA, jacobians_analytical) {
    const double k = 2e3;
    const double c = 40;
    const double l0 = 0.8;

    ChSystemNSC sys;
    std::shared_ptr<ChBody> body1, body2;
    CreateBodies(sys, body1, body2);
    body1->SetPos(ChVector3d(0.3, 0.2, 0.1));
    body2->SetPos(ChVector3d(-0.4, -0.3, 0.5));
    body1->SetRot(QuatFromAngleAxis(0.4, ChVector3d(1, 2, 3).GetNormalized()));
    body2->SetRot(QuatFromAngleAxis(-0.7, ChVector3d(3, -1, 2).GetNormalized()));
    body1->SetPosDt(ChVector3d(0.2, -0.1, 0.3));
    body2->SetPosDt(ChVector3d(0.2, -0.1, 0.3));

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body1, body2, true, VNULL, VNULL);
    spring->SetRestLength(l0);
    spring->SetSpringCoefficient(k);
    spring->SetDampingCoefficient(c);
    spring->IsStiff(true);
    sys.AddLink(spring);

    auto jac = LinkJacobians(sys);

    ChVector3d d = body1->GetPos() - body2->GetPos();
    double l = d.Length();
    ChVectorN<double, 3> n = (d / l).eigen();
    ChMatrix33<> nn = n * n.transpose();
    ChMatrix33<> A = k * nn + k * (l - l0) / l * (ChMatrix33<>::Identity() - nn);
    ChMatrix33<> B = c * nn;

    ChMatrixDynamic<> K(12, 12);
    ChMatrixDynamic<> R(12, 12);
    K.setZero();
    R.setZero();
    int o1 = body1->Variables().GetOffset();
    int o2 = body2->Variables().GetOffset();
    K.block(o1, o1, 3, 3) = A;
    K.block(o1, o2, 3, 3) = -A;
    K.block(o2, o1, 3, 3) = -A;
    K.block(o2, o2, 3, 3) = A;
    R.block(o1, o1, 3, 3) = B;
    R.block(o1, o2, 3, 3) = -B;
    R.block(o2, o1, 3, 3) = -B;
    R.block(o2, o2, 3, 3) = B;

    CheckMatrix(jac.K, K, 1e-6, "TSDA stiffness matrix");
    CheckMatrix(jac.R, R, 1e-6, "TSDA damping matrix");

    // The analytical Jacobians also agree with central finite differences
    auto jac_fd = FiniteDifferenceJacobians(sys, *spring);
    CheckMatrix(jac_fd.K, K, 1e-5, "TSDA stiffness matrix (finite differences)");
    CheckMatrix(jac_fd.R, R, 1e-5, "TSDA damping matrix (finite differences)");
}

// Nonlinear TSDA with attachment offsets, in a general configuration.
TEST(ChLinkTSDA, jacobians_finite_differences) {
    ChSystemNSC sys;
    std::shared_ptr<ChBody> body1, body2;
    CreateBodies(sys, body1, body2);
    body1->SetPos(ChVector3d(0.3, 0.2, 0.1));
    body2->SetPos(ChVector3d(-0.4, -0.3, 0.5));
    body1->SetRot(QuatFromAngleAxis(0.4, ChVector3d(1, 2, 3).GetNormalized()));
    body2->SetRot(QuatFromAngleAxis(-0.7, ChVector3d(3, -1, 2).GetNormalized()));
    body1->SetPosDt(ChVector3d(0.5, -0.2, 0.1));
    body2->SetPosDt(ChVector3d(-0.3, 0.4, -0.2));
    body1->SetAngVelLocal(ChVector3d(0.7, -0.4, 1.1));
    body2->SetAngVelLocal(ChVector3d(-0.6, 0.9, 0.3));

    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body1, body2, true, ChVector3d(0.1, -0.05, 0.2), ChVector3d(-0.15, 0.1, 0.05));
    spring->SetRestLength(0.6);
    spring->RegisterForceFunctor(chrono_types::make_shared<TSDAForce>());
    spring->IsStiff(true);
    sys.AddLink(spring);

    auto jac = LinkJacobians(sys);
    auto jac_fd = FiniteDifferenceJacobians(sys, *spring);

    CheckMatrix(jac.K, jac_fd.K, 1e-5, "TSDA stiffness matrix");
    CheckMatrix(jac.R, jac_fd.R, 1e-5, "TSDA damping matrix");
}

// Linear RSDA between bodies with aligned frames, at the rest angle and at zero relative angular velocity. The torques
// on the bodies are -/+ t z, with z the RSDA axis and t = -k a - c da/dt, where a is the relative rotation of body 2
// about z. Then the only nonzero entries of K and R are those for the rotations about z, [k -k; -k k] and [c -c; -c c].
// The bodies spin about z only: a spin about another axis makes the relative angle rate depend on the body orientations,
// which adds damping terms to K.
TEST(ChLinkRSDA, jacobians_analytical) {
    const double k = 300;
    const double c = 5;

    ChSystemNSC sys;
    std::shared_ptr<ChBody> body1, body2;
    CreateBodies(sys, body1, body2);
    body1->SetPos(ChVector3d(0.3, 0.2, 0.1));
    body2->SetPos(ChVector3d(-0.4, -0.3, 0.5));
    body1->SetAngVelLocal(ChVector3d(0, 0, 0.4));
    body2->SetAngVelLocal(ChVector3d(0, 0, 0.4));

    auto spring = chrono_types::make_shared<ChLinkRSDA>();
    spring->Initialize(body1, body2, ChFramed(ChVector3d(0, 0, 0.2)));
    spring->SetSpringCoefficient(k);
    spring->SetDampingCoefficient(c);
    spring->IsStiff(true);
    sys.AddLink(spring);

    auto jac = LinkJacobians(sys);

    ChMatrixDynamic<> K(12, 12);
    ChMatrixDynamic<> R(12, 12);
    K.setZero();
    R.setZero();
    int z1 = body1->Variables().GetOffset() + 5;
    int z2 = body2->Variables().GetOffset() + 5;
    K(z1, z1) = k;
    K(z1, z2) = -k;
    K(z2, z1) = -k;
    K(z2, z2) = k;
    R(z1, z1) = c;
    R(z1, z2) = -c;
    R(z2, z1) = -c;
    R(z2, z2) = c;

    CheckMatrix(jac.K, K, 1e-6, "RSDA stiffness matrix");
    CheckMatrix(jac.R, R, 1e-6, "RSDA damping matrix");

    // The analytical Jacobians also agree with central finite differences
    auto jac_fd = FiniteDifferenceJacobians(sys, *spring);
    CheckMatrix(jac_fd.K, K, 1e-5, "RSDA stiffness matrix (finite differences)");
    CheckMatrix(jac_fd.R, R, 1e-5, "RSDA damping matrix (finite differences)");
}

// Nonlinear RSDA, preloaded (away from the rest angle), in a general configuration.
TEST(ChLinkRSDA, jacobians_finite_differences) {
    ChSystemNSC sys;
    std::shared_ptr<ChBody> body1, body2;
    CreateBodies(sys, body1, body2);
    body1->SetPos(ChVector3d(0.3, 0.2, 0.1));
    body2->SetPos(ChVector3d(-0.4, -0.3, 0.5));
    body1->SetRot(QuatFromAngleAxis(0.4, ChVector3d(1, 2, 3).GetNormalized()));
    body2->SetRot(QuatFromAngleAxis(-0.7, ChVector3d(3, -1, 2).GetNormalized()));
    body1->SetAngVelLocal(ChVector3d(0.7, -0.4, 1.1));
    body2->SetAngVelLocal(ChVector3d(-0.6, 0.9, 0.3));

    auto spring = chrono_types::make_shared<ChLinkRSDA>();
    spring->Initialize(body1, body2, false, ChFramed(VNULL, QuatFromAngleX(0.3)), ChFramed(VNULL, QuatFromAngleY(-0.2)));
    spring->SetRestAngle(spring->GetAngle() - 0.25);
    spring->RegisterTorqueFunctor(chrono_types::make_shared<RSDATorque>());
    spring->IsStiff(true);
    sys.AddLink(spring);

    auto jac = LinkJacobians(sys);
    auto jac_fd = FiniteDifferenceJacobians(sys, *spring);

    CheckMatrix(jac.K, jac_fd.K, 1e-5, "RSDA stiffness matrix");
    CheckMatrix(jac.R, jac_fd.R, 1e-5, "RSDA damping matrix");
}

// A link that is not declared stiff loads no Jacobians.
TEST(ChLinkRSDA, jacobians_not_stiff) {
    ChSystemNSC sys;
    std::shared_ptr<ChBody> body1, body2;
    CreateBodies(sys, body1, body2);

    auto spring = chrono_types::make_shared<ChLinkRSDA>();
    spring->Initialize(body1, body2, ChFramed());
    spring->SetSpringCoefficient(300);
    spring->SetDampingCoefficient(5);
    sys.AddLink(spring);

    auto jac = LinkJacobians(sys);
    EXPECT_EQ(jac.K.cwiseAbs().maxCoeff(), 0.0);
    EXPECT_EQ(jac.R.cwiseAbs().maxCoeff(), 0.0);
}
