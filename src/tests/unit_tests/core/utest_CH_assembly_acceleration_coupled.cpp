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
// Authors: Noah Parsons
// =============================================================================
//
// Tests for DoAssembly at the ACCELERATION level on systems whose constraints are
// COUPLED, complementing the single-link pendulum in utest_CH_assembly_acceleration.
// Each system is built from point masses (ChBody, every attachment at the COM),
// massless rods (ChLinkDistance) and x-axis slides (ChLinkLockPrismatic), and has a
// closed-form constrained acceleration written directly from its Lagrangian:
//
//   N-link pendulum chain   absolute angles th_i from the downward vertical, equal
//                           masses m and rods l. With a_ij = (N - max(i,j)) m l^2,
//                               M_ij = a_ij cos(th_i - th_j)
//                               M thdd = -( sum_j a_ij sin(th_i - th_j) thd_j^2
//                                           + (N - i) m g l sin th_i )
//                           Every rod's quadratic-velocity term feeds every link.
//
//   Cart-pole               cart M on the x-axis, point mass m on a rod of length l
//                           at angle th from the downward vertical:
//                               [M+m        m l cos th] [xdd ]   [ m l thd^2 sin th]
//                               [m l cos th m l^2     ] [thdd] = [-m g l sin th    ]
//                           A light cart (M << m) makes the matrix nearly singular at
//                           th = 0.
//
//   Slider-crank            crank point mass m_c at radius r, slider m_s on the x-axis,
//                           rod of length l > r. q = (th, x), one loop constraint
//                               g = x^2 - 2 r x cos th + r^2 - l^2 = 0,
//                           solved as the saddle-point system
//                               [M  J^T] [qdd]   [F    ]
//                               [J  0  ] [lam] = [gamma]
//                           with M = diag(m_c r^2, m_s), F = (-m_c g r cos th, 0),
//                           J = (2 r x sin th, 2 x - 2 r cos th) and
//                           gamma = -(2 xd^2 + 4 r xd thd sin th + 2 r x thd^2 cos th).
//                           Probed through dead centre (th = 0, pi), where the slider
//                           stops contributing to the effective inertia.
//
// The exact generalized accelerations are mapped to body accelerations and compared
// with GetPosDt2(). Assertions use the direct sparse solver, with the tolerance of
// utest_CH_assembly_acceleration (1e-3 for unit lengths): the acceleration is a
// velocity increment divided by the internal 1e-6 step, so its floor is set by that
// step, not by the solve. The default PSOR result is printed for the record, with
// the two signals a user could inspect: the exit flag returned by DoAssembly and the
// iteration count and error of the iterative solver's last solve (the acceleration-
// level step). PSOR is also run with ten times its default iteration limit.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/core/ChMatrix.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"

using namespace chrono;

namespace {

const double G = 9.81;
const double PI = 3.14159265358979323846;
const double TOL = 1e-3;

// Unit vector from pivot to mass, and unit tangent, for an angle from the downward vertical.
ChVector3d Dir(double th) {
    return ChVector3d(std::sin(th), -std::cos(th), 0);
}
ChVector3d Tan(double th) {
    return ChVector3d(std::cos(th), std::sin(th), 0);
}

// Outcome of one assembly: the acceleration error and what Chrono reported.
struct Outcome {
    double err = 0;                                             // max |a_chrono - a_exact| over the bodies
    AssemblyAnalysis::ExitFlag flag = AssemblyAnalysis::ExitFlag::SUCCESS;
    int iterations = -1, max_iterations = -1;                   // iterative solver only
    double solver_error = 0, solver_tolerance = 0;              // iterative solver only
};

// A solver choice: type, plus an iteration limit for the iterative solver (0 = default).
struct Solver {
    ChSolver::Type type;
    int max_iterations;
};

// A system of point-mass bodies joined by rods and slides.
struct Mechanism {
    ChSystemNSC sys;
    std::shared_ptr<ChBody> ground;
    std::vector<std::shared_ptr<ChBody>> bodies;

    explicit Mechanism(Solver solver) {
        sys.SetGravitationalAcceleration(ChVector3d(0, -G, 0));
        if (solver.type != ChSolver::Type::PSOR)
            sys.SetSolverType(solver.type);
        if (solver.max_iterations > 0 && sys.GetSolver()->IsIterative())
            sys.GetSolver()->AsIterative()->SetMaxIterations(solver.max_iterations);
        ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);
        sys.Add(ground);
    }

    std::shared_ptr<ChBody> AddMass(double m, const ChVector3d& pos, const ChVector3d& vel) {
        auto b = chrono_types::make_shared<ChBody>();
        b->SetMass(m);
        b->SetInertiaXX(ChVector3d(1e-2 * m, 1e-2 * m, 1e-2 * m));
        b->SetPos(pos);
        b->SetPosDt(vel);
        sys.Add(b);
        bodies.push_back(b);
        return b;
    }

    // Massless rod between two body centres (a null body means the ground origin).
    void AddRod(std::shared_ptr<ChBody> a, std::shared_ptr<ChBody> b) {
        auto link = chrono_types::make_shared<ChLinkDistance>();
        ChVector3d pa = a ? a->GetPos() : ChVector3d(0, 0, 0);
        link->Initialize(a ? a : ground, b, false, pa, b->GetPos(), true);
        sys.AddLink(link);
    }

    // Slide along the world x-axis (the prismatic link's free axis is its local z).
    void AddSlideX(std::shared_ptr<ChBody> b) {
        auto link = chrono_types::make_shared<ChLinkLockPrismatic>();
        link->Initialize(ground, b, ChFrame<>(b->GetPos(), QuatFromAngleY(PI / 2)));
        sys.AddLink(link);
    }

    // Assemble, then compare each body's acceleration with the exact one.
    Outcome Assemble(const std::vector<ChVector3d>& exact) {
        Outcome out;
        out.flag = sys.DoAssembly(AssemblyAnalysis::Level::FULL);
        for (size_t i = 0; i < bodies.size(); i++)
            out.err = std::max(out.err, (bodies[i]->GetPosDt2() - exact[i]).Length());
        if (auto it = sys.GetSolver()->AsIterative()) {
            out.iterations = it->GetIterations();
            out.max_iterations = it->GetMaxIterations();
            out.solver_error = it->GetError();
            out.solver_tolerance = it->GetTolerance();
        }
        return out;
    }
};

// ---- N-link pendulum chain ---------------------------------------------------------------------

struct Chain {
    int N;
    double m = 1.0, l = 1.0;
    std::vector<double> th, thd;

    std::vector<double> Exact() const {
        ChMatrixDynamic<> M(N, N);
        ChVectorDynamic<> rhs(N);
        for (int i = 0; i < N; i++) {
            rhs(i) = -(N - i) * m * G * l * std::sin(th[i]);
            for (int j = 0; j < N; j++) {
                double a = (N - std::max(i, j)) * m * l * l;
                M(i, j) = a * std::cos(th[i] - th[j]);
                rhs(i) -= a * std::sin(th[i] - th[j]) * thd[j] * thd[j];
            }
        }
        ChVectorDynamic<> thdd = M.partialPivLu().solve(rhs);
        return std::vector<double>(thdd.data(), thdd.data() + N);
    }

    Outcome Run(Solver solver) const {
        Mechanism mech(solver);
        ChVector3d pos(0, 0, 0), vel(0, 0, 0);
        std::shared_ptr<ChBody> prev;
        for (int i = 0; i < N; i++) {
            pos += Dir(th[i]) * l;
            vel += Tan(th[i]) * (l * thd[i]);
            auto b = mech.AddMass(m, pos, vel);
            mech.AddRod(prev, b);
            prev = b;
        }
        std::vector<double> thdd = Exact();
        std::vector<ChVector3d> acc;
        ChVector3d a(0, 0, 0);
        for (int i = 0; i < N; i++) {
            a += (Tan(th[i]) * thdd[i] - Dir(th[i]) * (thd[i] * thd[i])) * l;
            acc.push_back(a);
        }
        return mech.Assemble(acc);
    }
};

// ---- Cart-pole ---------------------------------------------------------------------------------

struct CartPole {
    double M, m = 1.0, l = 1.0;
    double x, xd, th, thd;

    Outcome Run(Solver solver) const {
        ChMatrixDynamic<> A(2, 2);
        ChVectorDynamic<> f(2);
        A << M + m, m * l * std::cos(th), m * l * std::cos(th), m * l * l;
        f << m * l * thd * thd * std::sin(th), -m * G * l * std::sin(th);
        ChVectorDynamic<> qdd = A.partialPivLu().solve(f);

        Mechanism mech(solver);
        ChVector3d c(x, 0, 0), cv(xd, 0, 0);
        auto cart = mech.AddMass(M, c, cv);
        auto pole = mech.AddMass(m, c + Dir(th) * l, cv + Tan(th) * (l * thd));
        mech.AddSlideX(cart);
        mech.AddRod(cart, pole);

        ChVector3d a_cart(qdd(0), 0, 0);
        ChVector3d a_pole = a_cart + (Tan(th) * qdd(1) - Dir(th) * (thd * thd)) * l;
        return mech.Assemble({a_cart, a_pole});
    }
};

// ---- Slider-crank ------------------------------------------------------------------------------

struct SliderCrank {
    double ratio, mass_ratio;  // l / r and m_s / m_c
    double th, thd;
    double r = 1.0, m_c = 1.0;

    Outcome Run(Solver solver) const {
        double l = ratio * r, m_s = mass_ratio * m_c;
        double s = std::sin(th), c = std::cos(th);
        double x = r * c + std::sqrt(l * l - r * r * s * s);  // slider on the +x branch
        double xd = -r * x * s * thd / (x - r * c);              // from gdot = 0

        ChMatrixDynamic<> K(3, 3);
        ChVectorDynamic<> f(3);
        double J0 = 2 * r * x * s, J1 = 2 * x - 2 * r * c;
        K << m_c * r * r, 0, J0,  //
            0, m_s, J1,           //
            J0, J1, 0;
        f << -m_c * G * r * c, 0, -(2 * xd * xd + 4 * r * xd * s * thd + 2 * r * x * c * thd * thd);
        ChVectorDynamic<> sol = K.partialPivLu().solve(f);
        double thdd = sol(0), xdd = sol(1);

        Mechanism mech(solver);
        auto crank = mech.AddMass(m_c, ChVector3d(r * c, r * s, 0), ChVector3d(-r * thd * s, r * thd * c, 0));
        auto slider = mech.AddMass(m_s, ChVector3d(x, 0, 0), ChVector3d(xd, 0, 0));
        mech.AddRod(nullptr, crank);
        mech.AddRod(crank, slider);
        mech.AddSlideX(slider);

        ChVector3d a_crank(r * (-thdd * s - thd * thd * c), r * (thdd * c - thd * thd * s), 0);
        return mech.Assemble({a_crank, ChVector3d(xdd, 0, 0)});
    }
};

const char* FlagName(AssemblyAnalysis::ExitFlag flag) {
    switch (flag) {
        case AssemblyAnalysis::ExitFlag::NOT_CONVERGED:
            return "NOT_CONVERGED";
        case AssemblyAnalysis::ExitFlag::SUCCESS:
            return "SUCCESS";
        case AssemblyAnalysis::ExitFlag::ABSTOL_RESIDUAL:
            return "ABSTOL_RESIDUAL";
        case AssemblyAnalysis::ExitFlag::RELTOL_UPDATE:
            return "RELTOL_UPDATE";
        case AssemblyAnalysis::ExitFlag::ABSTOL_UPDATE:
            return "ABSTOL_UPDATE";
    }
    return "?";
}

// Assert with the direct solver; print PSOR (default and 10x iteration limit) for the record.
template <typename Case>
void Check(const Case& c, const std::string& label) {
    const Solver solvers[] = {{ChSolver::Type::SPARSE_QR, 0}, {ChSolver::Type::PSOR, 0}, {ChSolver::Type::PSOR, 500}};
    const char* names[] = {"SPARSE_QR ", "PSOR      ", "PSOR x500 "};
    for (int k = 0; k < 3; k++) {
        Outcome r = c.Run(solvers[k]);
        std::cout << label << "  solver = " << names[k] << "  max |a err| = " << r.err << "  flag = " << FlagName(r.flag);
        if (r.iterations >= 0)
            std::cout << "  iterations = " << r.iterations << "/" << r.max_iterations << "  solver error = " << r.solver_error
                      << " (tolerance " << r.solver_tolerance << ")";
        std::cout << std::endl;
        if (k == 0) {
            EXPECT_LT(r.err, TOL) << label;
            EXPECT_NE(r.flag, AssemblyAnalysis::ExitFlag::NOT_CONVERGED) << label;
        }
    }
}

}  // namespace

TEST(AssemblyAnalysisCoupled, pendulum_chain) {
    // Moving links: the quadratic-velocity term of every rod enters every body's acceleration.
    Check(Chain{2, 1.0, 1.0, {0.4, -0.7}, {1.5, -2.0}}, "chain N=2");
    Check(Chain{3, 1.0, 1.0, {0.3, 1.1, -0.5}, {2.0, -1.0, 3.0}}, "chain N=3");
    Check(Chain{5, 1.0, 1.0, {0.2, 0.5, -0.4, 1.0, -1.2}, {1.0, -1.5, 2.0, -0.5, 1.0}}, "chain N=5");
    // Fully extended and spinning as a rigid rod: aligned rods, all terms radial.
    Check(Chain{3, 1.0, 1.0, {0.6, 0.6, 0.6}, {2.0, 2.0, 2.0}}, "chain N=3 aligned");
    // Folded back on itself.
    Check(Chain{3, 1.0, 1.0, {0.4, 0.4 + PI, 0.4}, {1.0, -1.0, 1.0}}, "chain N=3 folded");
}

TEST(AssemblyAnalysisCoupled, cart_pole) {
    for (double M : {10.0, 1.0, 1e-2}) {
        for (double th : {0.0, 1.0, 2.5}) {
            Check(CartPole{M, 1.0, 1.0, 0.2, 0.5, th, 3.0}, "cart-pole M/m=" + std::to_string(M) + " th=" + std::to_string(th));
        }
    }
}

TEST(AssemblyAnalysisCoupled, slider_crank_through_dead_centre) {
    for (double ratio : {3.0, 1.5}) {
        for (double mass_ratio : {1.0, 1e2}) {
            for (double th : {0.0, 0.3, 1.0, PI}) {
                Check(SliderCrank{ratio, mass_ratio, th, 4.0},
                      "slider-crank l/r=" + std::to_string(ratio) + " ms/mc=" + std::to_string(mass_ratio) + " th=" + std::to_string(th));
            }
        }
    }
}
