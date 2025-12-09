// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo code for using a physics item with external dynamics described by DAEs.
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChExternalDynamicsDAE.h"

#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------

/// Simple 2D pendulum modeled as an ODE.
/// - 1 state: pendulum angle.
/// - 1 state derivative.
/// - 0 constraints.
class Pendulum2D_ODE : public ChExternalDynamicsDAE {
  public:
    Pendulum2D_ODE(double L, double mass) : g(9.8), L(L), m(mass), I(mass * L * L * CH_1_3) {}

    virtual Pendulum2D_ODE* Clone() const override { return new Pendulum2D_ODE(*this); }

    virtual unsigned int GetNumStates() const override { return 1; }
    virtual unsigned int GetNumStateDerivatives() const override { return 1; }
    virtual unsigned int GetNumAlgebraicConstraints() const override { return 0; }

    virtual bool InExplicitForm() const override { return false; }
    virtual bool IsStiff() const override { return false; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0, ChVectorDynamic<>& yd0) override {
        y0(0) = 0;  // theta
        yd0(0) = 0;
    }

    virtual void CalculateMassMatrix(ChMatrixDynamic<>& M) override {
        M.setZero();
        M(0, 0) = I + m * L * L;
    }

    virtual bool CalculateMassTimesVector(const ChVectorDynamic<>& v, ChVectorDynamic<>& Mv) override {
        Mv(0) = (I + m * L * L) * v(0);

        return true;
    }

    virtual void CalculateForce(double time,
                                const ChVectorDynamic<>& y,
                                const ChVectorDynamic<>& yd,
                                ChVectorDynamic<>& F) override {
        F(0) = -m * g * std::cos(y(0));
    }

  private:
    double g;  // gravitational acceleration
    double L;  // pendulum half-length
    double m;  // pendulum mass
    double I;  // pendulum moment of inertia
};

// -----------------------------------------------------------------------------

/// Simple 2D pendulum modeled as a DAE.
/// - 3 states: (x,y) and pendulum angle.
/// - 3 state derivatives.
/// - 2 constraints.
class Pendulum2D_DAE : public ChExternalDynamicsDAE {
  public:
    Pendulum2D_DAE(double L, double mass) : g(9.8), L(L), m(mass), I(mass * L * L * CH_1_3) {}

    virtual Pendulum2D_DAE* Clone() const override { return new Pendulum2D_DAE(*this); }

    virtual unsigned int GetNumStates() const override { return 3; }
    virtual unsigned int GetNumStateDerivatives() const override { return 3; }
    virtual unsigned int GetNumAlgebraicConstraints() const override { return 2; }

    virtual bool InExplicitForm() const override { return false; }
    virtual bool IsStiff() const override { return false; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0, ChVectorDynamic<>& yd0) override {
        y0(0) = L;  // x
        y0(1) = 0;  // y
        y0(2) = 0;  // theta

        yd0.setZero();
    }

    virtual void CalculateMassMatrix(ChMatrixDynamic<>& M) override {
        M.setZero();
        M(0, 0) = m;
        M(1, 1) = m;
        M(2, 2) = I;
    }

    virtual bool CalculateMassTimesVector(const ChVectorDynamic<>& v, ChVectorDynamic<>& Mv) override {
        Mv(0) = m * v(0);
        Mv(1) = m * v(1);
        Mv(2) = I * v(2);

        return true;
    }

    virtual void CalculateForce(double time,
                                const ChVectorDynamic<>& y,
                                const ChVectorDynamic<>& yd,
                                ChVectorDynamic<>& F) override {
        F(0) = 0;
        F(1) = -m * g;
        F(2) = 0;
    }

    virtual void CalculateConstraintViolation(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& c) override {
        c(0) = y(0) - L * std::cos(y(2));  // x = L * cos(theta)
        c(1) = y(1) - L * std::sin(y(2));  // y = L * sin(theta)
    }

    virtual void CalculateConstraintJacobian(double time,
                                             const ChVectorDynamic<>& y,
                                             const ChVectorDynamic<>& c,
                                             ChMatrixDynamic<>& J) override {
        J(0, 0) = 1;
        J(0, 1) = 0;
        J(0, 2) = L * std::sin(y(2));

        J(1, 0) = 0;
        J(1, 1) = 1;
        J(1, 2) = -L * std::cos(y(2));
    }

  private:
    double g;  // gravitational acceleration
    double L;  // pendulum half-length
    double m;  // pendulum mass
    double I;  // pendulum moment of inertia
};

// -----------------------------------------------------------------------------

/// Simple 3D pendulum modeled as a DAE.
/// - 7 states: pendulum position and orientation (quaternions).
/// - 6 state derivatives: pendulum linear and angular velocities.
/// - 5 constraints.
class Pendulum3D_DAE : public ChExternalDynamicsDAE {
  public:
    Pendulum3D_DAE(double L, double mass)
        : g(9.8),
          L(L),
          m(mass),
          I(0.01, mass * L * L * CH_1_3, mass * L * L * CH_1_3),
          frame_loc(ChFrame<>(ChVector3d(-L, 0, 0), QUNIT)) {}

    virtual Pendulum3D_DAE* Clone() const override { return new Pendulum3D_DAE(*this); }

    virtual unsigned int GetNumStates() const override { return 7; }
    virtual unsigned int GetNumStateDerivatives() const override { return 6; }
    virtual unsigned int GetNumAlgebraicConstraints() const override { return 5; }

    virtual bool InExplicitForm() const override { return false; }
    virtual bool IsStiff() const override { return false; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0, ChVectorDynamic<>& yd0) override {
        ChVector3d p0(L, 0, 0);
        ChQuaterniond q0 = QUNIT;
        y0.segment(0, 3) = p0.eigen();
        y0.segment(3, 4) = q0.eigen();

        yd0.setZero();
    }

    virtual void CalculateMassMatrix(ChMatrixDynamic<>& M) override {
        M.setZero();
        M.diagonal() << m, m, m, I.x(), I.y(), I.z();
    }

    virtual bool CalculateMassTimesVector(const ChVectorDynamic<>& v, ChVectorDynamic<>& Mv) override {
        Mv(0) = m * v(0);
        Mv(1) = m * v(1);
        Mv(2) = m * v(2);
        Mv(3) = I.x() * v(3);
        Mv(4) = I.y() * v(4);
        Mv(5) = I.z() * v(5);

        return true;
    }

    virtual void OnUpdate(double time, const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) override {
        // Current body frame (expressed in absolute frame)
        ChFrame<> body(ChVector3d(y.segment(0, 3)), ChQuaterniond(y.segment(3, 4)));

        // Joint frame (expressed in absolute frame)
        frame_abs = frame_loc >> body;
    }

    virtual void CalculateForce(double time,
                                const ChVectorDynamic<>& y,
                                const ChVectorDynamic<>& yd,
                                ChVectorDynamic<>& F) override {
        F.setZero();
        F(1) = -m * g;
    }

    virtual void CalculateConstraintViolation(double time, const ChVectorDynamic<>& y, ChVectorDynamic<>& c) override {
        ChVector3d w(0, 0, 1);  // absolute z direction

        c(0) = frame_abs.GetPos().x();
        c(1) = frame_abs.GetPos().y();
        c(2) = frame_abs.GetPos().z();
        c(3) = Vdot(frame_abs.GetRotMat().GetAxisX(), w);
        c(4) = Vdot(frame_abs.GetRotMat().GetAxisY(), w);
    }

    virtual void CalculateConstraintJacobian(double time,
                                             const ChVectorDynamic<>& y,
                                             const ChVectorDynamic<>& c,
                                             ChMatrixDynamic<>& J) override {
        ChVector3d u_abs(1, 0, 0);  // absolute x direction
        ChVector3d v_abs(0, 1, 0);  // absolute y direction
        ChVector3d w_abs(0, 0, 1);  // absolute z direction

        // Body orientation matrix
        auto R = ChMatrix33<>(ChQuaterniond(y.segment(3, 4)));

        // Jacobian of spherical constraints
        {
            auto Phi_pi = R * ChStarMatrix33<>(frame_loc.GetPos());

            J(0, 0) = 1;
            J(0, 1) = 0;
            J(0, 2) = 0;
            J(0, 3) = -Phi_pi(0, 0);
            J(0, 4) = -Phi_pi(0, 1);
            J(0, 5) = -Phi_pi(0, 2);

            J(1, 0) = 0;
            J(1, 1) = 1;
            J(1, 2) = 0;
            J(1, 3) = -Phi_pi(1, 0);
            J(1, 4) = -Phi_pi(1, 1);
            J(1, 5) = -Phi_pi(1, 2);

            J(2, 0) = 0;
            J(2, 1) = 0;
            J(2, 2) = 1;
            J(2, 3) = -Phi_pi(2, 0);
            J(2, 4) = -Phi_pi(2, 1);
            J(2, 5) = -Phi_pi(2, 2);
        }

        // Jacobian of x-z dot constraint
        {
            auto w = frame_abs.GetRotMat().GetAxisZ();
            auto w_tilde = ChStarMatrix33<>(w);
            ChMatrix33<> mat = R * w_tilde;
            ChVector3d Phi_pi = mat.transpose() * u_abs;

            J(3, 0) = 0;
            J(3, 1) = 0;
            J(3, 2) = 0;
            J(3, 3) = -Phi_pi.x();
            J(3, 4) = -Phi_pi.y();
            J(3, 5) = -Phi_pi.z();
        }

        // Jacobian of y-z dot constraint
        {
            auto w = frame_abs.GetRotMat().GetAxisZ();
            auto w_tilde = ChStarMatrix33<>(w);
            ChMatrix33<> mat = R * w_tilde;
            ChVector3d Phi_pi = mat.transpose() * v_abs;

            J(4, 0) = 0;
            J(4, 1) = 0;
            J(4, 2) = 0;
            J(4, 3) = -Phi_pi.x();
            J(4, 4) = -Phi_pi.y();
            J(4, 5) = -Phi_pi.z();
        }
    }

    virtual void IncrementState(const ChVectorDynamic<>& x,
                                const ChVectorDynamic<>& Dv,
                                ChVectorDynamic<>& x_new) override {
        x_new(0) = x(0) + Dv(0);
        x_new(1) = x(1) + Dv(1);
        x_new(2) = x(2) + Dv(2);

        ChQuaternion<> q_old(x.segment(3, 4));
        ChQuaternion<> rel_q;
        rel_q.SetFromRotVec(Dv.segment(3, 3));
        ChQuaternion<> q_new = q_old * rel_q;
        x_new.segment(3, 4) = q_new.eigen();
    }

    virtual void CalculateStateIncrement(const ChVectorDynamic<>& x,
                                         const ChVectorDynamic<>& x_new,
                                         ChVectorDynamic<>& Dv) override {
        Dv(0) = x_new(0) - x(0);
        Dv(1) = x_new(0) - x(0);
        Dv(2) = x_new(0) - x(0);

        ChQuaternion<> q_old(x.segment(3, 4));
        ChQuaternion<> q_new(x_new.segment(3, 4));
        ChQuaternion<> rel_q = q_old.GetConjugate() * q_new;
        Dv.segment(3, 3) = rel_q.GetRotVec().eigen();
    }

  private:
    double g;             // gravitational acceleration
    double L;             // pendulum half-length
    double m;             // pendulum mass
    ChVector3d I;         // pendulum moments of inertia
    ChFrame<> frame_loc;  // pendulum joint frame (in absolute coordinates)
    ChFrame<> frame_abs;  // pendulum joint frame (in absolute coordinates)
};

// -----------------------------------------------------------------------------

void Test_pendulum2D_ODE(double L, double m, double t_end, double t_step, const std::string& out_dir) {
    cout << "2D pendulum modeled as an ODE with 1 state and 0 constraints." << endl;

    ChSystemSMC sys;

    auto pendulum = chrono_types::make_shared<Pendulum2D_ODE>(L, m);
    pendulum->Initialize();
    sys.Add(pendulum);

    double t = 0;
    ChVectorDynamic<> y(1);
    ChVectorDynamic<> yd(1);

    ChWriterCSV csv(" ");
    y = pendulum->GetInitialStates();
    yd = pendulum->GetInitialStateDerivatives();
    csv << t << " " << L * std::cos(y(0)) << " " << L * std::sin(y(0)) << y(0) << " ";
    csv << -L * std::sin(y(0)) * yd(0) << " " << L * std::cos(y(0)) * yd(0) << " " << yd(0) << endl;

    while (t < t_end) {
        sys.DoStepDynamics(t_step);
        y = pendulum->GetStates();
        yd = pendulum->GetStateDerivatives();
        csv << t << " " << L * std::cos(y(0)) << " " << L * std::sin(y(0)) << y(0) << " ";
        csv << -L * std::sin(y(0)) * yd(0) << " " << L * std::cos(y(0)) * yd(0) << " " << yd(0) << endl;
        t += t_step;
    }

    std::string out_file = out_dir + "/pendulum2D_ODE.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/pendulum2D_ODE.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time");
    gplot.SetLabelY("");
    gplot.SetTitle("Pendulum 2D (ODE)");
    gplot.Plot(out_file, 1, 2, "x", " with lines lt rgb '#FF5500' lw 2");
    gplot.Plot(out_file, 1, 3, "y", " with lines lt rgb '#0055FF' lw 2");
    gplot.Plot(out_file, 1, 4, "theta", " with lines lt rgb '#55FF00' lw 2");
#endif
}

void Test_pendulum2D_DAE(double L, double m, double t_end, double t_step, const std::string& out_dir) {
    cout << "2D pendulum modeled as a DAE with 3 states and 2 constraints." << endl;

    ChSystemSMC sys;

    auto pendulum = chrono_types::make_shared<Pendulum2D_DAE>(L, m);
    pendulum->Initialize();
    sys.Add(pendulum);

    double t = 0;
    ChVectorDynamic<> y(3);
    ChVectorDynamic<> yd(3);

    Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
    ChWriterCSV csv(" ");
    y = pendulum->GetInitialStates();
    yd = pendulum->GetInitialStateDerivatives();
    csv << t << y.format(rowFmt) << yd.format(rowFmt) << endl;

    while (t < t_end) {
        sys.DoStepDynamics(t_step);
        y = pendulum->GetStates();
        yd = pendulum->GetStateDerivatives();
        csv << t << y.format(rowFmt) << yd.format(rowFmt) << endl;
        t += t_step;

        ////cout << "--------- " << t << "  " << std::sqrt(y(0) * y(0) + y(1) * y(1)) << endl;
    }

    std::string out_file = out_dir + "/pendulum2D_DAE.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/pendulum2D_DAE.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time");
    gplot.SetLabelY("");
    gplot.SetTitle("Pendulum 2D (DAE)");
    gplot.Plot(out_file, 1, 2, "x", " with lines lt rgb '#FF5500' lw 2");
    gplot.Plot(out_file, 1, 3, "y", " with lines lt rgb '#0055FF' lw 2");
    gplot.Plot(out_file, 1, 4, "theta", " with lines lt rgb '#55FF00' lw 2");
#endif
}

void Test_pendulum3D_DAE(double L, double m, double t_end, double t_step, const std::string& out_dir) {
    cout << "3D pendulum modeled as a DAE with 7/6 states and 5 constraints." << endl;

    ChSystemSMC sys;

    auto pendulum = chrono_types::make_shared<Pendulum3D_DAE>(L, m);
    pendulum->Initialize();
    sys.Add(pendulum);

    double t = 0;
    ChVectorDynamic<> y(7);
    ChVectorDynamic<> yd(6);

    Eigen::IOFormat rowFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "  ", "  ", "", "", "", "");
    ChWriterCSV csv(" ");
    y = pendulum->GetInitialStates();
    yd = pendulum->GetInitialStateDerivatives();
    csv << t << y.format(rowFmt) << yd.format(rowFmt) << endl;

    while (t < t_end) {
        sys.DoStepDynamics(t_step);
        y = pendulum->GetStates();
        yd = pendulum->GetStateDerivatives();
        csv << t << y.format(rowFmt) << yd.format(rowFmt) << endl;
        t += t_step;

        ////cout << "--------- " << t << "  " << std::sqrt(y(0) * y(0) + y(1) * y(1)) << endl;
    }

    std::string out_file = out_dir + "/pendulum3D_DAE.out";
    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/pendulum3D_DAE.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time");
    gplot.SetLabelY("");
    gplot.SetTitle("Pendulum 3D (DAE)");
    gplot.Plot(out_file, 1, 2, "x", " with lines lt rgb '#FF5500' lw 2");
    gplot.Plot(out_file, 1, 3, "y", " with lines lt rgb '#0055FF' lw 2");
    gplot.Plot(out_file, 1, 4, "z", " with lines lt rgb '#55FF00' lw 2");
#endif
}

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Create output directory
    std::string out_dir = GetChronoOutputPath() + "DEMO_EXTERNAL_DYNAMICS_DAE";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    double L = 1;  // pendulum half-length
    double m = 1;  // pendulum mass

    double t_end = 5;
    double t_step = 1e-2;

    Test_pendulum2D_ODE(L, m, t_end, t_step, out_dir);
    Test_pendulum2D_DAE(L, m, t_end, t_step, out_dir);
    Test_pendulum3D_DAE(L, m, t_end, t_step, out_dir);
}
