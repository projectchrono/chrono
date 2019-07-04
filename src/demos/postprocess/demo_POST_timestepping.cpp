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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo on low-level functionality for time integration of differential equations.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChLog.h"
#include "chrono/physics/ChGlobal.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace postprocess;

void example1(const std::string& out_dir) {
    GetLog() << " Example 1: integrate dx/dt=e^t \n";

    // Define a class inherited from ChIntegrable,
    // it will represent the differential equations
    // by implementing the StateSolve() function, and few other interfaces:
    class MyIntegrable : public ChIntegrable {
      private:
      public:
        MyIntegrable() {}

        /// the number of coordinates in the state:
        virtual int GetNcoords_y() override { return 1; }

        /// compute  dy/dt=f(y,t)
        virtual bool StateSolve(ChStateDelta& dydt,              ///< result: computed dy/dt
                                ChVectorDynamic<>& L,            ///< result: computed lagrangian multipliers, if any
                                const ChState& y,                ///< current state y
                                const double T,                  ///< current time T
                                const double dt,                 ///< timestep (if needed)
                                bool force_state_scatter = true  ///< if false, y and T are not scattered to the system
                                ) override {
            if (force_state_scatter)
                StateScatter(y, T);  // state -> system   (not needed here, btw.)

            dydt(0) = exp(T);  // dx/dt=e^t

            return true;
        }
    };

    // File to dump results
    std::string logfile = out_dir + "/log_timestepper_1.dat";
    ChStreamOutAsciiFile log_file1(logfile.c_str());

    // Create and object from your custom integrable class:
    MyIntegrable mintegrable;

    // Create a time-integrator, class: Euler explicit
    ChTimestepperEulerExpl mystepper(&mintegrable);

    // Execute the time integration
    while (mystepper.GetTime() < 4) {
        mystepper.Advance(0.1);

        double exact_solution = exp(mystepper.GetTime()) - 1;
        GetLog() << " T = " << mystepper.GetTime() << "  x=" << mystepper.get_Y()(0) << "  x_exact=" << exact_solution
                 << "\n";
        log_file1 << mystepper.GetTime() << ", " << mystepper.get_Y()(0) << ", " << exact_solution << "\n";
    }

    // Plot results
    std::string gplfile = out_dir + "/tmp_timestepping_1.gpl";
    ChGnuPlot mplot(gplfile.c_str());
    mplot.SetGrid();
    mplot.SetTitle("Integrate dx/dt=e^t ");
    mplot.SetLabelX("t");
    mplot.SetLabelY("x");
    mplot.Plot(logfile.c_str(), 1, 2, "Euler explicit", " with lines lt -1 lw 2");
    mplot.Plot(logfile.c_str(), 1, 3, "Exact, analytical", " with lines lt 2 lw 2");
}

void example2(const std::string& out_dir) {
    GetLog() << "\n\n Example 2: integrate 2nd order oscillator: M*ddx/dtdt + R*dx/dt + K*w = F(t) with a 1st "
                "order integrator. \n";

    // Define a class inherited from ChIntegrable,
    // it will represent the differential equations
    // by implementing the StateSolve() function, and few other interfaces:
    class MyIntegrable : public ChIntegrable {
      private:
        double M;
        double K;
        double R;
        double T;
        double x;
        double v;

      public:
        MyIntegrable() {
            T = 0;
            M = 10;
            K = 30;
            R = 1;
            x = 0;
            v = 0;
        }

        /// the number of coordinates in the state:
        virtual int GetNcoords_y() override { return 2; }

        /// system -> state
        virtual void StateGather(ChState& y, double& mT) override {
            y(0) = x;
            y(1) = v;
            mT = T;
        };

        /// state -> system
        virtual void StateScatter(const ChState& y, const double mT) override {
            x = y(0);
            v = y(1);
            T = mT;
        };

        /// compute  dy/dt=f(y,t)
        virtual bool StateSolve(ChStateDelta& dydt,              ///< result: computed dy/dt
                                ChVectorDynamic<>& L,            ///< result: computed lagrangian multipliers, if any
                                const ChState& y,                ///< current state y
                                const double T,                  ///< current time T
                                const double dt,                 ///< timestep (if needed)
                                bool force_state_scatter = true  ///< if false, y and T are not scattered to the system
                                ) override {
            if (force_state_scatter)
                StateScatter(y, T);

            double F = cos(T * 20) * 2;

            dydt(0) = v;                               // speed
            dydt(1) = (1. / M) * (F - K * x - R * v);  // acceleration

            return true;
        }
    };

    // File to dump results
    std::string logfile = out_dir + "/log_timestepper_2.dat";
    ChStreamOutAsciiFile log_file2(logfile.c_str());

    // Try integrator Euler explicit

    // Create and object from your custom integrable class:
    MyIntegrable mintegrable;
    // Create a time-integrator:
    ChTimestepperEulerExpl mystepper(&mintegrable);

    // Try integrator Runge Kutta 4st  explicit

    // Create and object from your custom integrable class:
    MyIntegrable mintegrable_rk;
    // Create a time-integrator, class: Runge Kutta 4 explicit
    ChTimestepperRungeKuttaExpl mystepper_rk(&mintegrable_rk);

    // Execute the time integration
    while (mystepper.GetTime() < 1) {
        mystepper.Advance(0.01);
        mystepper_rk.Advance(0.01);

        GetLog() << " T = " << mystepper.GetTime() << "  x=" << mystepper.get_Y()(0) << "  v=" << mystepper.get_Y()(1)
                 << "\n";
        log_file2 << mystepper.GetTime() << ", " << mystepper.get_Y()(0) << ", " << mystepper.get_Y()(1) << ", "
                  << mystepper_rk.get_Y()(0) << ", " << mystepper_rk.get_Y()(1) << "\n";
    }

    // Plot results
    std::string gplfile = out_dir + "/tmp_timestepping_2.gpl";
    ChGnuPlot mplot(gplfile.c_str());
    mplot.SetGrid();
    mplot.SetTitle("Integrate 2nd order oscillator with 1st order timestepper");
    mplot.SetLabelX("t");
    mplot.SetLabelY("x, v");
    mplot.Plot(logfile.c_str(), 1, 2, "Euler exp. x", " with lines");
    mplot.Plot(logfile.c_str(), 1, 3, "Euler exp. v", " with lines");
    mplot.Plot(logfile.c_str(), 1, 4, "RungeKutta x", " with lines");
    mplot.Plot(logfile.c_str(), 1, 5, "RungeKutta v", " with lines");
}

void example3(const std::string& out_dir) {
    GetLog() << "\n\n Example 3: integrate 2nd order oscillator: M*ddx/dtdt + R*dx/dt + K*w = F(t) with a 2nd "
                "order integrator. \n";

    // Define a class inherited from ChIntegrableIIorder,
    // it will represent the differential equations
    // by implementing the StateSolveA() function, and few other interfaces.
    // Compared to previous example 2, this is inherited from ChIntegrableIIorder,
    // so one can use more advanced integrators such as ChTimestepperEulerSemiImplicit,
    // that can exploit the II order nature of the problem. Anyway, also all I order
    // integrators can still be used.
    class MyIntegrable : public ChIntegrableIIorder {
      private:
        double M;
        double K;
        double R;
        double T;
        double mx;
        double mv;

      public:
        MyIntegrable() {
            T = 0;
            M = 10;
            K = 30;
            R = 1;
            mx = 0;
            mv = 0;
        }

        /// the number of coordinates in the state, x position part:
        virtual int GetNcoords_x() override { return 1; }

        /// system -> state
        virtual void StateGather(ChState& x, ChStateDelta& v, double& mT) override {
            x(0) = mx;
            v(0) = mv;
            mT = T;
        };

        /// state -> system
        virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double mT) override {
            mx = x(0);
            mv = v(0);
            T = mT;
        };

        /// compute  dy/dt=f(y,t)
        virtual bool StateSolveA(ChStateDelta& dvdt,              ///< result: computed accel. a = dv/dt
                                 ChVectorDynamic<>& L,            ///< result: computed lagrangian multipliers, if any
                                 const ChState& x,                ///< current state, x
                                 const ChStateDelta& v,           ///< current state, v
                                 const double T,                  ///< current time T
                                 const double dt,                 ///< timestep (if needed)
                                 bool force_state_scatter = true  ///< if false, y and T are not scattered to the system
                                 ) override {
            if (force_state_scatter)
                StateScatter(x, v, T);

            double F = cos(T * 5) * 2;
            dvdt(0) = (1. / M) * (F - K * mx - R * mv);

            return true;
        }
    };

    // Create a file to dump results
    std::string logfile = out_dir + "/log_timestepper_3.dat";
    ChStreamOutAsciiFile log_file3(logfile.c_str());

    // Create and object from your custom integrable class:
    MyIntegrable mintegrable1;
    MyIntegrable mintegrable2;
    MyIntegrable mintegrable3;

    // Create few time-integrators to be compared:
    ChTimestepperRungeKuttaExpl mystepper1(&mintegrable1);
    ChTimestepperEulerExplIIorder mystepper2(&mintegrable2);
    ChTimestepperEulerSemiImplicit mystepper3(&mintegrable3);

    // Execute the time integration
    while (mystepper1.GetTime() < 4) {
        mystepper1.Advance(0.01);
        mystepper2.Advance(0.01);
        mystepper3.Advance(0.01);

        GetLog() << "T = " << mystepper1.GetTime() << "  x=" << mystepper1.get_Y()(0) << "  v=" << mystepper1.get_Y()(1)
                 << "\n";
        log_file3 << mystepper1.GetTime() << ", " << mystepper1.get_Y()(0) << ", " << mystepper1.get_Y()(1) << ", "
                  << mystepper2.get_X()(0) << ", " << mystepper2.get_V()(0) << ", " << mystepper3.get_X()(0) << ", "
                  << mystepper3.get_V()(0) << "\n";
    }

    // Plot results
    std::string gplfile = out_dir + "/tmp_timestepping_3.gpl";
    ChGnuPlot mplot(gplfile.c_str());
    mplot.SetGrid();
    mplot.SetTitle("Integrate 2nd order oscillator with 2nd order timestepper");
    mplot.SetLabelX("t");
    mplot.SetLabelY("x");
    mplot.Plot(logfile.c_str(), 1, 2, "RungeKutta", " with lines");
    mplot.Plot(logfile.c_str(), 1, 4, "Euler exp. IIorder", " with lines");
    mplot.Plot(logfile.c_str(), 1, 6, "Euler semi-implicit", " with lines");
}

void example4(const std::string& out_dir) {
    GetLog() << "\n\n Example 4: integrate 2nd order oscillator: M*ddx/dtdt + R*dx/dt + K*w = F(t) with an "
                "implicit integrator \n";

    // Define a class inherited from ChIntegrableIIorder,
    // it will represent the differential equations
    // by implementing the interfaces to implicit solvers.
    //  We assume   M*a = F(x,v,t)

    class MyIntegrable : public ChIntegrableIIorder {
      private:
        double M;
        double K;
        double R;
        double mT;
        double mx;
        double mv;

      public:
        MyIntegrable() {
            mT = 0;
            M = 1;
            K = 30;
            R = 0;
            mx = 0;
            mv = 0.6;
        }

        /// the number of coordinates in the state, x position part:
        virtual int GetNcoords_x() override { return 1; }

        /// system -> state
        virtual void StateGather(ChState& x, ChStateDelta& v, double& T) override {
            x(0) = mx;
            v(0) = mv;
            T = mT;
        };

        /// state -> system
        virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T) override {
            mx = x(0);
            mv = v(0);
            mT = T;
        };

        /// compute  dy/dt=f(y,t)
        virtual bool StateSolveA(ChStateDelta& dvdt,              ///< result: computed accel. a=dv/dt
                                 ChVectorDynamic<>& L,            ///< result: computed lagrangian multipliers, if any
                                 const ChState& x,                ///< current state, x
                                 const ChStateDelta& v,           ///< current state, v
                                 const double T,                  ///< current time T
                                 const double dt,                 ///< timestep (if needed)
                                 bool force_state_scatter = true  ///< if false, y and T are not scattered to the system
                                 ) override {
            if (force_state_scatter)
                StateScatter(x, v, T);
            double F = sin(mT * 20) * 0.02;
            dvdt(0) = (1. / M) * (F - K * mx - R * mv);

            return true;
        }

        /// Compute the correction with linear system
        ///  Dv = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]^-1 * R
        virtual bool StateSolveCorrection(
            ChStateDelta& Dv,                 ///< result: computed Dv
            ChVectorDynamic<>& L,             ///< result: computed lagrangian multipliers, if any
            const ChVectorDynamic<>& R,       ///< the R residual
            const ChVectorDynamic<>& Qc,      ///< the Qc residual
            const double c_a,                 ///< the factor in c_a*M
            const double c_v,                 ///< the factor in c_v*dF/dv
            const double c_x,                 ///< the factor in c_x*dF/dv
            const ChState& x,                 ///< current state, x part
            const ChStateDelta& v,            ///< current state, v part
            const double T,                   ///< current time T
            bool force_state_scatter = true,  ///< if false, x,v and T are not scattered to the system
            bool force_setup = true           ///< if true, call the solver's Setup() function
            ) override {
            if (force_state_scatter)
                this->StateScatter(x, v, T);

            Dv(0) = R(0) * 1.0 / (c_a * this->M + c_v * (-this->R) + c_x * (-this->K));

            return true;
        }

        ///    R += c*F
        void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                            const double c         ///< a scaling factor
                            ) override {
            R(0) += c * (sin(mT * 20) * 0.02 - this->K * mx - this->R * mv);
        };

        ///    R += c*M*w
        void LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                             const ChVectorDynamic<>& w,  ///< the w vector
                             const double c               ///< a scaling factor
                             ) override {
            R(0) += c * this->M * w(0);
        };

        /// nothing to do here- no constraints
        virtual void LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                      const ChVectorDynamic<>& L,  ///< the L vector
                                      const double c               ///< a scaling factor
                                      ) override {}

        /// nothing to do here- no constraints
        virtual void LoadConstraint_C(ChVectorDynamic<>& Qc,        ///< result: the Qc residual, Qc += c*C
                                      const double c,               ///< a scaling factor
                                      const bool do_clamp = false,  ///< enable optional clamping of Qc
                                      const double mclam = 1e30     ///< clamping value
                                      ) override {}

        /// nothing to do here- no constraints
        virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                       const double c          ///< a scaling factor
                                       ) override {}
    };

    // Create a file to dump results
    std::string logfile = out_dir + "/log_timestepper_4.dat";
    ChStreamOutAsciiFile log_file4(logfile.c_str());

    // Create and object from your custom integrable class:
    MyIntegrable mintegrable1;
    MyIntegrable mintegrable2;
    MyIntegrable mintegrable3;
    MyIntegrable mintegrable4;
    MyIntegrable mintegrable5;
    MyIntegrable mintegrable6;
    MyIntegrable mintegrable7;
    MyIntegrable mintegrable8;

    // Create few time-integrators to be compared:
    ChTimestepperEulerImplicit mystepper1(&mintegrable1);
    ChTimestepperTrapezoidal mystepper2(&mintegrable2);
    ChTimestepperEulerExplIIorder mystepper3(&mintegrable3);
    ChTimestepperHHT mystepper4(&mintegrable4);
    mystepper4.SetAlpha(0);  // HHT with no dissipation -> trapezoidal
    ChTimestepperHHT mystepper5(&mintegrable5);
    mystepper5.SetAlpha(-0.33);  // HHT with max dissipation
    ChTimestepperNewmark mystepper6(&mintegrable6);
    mystepper6.SetGammaBeta(0.5, 0.25);  // Newmark as const accel. method
    ChTimestepperNewmark mystepper7(&mintegrable7);
    mystepper7.SetGammaBeta(0.5, 1 / 6);  // Newmark as linear accel. method
    ChTimestepperNewmark mystepper8(&mintegrable8);
    mystepper8.SetGammaBeta(1.0, 0.25);  // Newmark with max numerical damping

    // Execute the time integration
    while (mystepper1.GetTime() < 4) {
        double timestep = 0.05;
        mystepper1.Advance(timestep);
        mystepper2.Advance(timestep);
        mystepper3.Advance(timestep);
        mystepper4.Advance(timestep);
        mystepper5.Advance(timestep);
        mystepper6.Advance(timestep);
        mystepper7.Advance(timestep);
        mystepper8.Advance(timestep);

        GetLog() << "T = " << mystepper1.GetTime() << "  x=" << mystepper1.get_X()(0) << "  v=" << mystepper1.get_V()(0)
                 << "\n";
        log_file4 << mystepper1.GetTime() << ", " << mystepper1.get_X()(0) << ", " << mystepper1.get_V()(0) << ", "
                  << mystepper2.get_X()(0) << ", " << mystepper2.get_V()(0) << ", " << mystepper3.get_X()(0) << ", "
                  << mystepper3.get_V()(0) << ", " << mystepper4.get_X()(0) << ", " << mystepper4.get_V()(0) << ", "
                  << mystepper5.get_X()(0) << ", " << mystepper5.get_V()(0) << ", " << mystepper6.get_X()(0) << ", "
                  << mystepper6.get_V()(0) << ", " << mystepper7.get_X()(0) << ", " << mystepper7.get_V()(0) << ", "
                  << mystepper8.get_X()(0) << ", " << mystepper8.get_V()(0) << "\n";
    }

    // Plot results
    std::string gplfile = out_dir + "/tmp_timestepping_4.gpl";
    ChGnuPlot mplot(gplfile.c_str());
    mplot.SetGrid();
    mplot.SetTitle("Test: oscillator with implicit integrators");
    mplot.SetLabelX("t");
    mplot.SetLabelY("x");
    mplot.Plot(logfile.c_str(), 1, 2, "Euler implicit", " with lines");
    mplot.Plot(logfile.c_str(), 1, 4, "Trapezoidal", " with lines");
    mplot.Plot(logfile.c_str(), 1, 6, "Euler expl.IIorder", " with lines");
    mplot.Plot(logfile.c_str(), 1, 8, "HHT alpha=0", " with lines");
    mplot.Plot(logfile.c_str(), 1, 10, "HHT alpha=-0.33", " with lines");
    mplot.Plot(logfile.c_str(), 1, 12, "Newmark g=0.5, b=1/4", " with lines");
    mplot.Plot(logfile.c_str(), 1, 14, "Newmark g=0.5, b=1/6", " with lines");
    mplot.Plot(logfile.c_str(), 1, 16, "Newmark g=1.0, b=1/4", " with lines");
}

void example5(const std::string& out_dir) {
    GetLog() << "\n\n Example 5: integrate pendulum DAE \n";

    // Define a class inherited from ChIntegrableIIorder,
    // it will represent the differential equations
    // by implementing the interfaces to implicit solvers.
    //  We assume   M*a = F(x,v,t)
    //            C(x,t)=0;

    class MyIntegrable : public ChIntegrableIIorder {
      private:
        double M;
        double K;
        double R;
        double mT;
        double mpx;
        double mpy;
        double mvx;
        double mvy;
        double mlength;
        double mreaction;

      public:
        MyIntegrable() {
            mlength = 1;
            M = 1;
            K = 2;
            R = 0;
            mT = 0;
            mpx = 0;
            mpy = -mlength;
            mvx = 0.2;
            mvy = 0;
            mreaction = 0;
        }

        /// the number of coordinates in the state, x position part:
        virtual int GetNcoords_x() override { return 2; }

        /// Tells the number of lagrangian multipliers (constraints)
        virtual int GetNconstr() override { return 1; }

        /// system -> state
        virtual void StateGather(ChState& x, ChStateDelta& v, double& T) override {
            x(0) = mpx;
            x(1) = mpy;
            v(0) = mvx;
            v(1) = mvy;
            T = mT;
        };

        virtual void StateGatherAcceleration(ChStateDelta& a) override {
            a(0) = 0;
            a(1) = 0;
        }

        /// state -> system
        virtual void StateScatter(const ChState& x, const ChStateDelta& v, const double T) override {
            mpx = x(0);
            mpy = x(1);
            mvx = v(0);
            mvy = v(1);
            mT = T;
        };

        /// Some timesteppers exploit persistence of reaction information
        virtual void StateGatherReactions(ChVectorDynamic<>& L) override { L(0) = mreaction; };
        virtual void StateScatterReactions(const ChVectorDynamic<>& L) override { mreaction = L(0); };

        /// Compute the correction with linear system
        ///  Dv = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]^-1 * R
        virtual bool StateSolveCorrection(
            ChStateDelta& Dv,                 ///< result: computed Dv
            ChVectorDynamic<>& L,             ///< result: computed lagrangian multipliers, if any
            const ChVectorDynamic<>& R,       ///< the R residual
            const ChVectorDynamic<>& Qc,      ///< the Qc residual
            const double c_a,                 ///< the factor in c_a*M
            const double c_v,                 ///< the factor in c_v*dF/dv
            const double c_x,                 ///< the factor in c_x*dF/dv
            const ChState& x,                 ///< current state, x part
            const ChStateDelta& v,            ///< current state, v part
            const double T,                   ///< current time T
            bool force_state_scatter = true,  ///< if false, x,v and T are not scattered to the system
            bool force_setup = true           ///< if true, call the solver's Setup() function
            ) override {
            if (force_state_scatter)
                this->StateScatter(x, v, T);

            ChVector<> dirpend(-mpx, -mpy, 0);
            dirpend.Normalize();
            ChVectorDynamic<> b(3);
            b(0) = R(0);
            b(1) = R(1);
            b(2) = Qc(0);
            ChMatrixDynamic<> A(3, 3);
            A.Reset();
            A(0, 0) = c_a * this->M + c_v * (-this->R) + c_x * (-this->K);
            A(1, 1) = c_a * this->M;
            A(0, 2) = dirpend.x();
            A(1, 2) = dirpend.y();
            A(2, 0) = dirpend.x();
            A(2, 1) = dirpend.y();
            ChVectorDynamic<> w(3);
            ChLinearAlgebra::Solve_LinSys(A, &b, &w);
            Dv(0) = w(0);
            Dv(1) = w(1);
            L(0) = -w(2);  // note assume result sign in multiplier is flipped

            return true;
        }

        ///    R += c*F
        void LoadResidual_F(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
                            const double c         ///< a scaling factor
                            ) override {
            R(0) += c * (sin(mT * 20) * 0.000 - this->K * mpx - this->R * mvx);
            R(1) += c * -5;  // vertical force
        };

        ///    R += c*M*w
        void LoadResidual_Mv(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                             const ChVectorDynamic<>& w,  ///< the w vector
                             const double c               ///< a scaling factor
                             ) override {
            R(0) += c * this->M * w(0);
            R(1) += c * this->M * w(1);
        };

        ///   R += Cq'*l
        virtual void LoadResidual_CqL(ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                      const ChVectorDynamic<>& L,  ///< the L vector
                                      const double c               ///< a scaling factor
                                      ) override {
            ChVector<> dirpend(-mpx, -mpy, 0);
            dirpend.Normalize();
            R(0) += c * dirpend.x() * L(0);
            R(1) += c * dirpend.y() * L(0);
        };

        ///  Qc += c * C
        virtual void LoadConstraint_C(ChVectorDynamic<>& Qc,        ///< result: the Qc residual, Qc += c*C
                                      const double c,               ///< a scaling factor
                                      const bool do_clamp = false,  ///< enable optional clamping of Qc
                                      const double mclam = 1e30     ///< clamping value
                                      ) override {
            ChVector<> distpend(-mpx, -mpy, 0);
            Qc(0) += -c * (-distpend.Length() + mlength);
        };

        /// nothing to do here- no rheonomic part
        virtual void LoadConstraint_Ct(ChVectorDynamic<>& Qc,  ///< result: the Qc residual, Qc += c*Ct
                                       const double c          ///< a scaling factor
                                       ) override {}
    };

    // Create a file to dump results
    std::string logfile5 = out_dir + "/log_timestepper_5.dat";
    ChStreamOutAsciiFile log_file5(logfile5.c_str());
    std::string logfile5r = out_dir + "/log_timestepper_5r.dat";
    ChStreamOutAsciiFile log_file5r(logfile5r.c_str());

    // Create and object from your custom integrable class:
    MyIntegrable mintegrable1;
    MyIntegrable mintegrable2;
    MyIntegrable mintegrable3;
    MyIntegrable mintegrable4;
    MyIntegrable mintegrable5;
    MyIntegrable mintegrable6;
    MyIntegrable mintegrable7;

    // Create few time-integrators to be compared:
    ChTimestepperEulerImplicitLinearized mystepper1(&mintegrable1);
    ChTimestepperEulerImplicit mystepper2(&mintegrable2);
    ChTimestepperTrapezoidal mystepper3(&mintegrable3);
    ChTimestepperHHT mystepper4(&mintegrable4);
    mystepper4.SetAlpha(0);  // HHT with no dissipation -> trapezoidal
    ChTimestepperHHT mystepper5(&mintegrable5);
    mystepper5.SetAlpha(-0.2);  // HHT with dissipation
    ChTimestepperNewmark mystepper6(&mintegrable6);
    mystepper6.SetGammaBeta(0.6, 0.3);  // Newmark

    // Execute the time integration
    while (mystepper1.GetTime() < 12) {
        mystepper1.Advance(0.05);
        mystepper2.Advance(0.05);
        mystepper3.Advance(0.05);
        mystepper4.Advance(0.05);
        mystepper5.Advance(0.05);
        mystepper6.Advance(0.05);

        GetLog() << "T = " << mystepper1.GetTime() << "  x=" << mystepper1.get_X()(0) << "  y=" << mystepper1.get_X()(1)
                 << "\n";
        log_file5 << mystepper1.GetTime() << ", " << mystepper1.get_X()(0) << ", " << mystepper1.get_X()(1) << ", "
                  << mystepper1.get_V()(0) << ", " << mystepper1.get_V()(1) << ", " << mystepper2.get_X()(0) << ", "
                  << mystepper2.get_X()(1) << ", " << mystepper2.get_V()(0) << ", " << mystepper2.get_V()(1) << ", "
                  << mystepper3.get_X()(0) << ", " << mystepper3.get_X()(1) << ", " << mystepper3.get_V()(0) << ", "
                  << mystepper3.get_V()(1) << ", " << mystepper4.get_X()(0) << ", " << mystepper4.get_X()(1) << ", "
                  << mystepper4.get_V()(0) << ", " << mystepper4.get_V()(1) << ", " << mystepper5.get_X()(0) << ", "
                  << mystepper5.get_X()(1) << ", " << mystepper5.get_V()(0) << ", " << mystepper5.get_V()(1) << ", "
                  << mystepper6.get_X()(0) << ", " << mystepper6.get_X()(1) << ", " << mystepper6.get_V()(0) << ", "
                  << mystepper6.get_V()(1) << "\n";
        log_file5r << mystepper1.GetTime() << ", " << mystepper1.get_L()(0) << ", " << mystepper2.get_L()(0) << ", "
                   << mystepper3.get_L()(0) << ", " << mystepper4.get_L()(0) << ", " << mystepper5.get_L()(0) << ", "
                   << mystepper6.get_L()(0) << "\n";
    }

    std::string gplfile = out_dir + "/tmp_timestepping_5.gpl";
    ChGnuPlot mplot(gplfile.c_str());
    mplot.OutputWindow(0);
    mplot.SetGrid();
    mplot.SetTitle("Test: DAE, constrained pendulum");
    mplot.SetLabelX("t");
    mplot.SetLabelY("x");
    mplot.Plot(logfile5.c_str(), 1, 2, "Euler impl. lineariz.", " with lines");
    mplot.Plot(logfile5.c_str(), 1, 6, "Euler impl.", " with lines");
    mplot.Plot(logfile5.c_str(), 1, 10, "Trapezoidal*", " with lines");
    mplot.Plot(logfile5.c_str(), 1, 14, "HHT alpha=0", " with lines");
    mplot.Plot(logfile5.c_str(), 1, 18, "HHT alpha=-0.2", " with lines");
    mplot.Plot(logfile5.c_str(), 1, 22, "Newmark g=0.6,b=0.3", " with lines");

    mplot.OutputWindow(1);
    mplot.SetGrid();
    mplot.SetTitle("Test: DAE, constrained pendulum reactions");
    mplot.SetLabelX("t [s]");
    mplot.SetLabelY("R [N]");
    mplot.Plot(logfile5r.c_str(), 1, 2, "Euler impl. lineariz.", " with lines");
    mplot.Plot(logfile5r.c_str(), 1, 3, "Euler impl.", " with lines");
    mplot.Plot(logfile5r.c_str(), 1, 4, "Trapezoidal*", " with lines");
    mplot.Plot(logfile5r.c_str(), 1, 5, "HHT alpha=0", " with lines");
    mplot.Plot(logfile5r.c_str(), 1, 6, "HHT alpha=-0.2", " with lines");
    mplot.Plot(logfile5r.c_str(), 1, 7, "Newmark g=0.6,b=0.3", " with lines");

    mplot.OutputWindow(2);
    mplot.SetGrid();
    mplot.SetTitle("Test: DAE, constrained pendulum trajectory");
    mplot.SetLabelX("x");
    mplot.SetLabelY("y");
    mplot.SetRangeX(-0.15, 0.15);
    mplot.SetRangeY(-1.025, -0.95);
    mplot.SetCommand("set size ratio 0.5");
    mplot.Plot(logfile5.c_str(), 2, 3, "Euler impl. lineariz.", " pt 0");
    mplot.Plot(logfile5.c_str(), 6, 7, "Euler impl.", " pt 1");
    mplot.Plot(logfile5.c_str(), 10, 11, "Trapezoidal*", " pt 2");
    mplot.Plot(logfile5.c_str(), 14, 15, "HHT alpha=0", " pt 3");
    mplot.Plot(logfile5.c_str(), 18, 19, "HHT alpha=-0.2", " pt 4");
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    GetLog() << "CHRONO demo about low-level time integration of differential equations: \n\n";

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_TIMESTEPPER";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    example1(out_dir);
    example2(out_dir);
    example3(out_dir);
    example4(out_dir);
    example5(out_dir);
}
