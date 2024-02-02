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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHTIMESTEPPER_H
#define CHTIMESTEPPER_H

#include <cstdlib>
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/timestepper/ChIntegrable.h"
#include "chrono/timestepper/ChState.h"

namespace chrono {

/// @addtogroup chrono_timestepper
/// @{

/// Base class for timesteppers, i.e., time integrators that can advance a system state.
/// It operates on systems inherited from ChIntegrable.
class ChApi ChTimestepper {
  public:
    /// Available methods for time integration (time steppers).
    enum class Type {
        EULER_IMPLICIT_LINEARIZED = 0,
        EULER_IMPLICIT_PROJECTED = 1,
        EULER_IMPLICIT = 2,
        TRAPEZOIDAL = 3,
        TRAPEZOIDAL_LINEARIZED = 4,
        HHT = 5,
        HEUN = 6,
        RUNGEKUTTA45 = 7,
        EULER_EXPLICIT = 8,
        LEAPFROG = 9,
        NEWMARK = 10,
        CUSTOM = 20
    };

    /// Constructor
    ChTimestepper(ChIntegrable* intgr = nullptr)
        : integrable(intgr), T(0), verbose(false), Qc_do_clamp(false), Qc_clamping(0) {}

    /// Destructor
    virtual ~ChTimestepper() {}

    /// Return type of the integration method.
    /// Default is CUSTOM. Derived classes should override this function.
    virtual Type GetType() const { return Type::CUSTOM; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) = 0;

    /// Access the lagrangian multipliers, if any.
    virtual ChVectorDynamic<>& get_L() { return L; }

    /// Set the integrable object.
    virtual void SetIntegrable(ChIntegrable* intgr) { integrable = intgr; }

    /// Get the integrable object.
    ChIntegrable* GetIntegrable() { return integrable; }

    /// Get the current time.
    virtual double GetTime() const { return T; }

    /// Set the current time.
    virtual void SetTime(double mt) { T = mt; }

    /// Turn on/off logging of messages.
    void SetVerbose(bool verb) { verbose = verb; }

    /// Turn on/off clamping on the Qcterm.
    void SetQcDoClamp(bool dc) { Qc_do_clamp = dc; }

    /// Turn on/off clamping on the Qcterm.
    void SetQcClamping(double cl) { Qc_clamping = cl; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive);

  protected:
    ChIntegrable* integrable;
    double T;

    ChVectorDynamic<> L;

    bool verbose;

    bool Qc_do_clamp;
    double Qc_clamping;
};

/// Base class for 1st order timesteppers, that is a time integrator for a ChIntegrable.
class ChApi ChTimestepperIorder : public ChTimestepper {
  protected:
    ChState Y;
    ChStateDelta dYdt;

  public:
    /// Constructor
    ChTimestepperIorder(ChIntegrable* intgr = nullptr) : ChTimestepper(intgr) { SetIntegrable(intgr); }

    /// Destructor
    virtual ~ChTimestepperIorder() {}

    /// Access the state at current time
    virtual ChState& get_Y() { return Y; }

    /// Access the derivative of state at current time
    virtual ChStateDelta& get_dYdt() { return dYdt; }

    /// Set the integrable object
    virtual void SetIntegrable(ChIntegrable* intgr) {
        ChTimestepper::SetIntegrable(intgr);
        Y.setZero(1, intgr);
        dYdt.setZero(1, intgr);
    }
};

/// Base class for 2nd order timesteppers, i.e., a time integrator for a ChIntegrableIIorder.
/// A ChIntegrableIIorder is a special subclass of integrable objects that have a state comprised
/// of position and velocity y={x,v}, and state derivative dy/dt={v,a}, where a=acceleration.
class ChApi ChTimestepperIIorder : public ChTimestepper {
  protected:
    ChState X;
    ChStateDelta V;
    ChStateDelta A;

  public:
    /// Constructor
    ChTimestepperIIorder(ChIntegrableIIorder* intgr = nullptr) : ChTimestepper(intgr) { SetIntegrable(intgr); }

    /// Destructor
    virtual ~ChTimestepperIIorder() {}

    /// Access the state, position part, at current time
    virtual ChState& get_X() { return X; }

    /// Access the state, speed part, at current time
    virtual ChStateDelta& get_V() { return V; }

    /// Access the acceleration, at current time
    virtual ChStateDelta& get_A() { return A; }

    /// Set the integrable object
    virtual void SetIntegrable(ChIntegrableIIorder* intgr) {
        ChTimestepper::SetIntegrable(intgr);
        X.setZero(1, intgr);
        V.setZero(1, intgr);
        A.setZero(1, intgr);
    }

  private:
    using ChTimestepper::SetIntegrable;
};


/// Base properties for explicit solvers.
/// Such integrators might require solution of a nonlinear problem if constraints 
/// are added, otherwise they can use penalty in constraints and lumped masses to avoid the linear system.
/// Diagonal lumping is off by default.
/// Note that if you apply this 
class ChApi ChExplicitTimestepper {
  protected:
      ChLumpingParms* lumping_parameters;

  public:
      ChExplicitTimestepper() : lumping_parameters(nullptr) {}
      virtual ~ChExplicitTimestepper() { if (lumping_parameters) delete (lumping_parameters); }

    /// Turn on the diagonal lumping. This can achieve a large speedup because no linear system is needeed 
    /// to compute the derivative (i.e. acceleration in II order systems), but not all Chintegrable might 
    /// support the diagonal lumping. 
    /// If lumping not supported because ChIntegrable::LoadLumpedMass_Md() not implemented, throw exception. 
    /// If lumping introduces some approximation, you'll get nonzero in GetLumpingError().
    /// Optionally paramters: the stiffness penalty for constraints, and damping penalty for constraints.
    void SetDiagonalLumpingON(double Ck = 1000, double Cr = 0) { lumping_parameters = new ChLumpingParms(Ck,Cr); }

    /// Turn off the diagonal lumping (default is off)
    void SetDiagonalLumpingOFF() { if (lumping_parameters) delete (lumping_parameters); }

    /// Gets the diagonal lumping error done last time the integrator has been called
    double GetLumpingError() {
        if (this->lumping_parameters)
            return lumping_parameters->error;
        else
            return 0;
    }
    /// Resets the diagonal lumping error.
    void ResetLumpingError() {
        if (this->lumping_parameters)
            lumping_parameters->error = 0;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) {
        // version number
        archive.VersionWrite(1);
        // serialize all member data:
        archive << CHNVP(lumping_parameters);
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) {
        // version number
        /*int version =*/ archive.VersionRead();
        // stream in all member data:
        archive >> CHNVP(lumping_parameters);
    }
};

/// Base class for implicit solvers (double inheritance)
class ChApi ChImplicitTimestepper {};

/// Base properties for implicit solvers.
/// Such integrators require solution of a nonlinear problem, typically solved
/// using an iterative process, up to a desired tolerance. At each iteration,
/// a linear system must be solved.
class ChApi ChImplicitIterativeTimestepper : public ChImplicitTimestepper {
  protected:
    int maxiters;    ///< maximum number of iterations
    double reltol;   ///< relative tolerance
    double abstolS;  ///< absolute tolerance (states)
    double abstolL;  ///< absolute tolerance (Lagrange multipliers)

    int numiters;   ///< number of iterations
    int numsetups;  ///< number of calls to the solver's Setup function
    int numsolves;  ///< number of calls to the solver's Solve function

  public:
    ChImplicitIterativeTimestepper()
        : maxiters(6), reltol(1e-4), abstolS(1e-10), abstolL(1e-10), numiters(0), numsetups(0), numsolves(0) {}
    virtual ~ChImplicitIterativeTimestepper() {}

    /// Set the max number of iterations using the Newton Raphson procedure
    void SetMaxiters(int iters) { maxiters = iters; }
    /// Get the max number of iterations using the Newton Raphson procedure
    double GetMaxiters() { return maxiters; }

    /// Set the relative tolerance.
    /// This tolerance is optionally used by derived classes in the Newton-Raphson
    /// convergence test.
    void SetRelTolerance(double rel_tol) { reltol = rel_tol; }

    /// Set the absolute tolerances.
    /// These tolerances are optionally used by derived classes in the Newton-Raphson
    /// convergence test.  This version sets separate absolute tolerances for states
    /// and Lagrange multipliers.
    void SetAbsTolerances(double abs_tolS, double abs_tolL) {
        abstolS = abs_tolS;
        abstolL = abs_tolL;
    }

    /// Set the absolute tolerances.
    /// These tolerances are optionally used by derived classes in the Newton-Raphson
    /// convergence test.  This version sets equal absolute tolerances for states and
    /// Lagrange multipliers.
    void SetAbsTolerances(double abs_tol) {
        abstolS = abs_tol;
        abstolL = abs_tol;
    }

    /// Return the number of iterations.
    int GetNumIterations() const { return numiters; }

    /// Return the number of calls to the solver's Setup function.
    int GetNumSetupCalls() const { return numsetups; }

    /// Return the number of calls to the solver's Solve function.
    int GetNumSolveCalls() const { return numsolves; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) {
        // version number
        archive.VersionWrite(1);
        // serialize all member data:
        archive << CHNVP(maxiters);
        archive << CHNVP(reltol);
        archive << CHNVP(abstolS);
        archive << CHNVP(abstolL);
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) {
        // version number
        /*int version =*/ archive.VersionRead();
        // stream in all member data:
        archive >> CHNVP(maxiters);
        archive >> CHNVP(reltol);
        archive >> CHNVP(abstolS);
        archive >> CHNVP(abstolL);
    }
};

/// Euler explicit timestepper.
/// This performs the typical  y_new = y+ dy/dt * dt integration with Euler formula.
class ChApi ChTimestepperEulerExpl : public ChTimestepperIorder, public ChExplicitTimestepper {
  public:
    /// Constructors (default empty)
    ChTimestepperEulerExpl(ChIntegrable* intgr = nullptr) : ChTimestepperIorder(intgr) {}

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
    ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Euler explicit timestepper customized for II order.
/// (It gives the same results of ChTimestepperEulerExpl, but performs a bit faster because it
/// can exploit the special structure of ChIntegrableIIorder)
/// This integrator implements the typical Euler scheme:
///    x_new = x + v * dt
///    v_new = v + a * dt
class ChApi ChTimestepperEulerExplIIorder : public ChTimestepperIIorder, public ChExplicitTimestepper {
  protected:
    ChStateDelta Dv;

  public:
    /// Constructors (default empty)
    ChTimestepperEulerExplIIorder(ChIntegrableIIorder* intgr = nullptr) : ChTimestepperIIorder(intgr) {}

    virtual Type GetType() const override { return Type::EULER_EXPLICIT; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Euler semi-implicit timestepper.
/// This performs the typical
///    v_new = v + a * dt
///    x_new = x + v_new * dt
/// integration with Euler semi-implicit formula.
class ChApi ChTimestepperEulerSemiImplicit : public ChTimestepperIIorder, public ChExplicitTimestepper {
  public:
    /// Constructors (default empty)
    ChTimestepperEulerSemiImplicit(ChIntegrableIIorder* intgr = nullptr) : ChTimestepperIIorder(intgr) {}

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
    ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of a 4th order explicit Runge-Kutta integration scheme.
class ChApi ChTimestepperRungeKuttaExpl : public ChTimestepperIorder, public ChExplicitTimestepper {
  protected:
    ChState y_new;
    ChStateDelta Dydt1;
    ChStateDelta Dydt2;
    ChStateDelta Dydt3;
    ChStateDelta Dydt4;

  public:
    /// Constructors (default empty)
    ChTimestepperRungeKuttaExpl(ChIntegrable* intgr = nullptr) : ChTimestepperIorder(intgr) {}

    virtual Type GetType() const override { return Type::RUNGEKUTTA45; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of a Heun explicit integrator. It is like a 2nd Runge Kutta.
class ChApi ChTimestepperHeun : public ChTimestepperIorder, public ChExplicitTimestepper {
  protected:
    ChState y_new;
    ChStateDelta Dydt1;
    ChStateDelta Dydt2;

  public:
    /// Constructors (default empty)
    ChTimestepperHeun(ChIntegrable* intgr = nullptr) : ChTimestepperIorder(intgr) {}

    virtual Type GetType() const override { return Type::HEUN; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of a Leapfrog explicit integrator.
/// This is a symplectic method, with 2nd order accuracy, at least when F depends on positions only.
/// Note: uses last step acceleration: changing or resorting  the numbering of DOFs will invalidate it.
/// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives the same accuracy with better performance.
class ChApi ChTimestepperLeapfrog : public ChTimestepperIIorder, public ChExplicitTimestepper {
  protected:
    ChStateDelta Aold;

  public:
    /// Constructors (default empty)
    ChTimestepperLeapfrog(ChIntegrableIIorder* intgr = nullptr) : ChTimestepperIIorder(intgr) {}

    virtual Type GetType() const override { return Type::LEAPFROG; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of Euler implicit for II order systems.
class ChApi ChTimestepperEulerImplicit : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
  protected:
    ChStateDelta Dv;
    ChVectorDynamic<> Dl;
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperEulerImplicit(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChImplicitIterativeTimestepper() {}

    virtual Type GetType() const override { return Type::EULER_IMPLICIT; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of Euler implicit for II order systems using the Anitescu/Stewart/Trinkle
/// single-iteration method, that is a bit like an implicit Euler where one performs only the
/// first Newton corrector iteration.
/// If using an underlying CCP complementarity solver, this is the typical Anitescu stabilized
/// timestepper for DVIs.
class ChApi ChTimestepperEulerImplicitLinearized : public ChTimestepperIIorder, public ChImplicitTimestepper {
  protected:
    ChStateDelta Vold;
    ChVectorDynamic<> Dl;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperEulerImplicitLinearized(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChImplicitTimestepper() {}

    virtual Type GetType() const override { return Type::EULER_IMPLICIT_LINEARIZED; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of Euler implicit for II order systems using a semi implicit Euler without
/// constraint stabilization, followed by a projection. That is: a speed problem followed by a
/// position problem that keeps constraint drifting 'closed' by using a projection.
/// If using an underlying CCP complementarity solver, this is the typical Tasora stabilized
/// timestepper for DVIs.
class ChApi ChTimestepperEulerImplicitProjected : public ChTimestepperIIorder, public ChImplicitTimestepper {
  protected:
    ChStateDelta Vold;
    ChVectorDynamic<> Dl;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperEulerImplicitProjected(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChImplicitTimestepper() {}

    virtual Type GetType() const override { return Type::EULER_IMPLICIT_PROJECTED; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of trapezoidal implicit for II order systems.
/// NOTE this is a modified version of the trapezoidal for DAE: the original derivation would lead
/// to a scheme that produces oscillatory reactions in constraints, so this is a modified version
/// that is first order in constraint reactions. Use damped HHT or damped Newmark for more advanced options.
class ChApi ChTimestepperTrapezoidal : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
  protected:
    ChStateDelta Dv;
    ChVectorDynamic<> Dl;
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Rold;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperTrapezoidal(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChImplicitIterativeTimestepper() {}

    virtual Type GetType() const override { return Type::TRAPEZOIDAL; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of trapezoidal implicit linearized for II order systems.
class ChApi ChTimestepperTrapezoidalLinearized : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
  protected:
    ChStateDelta Dv;
    ChVectorDynamic<> Dl;
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Rold;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperTrapezoidalLinearized(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChImplicitIterativeTimestepper() {}

    virtual Type GetType() const override { return Type::TRAPEZOIDAL_LINEARIZED; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of trapezoidal implicit linearized for II order systems.
///*** SIMPLIFIED VERSION -DOES NOT WORK - PREFER ChTimestepperTrapezoidalLinearized
class ChApi ChTimestepperTrapezoidalLinearized2 : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
  protected:
    ChStateDelta Dv;
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperTrapezoidalLinearized2(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChImplicitIterativeTimestepper() {}

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
    ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// Performs a step of Newmark constrained implicit for II order DAE systems.
/// See Negrut et al. 2007.
class ChApi ChTimestepperNewmark : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
  private:
    double gamma;
    double beta;
    ChStateDelta Da;
    ChVectorDynamic<> Dl;
    ChState Xnew;
    ChStateDelta Vnew;
    ChStateDelta Anew;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Rold;
    ChVectorDynamic<> Qc;
    bool modified_Newton;

  public:
    /// Constructors (default empty)
    ChTimestepperNewmark(ChIntegrableIIorder* intgr = nullptr)
        : ChTimestepperIIorder(intgr), ChImplicitIterativeTimestepper() {
        SetGammaBeta(0.6, 0.3);  // default values with some damping, and that works also with DAE constraints
        modified_Newton = true; // default use modified Newton with jacobian factorization only at beginning
    }

    virtual Type GetType() const override { return Type::NEWMARK; }

    /// Set the numerical damping parameter gamma and the beta parameter.
    /// Gamma: in the [1/2, 1] interval.
    /// For gamma = 1/2, no numerical damping
    /// For gamma > 1/2, more damping
    /// Beta: in the [0, 1] interval.
    /// For beta = 1/4, gamma = 1/2 -> constant acceleration method
    /// For beta = 1/6, gamma = 1/2 -> linear acceleration method
    /// Method is second order accurate only for gamma = 1/2
    void SetGammaBeta(double mgamma, double mbeta);

    double GetGamma() { return gamma; }

    double GetBeta() { return beta; }

    /// Enable/disable modified Newton.
    /// If enabled, the Newton matrix is evaluated, assembled, and factorized only once per step.
    /// If disabled, the Newton matrix is evaluated at every iteration of the nonlinear solver.
    /// Modified Newton iteration is enabled by default.
    void SetModifiedNewton(bool val) { modified_Newton = val; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;
};

/// @} chrono_timestepper

}  // end namespace chrono

#endif
