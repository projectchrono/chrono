//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHTIMESTEPPER_H
#define CHTIMESTEPPER_H

#include <stdlib.h>
#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "core/ChVectorDynamic.h"
#include "timestepper/ChState.h"
#include "timestepper/ChIntegrable.h"
#include "serialization/ChArchive.h"

namespace chrono {

/// @addtogroup chrono_timestepper
/// @{

/// Base class for timesteppers, that is
/// a time integrator which can advance a system state.
/// It operates on systems inherited from ChIntegrable.
class ChApi ChTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChTimestepper);

  protected:
    ChIntegrable* integrable;
    double T;

    ChVectorDynamic<> L;

    bool verbose;

    bool Qc_do_clamp;
    double Qc_clamping;

  public:
    /// Constructor
    ChTimestepper(ChIntegrable* mintegrable =0) {
        integrable = mintegrable;
        T = 0;
        L.Reset(0);
        verbose = false;
        Qc_do_clamp = false;
        Qc_clamping = 1e30;
    };

    /// Destructor
    virtual ~ChTimestepper(){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         ) = 0;

    /// Access the lagrangian multipliers, if any
    virtual ChVectorDynamic<>& get_L() { return L; }

    /// Set the integrable object
    virtual void SetIntegrable(ChIntegrable* mintegrable) { integrable = mintegrable; }

    /// Get the integrable object
    ChIntegrable* GetIntegrable() { return integrable; }

    /// Get the current time
    virtual double GetTime() { return T; }

    /// Set the current time
    virtual void SetTime(double mt) { T = mt; }

    /// Turn on/off logging of messages
    void SetVerbose(bool mverbose) { verbose = mverbose; }

    /// Turn on/off clamping on the Qcterm
    void SetQcDoClamp(bool mdc) { Qc_do_clamp = mdc; }

    /// Turn on/off clamping on the Qcterm
    void SetQcClamping(double mcl) { Qc_clamping = mcl; }

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite(1);
        // serialize all member data:
        marchive << CHNVP(verbose);
        marchive << CHNVP(Qc_do_clamp);
        marchive << CHNVP(Qc_clamping);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead();
        // stream in all member data:
        marchive >> CHNVP(verbose);
        marchive >> CHNVP(Qc_do_clamp);
        marchive >> CHNVP(Qc_clamping);
    }

};

/// Base class for 1st order timesteppers, that is
/// a time integrator for whatever ChIntegrable.
class ChApi ChTimestepperIorder : public ChTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperIorder, ChTimestepper);

  protected:
    ChState Y;
    ChStateDelta dYdt;

  public:
    /// Constructor
    ChTimestepperIorder(ChIntegrable* mintegrable =0) : ChTimestepper(mintegrable) {
        SetIntegrable(mintegrable);
    };

    /// Destructor
    virtual ~ChTimestepperIorder(){};

    /// Access the state at current time
    virtual ChState& get_Y() { return Y; }

    /// Access the derivative of state at current time
    virtual ChStateDelta& get_dYdt() { return dYdt; }

    /// Set the integrable object
    virtual void SetIntegrable(ChIntegrable* mintegrable) { 
            ChTimestepper::SetIntegrable(mintegrable);
            Y.Reset(1, mintegrable);
            dYdt.Reset(1, mintegrable);
    }
};

/// Base class for 2nd order timesteppers, that is
/// a time integrator for whatever ChIntegrableIIorder
/// (special sub lass of integrable objects that have a state
/// made with position and velocity y={x,v}, and dy/dt={v,a}
/// with a=acceleration)
class ChApi ChTimestepperIIorder : public ChTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperIIorder, ChTimestepper);

  protected:
    ChState X;
    ChStateDelta V;
    ChStateDelta A;

  public:
    /// Constructor
    ChTimestepperIIorder(ChIntegrableIIorder* mintegrable =0) : ChTimestepper(mintegrable) {
        SetIntegrable(mintegrable);
    };

    /// Destructor
    virtual ~ChTimestepperIIorder(){};

    /// Access the state, position part, at current time
    virtual ChState& get_X() { return X; }

    /// Access the state, speed part, at current time
    virtual ChStateDelta& get_V() { return V; }

    /// Access the acceleration, at current time
    virtual ChStateDelta& get_A() { return A; }

    /// Set the integrable object
    virtual void SetIntegrable(ChIntegrableIIorder* mintegrable) { 
            ChTimestepper::SetIntegrable(mintegrable);
            X.Reset(1, mintegrable);
            V.Reset(1, mintegrable);
            A.Reset(1, mintegrable);
    }
};

/// Base properties for implicit solvers (double inheritance)
class ChApi ChImplicitTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChImplicitTimestepper);

};

/// Base properties for implicit solvers that compute the solution by iterative
/// process up to a desired tolerance
class ChApi ChImplicitIterativeTimestepper : public ChImplicitTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChImplicitIterativeTimestepper, ChImplicitTimestepper);

  protected:
    int maxiters;
    double reltol;   // relative tolerance
    double abstolS;  // absolute tolerance (states)
    double abstolL;  // absolute tolerance (Lagrange multipliers)

  public:
    /// Constructors
    ChImplicitIterativeTimestepper() : maxiters(6), reltol(1e-4), abstolS(1e-10), abstolL(1e-10) {}

    /// Set the max number of iterations using the Newton Raphson procedure
    void SetMaxiters(int miters) { maxiters = miters; }
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

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite(1);
        // serialize all member data:
        marchive << CHNVP(maxiters);
        marchive << CHNVP(reltol);
        marchive << CHNVP(abstolS);
        marchive << CHNVP(abstolL);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead();
        // stream in all member data:
        marchive >> CHNVP(maxiters);
        marchive >> CHNVP(reltol);
        marchive >> CHNVP(abstolS);
        marchive >> CHNVP(abstolL);
    }
};

/// Euler explicit timestepper
/// This performs the typical  y_new = y+ dy/dt * dt
/// integration with Euler formula.
class ChApi ChTimestepperEulerExpl : public ChTimestepperIorder {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperEulerExpl, ChTimestepperIorder);

  public:
    /// Constructors (default empty)
    ChTimestepperEulerExpl(ChIntegrable* mintegrable =0) : ChTimestepperIorder(mintegrable){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Euler explicit timestepper customized for II order.
/// (It gives the same results of ChTimestepperEulerExpl,
/// but this performes a bit faster because it can exploit
/// the special structure of ChIntegrableIIorder)
/// This performs the typical
///    x_new = x + v * dt
///    v_new = v + a * dt
/// integration with Euler formula.
class ChApi ChTimestepperEulerExplIIorder : public ChTimestepperIIorder {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperEulerExplIIorder, ChTimestepperIIorder);

  protected:
    ChStateDelta Dv;

  public:
    /// Constructors (default empty)
    ChTimestepperEulerExplIIorder(ChIntegrableIIorder* mintegrable =0) : ChTimestepperIIorder(mintegrable){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Euler semi-implicit timestepper
/// This performs the typical
///    v_new = v + a * dt
///    x_new = x + v_new * dt
/// integration with Euler semi-implicit formula.
class ChApi ChTimestepperEulerSemiImplicit : public ChTimestepperIIorder {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperEulerSemiImplicit, ChTimestepperIIorder);

  public:
    /// Constructors (default empty)
    ChTimestepperEulerSemiImplicit(ChIntegrableIIorder* mintegrable =0) : ChTimestepperIIorder(mintegrable){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of a 4th order explicit Runge-Kutta
/// integration scheme.
class ChApi ChTimestepperRungeKuttaExpl : public ChTimestepperIorder {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperRungeKuttaExpl, ChTimestepperIorder);

  protected:
    ChState y_new;
    ChStateDelta Dydt1;
    ChStateDelta Dydt2;
    ChStateDelta Dydt3;
    ChStateDelta Dydt4;

  public:
    /// Constructors (default empty)
    ChTimestepperRungeKuttaExpl(ChIntegrable* mintegrable =0) : ChTimestepperIorder(mintegrable){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of a Heun explicit integrator. It is like
/// a 2nd Runge Kutta.
class ChApi ChTimestepperHeun : public ChTimestepperIorder {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperHeun, ChTimestepperIorder);

  protected:
    ChState y_new;
    ChStateDelta Dydt1;
    ChStateDelta Dydt2;

  public:
    /// Constructors (default empty)
    ChTimestepperHeun(ChIntegrable* mintegrable =0) : ChTimestepperIorder(mintegrable){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of a Leapfrog explicit integrator.
/// It is a symplectic method, with 2nd order accuracy,
/// at least when F depends on positions only.
/// Note: uses last step acceleration: changing or resorting
/// the numbering of DOFs will invalidate it.
/// Suggestion: use the ChTimestepperEulerSemiImplicit, it gives
/// the same accuracy with a bit of faster performance.
class ChApi ChTimestepperLeapfrog : public ChTimestepperIIorder {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperLeapfrog, ChTimestepperIIorder);

  protected:
    ChStateDelta Aold;

  public:
    /// Constructors (default empty)
    ChTimestepperLeapfrog(ChIntegrableIIorder* mintegrable =0) : ChTimestepperIIorder(mintegrable){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of Euler implicit for II order systems
class ChApi ChTimestepperEulerImplicit : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperEulerImplicit, ChTimestepperIIorder);

  protected:
    ChStateDelta Dv;
    ChVectorDynamic<> Dl;
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperEulerImplicit(ChIntegrableIIorder* mintegrable =0)
        : ChTimestepperIIorder(mintegrable), ChImplicitIterativeTimestepper(){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of Euler implicit for II order systems
/// using the Anitescu/Stewart/Trinkle single-iteration method,
/// that is a bit like an implicit Euler where one performs only
/// the first NR corrector iteration.
/// If the solver in StateSolveCorrection is a CCP complementarity
/// solver, this is the typical Anitescu stabilized timestepper for DVIs.
class ChApi ChTimestepperEulerImplicitLinearized : public ChTimestepperIIorder, public ChImplicitTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperEulerImplicitLinearized, ChTimestepperIIorder);

  protected:
    ChStateDelta Vold;
    ChVectorDynamic<> Dl;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperEulerImplicitLinearized(ChIntegrableIIorder* mintegrable =0)
        : ChTimestepperIIorder(mintegrable), ChImplicitTimestepper(){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of Euler implicit for II order systems
/// using a semi implicit Euler without constr.stabilization, followed by a projection,
/// that is: a speed problem followed by a position problem that
/// keeps constraint drifting 'closed' by using a projection.
/// If the solver in StateSolveCorrection is a CCP complementarity
/// solver, this is the Tasora stabilized timestepper for DVIs.
class ChApi ChTimestepperEulerImplicitProjected : public ChTimestepperIIorder, public ChImplicitTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperEulerImplicitProjected, ChTimestepperIIorder);

  protected:
    ChStateDelta Vold;
    ChVectorDynamic<> Dl;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperEulerImplicitProjected(ChIntegrableIIorder* mintegrable =0)
        : ChTimestepperIIorder(mintegrable), ChImplicitTimestepper(){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of trapezoidal implicit for II order systems.
/// NOTE this is a modified version of the trapezoidal for DAE: the
/// original derivation would lead to a scheme that produces oscillatory
/// reactions in constraints, so this is a modified version that is first
/// order in constraint reactions. Use damped HHT or damped Newmark for
/// more advanced options.
class ChApi ChTimestepperTrapezoidal : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperTrapezoidal, ChTimestepperIIorder);

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
    ChTimestepperTrapezoidal(ChIntegrableIIorder* mintegrable =0)
        : ChTimestepperIIorder(mintegrable), ChImplicitIterativeTimestepper(){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of trapezoidal implicit linearized for II order systems
class ChApi ChTimestepperTrapezoidalLinearized : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperTrapezoidalLinearized, ChTimestepperIIorder);

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
    ChTimestepperTrapezoidalLinearized(ChIntegrableIIorder* mintegrable =0)
        : ChTimestepperIIorder(mintegrable), ChImplicitIterativeTimestepper(){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of trapezoidal implicit linearized for II order systems
///*** SIMPLIFIED VERSION -DOES NOT WORK - PREFER ChTimestepperTrapezoidalLinearized
class ChApi ChTimestepperTrapezoidalLinearized2 : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperTrapezoidalLinearized2, ChTimestepperIIorder);

  protected:
    ChStateDelta Dv;
    ChState Xnew;
    ChStateDelta Vnew;
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;

  public:
    /// Constructors (default empty)
    ChTimestepperTrapezoidalLinearized2(ChIntegrableIIorder* mintegrable =0)
        : ChTimestepperIIorder(mintegrable), ChImplicitIterativeTimestepper(){};

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );
};

/// Performs a step of HHT (generalized alpha) implicit for II order systems
/// See Negrut et al. 2007.
class ChApi ChTimestepperHHT : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperHHT, ChTimestepperIIorder);

  public:
    enum HHT_Mode {
        ACCELERATION,
        POSITION,
    };
    CH_ENUM_MAPPER_BEGIN(HHT_Mode);
      CH_ENUM_VAL(ACCELERATION);
      CH_ENUM_VAL(POSITION);
    CH_ENUM_MAPPER_END(HHT_Mode);

  private:
    double alpha;   // HHT method parameters:  -1/3 <= alpha <= 0
    double gamma;   //                         gamma = 1/2 - alpha
    double beta;    //                         beta = (1 - alpha)^2 / 4
    HHT_Mode mode;  // HHT formulation (ACCELERATION or POSITION)
    bool scaling;   // include scaling by beta * h * h (POSITION only)
    int num_it;     // total number of NR iterations over the last step

    ChStateDelta Da;         // state update
    ChStateDelta Dx;         // cummulative state updates (POSITION only)
    ChVectorDynamic<> Dl;    // Lagrange multiplier update
    ChState Xnew;            // current estimate of new positions
    ChState Xprev;           // previous estimate of new positions (POSITION only)
    ChStateDelta Vnew;       // current estimate of new velocities
    ChStateDelta Anew;       // current estimate of new accelerations
    ChVectorDynamic<> Lnew;  // current estimate of Lagrange multipliers
    ChVectorDynamic<> R;     // residual of nonlinear system (dynamics portion)
    ChVectorDynamic<> Rold;  // residual terms depending on previous state
    ChVectorDynamic<> Qc;    // residual of nonlinear system (constranints portion)

    bool step_control;            // step size control enabled?
    int maxiters_success;         // maximum number of NR iterations to declare a step successful
    int req_successful_steps;     // required number of successive successful steps for a stepsize increase
    double step_increase_factor;  // factor used in increasing stepsize (>1)
    double step_decrease_factor;  // factor used in decreasing stepsize (<1)
    double h_min;                 // minimum allowable stepsize
    double h;                     // internal stepsize
    int num_successful_steps;     // number of successful steps

    ChVectorDynamic<> ewtS;  // vector of error weights (states)
    ChVectorDynamic<> ewtL;  // vector of error weights (Lagrange multipliers)

  public:
    /// Constructors (default empty)
    ChTimestepperHHT(ChIntegrableIIorder* mintegrable = 0)
        : ChTimestepperIIorder(mintegrable),
          ChImplicitIterativeTimestepper(),
          mode(ACCELERATION),
          scaling(false),
          step_control(true),
          maxiters_success(3),
          req_successful_steps(5),
          step_increase_factor(2),
          step_decrease_factor(0.5),
          h_min(1e-10),
          h(1e6),
          num_successful_steps(0) {
        SetAlpha(-0.2);  // default: some dissipation
    };

    /// Set the numerical damping parameter.
    /// It must be in the [-1/3, 0] interval. The closer to -1/3, the more damping.
    /// The closer to 0, the less damping (for 0, it is the trapezoidal method).
    /// The method coefficients gamma and beta are set automatically, based on alpha.
    void SetAlpha(double malpha) {
        alpha = malpha;
        if (alpha < -1.0 / 3.0)
            alpha = -1.0 / 3.0;
        if (alpha > 0)
            alpha = 0;
        gamma = (1.0 - 2.0 * alpha) / 2.0;
        beta = pow((1.0 - alpha), 2) / 4.0;
    }

    /// Return the current value of the method parameter alpha.
    double GetAlpha() { return alpha; }

    /// Set the HHT formulation.
    void SetMode(HHT_Mode mmode) { mode = mmode; }

    /// Turn scaling on/off.
    void SetScaling(bool mscaling) { scaling = mscaling; }

    /// Turn step size control on/off.
    void SetStepControl(bool val) { step_control = val; }

    /// Set the minimum step size.
    /// An exception is thrown if the internal step size decreases below this limit.
    void SetMinStepSize(double min_step) { h_min = min_step; }

    /// Set the maximum allowable number of iterations for counting a step towards a stepsize increase.
    void SetMaxItersSuccess(int iters) { maxiters_success = iters; }

    /// Set the minimum number of (internal) steps that require at most maxiters_success
    /// before considering a stepsize increase.
    void SetRequiredSuccessfulSteps(int num_steps) { req_successful_steps = num_steps; }

    /// Set the multiplicative factor for a stepsize increase.
    /// Must be a value larger than 1.
    void SetStepIncreaseFactor(double factor) { step_increase_factor = factor; }
    
    /// Set the multiplicative factor for a stepsize decrease.
    /// Must be a value smaller than 1.
    void SetStepDecreaseFactor(double factor) { step_decrease_factor = factor; }

    /// Return the number of iterations over the last step.
    /// Note that this is a cummulative iteration count, over all internal steps.
    int GetNumIterations() const { return num_it; }

    /// Perform an integration timestep.
    virtual void Advance(const double dt  ///< timestep to advance
                         );

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class:
        ChTimestepperIIorder::ArchiveOUT(marchive);
        ChImplicitIterativeTimestepper::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(alpha);
        marchive << CHNVP(beta);
        marchive << CHNVP(gamma);
        marchive << CHNVP(scaling);
        marchive << CHNVP(num_it);
        HHT_Mode_mapper modemapper;
        marchive << CHNVP(modemapper(mode),"mode");
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class:
        ChTimestepperIIorder::ArchiveIN(marchive);
        ChImplicitIterativeTimestepper::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(alpha);
        marchive >> CHNVP(beta);
        marchive >> CHNVP(gamma);
        marchive >> CHNVP(scaling);
        marchive >> CHNVP(num_it);
        HHT_Mode_mapper modemapper;
        marchive >> CHNVP(modemapper(mode),"mode");
    }

  private:
      void Prepare(ChIntegrableIIorder* integrable, double scaling_factor);
      void Increment(ChIntegrableIIorder* integrable, double scaling_factor);
      bool CheckConvergence(double scaling_factor);
      void CalcErrorWeights(const ChVectorDynamic<>& x, double rtol, double atol, ChVectorDynamic<>& ewt);
};

/// Performs a step of Newmark constrained implicit for II order DAE systems
/// See Negrut et al. 2007.
class ChApi ChTimestepperNewmark : public ChTimestepperIIorder, public ChImplicitIterativeTimestepper {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChTimestepperNewmark, ChTimestepperIIorder);

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

  public:
    /// Constructors (default empty)
    ChTimestepperNewmark(ChIntegrableIIorder* mintegrable =0)
        : ChTimestepperIIorder(mintegrable), ChImplicitIterativeTimestepper() {
        SetGammaBeta(0.6, 0.3);  // default values with some damping, and that works also with DAE constraints
    };

    /// Set the numerical damping parameter gamma and the beta parameter.
    /// Gamma: in the [1/2, 1] interval.
    /// For gamma = 1/2, no numerical damping
    /// For gamma > 1/2, more damping
    /// Beta: in the [0, 1] interval.
    /// For beta = 1/4, gamma = 1/2 -> constant acceleration method
    /// For beta = 1/6, gamma = 1/2 -> linear acceleration method
    /// Method is second order accurate only for gamma = 1/2
    void SetGammaBeta(double mgamma, double mbeta) {
        gamma = mgamma;
        if (gamma < 0.5)
            gamma = 0.5;
        if (gamma > 1)
            gamma = 1;
        beta = mbeta;
        if (beta < 0)
            beta = 0;
        if (beta > 1)
            beta = 1;
    }

    double GetGamma() { return gamma; }

    double GetBeta() { return beta; }

    /// Performs an integration timestep
    virtual void Advance(const double dt  ///< timestep to advance
                         );

        // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class:
        ChTimestepperIIorder::ArchiveOUT(marchive);
        ChImplicitIterativeTimestepper::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(beta);
        marchive << CHNVP(gamma);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class:
        ChTimestepperIIorder::ArchiveIN(marchive);
        ChImplicitIterativeTimestepper::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(beta);
        marchive >> CHNVP(gamma);
    }
};

/// @} chrono_timestepper

}  // END_OF_NAMESPACE____

#endif
