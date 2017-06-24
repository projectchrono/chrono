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

#ifndef CHSTATICANALYSIS_H
#define CHSTATICANALYSIS_H

#include <cstdlib>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/timestepper/ChIntegrable.h"

namespace chrono {

/// Base class for static analysis

class ChStaticAnalysis {
  protected:
    ChIntegrableIIorder* integrable;

    ChState X;
    ChStateDelta V;
    ChStateDelta A;
    ChVectorDynamic<> L;

  public:
    /// Constructor
    ChStaticAnalysis(ChIntegrableIIorder& mintegrable) {
        integrable = &mintegrable;
        L.Reset(0);
        X.Reset(1, &mintegrable);
        V.Reset(1, &mintegrable);
        A.Reset(1, &mintegrable);
    };

    /// Destructor
    virtual ~ChStaticAnalysis(){};

    /// Performs the static analysis
    virtual void StaticAnalysis() = 0;

    /// Access the lagrangian multipliers, if any
    virtual ChVectorDynamic<>& get_L() { return L; }

    /// Get the integrable object
    ChIntegrable* GetIntegrable() { return integrable; }

    /// Access the state, position part, at current analysis
    virtual ChState& get_X() { return X; }

    /// Access the state, speed part, at current analysis
    virtual ChStateDelta& get_V() { return V; }

    /// Access the acceleration, at current analysis
    virtual ChStateDelta& get_A() { return A; }
};

/// Linear static analysis

class ChStaticLinearAnalysis : public ChStaticAnalysis {
  protected:
  public:
    /// Constructor
    ChStaticLinearAnalysis(ChIntegrableIIorder& mintegrable) : ChStaticAnalysis(mintegrable){};

    /// Destructor
    virtual ~ChStaticLinearAnalysis(){};

    /// Performs the static analysis,
    /// doing a linear solve.

    virtual void StaticAnalysis() {
        ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

        // setup main vectors
        mintegrable->StateSetup(X, V, A);

        ChStateDelta Dx;
        ChVectorDynamic<> R;
        ChVectorDynamic<> Qc;
        double T;

        // setup auxiliary vectors
        Dx.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
        R.Reset(mintegrable->GetNcoords_v());
        Qc.Reset(mintegrable->GetNconstr());
        L.Reset(mintegrable->GetNconstr());

        mintegrable->StateGather(X, V, T);  // state <- system

        // Set V speed to zero
        V.FillElem(0);
        mintegrable->StateScatter(X, V, T);  // state -> system

        // Solve:
        //
        // [-dF/dx     Cq' ] [ dx  ] = [ f]
        // [ Cq        0   ] [  l  ] = [ C]

        mintegrable->LoadResidual_F(R, 1.0);
        mintegrable->LoadConstraint_C(Qc, 1.0);

        mintegrable->StateSolveCorrection(
            Dx, L, R, Qc,
            0,        // factor for  M
            0,        // factor for  dF/dv
            -1.0,     // factor for  dF/dx (the stiffness matrix)
            X, V, T,  // not needed here
            false,    // do not StateScatter update to Xnew Vnew T+dt before computing correction
            true      // force a call to the solver's Setup() function
            );

        X += Dx;

        mintegrable->StateScatter(X, V, T);  // state -> system
        mintegrable->StateScatterReactions(L);  // -> system auxiliary data 
    }
};

/// Non-Linear static analysis

class ChStaticNonLinearAnalysis : public ChStaticAnalysis {
  protected:
    int maxiters;
    double tolerance;
    int incremental_steps;

  public:
    /// Constructor
    ChStaticNonLinearAnalysis(ChIntegrableIIorder& mintegrable)
        : ChStaticAnalysis(mintegrable), maxiters(20), incremental_steps(6), tolerance(1e-10){};

    /// Destructor
    virtual ~ChStaticNonLinearAnalysis(){};

    /// Performs the static analysis,
    /// doing a linear solve.

    virtual void StaticAnalysis() {
        ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

        // setup main vectors
        mintegrable->StateSetup(X, V, A);

        ChState Xnew;
        ChStateDelta Dx;
        ChVectorDynamic<> R;
        ChVectorDynamic<> Qc;
        double T;

        // setup auxiliary vectors
        Dx.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
        Xnew.Reset(mintegrable->GetNcoords_x(), mintegrable);
        R.Reset(mintegrable->GetNcoords_v());
        Qc.Reset(mintegrable->GetNconstr());
        L.Reset(mintegrable->GetNconstr());

        mintegrable->StateGather(X, V, T);  // state <- system

        // Set speed to zero
        V.FillElem(0);

        // Extrapolate a prediction as warm start
        Xnew = X;

        // use Newton Raphson iteration to solve implicit Euler for v_new
        //
        // [ - dF/dx    Cq' ] [ Dx  ] = [ f ]
        // [ Cq         0   ] [ L   ] = [ C ]

        for (int i = 0; i < this->GetMaxiters(); ++i) {
            mintegrable->StateScatter(Xnew, V, T);  // state -> system
            R.Reset();
            Qc.Reset();
            mintegrable->LoadResidual_F(R, 1.0);
            mintegrable->LoadConstraint_C(Qc, 1.0);

            double cfactor = ChMin(1.0, ((double)(i + 2) / (double)(incremental_steps + 1)));
            R *= cfactor;
            Qc *= cfactor;

            //	GetLog()<< "Non-linear statics iteration=" << i << "  |R|=" << R.NormInf() << "  |Qc|=" << Qc.NormInf()
            //<< "\n";
            if ((R.NormInf() < this->GetTolerance()) && (Qc.NormInf() < this->GetTolerance()))
                break;

            mintegrable->StateSolveCorrection(
                Dx, L, R, Qc,
                0,           // factor for  M
                0,           // factor for  dF/dv
                -1.0,        // factor for  dF/dx (the stiffness matrix)
                Xnew, V, T,  // not needed here
                false,       // do not StateScatter update to Xnew Vnew T+dt before computing correction
                true         // force a call to the solver's Setup() function
                );

            Xnew += Dx;
        }

        X = Xnew;

        mintegrable->StateScatter(X, V, T);  // state -> system
        mintegrable->StateScatterReactions(L);  // -> system auxiliary data 
    }

    /// Set the max number of iterations using the Newton Raphson procedure
    void SetMaxiters(int miters) {
        maxiters = miters;
        if (incremental_steps > miters)
            incremental_steps = miters;
    }
    /// Get the max number of iterations using the Newton Raphson procedure
    double GetMaxiters() { return maxiters; }

    /// Set the number of steps that, for the first iterations, make the residual
    /// growing linearly. If =0, no incremental application of residual, so it is
    /// a classic Newton Raphson iteration, otherwise acts as  continuation strategy.
    /// For values > 0 , it might help convergence. Must be less than maxiters.
    void SetIncrementalSteps(int mist) {
        incremental_steps = mist;
        if (maxiters < incremental_steps)
            maxiters = incremental_steps;
    }
    /// Set the number of steps that, for the first iterations, make the residual
    /// growing linearly.
    double GetIncrementalSteps() { return incremental_steps; }

    /// Set the tolerance for terminating the Newton Raphson procedure
    void SetTolerance(double mtol) { tolerance = mtol; }
    /// Get the tolerance for terminating the Newton Raphson procedure
    double GetTolerance() { return tolerance; }
};

}  // end namespace chrono

#endif
