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

#ifndef CHASSEMBLYANALYSIS_H
#define CHASSEMBLYANALYSIS_H

#include <stdlib.h>
#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "core/ChVectorDynamic.h"
#include "timestepper/ChState.h"
#include "timestepper/ChIntegrable.h"

namespace chrono {


//////////////////////////////////////
// Define flags for "action" of
// AssemblyAnalysis()  function

#define ASS_POSITION (1L << 0)
#define ASS_SPEED (1L << 1)
#define ASS_ACCEL (1L << 2)

// define other flags for "flags"
// argument of AssemblyAnalysis() function
#define ASF_COLLISIONS (1L << 6)



/// Base class for assembly analysis
/// It assemblies the parts by satisfying constraints at a position level (using a non-linear 
/// solver) and at the speed level, and at acceleration level too. 

class ChAssemblyAnalysis {
  protected:
    ChIntegrableIIorder* integrable;

    ChState X;
    ChStateDelta V;
    ChStateDelta A;
    ChVectorDynamic<> L;
    int max_assembly_iters;

  public:
    /// Constructor
    ChAssemblyAnalysis(ChIntegrableIIorder& mintegrable) {
        integrable = &mintegrable;
        L.Reset(0);
        X.Reset(1, &mintegrable);
        V.Reset(1, &mintegrable);
        A.Reset(1, &mintegrable);
        max_assembly_iters = 4;
    };

    /// Destructor
    virtual ~ChAssemblyAnalysis(){};

    /// Performs the assembly analysis.
    /// It assemblies the parts by satisfying constraints at a position level (using a non-linear 
    /// solver) and at the speed level, and at acceleration level too. 

    virtual void AssemblyAnalysis(int action,  int mflags) {
        ChIntegrableIIorder* mintegrable = (ChIntegrableIIorder*)this->integrable;

        ChStateDelta Dx;
        ChVectorDynamic<> R;
        ChVectorDynamic<> Qc;
        double T;

        // setup main vectors
        mintegrable->StateSetup(X, V, A);

        if (action & ASS_POSITION) {
            for (int m_iter = 0; m_iter < max_assembly_iters; m_iter++) {
                
                // setup auxiliary vectors
                Dx.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
                R.Reset(mintegrable->GetNcoords_v());
                Qc.Reset(mintegrable->GetNconstr());
                L.Reset(mintegrable->GetNconstr());

                mintegrable->StateGather(X, V, T);  // state <- system

                // Solve:
                //
                // [M          Cq' ] [ dx  ] = [ 0]
                // [ Cq        0   ] [  l  ] = [ C]

                mintegrable->LoadConstraint_C(Qc, 1.0);

                mintegrable->StateSolveCorrection(
                    Dx, L, R, Qc,
                    1.0,      // factor for  M
                    0,        // factor for  dF/dv
                    0,        // factor for  dF/dx (the stiffness matrix)
                    X, V, T,  // not needed
                    false     // do not StateScatter update to Xnew Vnew T+dt before computing correction
                    );

                X += Dx;

                mintegrable->StateScatter(X, V, T);  // state -> system
            }
        }

        if ((action & ASS_SPEED) || (action & ASS_ACCEL)) {
            ChStateDelta Vold;

            // setup auxiliary vectors
            Vold.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
            Dx.Reset(mintegrable->GetNcoords_v(), GetIntegrable());
            R.Reset(mintegrable->GetNcoords_v());
            Qc.Reset(mintegrable->GetNconstr());
            L.Reset(mintegrable->GetNconstr());

            mintegrable->StateGather(X, V, T);  // state <- system

            Vold = V;

            // As a small Anitescu/Trinkle time step:
            //
            // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
            // [ Cq                           0   ] [ -dt*l  ] = [ C/dt + Ct ]

            double mdt = 1e-7;
            double mclamping = 1e-5; // or better as zero to disable stbilization

            mintegrable->LoadResidual_F(R, mdt);
            mintegrable->LoadResidual_Mv(R, V, 1.0);
            mintegrable->LoadConstraint_C(Qc, 1.0 / mdt, true, mclamping);
            mintegrable->LoadConstraint_Ct(Qc, 1.0);
            
            mintegrable->StateSolveCorrection(V, L, R, Qc,
                                              1.0,           // factor for  M
                                              -mdt,          // factor for  dF/dv
                                              -mdt * mdt,    // factor for  dF/dx
                                              X, V, T + mdt, // not needed
                                              false  // do not StateScatter update to Xnew Vnew T+dt before computing correction
                                              );
            
            mintegrable->StateScatter(X, V, T);     // state -> system

            L *= (1.0 / mdt);  // Note it is not -(1.0/mdt) because we assume StateSolveCorrection already flips sign of Dl

            if (action & ASS_ACCEL) {
                mintegrable->StateScatterAcceleration(
                    (V - Vold) * (1 / mdt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)
            
                mintegrable->StateScatterReactions(L);  // -> system auxiliary data
            }
        }
    }

    /// Set the max number of Newton-Raphson iterations for the positional assembly procedure
    void SetMaxAssemblyIters(int mi) { max_assembly_iters = mi;}
    /// Get the max number of Newton-Raphson iterations for the positional assembly procedure
    int GetMaxAssemblyIters() {return max_assembly_iters;}

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



}  // END_OF_NAMESPACE____
#endif
