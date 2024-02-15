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

#include "chrono/core/ChMath.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"

namespace chrono {

ChAssemblyAnalysis::ChAssemblyAnalysis(ChIntegrableIIorder& mintegrable) {
    integrable = &mintegrable;
    X.setZero(1, &mintegrable);
    V.setZero(1, &mintegrable);
    A.setZero(1, &mintegrable);
    max_assembly_iters = 4;
}

void ChAssemblyAnalysis::AssemblyAnalysis(int action, double dt) {
    ChVectorDynamic<> R;
    ChVectorDynamic<> Qc;
    double T;

    // Set up main vectors
    integrable->StateSetup(X, V, A);

    if (action & AssemblyLevel::POSITION) {
        ChStateDelta Dx;

        for (int m_iter = 0; m_iter < max_assembly_iters; m_iter++) {
            // Set up auxiliary vectors
            Dx.setZero(integrable->GetNcoords_v(), GetIntegrable());
            R.setZero(integrable->GetNcoords_v());
            Qc.setZero(integrable->GetNconstr());
            L.setZero(integrable->GetNconstr());

            integrable->StateGather(X, V, T);  // state <- system

            // Solve:
            //
            // [M          Cq' ] [ dx  ] = [  0]
            // [ Cq        0   ] [ -l  ] = [ -C]

            integrable->LoadConstraint_C(Qc, 1.0); // sign flipped later in StateSolveCorrection

            integrable->StateSolveCorrection(
                Dx, L, R, Qc,
                1.0,      // factor for  M
                0,        // factor for  dF/dv
                0,        // factor for  dF/dx (the stiffness matrix)
                X, V, T,  // not needed
                false,    // do not scatter Xnew Vnew T+dt before computing correction
                false,    // full update? (not used, since no scatter)
                true      // force a call to the solver's Setup function
                );

            X += Dx;

            integrable->StateScatter(X, V, T, true);  // state -> system
        }
    }

    if ((action & AssemblyLevel::VELOCITY) || (action & AssemblyLevel::ACCELERATION)) {
        ChStateDelta Vold;

        // setup auxiliary vectors
        Vold.setZero(integrable->GetNcoords_v(), GetIntegrable());
        R.setZero(integrable->GetNcoords_v());
        Qc.setZero(integrable->GetNconstr());
        L.setZero(integrable->GetNconstr());

        integrable->StateGather(X, V, T);  // state <- system

        Vold = V;

        // Perform a linearized semi-implicit Euler integration step
        //
        // [ M - dt*dF/dv - dt^2*dF/dx    Cq' ] [ v_new  ] = [ M*(v_old) + dt*f]
        // [ Cq                           0   ] [ -dt*l  ] = [ -C/dt - Ct ]

        integrable->LoadResidual_F(R, dt);
        integrable->LoadResidual_Mv(R, V, 1.0);
        integrable->LoadConstraint_C(Qc, 1.0 / dt, false); // sign later flipped in StateSolveCorrection
        integrable->LoadConstraint_Ct(Qc, 1.0);  // sign later flipped in StateSolveCorrection

        integrable->StateSolveCorrection(
            V, L, R, Qc,
            1.0,           // factor for  M
            -dt,           // factor for  dF/dv
            -dt * dt,      // factor for  dF/dx
            X, V, T + dt,  // not needed
            false,         // do not scatter Xnew Vnew T+dt before computing correction
            false,         // full update? (not used, since no scatter)
            true           // force a call to the solver's Setup() function
        );

        integrable->StateScatter(X, V, T, true);  // state -> system

        L *= (1.0 / dt);  // Note it is not -(1.0/dt) because we assume StateSolveCorrection already flips sign of L

        if (action & AssemblyLevel::ACCELERATION) {
            integrable->StateScatterAcceleration(
                (V - Vold) * (1 / dt));  // -> system auxiliary data (i.e acceleration as measure, fits DVI/MDI)

            integrable->StateScatterReactions(L);  // -> system auxiliary data
        }
    }
}

}  // end namespace chrono