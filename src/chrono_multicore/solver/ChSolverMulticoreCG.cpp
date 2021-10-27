// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================

#include "chrono_multicore/solver/ChSolverMulticore.h"

using namespace chrono;

real Convergence_Norm(const DynamicVector<real>& r) {
    real result = (real)0.;
    for (int i = 0; i < r.size(); i += 3) {
        real3 v(r[i + 0], r[i + 1], r[i + 2]);
        real mag = Length(v);
        result = Max(result, mag);
    }
    return result;
}
uint ChSolverMulticoreCG::Solve(ChShurProduct& ShurProduct,
                               ChProjectConstraints& Project,
                               const uint max_iter,
                               const uint size,
                               const DynamicVector<real>& b,
                               DynamicVector<real>& x) {
    r.resize(b.size());
    q.resize(b.size());
    s.resize(b.size());

    real rho_old = C_REAL_MAX;
    real convergence_norm = 0;
    real tolerance = 1e-4;  // Max(1e-4 * Convergence_Norm(b), 1e-6);
    uint min_iterations = 0;

    uint restart_iterations = 100;
    for (uint iterations = 0;; iterations++) {
        bool restart = !iterations || (restart_iterations && iterations % restart_iterations == 0);
        if (restart) {
            printf("restarting cg\n");
            r = b;
            ShurProduct(x, q);
            r -= q;
            // Project(r.data());
        }

        convergence_norm = Convergence_Norm(r);
        printf("%f\n", convergence_norm);

        if (convergence_norm <= tolerance && (iterations >= min_iterations || convergence_norm < C_REAL_EPSILON)) {
            printf("cg iterations %d\n", iterations);
            break;
        }
        if (iterations == max_iter) {
            break;
        }

        real rho = (r, r);
        if (restart) {
            s = r;
        } else {
            s = rho / rho_old * s + r;
        }
        ShurProduct(s, q);
        // Project(r.data());
        real s_dot_q = (s, q);
        real alpha = s_dot_q ? rho / s_dot_q : C_REAL_MAX;
        x = alpha * s + x;
        r = -alpha * q + r;
        rho_old = rho;
    }

    return current_iteration;
}
