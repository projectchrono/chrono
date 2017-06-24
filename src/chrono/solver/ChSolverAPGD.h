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

#ifndef CHSOLVERAPGD_H
#define CHSOLVERAPGD_H

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

/// An iterative solver based on Nesterov's Projected Gradient Descent.\n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures
/// passed to the solver.

class ChApi ChSolverAPGD : public ChIterativeSolver {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSolverAPGD);

  protected:
    double residual = 0;
    int nc = 0;
    ChMatrixDynamic<> gamma_hat, gammaNew, g, y, gamma, yNew, r, tmp;

  public:
    ChSolverAPGD(int mmax_iters = 1000,     ///< max.number of iterations
                 bool mwarm_start = false,  ///< uses warm start?
                 double mtolerance = 0.0    ///< tolerance for termination criterion
                 )
        : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, 0.0001) {}

    virtual ~ChSolverAPGD() {}

    virtual Type GetType() const override { return Type::APGD; }

    /// Performs the solution of the problem.
    virtual double Solve(ChSystemDescriptor& sysd) override;

    void ShurBvectorCompute(ChSystemDescriptor& sysd);
    double Res4(ChSystemDescriptor& sysd);

    double GetResidual() { return residual; }

    void Dump_Rhs(std::vector<double>& temp) {
        for (int i = 0; i < r.GetRows(); i++) {
            temp.push_back(r(i, 0));
        }
    }

    void Dump_Lambda(std::vector<double>& temp) {
        for (int i = 0; i < gamma_hat.GetRows(); i++) {
            temp.push_back(gamma_hat(i, 0));
        }
    }
};

} // end namespace chrono

#endif