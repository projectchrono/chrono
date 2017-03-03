// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSOLVERSORMULTITHREAD_H
#define CHSOLVERSORMULTITHREAD_H

#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/parallel/ChThreads.h"

namespace chrono {

class ChApi ChSolverSORmultithread : public ChIterativeSolver {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSolverSORmultithread)

  protected:
    ChThreads* solver_threads;

  public:
    ChSolverSORmultithread(const char* uniquename = "solver",  ///< this name must be unique.
                           int nthreads = 2,                   ///< number of threads
                           int mmax_iters = 50,                ///< max.number of iterations
                           bool mwarm_start = false,           ///< uses warm start?
                           double mtolerance = 0.0,            ///< tolerance for termination criterion
                           double momega = 1.0                 ///< overrelaxation criterion
                           );

    virtual ~ChSolverSORmultithread();

    /// Return type of the solver.
    virtual Type GetType() const override { return Type::SOR_MULTITHREAD; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Changes the number of threads which run in parallel (should be > 1 )
    void ChangeNumberOfThreads(int mthreads = 2);
};

}  // end namespace chrono

#endif
