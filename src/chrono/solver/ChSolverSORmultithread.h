//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSOLVERSORMULTITHREAD_H
#define CHSOLVERSORMULTITHREAD_H

#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/parallel/ChThreads.h"

namespace chrono {

class ChApi ChSolverSORmultithread : public ChIterativeSolver {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChSolverSORmultithread, ChIterativeSolver);

  protected:
    ChThreads* solver_threads;

  public:
    //
    // CONSTRUCTORS
    //

    ChSolverSORmultithread(char* uniquename = (char*)"solver",  ///< this name must be unique.
                           int nthreads = 2,                    ///< number of threads
                           int mmax_iters = 50,                 ///< max.number of iterations
                           bool mwarm_start = false,            ///< uses warm start?
                           double mtolerance = 0.0,             ///< tolerance for termination criterion
                           double momega = 1.0                  ///< overrelaxation criterion
                           );

    virtual ~ChSolverSORmultithread();

    //
    // FUNCTIONS
    //

    /// Performs the solution of the LCP.
    /// \return  the maximum constraint violation after termination.

    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         );

    /// Changes the number of threads which run in parallel (should be > 1 )
    void ChangeNumberOfThreads(int mthreads = 2);
};

}  // end namespace chrono

#endif
