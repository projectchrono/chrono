//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPMKLSOLVER_H
#define CHLCPMKLSOLVER_H

#include "ChApiMkl.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolver.h"

#include "ChMklEngine.h"

namespace chrono {

/// Class for usin Intel® MKL from a Chrono::Engine programs.

class ChApiMkl ChLcpMklSolver : public ChLcpSolver {
  protected:
    ChMklEngine* mklengine;

  public:
    ChLcpMklSolver(ChMklEngine& me);
    virtual ~ChLcpMklSolver() {}

    /// Solve using the MKL PARDISO sparse direct solver
    virtual double Solve(ChLcpSystemDescriptor& sysd);  ///< system description with constraints and variables
};

}  // END_OF_NAMESPACE____

#endif
