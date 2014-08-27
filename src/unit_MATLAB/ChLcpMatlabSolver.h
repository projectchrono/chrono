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

#ifndef CHLCPMATLABSOLVER_H
#define CHLCPMATLABSOLVER_H

#include "ChApiMatlab.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolver.h"

#include "ChMatlabEngine.h"

namespace chrono {

/// Class for usin Matlab from a Chrono::Engine programs.

class ChApiMatlab ChLcpMatlabSolver : public ChLcpSolver {
protected:
  ChMatlabEngine* mengine;

public:
  ChLcpMatlabSolver(ChMatlabEngine& me);
  virtual ~ChLcpMatlabSolver() {}

  /// Solve using the Matlab default direct solver (as in x=A\b)
  virtual double Solve(
    ChLcpSystemDescriptor& sysd);      ///< system description with constraints and variables
};

} // END_OF_NAMESPACE____

#endif

