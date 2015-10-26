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

#include "chrono_matlab/ChApiMatlab.h"
#include "chrono_matlab/ChMatlabEngine.h"

#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolver.h"

namespace chrono {

/// Class for usin Matlab from a Chrono::Engine programs.

class ChApiMatlab ChLcpMatlabSolver : public ChLcpSolver {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChLcpMatlabSolver, ChLcpSolver);

  protected:
    ChMatlabEngine* mengine;

  public:
    ChLcpMatlabSolver(ChMatlabEngine& me);
    ChLcpMatlabSolver();

    virtual ~ChLcpMatlabSolver() {}

    /// Set the Matlab engine.
    void SetEngine(ChMatlabEngine* me) {mengine = me;}

    /// Solve using the Matlab default direct solver (as in x=A\b)
    virtual double Solve(ChLcpSystemDescriptor& sysd);  ///< system description with constraints and variables


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChLcpSolver::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(mengine);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChLcpSolver::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(mengine);
    }

};

}  // END_OF_NAMESPACE____

#endif
