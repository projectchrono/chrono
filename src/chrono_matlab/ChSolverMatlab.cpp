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

#include "chrono_matlab/ChSolverMatlab.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverMatlab)

ChSolverMatlab::ChSolverMatlab(ChMatlabEngine& me) {
    mengine = &me;
}

ChSolverMatlab::ChSolverMatlab() {
    mengine = 0;
}

// Solve using the Matlab default direct solver (as in x=A\b)
double ChSolverMatlab::Solve(ChSystemDescriptor& sysd) {

    ChSparseMatrix Z;
    ChVectorDynamic<double> rhs;
    sysd.BuildSystemMatrix(&Z, &rhs);

    mengine->PutSparseMatrix(Z, "Z");
    mengine->PutVariable(rhs, "rhs");

    mengine->Eval("sol = mldivide(Z , rhs);");

    ChMatrixDynamic<> sol;
    if (!mengine->GetVariable(sol, "sol"))
        std::cerr << "ERROR!! cannot fetch sol" << std::endl;

    sysd.FromVectorToUnknowns(sol);

    mengine->Eval("residual = norm(Z*sol - rhs);");
    ChMatrixDynamic<> residual;
    mengine->GetVariable(residual, "residual");
    std::cout << " Matlab computed residual:" << residual(0, 0) << std::endl;

    return 0;
}

void ChSolverMatlab::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSolverMatlab>();
    // serialize parent class
    ChSolver::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(mengine);
}

void ChSolverMatlab::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSolverMatlab>();
    // deserialize parent class
    ChSolver::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(mengine);
}

}  // end namespace chrono
