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
// Authors: Radu Serban
// =============================================================================

#include "chrono/solver/ChIterativeSolverVI.h"

namespace chrono {

void ChIterativeSolverVI::AtIterationEnd(double mmaxviolation, double mdeltalambda, unsigned int iternum) {
    if (!record_violation_history)
        return;
    if (iternum != violation_history.size()) {
        violation_history.clear();
        violation_history.resize(iternum);
    }
    if (iternum != dlambda_history.size()) {
        dlambda_history.clear();
        dlambda_history.resize(iternum);
    }
    violation_history.push_back(mmaxviolation);
    dlambda_history.push_back(mdeltalambda);
}

void ChIterativeSolverVI::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChIterativeSolverVI>();
    // serialize parent class
    ChSolver::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(max_iterations);
    marchive << CHNVP(warm_start);
    marchive << CHNVP(tolerance);
    marchive << CHNVP(omega);
    marchive << CHNVP(shlambda);
}

void ChIterativeSolverVI::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChIterativeSolverVI>();
    // deserialize parent class
    ChSolver::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(max_iterations);
    marchive >> CHNVP(warm_start);
    marchive >> CHNVP(tolerance);
    marchive >> CHNVP(omega);
    marchive >> CHNVP(shlambda);
}

void ChIterativeSolverVI::SaveMatrix(ChSystemDescriptor& sysd) {
    // Assemble sparse matrix
    ChSparseMatrix Z1;
    sysd.ConvertToMatrixForm(&Z1, nullptr);

    // Create sparse matrix with SPMV
    ChMatrixDynamic<> Z2(Z1.rows(), Z1.cols());
    ChVectorDynamic<> e = ChVectorDynamic<>::Zero(Z1.cols());
    ChVectorDynamic<> v(Z1.cols());
    for (int i = 0; i < Z1.cols(); i++) {
        e(i) = 1;
        sysd.SystemProduct(v, e);
        Z2.col(i) = v;
        e(i) = 0;
    }
    
    // Save matrices to file
    {
        ChStreamOutAsciiFile file("matrix_Z1.dat");
        file.SetNumFormat("%.12g");
        StreamOUTsparseMatlabFormat(Z1, file);
    }
    {
        ChStreamOutAsciiFile file("matrix_Z2.dat");
        file.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(Z2, file);
    }

    // Assemble RHS
    ChVectorDynamic<> rhs1;
    sysd.ConvertToMatrixForm(nullptr, &rhs1);

    // RHS using d vector
    ChVectorDynamic<> rhs2;
    sysd.BuildDiVector(rhs2);
}

double ChIterativeSolverVI::CheckSolution(ChSystemDescriptor& sysd, const ChVectorDynamic<>& x) {
    ChSparseMatrix A;
    sysd.ConvertToMatrixForm(&A, nullptr);
    ChVectorDynamic<> b;
    sysd.ConvertToMatrixForm(nullptr, &b);
    return (A * x - b).norm();
}

}  // end namespace chrono
