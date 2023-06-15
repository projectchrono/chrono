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

#include "chrono/solver/ChIterativeSolver.h"

namespace chrono {

ChIterativeSolver::ChIterativeSolver(int max_iterations, double tolerance, bool use_precond, bool warm_start)
    : m_max_iterations(max_iterations), m_tolerance(tolerance), m_use_precond(use_precond), m_warm_start(warm_start) {}

void ChIterativeSolver::SaveMatrix(ChSystemDescriptor& sysd) {
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
        ChStreamOutAsciiFile file("Z1.dat");
        file.SetNumFormat("%.12g");
        StreamOutSparseMatlabFormat(Z1, file);
    }
    {
        ChStreamOutAsciiFile file("Z2.dat");
        file.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(Z2, file);
    }

    // Assemble RHS
    ChVectorDynamic<> rhs1;
    sysd.ConvertToMatrixForm(nullptr, &rhs1);

    // RHS using d vector
    ChVectorDynamic<> rhs2;
    sysd.BuildDiVector(rhs2);

    // Save vectors to file
    {
        ChStreamOutAsciiFile file("rhs1.dat");
        file.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(rhs1, file);
    }
    {
        ChStreamOutAsciiFile file("rhs2.dat");
        file.SetNumFormat("%.12g");
        StreamOutDenseMatlabFormat(rhs2, file);
    }
}

double ChIterativeSolver::CheckSolution(ChSystemDescriptor& sysd, const ChVectorDynamic<>& x) {
    ChVectorDynamic<> b;
    sysd.ConvertToMatrixForm(nullptr, &b);

    ChSparseMatrix Z;
    sysd.ConvertToMatrixForm(&Z, nullptr);
    double res_norm1 = (Z * x - b).norm();
    
    ChVectorDynamic<> Zx(x.size());
    sysd.SystemProduct(Zx, x);
    double res_norm2 = (Zx - b).norm();

    std::cout << "  Residual norm (using full matrix): " << res_norm1 << std::endl;
    std::cout << "  Residual norm (using SPMV):        " << res_norm2 << std::endl;

    return res_norm1;
}

}  // end namespace chrono
