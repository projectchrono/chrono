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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================
// Interfacing to the Pardiso Sparse Direct Solver from the Intel® MKL Library.
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_mkl/ChMklEngine.h"

namespace chrono {

// Constructor.
// m_nrhs   - number of RHS vectors (currently only 1 supported)
// m_maxfct - max. number of factors with identical sparsity structure that must be kept in
//            memory at the same time
// m_mnum   - actual matrix for the solution phase (1 ≤ m_mnum ≤ m_maxfct)
ChMklEngine::ChMklEngine(int pb_size, ChSparseMatrix::SymmetryType matrix_type)
    : m_a(nullptr),
      m_ia(nullptr),
      m_ja(nullptr),
      m_b(nullptr),
      m_x(nullptr),
      m_n(pb_size),
      m_nrhs(1),
      m_maxfct(1),
      m_mnum(1),
      m_last_phase(-1) {
    m_type = GetPardisoMatrixType(matrix_type);
    ResetSolver();
}

ChMklEngine::~ChMklEngine() {
    int phase = RELEASE_ALL;
    int msglvl = 1;
    int error;
    PARDISO(m_pt, &m_maxfct, &m_mnum, &m_type, &phase, &m_n, m_a, m_ia, m_ja, m_perm.data(), &m_nrhs, m_iparm, &msglvl,
            m_b, m_x, &error);
    if (error)
        printf("Error while releasing memory: %d", error);
}

void ChMklEngine::SetMatrix(ChSparseMatrix& Z) {
    assert(Z.GetNumRows() == Z.GetNumColumns());

    m_n = Z.GetNumRows();

    m_a = Z.GetCS_ValueArray();
    m_ia = Z.GetCS_LeadingIndexArray();
    m_ja = Z.GetCS_TrailingIndexArray();

    int type = GetPardisoMatrixType(Z.GetType());
    if (m_type != type) {
        m_type = type;
        ResetSolver();
    }
}

void ChMklEngine::SetMatrix(int pb_size, double* a, int* ia, int* ja) {
    m_n = pb_size;
    m_a = a;
    m_ia = ia;
    m_ja = ja;
}

void ChMklEngine::SetSolutionVector(ChMatrix<>& x) {
    assert(x.GetRows() >= m_n);
    m_x = x.GetAddress();
}

void ChMklEngine::SetSolutionVector(double* x) {
    m_x = x;
}

void ChMklEngine::SetRhsVector(ChMatrix<>& b) {
    assert(b.GetRows() >= m_n);
    m_b = b.GetAddress();
}

void ChMklEngine::SetRhsVector(double* b) {
    m_b = b;
}

void ChMklEngine::SetProblem(ChSparseMatrix& Z, ChMatrix<>& b, ChMatrix<>& x) {
    SetMatrix(Z);
    SetRhsVector(b);
    SetSolutionVector(x);
}

void ChMklEngine::UsePermutationVector(bool val) {
    // The perm array is not set yet; it is built by Pardiso during the next factorization phase.
    // iparm[4]=2 (perm as output) says to Pardiso to output the perm vector used by the factorization;
    // PardisoCall() will then switch iparm[4] to 1 (perm as input)
    m_iparm[4] = (val) ? 2 : 0;

    if (val) {
        resetIparmElement(30);
        resetIparmElement(35);
        m_perm.resize(m_n);
    }
}

void ChMklEngine::resetIparmElement(int iparm_num, int reset_value) {
    if (m_iparm[iparm_num] != reset_value) {
        m_iparm[iparm_num] = reset_value;

        switch (iparm_num) {
            case 3:
                printf("Preconditioned CGS has been disabled. iparm[3] = 0");
                break;
            case 4:
                printf("Permutation vector has been disabled.iparm[4] = 0");
                break;
            case 7:
                printf("Iterative refinement steps has been disabled. iparm[7] = 0");
                break;
            case 30:
                printf("Partial solution has been disabled. iparm[30] = 0");
                break;
            case 35:
                printf("Schur computation has been disabled. iparm[35] = 0");
                break;
            case 59:
                printf("Mkl is now running in-core. iparm[59] = 0");
                break;
            default:
                printf("WARN: IparmReset not handled");
        }
    }
}

void ChMklEngine::UsePartialSolution(int option, int start_row, int end_row) {
    assert(option == 0 || option == 1 || option == 2 || option == 3);

    m_iparm[30] = option;

    if (option) {
        resetIparmElement(3);
        resetIparmElement(4);
        resetIparmElement(7);
        resetIparmElement(35);
        resetIparmElement(59);

        m_perm.resize(m_n);

        if (option == 1 || option == 2) {
            for (int row_sel = 0; row_sel < m_n; row_sel++)
                m_perm[row_sel] = (m_b[row_sel] == 0) ? 0 : 1;
        } else if (option == 3) {
            for (int row_sel = 0; row_sel < m_n; row_sel++)
                m_perm[row_sel] = (row_sel < start_row || row_sel > end_row) ? 0 : 1;
        }
    }
}


void ChMklEngine::OutputSchurComplement(int option, int start_row, int end_row) {
    m_iparm[35] = option;

    if (option) {
        resetIparmElement(4);
        resetIparmElement(30);

        m_perm.resize(m_n);

        assert(!(start_row == 0) && (end_row == 0));

        if (end_row == 0)
            end_row = m_n - 1;

        for (int row_sel = 0; row_sel < m_n; row_sel++)
            m_perm[row_sel] = (row_sel < start_row || row_sel > end_row) ? 0 : 1;
    }
}

void ChMklEngine::SetPreconditionedCGS(bool val, int L) {
    if (val) {
        int K = (m_type == 11 || m_type == 1) ? 1 : 2;
        m_iparm[3] = 10 * L + K;
    } else {
        m_iparm[3] = 0;
    }
}

int ChMklEngine::PardisoCall(int phase, int message_level) {
    int error;
    m_last_phase = phase;
    PARDISO(m_pt, &m_maxfct, &m_mnum, &m_type, &phase, &m_n, m_a, m_ia, m_ja, m_perm.data(), &m_nrhs, m_iparm,
            &message_level, m_b, m_x, &error);

    if (m_iparm[4] == 2)
        m_iparm[4] = 1;

    return error;
}

// Convert the symmetry matrix type to the corresponding Pardiso code.
MKL_INT ChMklEngine::GetPardisoMatrixType(ChSparseMatrix::SymmetryType type) {
    switch (type) {
        case ChSparseMatrix::GENERAL:
            return 11;
        case ChSparseMatrix::SYMMETRIC_POSDEF:
            return 2;
        case ChSparseMatrix::SYMMETRIC_INDEF:
            return -2;
        case ChSparseMatrix::STRUCTURAL_SYMMETRIC:
            return 1;
    }

    return 11;
}

void ChMklEngine::ResetSolver() {
    // After the first call to pardiso do not directly modify m_pt, as that could cause a serious memory leak.
    pardisoinit(m_pt, &m_type, m_iparm);

    // NOTE: for highly indefinite symmetric matrices (e.g. interior point optimizations or saddle point problems)
    // use m_iparm[10] = 1 (scaling) and m_iparm[12] = 1 (matchings);

    // Main settings
    m_iparm[0] = 1;    // No default values for solver
    m_iparm[5] = 0;    // Write solution on u
    m_iparm[11] = 0;   // Solve with transposed/conjugate transposed matrix [def: 0, solve simply A*x=b]
    m_iparm[17] = -1;  // Report number of nonzeros
    m_iparm[18] = -1;  // Report number of floating point operations
    m_iparm[34] = 1;   // Zero based indexing
    m_iparm[26] = 0;   // Matrix checker
    m_iparm[27] = 0;   // Double precision
    m_iparm[35] = 0;   // Schur complement matrix computation control [def:0, do not compute Schur]
    m_iparm[55] = 0;   // Diagonal and pivoting control [def:0, disabled]
    m_iparm[59] = 0;   // In-Core (OC) / Out-Of-Core (OOC) switch [def:0, IC mode]

    // Fine settings
    m_iparm[1] = 2;   // Fill-in reducing ordering [def:2]
    m_iparm[3] = 0;   // Preconditioned CGS/CG [def:0] - HIGHLY RECOMMENDED
    m_iparm[4] = 0;   // User fill-in reducing permutation [def:0, default filling]
    m_iparm[7] = 10;  // Maximum number of iterative refinement steps
}

void ChMklEngine::GetResidual(ChMatrix<>& res) const {
    assert(res.GetRows() >= m_n);
    GetResidual(res.GetAddress());
}
void ChMklEngine::GetResidual(double* res) const {
    // Calculate A*x
    mkl_cspblas_dcsrgemv("N", &m_n, m_a, m_ia, m_ja, m_x, res);
    // Calculate b - A*x
    for (int i = 0; i < m_n; i++) {
        res[i] = m_b[i] - res[i];
    }
}

double ChMklEngine::GetResidualNorm() const {
    std::vector<double> res(m_n);
    GetResidual(res.data());
    double norm = 0;
    for (int i = 0; i < m_n; i++) {
        norm += res[i] * res[i];
    }
    return std::sqrt(norm);
}

void ChMklEngine::PrintPardisoParameters() const {
    printf("\n[6] Number of iterative refinement steps performed: %d", m_iparm[6]);
    if (m_type == 11 || m_type == 13 || m_type == -2 || m_type == -4 || m_type == -6)
        printf("\n[13] Number of perturbed pivots: %d", m_iparm[13]);
    if (m_last_phase == 11 || m_last_phase == 12 || m_last_phase == 13) {
        printf("\n[14] Peak memory on symbolic factorization (kB): %d", m_iparm[14]);
        printf("\n[15] Permanent memory on symbolic factorization (kB): %d", m_iparm[15]);
        printf("\n[16] Peak memory on numerical factorization and solution (kB): %d", m_iparm[16]);
        printf("\nTotal peak memory consumed (kB): %d", std::max(m_iparm[14], m_iparm[15] + m_iparm[16]));
    }

    printf("\n[17] Number of non-zero elements in the factors: %d", m_iparm[17]);
    printf("\n[18] Number of floating point operations necessary to factor the matrix (^6): %d", m_iparm[18]);
    printf("\n[19] Number of completed CG/CGS iterations: %d", m_iparm[19]);
    if (m_type == -2) {
        printf("\n[21] Number of positive eigenvalues: %d", m_iparm[21]);
        printf("\n[22] Number of negative eigenvalues: %d\n", m_iparm[22]);
    }
    if (m_type == 2 || m_type == 4)
        printf("\n[29] Number of zero or negative pivots: %d", m_iparm[29]);
}

}  // end namespace chrono
