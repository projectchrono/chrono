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
// Authors: Dario Mangoni
// =============================================================================

#include <mpi.h>
#include <bitset>

#include "chrono_mumps/ChMumpsEngine.h"

namespace chrono {

ChMumpsEngine::ChMumpsEngine(mumps_SYM symmetry, int mumps_mpi_comm, int activate_this_node) {
    int myrank;
    MPI_Init(nullptr, nullptr);
    MPI_Comm_rank(MPI_COMM_WORLD, &myrank);

    /* Initialize a MUMPS instance. Use MPI_COMM_WORLD */
    mumps_id.job = INIT;
    mumps_id.par = activate_this_node;
    mumps_id.sym = symmetry;
    mumps_id.comm_fortran = mumps_mpi_comm;

    dmumps_c(&mumps_id);

    /* Output messages */
    mumps_id.ICNTL(1) = 6;  // Error
    mumps_id.ICNTL(2) = 0;  // Diagnostic and warnings
    mumps_id.ICNTL(3) = 0;  // Global information
    mumps_id.ICNTL(4) = 1;  // Error, warning and diagnostic control

    /* Matrix control */
    mumps_id.ICNTL(5) = 0;   // COO Matrix format selection
    mumps_id.ICNTL(18) = 0;  // Matrix centralized on the host
}

ChMumpsEngine::~ChMumpsEngine() {
    mumps_id.job = END;
    dmumps_c(&mumps_id); /* Terminate instance */
    MPI_Finalize();
}

void ChMumpsEngine::SetProblem(const ChCOOMatrix& Z, const ChMatrix<>& rhs) {
    SetMatrix(Z);
    SetRhsVector(rhs);
}

void ChMumpsEngine::SetMatrix(const ChCOOMatrix& Z) {
    /* Define the problem on the host */
    mumps_id.n = Z.GetNumRows();
    mumps_id.nz = Z.GetNNZ();
    mumps_id.irn = Z.GetCOO_RowIndexArray();
    mumps_id.jcn = Z.GetCOO_ColIndexArray();
    mumps_id.a = Z.GetCOO_ValuesAddress();

    switch (Z.GetType()) {
        case ChSparseMatrix::GENERAL:
            mumps_id.sym = mumps_SYM::UNSYMMETRIC;
            break;
        case ChSparseMatrix::SYMMETRIC_POSDEF:
            mumps_id.sym = mumps_SYM::SYMMETRIC_POSDEF;
            break;
        case ChSparseMatrix::SYMMETRIC_INDEF:
            mumps_id.sym = mumps_SYM::SYMMETRIC_GENERAL;
            break;
        case ChSparseMatrix::STRUCTURAL_SYMMETRIC:
            mumps_id.sym = mumps_SYM::UNSYMMETRIC;
            break;
        default:
            mumps_SYM::UNSYMMETRIC;
            break;
    }
}

void ChMumpsEngine::SetMatrixSymmetry(mumps_SYM mat_type) {
    mumps_id.sym = mat_type;
}

void ChMumpsEngine::SetRhsVector(const ChMatrix<>& b) {
    mumps_id.rhs = const_cast<double*>(b.GetAddress());
}

void ChMumpsEngine::SetRhsVector(double* b) {
    mumps_id.rhs = b;
}

int ChMumpsEngine::MumpsCall(mumps_JOB job_call) {
    /* Call the MUMPS package. */
    mumps_id.job = job_call;
    dmumps_c(&mumps_id);
    return mumps_id.INFOG(1);
}

void ChMumpsEngine::PrintINFOG() {
    if (mumps_id.INFOG(1) > 0) {
        printf("WARN: INFOG(1)=");
        typedef std::bitset<sizeof(int)> IntBits;
        if (IntBits(mumps_id.INFOG(1)).test(0))
            printf("+1: Row or column index out of range. Faulty entries: %d\n", mumps_id.INFOG(2));
        if (IntBits(mumps_id.INFOG(1)).test(1))
            printf("+2: Solution max-norm = 0\n");
        if (IntBits(mumps_id.INFOG(1)).test(3))
            printf("+8: More than %d iterative refinements are required\n", mumps_id.ICNTL(10));
        return;
    }

    if (mumps_id.INFOG(1) == 0) {
        printf("INFOG(1)=0: Mumps is successful!\n");
        return;
    }

    printf("ERR: INFOG(1)=");
    switch (mumps_id.INFOG(1)) {
        case (0):
            printf("0: Mumps is successful!\n");
            break;
        case (-1):
            printf("-1: Error on processor %d\n", mumps_id.INFOG(2));
            break;
        case (-2):
            printf("-2: Number of nonzeros out of range NZ=%d\n", mumps_id.INFOG(2));
            break;
        case (-3):
            printf("-3: Mumps called with wrong JOB. JOB=%d\n", mumps_id.INFOG(2));
            break;
        case (-4):
            printf("-4: Error in user-provided permutation array PERM_IN at position: %d\n", mumps_id.INFOG(2));
            break;
        case (-5):
            printf("-5: Problem of real workspace allocation of size %d during analysis\n", mumps_id.INFOG(2));
            break;
        case (-6):
            printf("-6: Matrix is singular in structure. Matrix rank: %d\n", mumps_id.INFOG(2));
            break;
        case (-7):
            printf("-7: Problem of integer workspace allocation of size %d during analysis\n", mumps_id.INFOG(2));
            break;
        case (-10):
            printf("-10: Matrix is numerically singular.\n");
            break;
        case (-16):
            printf("-16: N is out of range. N=%d\n", mumps_id.INFOG(2));
            break;
        case (-21):
            printf("-21: PAR=1 not allowed because only one processor is available.\n");
            break;
        case (-22):
            printf("-22: Array pointers have problems. INFOG(2)=%d\n", mumps_id.INFOG(2));
            break;
        default:
            printf("%d: See the user guide. INFOG(2)=%d\n", mumps_id.INFOG(1), mumps_id.INFOG(2));
    }
}

}  // namespace chrono
