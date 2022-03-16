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


#include "chrono_pardisoproject/ChPardisoProjectEngine.h"
#include "chrono/utils/ChOpenMP.h"


/* PARDISO prototype. */
extern "C" void pardisoinit (void   *, int    *,   int *, int *, double *, int *);
extern "C" void pardiso     (void   *, int    *,   int *, int *,    int *, int *, 
                  double *, int    *,    int *, int *,   int *, int *,
                     int *, double *, double *, int *, double *);
extern "C" void pardiso_chkmatrix  (int *, int *, double *, int *, int *, int *);
extern "C" void pardiso_chkvec     (int *, int *, double *, int *);
extern "C" void pardiso_printstats (int *, int *, double *, int *, int *, int *, double *, int *);
extern "C" void pardiso_get_schur(void*, int*, int*, int*, double*, int*, int*);

namespace chrono {

ChPardisoProjectEngine::ChPardisoProjectEngine(parproj_SYM symmetry) :symmetry(symmetry) {
    Reinit();
}

ChPardisoProjectEngine::~ChPardisoProjectEngine() {
    PardisoProjectCall(parproj_PHASE::END);
}

void ChPardisoProjectEngine::SetProblem(const ChSparseMatrix& Z, ChVectorRef rhs, ChVectorRef sol) {
    SetMatrix(Z);
    SetRhsVector(rhs);
    SetSolutionVector(sol);
}

void ChPardisoProjectEngine::SetMatrix(const ChSparseMatrix& Z, bool isZeroIndexed) {
    this->SetMatrix(
        Z.rows(),
        const_cast<int*>(Z.outerIndexPtr()),
        const_cast<int*>(Z.innerIndexPtr()),
        const_cast<double*>(Z.valuePtr()),
        false
    );
    matOneIndexedFormat = !isZeroIndexed;
}

void ChPardisoProjectEngine::SetMatrix(int n, int *ia, int *ja, double *a, bool isZeroIndexed) {
    this->n = n;
    this->a = a;
    this->ia = ia;
    this->ja = ja;
    matOneIndexedFormat = !isZeroIndexed;
}

void ChPardisoProjectEngine::SetMatrixSymmetry(parproj_SYM symmetry) {
    PardisoProjectCall(parproj_PHASE::END);
    this->symmetry = symmetry;
    Reinit();
}

void ChPardisoProjectEngine::SetRhsVector(ChVectorRef b) {
    this->b = b.data();
}

void ChPardisoProjectEngine::SetRhsVector(double* b) {
    this->b = b;
}

void ChPardisoProjectEngine::SetSolutionVector(ChVectorRef x) {
    this->x = x.data();
}

void ChPardisoProjectEngine::SetSolutionVector(double* x) {
    this->x = x;
}


int ChPardisoProjectEngine::PardisoProjectCall(parproj_PHASE phase) {
    int phase_int = phase;
    int mtype_int = symmetry;

    SetOneIndexedFormat(); // make sure that is one-based format

    pardiso (pt, &maxfct, &mnum, &mtype_int, &phase_int, &n, a, ia, ja, &idum, &nrhs, iparm, &msglvl, b, x, &error,  dparm);

    return error;
}


int ChPardisoProjectEngine::CheckMatrix(bool print){
    /* -------------------------------------------------------------------- */
    /*  .. pardiso_chk_matrix(...)                                          */
    /*     Checks the consistency of the given matrix.                      */
    /*     Use this functionality only for debugging purposes               */
    /* -------------------------------------------------------------------- */
    bool matOneIndexedFormat_bkp = this->matOneIndexedFormat;
    SetOneIndexedFormat();
    int mtype_int = symmetry;
    pardiso_chkmatrix(&mtype_int, &n, a, ia, ja, &error);
    if (!matOneIndexedFormat_bkp) SetZeroIndexedFormat();

    if (error != 0) {
        if (print)
            printf("\nERROR in consistency of matrix: %d", error);
        return error;
    } else
        printf("\nMatrix consistency check passed\n");
    return error;
}

int ChPardisoProjectEngine::CheckMatrixStats(bool print) {
    /* -------------------------------------------------------------------- */
    /* .. pardiso_printstats(...)                                           */
    /*    prints information on the matrix to STDOUT.                       */
    /*    Use this functionality only for debugging purposes                */
    /* -------------------------------------------------------------------- */
    bool matOneIndexedFormat_bkp = this->matOneIndexedFormat;
    SetOneIndexedFormat();
    int mtype_int = symmetry;
    pardiso_printstats(&mtype_int, &n, a, ia, ja, &nrhs, b, &error);
    if (!matOneIndexedFormat_bkp) SetZeroIndexedFormat();

    if (error != 0) {
        if (print)
            printf("\nERROR in matrix stats: %d", error);
        return error;
    } else
        printf("\nMatrix stats passed\n");
    return error;
}


int ChPardisoProjectEngine::CheckRhsVectors(bool print){
    /* -------------------------------------------------------------------- */
    /* ..  pardiso_chkvec(...)                                              */
    /*     Checks the given vectors for infinite and NaN values             */
    /*     Input parameters (see PARDISO user manual for a description):    */
    /*     Use this functionality only for debugging purposes               */
    /* -------------------------------------------------------------------- */
    bool matOneIndexedFormat_bkp = this->matOneIndexedFormat;
    SetOneIndexedFormat();
    pardiso_chkvec (&n, &nrhs, b, &error);
    if (!matOneIndexedFormat_bkp) SetZeroIndexedFormat();

    if (error != 0) {
        if (print)
            printf("\nERROR in right hand side: %d", error);
        return error;
    } else
        printf("\nRhs check passed\n");
    return error;
}

void ChPardisoProjectEngine::GetSchurComplement(ChSparseMatrix& Z, int nrows) {
    SetIPARM(38-1, nrows);
    PardisoProjectCall(ChPardisoProjectEngine::parproj_PHASE::ANALYZE_FACTORIZE);
    int schurNNZ = GetIPARM(39-1);
    Z.resize(nrows, nrows);
    Z.reserve(schurNNZ);
    int *iS, *jS;
    double* S;
    getMatrixInternalArrays(Z, &iS, &jS, &S);
    int mtype_int = symmetry;
    pardiso_get_schur(pt, &maxfct, &mnum, &mtype_int, S, iS, jS);
}

void ChPardisoProjectEngine::shiftInternalMatrixIndices(int val) {
    shiftMatrixIndices(ia, ja, a, n, val, matOneIndexedFormat);
}

void ChPardisoProjectEngine::shiftMatrixIndices(int* ext_ia, int* ext_ja, double* ext_a, int ext_n, int val, bool isOneIndexed) {
    int nnz = isOneIndexed ? ext_ia[ext_n]-1 : ext_ia[ext_n];
    for (int i = 0; i < ext_n + 1; i++) {
        ext_ia[i] += val;
    }
    for (int i = 0; i < nnz; i++) {
        ext_ja[i] += val;
    }
}

bool ChPardisoProjectEngine::getMatrixInternalArrays(ChSparseMatrix& sparseMat, int** ext_ia, int** ext_ja, double** ext_a) {
    *ext_ia = sparseMat.outerIndexPtr();
    *ext_ja = sparseMat.innerIndexPtr();
    *ext_a =  sparseMat.valuePtr();
    return false;
}

inline void ChPardisoProjectEngine::SetZeroIndexedFormat() {
    if (matOneIndexedFormat)
        shiftInternalMatrixIndices(-1);
    matOneIndexedFormat = false;
}

inline void ChPardisoProjectEngine::SetOneIndexedFormat() {
    if (!matOneIndexedFormat)
        shiftInternalMatrixIndices(+1);
    matOneIndexedFormat = true;
}

/// Set the solver type. \p directsparse = true for direct sparse solver, false for multi-recursive iterative solver.
/// \warning{This function triggers a Reinit()}

inline void ChPardisoProjectEngine::SetSolverType(bool directsparse) {
    solver = directsparse ? 0 : 1;
    //SetIPARM(32-1, directsparse ? 0 : 1);
    Reinit();
}

void ChPardisoProjectEngine::Reinit() {
    this->iparm[2]  = ChOMP::GetNumProcs();

    int mtype_int = symmetry;
    pardisoinit(pt,  &mtype_int, &solver, iparm, dparm, &error);

    if (error != 0)
    {
        if (error == -10 )
           printf("No license file found \n");
        if (error == -11 )
           printf("License is expired \n");
        if (error == -12 )
           printf("Wrong username or hostname \n");
         //return 1;
    }
    else
        printf("[PARDISO]: License check was successful ... \n");
}

void ChPardisoProjectEngine::SetMessageLevel(int msglvl) {
    if (msglvl == 0 || msglvl == 1)
        this->msglvl = msglvl;
}


}  // namespace chrono
