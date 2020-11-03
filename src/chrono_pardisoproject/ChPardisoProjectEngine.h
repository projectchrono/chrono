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

#ifndef CHPARDISOPROJECTENGINE_H
#define CHPARDISOPROJECTENGINE_H

#include "chrono/core/ChMatrix.h"
#include "chrono_pardisoproject/ChApiPardisoProject.h"


namespace chrono {

/// @addtogroup pardisoproject_module
/// @{

/// Wrapper class for the PardisoProject direct linear solver.
/// This solver is not appropriate for VI and complementarity problems.
/// \warning {[WARNING for DEVELOPERS]: please consider that the solver does not handle C-like arrays so the matrices have to be set to one-indexed format manually.
/// Appropriate instructions have been put to avoid passing back the matrix in one-indexed state when not possible}

class ChApiPardisoProject ChPardisoProjectEngine {
  public:
    enum parproj_SYM {
        STRUCTURAL_SYMMETRIC = 1, ///< real and structurally symmetric, supernode pivoting
        SYMMETRIC_POSDEF = 2, ///< real and symmetric positive definite
        SYMMETRIC_GENERAL = -2, ///< real and symmetric indefinite, diagonal or Bunch-Kaufman pivoting
        UNSYMMETRIC = 11 ///< real and nonsymmetric, complete supernode pivoting
    };

    enum parproj_PHASE {
        END = -1,
        ANALYZE = 11,
        ANALYZE_FACTORIZE = 12,
        FACTORIZE = 22,
        SOLVE = 33,
        FACTORIZE_SOLVE = 23,
        COMPLETE = 13,
        SELECTED_INVERSION = -22
    };

    ChPardisoProjectEngine(parproj_SYM symmetry);
    ~ChPardisoProjectEngine();

    /// Set the problem matrix and the right-hand side.
    void SetProblem(const ChSparseMatrix& Z, ChVectorRef rhs, ChVectorRef sol);

    /// Set the problem matrix.
    void SetMatrix(const ChSparseMatrix& Z, bool isZeroIndexed = true);
    void SetMatrix(int n, int *ia, int *ja, double *a, bool isZeroIndexed = true);

    /// Set a new value for symmetry. \warning{This function triggers a Reinit()}
    void SetMatrixSymmetry(parproj_SYM symmetry);

    /// Set the right-hand side vector.
    /// Note that it is the caller's responsibility to ensure that the size is appropriate.
    void SetRhsVector(ChVectorRef b);
    void SetRhsVector(double* b);

    /// Set the solution vector.
    /// Note that it is the caller's responsibility to ensure that the size is appropriate.
    void SetSolutionVector(ChVectorRef x);
    void SetSolutionVector(double* x);

    /// Submit job to PardisoProject.
    int PardisoProjectCall(parproj_PHASE job_call);

    /// Check if the input matrix is in the appropriate form.
    int CheckMatrix(bool print=true);

    /// Check if the rhs vector is in the appropriate form.
    int CheckRhsVectors(bool print=true);

    /// Check if the input matrix has appropriate properties.
    int CheckMatrixStats(bool print=true);

    /// Return the value of the i-th IPARM coefficient; consider that is in zero-indexed format.
    int GetIPARM(int id) const { return iparm[id]; };
    /// Set the value of the i-th IPARM coefficient; consider that is in zero-indexed format.
    void SetIPARM(int id, int val){ iparm[id] = val; };

    /// Return the value of the i-th DPARM coefficient; consider that is in zero-indexed format.
    double GetDPARM(int id) const { return dparm[id]; };
    /// Set the value of the i-th DPARM coefficient; consider that is in zero-indexed format.
    void SetDPARM(int id, int val){ dparm[id] = val; };

    /// Return the value of the error flag.
    int GetLastError() {return error;};

    /// Set the index of the underlying arrays to zero-indexed.
    void SetZeroIndexedFormat();
    /// Set the index of the underlying arrays to one-indexed.
    void SetOneIndexedFormat();

    /// Set the solver type. \p directsparse = true for direct sparse solver, false for multi-recursive iterative solver.  \warning{This function triggers a Reinit()}
    void SetSolverType(bool directsparse = true);

    /// Reinitialize the solver (e.g. when a new symmetry option is set)
    void Reinit();

    /// Set the message level verbosity (0: no messages, 1:print stats). (Default: 0)
    void SetMessageLevel(int msglvl);

    /// Set the maximum number of numerical factorizations.
    void SetMaxNumericalFactorization(int maxfct){ maxfct = maxfct; };

    /// Set the maximum number of numerical factorizations.
    void GetSchurComplement(ChSparseMatrix& Z, int nrows);

  protected:
    /// Shift the matrix indeces of the internal matrix arrays of a value of \p val
    void shiftMatrixIndices(int* ext_ia, int* ext_ja, double* ext_a, int ext_n, int val, bool isOneIndexed);

    /// Shift the matrix indeces of a value of \p val
    void shiftInternalMatrixIndices(int val);

    /// Get internal arrays of sparse matrix
    bool getMatrixInternalArrays(ChSparseMatrix& sparseMat, int** ext_ia, int** ext_ja, double** ext_a);

  private:

    bool matOneIndexedFormat = false;

    /* RHS and solution vectors. */
    double   *b, *x;
    int      nrhs = 1;          /* Number of right hand sides. */

    /* Internal solver memory pointer pt,                  */
    /* 32-bit: int pt[64]; 64-bit: long int pt[64]         */
    /* or void *pt[64] should be OK on both architectures  */ 
    void     *pt[64];

    /* Pardiso control parameters. */
    int      iparm[64];
    double   dparm[64];
    int      solver = 0;            /* Solver type: 0: sparse direct solver; 1: multi-recursive iterative solver*/
    int      maxfct = 1;            /* Maximum number of numerical factorizations in memory.*/
    int      mnum = 1;              /* Which factorization to use. */
    int      error = 0;             /* Initialize error flag */
    int      msglvl = 0;            /* Print statistical information  */

    /* Number of processors. */
    int      num_procs;

    /* Auxiliary variables. */
    char    *var;
    int      i;

    double   ddum;              /* Double dummy */
    int      idum;              /* Integer dummy. */
    
    // Matrix variables
    int    n = 0;
    int    *ia;
    int    *ja;
    double  *a;

    parproj_SYM symmetry;

};

/// @} pardisoproject_module

}  // end of namespace chrono

#endif
