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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <chrono_pardisoproject\ChPardisoProjectEngine.h>
#include "chrono/core/ChMatrix.h"

using namespace std;
using namespace chrono;

int main( void ) 
{
    ChPardisoProjectEngine ppengine(ChPardisoProjectEngine::parproj_SYM::UNSYMMETRIC);
    
    /* Matrix data. */
    int    n = 8;
    int    ia[ 9] = { 0, 4, 7, 9, 11, 12, 15, 17, 20 };
    int    ja[20] = { 0,    2,         5,  6, 
                         1, 2,     4,
                            2,             7,
                               3,       6,
                         1,
                            2,       5,    7,
                         1,             6,
                            2,          6, 7 };
    double  a[20] = { 7.0,      1.0,           2.0, 7.0, 
                     -4.0, 8.0,      2.0,
                           1.0,                     5.0,
                                7.0,           9.0,
                     -4.0,
                        7.0,           3.0,      8.0,
                    1.0,                    11.0,
                        -3.0,                2.0, 5.0 };

    ChSparseMatrix mmat;
    mmat.resize(8, 8);
    mmat.reserve(20);
    for (int row_sel = 0; row_sel<n; ++row_sel){
        for (int col_pt = ia[row_sel]; col_pt<ia[row_sel+1]; ++col_pt){
            mmat.insert(row_sel, ja[col_pt]) = a[col_pt];
        }
    }
    mmat.makeCompressed();

    ChVectorDynamic<> mb;
    mb.resize(mmat.rows());
    mb.fill(1.0);
    ChVectorDynamic<> mx;
    mx.resize(mmat.rows());
    mx.fill(999.9);

    ppengine.SetMatrix(mmat);
    ppengine.SetSolutionVector(mx);
    ppengine.SetRhsVector(mb);

    /* -------------------------------------------------------------------- */
    /*  .. pardiso_chk_matrix(...)                                          */
    /*     Checks the consistency of the given matrix.                      */
    /*     Use this functionality only for debugging purposes               */
    /* -------------------------------------------------------------------- */
    
    ppengine.CheckMatrix();

    /* -------------------------------------------------------------------- */
    /* ..  pardiso_chkvec(...)                                              */
    /*     Checks the given vectors for infinite and NaN values             */
    /*     Input parameters (see PARDISO user manual for a description):    */
    /*     Use this functionality only for debugging purposes               */
    /* -------------------------------------------------------------------- */

    ppengine.CheckRhsVectors();

    /* -------------------------------------------------------------------- */
    /* .. pardiso_printstats(...)                                           */
    /*    prints information on the matrix to STDOUT.                       */
    /*    Use this functionality only for debugging purposes                */
    /* -------------------------------------------------------------------- */

    ppengine.CheckMatrixStats();
    
    /* -------------------------------------------------------------------- */
    /* ..  Reordering and Symbolic Factorization. This step also allocates  */
    /*     all memory that is necessary for the factorization.              */
    /*     ppengine.PardisoProjectCall                                      */
    /* -------------------------------------------------------------------- */

    ppengine.PardisoProjectCall(ChPardisoProjectEngine::parproj_PHASE::ANALYZE);
    
    if (ppengine.GetLastError() != 0) {
        printf("\nERROR during symbolic factorization: %d", ppengine.GetLastError());
        exit(1);
    }
    printf("\nReordering completed ... ");
    printf("\nNumber of nonzeros in factors  = %d", ppengine.GetIPARM(17));
    printf("\nNumber of factorization MFLOPS = %d", ppengine.GetIPARM(18));
   
    /* -------------------------------------------------------------------- */
    /* ..  Numerical factorization.                                         */
    /* -------------------------------------------------------------------- */
    
    ppengine.SetIPARM(32, 1);
    ppengine.PardisoProjectCall(ChPardisoProjectEngine::parproj_PHASE::FACTORIZE);

    if (ppengine.GetLastError() != 0) {
        printf("\nERROR during numerical factorization: %d", ppengine.GetLastError());
        exit(2);
    }
    printf("\nFactorization completed...\n");
    std::cout << "Determinant is: " << ppengine.GetDPARM(32) << std::endl;

    /* -------------------------------------------------------------------- */
    /* ..  Back substitution and iterative refinement.                      */
    /* -------------------------------------------------------------------- */

    ppengine.PardisoProjectCall(ChPardisoProjectEngine::parproj_PHASE::SOLVE);

   
    if (ppengine.GetLastError() != 0) {
        printf("\nERROR during solution: %d", ppengine.GetLastError());
        exit(3);
    }

    printf("\nSolve completed... \n");

    ppengine.SetZeroIndexedFormat();

    std::cout << "mmat is:" << std::endl << mmat << std::endl;
    std::cout << "nnz is:" << mmat.nonZeros() << std::endl;
    std::cout << "rhs is:" << std::endl << mb << std::endl;
    std::cout << "sol is:" << std::endl << mx << std::endl;

    ppengine.PardisoProjectCall(ChPardisoProjectEngine::parproj_PHASE::END);

    return 0;
} 