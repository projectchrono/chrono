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
// Author: Milad Rakhsha
// =============================================================================
//
// Class for solving a linear linear system via iterative methods.//
// =============================================================================

#include <ctype.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <typeinfo>
#include "cublas_v2.h"
#include "cusparse_v2.h"

#include "chrono_fsi/sph/math/SphLinearSolverBiCGStab.h"

namespace chrono {
namespace fsi {
namespace sph {

void SphLinearSolverBiCGStab::Solve(int SIZE,
                                 int NNZ,
                                 Real* A,
                                 unsigned int* ArowIdx,
                                 unsigned int* AcolIdx,
                                 Real* x,
                                 Real* b) {
#ifndef CUDART_VERSION
    #error CUDART_VERSION Undefined!
#elif (CUDART_VERSION == 11000)

    Real *r, *rh, *p, *ph, *v, *s, *t, *Ac;

    cudaMalloc((void**)&r, sizeof(Real) * SIZE);
    cudaMalloc((void**)&rh, sizeof(Real) * SIZE);
    cudaMalloc((void**)&p, sizeof(Real) * SIZE);
    cudaMalloc((void**)&ph, sizeof(Real) * SIZE);
    cudaMalloc((void**)&v, sizeof(Real) * SIZE);
    cudaMalloc((void**)&s, sizeof(Real) * SIZE);
    cudaMalloc((void**)&t, sizeof(Real) * SIZE);

    cudaMalloc((void**)&Ac, sizeof(Real) * NNZ);
    cudaDeviceSynchronize();

    cudaMemset((void*)r, 0, sizeof(Real) * SIZE);
    cudaMemset((void*)rh, 0, sizeof(Real) * SIZE);
    cudaMemset((void*)p, 0, sizeof(Real) * SIZE);
    cudaMemset((void*)ph, 0, sizeof(Real) * SIZE);
    cudaMemset((void*)v, 0, sizeof(Real) * SIZE);
    cudaMemset((void*)s, 0, sizeof(Real) * SIZE);
    cudaMemset((void*)t, 0, sizeof(Real) * SIZE);

    cudaMemset((void*)Ac, 0, sizeof(Real) * NNZ);
    cudaDeviceSynchronize();

    //====== Get handle to the CUBLAS context ========
    cublasHandle_t cublasHandle = 0;
    cusparseHandle_t cusparseHandle = 0;

    cublasStatus_t cublasStatus;
    cublasCreate(&cublasHandle);
    cudaDeviceSynchronize();

    //====== Get handle to the CUSPARSE context ======
    cusparseStatus_t cusparseStatus;
    cusparseCreate(&cusparseHandle);
    cudaDeviceSynchronize();

    //===========================Incomplete-LU-Preconditioner=================================
    // Suppose that A is m x m sparse matrix represented by CSR format,
    // Assumption:
    // - (d_csrRowPtr, d_csrColInd, d_csrVal) is CSR of A on device memory,
    // copy A into Ac before factorizing A in place
    cublasDcopy(cublasHandle, NNZ, A, 1, Ac, 1);

    cusparseSpMatDescr_t descrA;
    cusparseDnVecDescr_t vecX, vecPh, vecS, vecV, vecT, vecR;
    cusparseMatDescr_t descr_M = 0;
    cusparseMatDescr_t descr_L = 0;
    cusparseMatDescr_t descr_U = 0;
    csrilu02Info_t info_M = 0;
    csrsv2Info_t info_L = 0;
    csrsv2Info_t info_U = 0;
    int pBufferSize_M;
    int pBufferSize_L;
    int pBufferSize_U;
    int pBufferSize;
    size_t bufferSize = 0;
    void* pBuffer = NULL;
    void* bufferX = NULL;
    void* bufferP = NULL;
    void* bufferS = NULL;
    int structural_zero;
    int numerical_zero;
    const cusparseSolvePolicy_t policy_M = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
    const cusparseSolvePolicy_t policy_L = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
    const cusparseSolvePolicy_t policy_U = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
    const cusparseOperation_t trans_A = CUSPARSE_OPERATION_NON_TRANSPOSE;
    const cusparseOperation_t trans_L = CUSPARSE_OPERATION_NON_TRANSPOSE;
    const cusparseOperation_t trans_U = CUSPARSE_OPERATION_NON_TRANSPOSE;

    // step 1: create a descriptor which contains
    // - matrix M is base-1
    // - matrix L is base-1
    // - matrix L is lower triangular
    // - matrix L has unit diagonal
    // - matrix U is base-1
    // - matrix U is upper triangular
    // - matrix U has non-unit diagonal
    cusparseCreateCsr(&descrA, SIZE, SIZE, NNZ, (int*)ArowIdx, (int*)AcolIdx, A, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                      CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F);

    cusparseCreateMatDescr(&descr_M);
    cusparseSetMatIndexBase(descr_M, CUSPARSE_INDEX_BASE_ZERO);
    cusparseSetMatType(descr_M, CUSPARSE_MATRIX_TYPE_GENERAL);

    cusparseCreateMatDescr(&descr_L);
    cusparseSetMatIndexBase(descr_L, CUSPARSE_INDEX_BASE_ZERO);
    cusparseSetMatType(descr_L, CUSPARSE_MATRIX_TYPE_GENERAL);
    cusparseSetMatFillMode(descr_L, CUSPARSE_FILL_MODE_LOWER);
    cusparseSetMatDiagType(descr_L, CUSPARSE_DIAG_TYPE_UNIT);

    cusparseCreateMatDescr(&descr_U);
    cusparseSetMatIndexBase(descr_U, CUSPARSE_INDEX_BASE_ZERO);
    cusparseSetMatType(descr_U, CUSPARSE_MATRIX_TYPE_GENERAL);
    cusparseSetMatFillMode(descr_U, CUSPARSE_FILL_MODE_UPPER);
    cusparseSetMatDiagType(descr_U, CUSPARSE_DIAG_TYPE_UNIT);

    // step 2: create an empty info structure
    // we need one info for csrilu02 and two info's for csrsv2
    cusparseCreateCsrilu02Info(&info_M);
    cusparseCreateCsrsv2Info(&info_L);
    cusparseCreateCsrsv2Info(&info_U);

    // step 3: query how much memory used in csrilu02 and csrsv2, and allocate the buffer
    cusparseDcsrilu02_bufferSize(cusparseHandle, SIZE, NNZ, descr_M, Ac, (int*)ArowIdx, (int*)AcolIdx, info_M,
                                 &pBufferSize_M);
    cusparseDcsrsv2_bufferSize(cusparseHandle, trans_L, SIZE, NNZ, descr_L, Ac, (int*)ArowIdx, (int*)AcolIdx, info_L,
                               &pBufferSize_L);
    cusparseDcsrsv2_bufferSize(cusparseHandle, trans_U, SIZE, NNZ, descr_U, Ac, (int*)ArowIdx, (int*)AcolIdx, info_U,
                               &pBufferSize_U);

    pBufferSize = std::max(pBufferSize_M, std::max(pBufferSize_L, pBufferSize_U));
    // pBuffer returned by cudaMalloc is automatically aligned to 128 bytes.
    cudaMalloc((void**)&pBuffer, pBufferSize);

    // step 4: perform analysis of incomplete Cholesky on M
    //         perform analysis of triangular solve on L
    //         perform analysis of triangular solve on U
    // The lower(upper) triangular part of M has the same sparsity pattern as L(U),
    // we can do analysis of csrilu0 and csrsv2 simultaneously.

    // analysis phase
    cusparseDcsrilu02_analysis(cusparseHandle, SIZE, NNZ, descr_M, Ac, (int*)ArowIdx, (int*)AcolIdx, info_M, policy_M,
                               pBuffer);
    cusparseStatus = cusparseXcsrilu02_zeroPivot(cusparseHandle, info_M, &structural_zero);
    if (CUSPARSE_STATUS_ZERO_PIVOT == cusparseStatus) {
        printf("A(%d,%d) is missing\n", structural_zero, structural_zero);
        exit(0);
    }
    cusparseStatus = cusparseXcsrilu02_zeroPivot(cusparseHandle, info_M, &numerical_zero);
    if (CUSPARSE_STATUS_ZERO_PIVOT == cusparseStatus) {
        printf("U(%d,%d) is zero\n", numerical_zero, numerical_zero);
    }
    // analysis phase for L and U solve
    cusparseDcsrsv2_analysis(cusparseHandle, trans_L, SIZE, NNZ, descr_L, Ac, (int*)ArowIdx, (int*)AcolIdx, info_L,
                             policy_L, pBuffer);

    cusparseDcsrsv2_analysis(cusparseHandle, trans_U, SIZE, NNZ, descr_U, Ac, (int*)ArowIdx, (int*)AcolIdx, info_U,
                             policy_U, pBuffer);
    cusparseStatus = cusparseXcsrsv2_zeroPivot(cusparseHandle, info_U, &numerical_zero);
    if (CUSPARSE_STATUS_ZERO_PIVOT == cusparseStatus) {
        printf("U(%d,%d) is zero\n", numerical_zero, numerical_zero);
    }
    // step 5: M = L * U
    cusparseStatus = cusparseDcsrilu02(cusparseHandle, SIZE, NNZ, descr_M, Ac, (int*)ArowIdx, (int*)AcolIdx, info_M,
                                       policy_M, pBuffer);
    if (cusparseStatus != CUSPARSE_STATUS_SUCCESS) {
        printf("look into ilu\n");
        if (cusparseStatus == CUSPARSE_STATUS_NOT_INITIALIZED)
            printf("not initialized\n");
        if (cusparseStatus == CUSPARSE_STATUS_ALLOC_FAILED)
            printf("alloc failed\n");
        if (cusparseStatus == CUSPARSE_STATUS_INVALID_VALUE)
            printf("invalid value\n");
        if (cusparseStatus == CUSPARSE_STATUS_INTERNAL_ERROR)
            printf("internal error\n");
        if (cusparseStatus == CUSPARSE_STATUS_MATRIX_TYPE_NOT_SUPPORTED)
            printf("matrix type\n");
        exit(0);
    } else
        printf("ilu success\n");
    cudaDeviceSynchronize();
    if (CUSPARSE_STATUS_ZERO_PIVOT == cusparseStatus) {
        printf("U(%d,%d) is zero\n", numerical_zero, numerical_zero);
    }

    //===========================Solution=====================================================
    Real rho = 1, rho_old = 1, beta = 1, alpha = 1, negalpha = -1, omega = 1, negomega = -1, temp = 1, temp2 = 1;
    Real nrmr = 0.0, nrmr0 = 0.0;
    Real zero = 0.0;
    Real one = 1.0;
    // negative one
    Real none = -1.0;

    // compute initial residual r0=b-Ax0 (using initial guess in x)
    // 1. get Ax0 - Dcsrmv, can be mitigated to cusparseSpMV() later
    // 2. get -Ax0 - scal by -1.0
    // 3. get b + (-Ax0) - axpy by 1.0
    // 4. get norm of r0 as a base line for determining convergence
    cusparseCreateDnVec(&vecX, SIZE, x, CUDA_R_64F);
    cusparseCreateDnVec(&vecR, SIZE, r, CUDA_R_64F);
    cusparseCreateDnVec(&vecPh, SIZE, ph, CUDA_R_64F);
    cusparseCreateDnVec(&vecV, SIZE, v, CUDA_R_64F);
    cusparseCreateDnVec(&vecS, SIZE, s, CUDA_R_64F);
    cusparseCreateDnVec(&vecT, SIZE, t, CUDA_R_64F);
    cusparseStatus = cusparseSpMV_bufferSize(cusparseHandle, trans_A, &one, descrA, vecX, &zero, vecR, CUDA_R_64F,
                                             CUSPARSE_MV_ALG_DEFAULT, &bufferSize);
    cudaMalloc((void**)&bufferX, bufferSize);
    cusparseStatus = cusparseSpMV(cusparseHandle, trans_A, &one, descrA, vecX, &zero, vecR, CUDA_R_64F,
                                  CUSPARSE_MV_ALG_DEFAULT, bufferX);
    cublasStatus = cublasDscal(cublasHandle, SIZE, &none, r, 1);

    cublasStatus = cublasDaxpy(cublasHandle, SIZE, &one, b, 1, r, 1);
    cublasStatus = cublasDnrm2(cublasHandle, SIZE, r, 1, &nrmr0);
    // copy residual r into rh
    cublasDcopy(cublasHandle, SIZE, r, 1, rh, 1);

    printf("nrmr0=%e\n", nrmr0);
    solver_status = 0;

    for (Iterations = 0; Iterations < max_iter; Iterations++) {
        if (nrmr > nrmr0)
            break;
        rho_old = rho;
        // compute rho_i
        cublasDdot(cublasHandle, SIZE, rh, 1, r, 1, &rho);

        // compute beta
        // Real rho_old_m = (rho_old < 1e-15) ? rho_old * 1e6 : rho_old;
        // Real rho_m = (rho_old < 1e-15) ? rho * 1e6 : rho;
        // Real omega_m = (omega < 1e-15) ? omega * 1e6 : omega;
        // Real alpha_m = (omega < 1e-15) ? alpha * 1e6 : alpha;

        beta = (rho / rho_old) * (alpha / omega);
        // p_i = r_{i-1} + beta * (p_{i-1} - omega_{i-1} * v_{i-1})
        cublasDaxpy(cublasHandle, SIZE, &negomega, v, 1, p, 1);
        cublasDscal(cublasHandle, SIZE, &beta, p, 1);
        cublasDaxpy(cublasHandle, SIZE, &one, r, 1, p, 1);
        // cublasDnrm2(cublasHandle, SIZE, p, 1, &nrmr);
        // printf("p is: %e\n", nrmr);
        // M p^hat = p
        cusparseDcsrsv2_solve(cusparseHandle, trans_L, SIZE, NNZ, &one, descr_L, Ac, (int*)ArowIdx, (int*)AcolIdx,
                              info_L, p, t, policy_L, pBuffer);
        cusparseDcsrsv2_solve(cusparseHandle, trans_U, SIZE, NNZ, &one, descr_U, Ac, (int*)ArowIdx, (int*)AcolIdx,
                              info_U, t, ph, policy_U, pBuffer);
        cudaDeviceSynchronize();

        // v = A p^hat
        cusparseStatus = cusparseSpMV_bufferSize(cusparseHandle, trans_A, &one, descrA, vecPh, &zero, vecV, CUDA_R_64F,
                                                 CUSPARSE_MV_ALG_DEFAULT, &bufferSize);
        cudaMalloc((void**)&bufferP, bufferSize);
        cusparseStatus = cusparseSpMV(cusparseHandle, trans_A, &one, descrA, vecPh, &zero, vecV, CUDA_R_64F,
                                      CUSPARSE_MV_ALG_DEFAULT, bufferP);

        // cublasDnrm2(cublasHandle, SIZE, t, 1, &nrmr);
        // printf("t is: %e\n", nrmr);
        // alpha = rho_i / (rh * v_i)
        cublasDdot(cublasHandle, SIZE, rh, 1, v, 1, &temp);
        if (isnan(temp))
            break;

        alpha = rho / temp;
        negalpha = -alpha;

        // compute x_i = x_{i-1} + alpha * p_i
        cublasDaxpy(cublasHandle, SIZE, &alpha, ph, 1, x, 1);

        // compute s = r_{i-1} - alpha * v_i, here can overwrite r_{i-1} by s
        // since it's an intermediate result
        cublasDaxpy(cublasHandle, SIZE, &negalpha, v, 1, r, 1);
        cublasDnrm2(cublasHandle, SIZE, r, 1, &nrmr);
        // if h is accurate enough, set x=h and exit
        if (nrmr < rel_res * nrmr0 || nrmr < abs_res) {
            residual = nrmr;
            solver_status = 1;
            printf("abs_res is %e, 2-norm of r is %e\n", abs_res, nrmr);
            // printf("relative residual is %f, absolute residual is %f\n", rel_res, residual);
            break;
        } else {
            solver_status = 0;
        }
        // M s^hat = r
        cusparseDcsrsv2_solve(cusparseHandle, trans_L, SIZE, NNZ, &one, descr_L, Ac, (int*)ArowIdx, (int*)AcolIdx,
                              info_L, r, t, policy_L, pBuffer);
        cusparseDcsrsv2_solve(cusparseHandle, trans_U, SIZE, NNZ, &one, descr_U, Ac, (int*)ArowIdx, (int*)AcolIdx,
                              info_U, t, s, policy_U, pBuffer);
        // t = A*s
        cusparseStatus = cusparseSpMV_bufferSize(cusparseHandle, trans_A, &one, descrA, vecS, &zero, vecT, CUDA_R_64F,
                                                 CUSPARSE_MV_ALG_DEFAULT, &bufferSize);
        cudaMalloc((void**)&bufferS, bufferSize);
        cusparseStatus = cusparseSpMV(cusparseHandle, trans_A, &one, descrA, vecS, &zero, vecT, CUDA_R_64F,
                                      CUSPARSE_MV_ALG_DEFAULT, bufferS);

        // cublasStatus = cublasDnrm2(cublasHandle, NNZ, Ac, 1, &nrmr);
        // omega_i = t * s / (t * t)
        cublasDdot(cublasHandle, SIZE, t, 1, r, 1, &temp);
        cublasDdot(cublasHandle, SIZE, t, 1, t, 1, &temp2);

        omega = temp / temp2;
        negomega = -omega;

        // x_i = x_{i-1} + omega_i * s^hat
        cublasDaxpy(cublasHandle, SIZE, &omega, s, 1, x, 1);

        // r_i = r_{i-1} - omega_i * t
        cublasDaxpy(cublasHandle, SIZE, &negomega, t, 1, r, 1);
        cublasDnrm2(cublasHandle, SIZE, r, 1, &nrmr);

        // if h is accurate enough, set x=h and exit
        if (nrmr < rel_res * nrmr0 || nrmr < abs_res) {
            residual = nrmr;
            solver_status = 1;
            printf("abs_res is %e, 2-norm of r is %e\n", abs_res, nrmr);
            // printf("relative residual is %f, absolute residual is %f\n", rel_res, residual);
            break;
        } else {
            solver_status = 0;
        }

        residual = nrmr;

        if (verbose)
            printf("Iterations=%d\t ||b-A*x||=%.4e\n", Iterations, nrmr);
        if (temp2 == 0.0) {
            printf("Zero (t * t)\n");
            break;
        }
    }

    cudaFree(r);
    cudaFree(rh);
    cudaFree(p);
    cudaFree(ph);
    cudaFree(s);
    cudaFree(v);
    cudaFree(t);
    cudaFree(Ac);
    // step 6: free resources
    cudaFree(pBuffer);
    cudaFree(bufferX);
    cudaFree(bufferP);
    cudaFree(bufferS);
    cusparseDestroyMatDescr(descr_M);
    cusparseDestroyMatDescr(descr_L);
    cusparseDestroyMatDescr(descr_U);
    cusparseDestroyCsrilu02Info(info_M);
    cusparseDestroyCsrsv2Info(info_L);
    cusparseDestroyCsrsv2Info(info_U);
#endif
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
