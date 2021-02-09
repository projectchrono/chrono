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

#include <typeinfo>
#include "cublas_v2.h"
#include "cusparse_v2.h"
#include "chrono_fsi/math/ChFsiLinearSolverGMRES.h"

namespace chrono {
namespace fsi {

void printMatrix(const double* A, int Ny, int Nx) {
    printf("Matrix A(%d,%d):\n", Ny, Nx);
    for (int i = 0; i < Ny; i++) {
        for (int j = 0; j < Nx; j++) {
            printf("%d,%d,%f  ", i, j, A[i * Nx + j]);
        }
        printf("\n");
    }
}

void GeneratePlaneRotation(const double& dx, const double& dy, double& cs, double& sn) {
    if (dx == double(0.0)) {
        cs = double(0.0);
        sn = double(1.0);
    } else {
        double scale = abs(dx) + abs(dy);
        double norm = scale * sqrt(abs(dx / scale) * abs(dx / scale) + abs(dy / scale) * abs(dy / scale));
        double alpha = dx / abs(dx);
        cs = abs(dx) / norm;
        sn = alpha * (dy) / norm;
    }
}

void ApplyPlaneRotation(double& dx, double& dy, const double& cs, const double& sn) {
    double temp = cs * dx + sn * dy;
    dy = -sn * dx + cs * dy;
    dx = temp;
}

void PlaneRotation(double* H, double* cs, double* sn, double* s, const int i, const int restart) {
    for (int k = 0; k < i; k++) {
        ApplyPlaneRotation(H[k * restart + i], H[(k + 1) * restart + i], cs[k], sn[k]);
    }
    GeneratePlaneRotation(H[i * restart + i], H[(i + 1) * restart + i], cs[i], sn[i]);
    ApplyPlaneRotation(H[i * restart + i], H[(i + 1) * restart + i], cs[i], sn[i]);
    ApplyPlaneRotation(s[i], s[i + 1], cs[i], sn[i]);
}

void ChFsiLinearSolverGMRES::Solve(int SIZE,
                                   int NNZ,
                                   double* A,
                                   unsigned int* ArowIdx,
                                   unsigned int* AcolIdx,
                                   double* x,
                                   double* b) {
#ifndef CUDART_VERSION
#error CUDART_VERSION Undefined!
#elif (CUDART_VERSION == 11000)

    restart = 10;
    cublasHandle_t cublasHandle = 0;
    cusparseHandle_t cusparseHandle = 0;
    cusparseDnVecDescr_t vecX, vecW, vecV0;
    cusparseSpMatDescr_t descrA;
    size_t bufferSize = 0;
    void *bufferX = NULL;
    void *bufferW = NULL;
    const cusparseOperation_t trans_A  = CUSPARSE_OPERATION_NON_TRANSPOSE;
    cusparseCreateCsr(&descrA, SIZE, SIZE, NNZ, (int*)ArowIdx, (int*)AcolIdx, A,
                      CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F);

    double *w, *v0, *V, *sDev, *H, *s, *cs, *sn;

    cudaMalloc((void**)&w, sizeof(double) * SIZE);
    cudaMalloc((void**)&v0, sizeof(double) * SIZE);
    cudaMalloc((void**)&V, sizeof(double) * (restart + 1) * SIZE);  // Arnoldi Matrix
    // Note that this is in fact the transpose of the matrix because of performance reasons
    cudaMalloc((void**)&sDev, sizeof(double) * (restart + 1));

    H = (double*)malloc(sizeof(double) * (restart + 1) * restart);
    s = (double*)malloc(sizeof(double) * (restart + 1));
    cs = (double*)malloc(sizeof(double) * restart);
    sn = (double*)malloc(sizeof(double) * restart);

    cudaDeviceSynchronize();

    //    cudaMemset((void*)x, 0.0, sizeof(double) * SIZE);

    cudaMemset((void*)w, 0, sizeof(double) * SIZE);
    cudaMemset((void*)v0, 0, sizeof(double) * SIZE);
    cudaMemset((void*)V, 0, sizeof(double) * (restart + 1) * SIZE);
    cudaMemset((void*)sDev, 0, sizeof(double) * (restart + 1));

    memset(H, 0, sizeof(double) * (restart + 1) * restart);
    memset(s, 0, sizeof(double) * (restart + 1));
    memset(cs, 0, sizeof(double) * restart);
    memset(sn,0, sizeof(double) * restart);

    cudaDeviceSynchronize();

    //====== Get handle to the CUBLAS context ========
    cublasStatus_t cublasStatus;
    cublasCreate(&cublasHandle);
    cudaDeviceSynchronize();

    //====== Get handle to the CUSPARSE context ======
    cusparseStatus_t cusparseStatus;
    cusparseCreate(&cusparseHandle);
    cudaDeviceSynchronize();

    //===========================Solution=====================================================
    double beta = 1.0, mOneOverBeta = 1, temp = 1, mtemp = -1, Hnew = 1, oneOverHnew = 1;
    double nrmr0 = 1e5, nrmr = 1e5, res = 1e5;
    double zero = 0.0;
    double one = 1.0;
    double mone = -1.0;
    cudaCheckError();

    for (Iterations = 0; Iterations < max_iter; Iterations++) {
        //        printf("-----1\n");
        // compute initial residual w=Ax0 (using initial guess in x)
        // w = A * x
        cusparseCreateDnVec(&vecX, SIZE, x, CUDA_R_64F);
        cusparseCreateDnVec(&vecW, SIZE, w, CUDA_R_64F);
        cusparseStatus = cusparseSpMV_bufferSize(cusparseHandle, trans_A, &one, descrA, vecX, &zero, vecW,
                     CUDA_R_64F, CUSPARSE_MV_ALG_DEFAULT, &bufferSize);
        cudaMalloc((void**)&bufferX, bufferSize);
        cusparseStatus = cusparseSpMV(cusparseHandle, trans_A, &one, descrA, vecX, &zero, vecW, 
                     CUDA_R_64F, CUSPARSE_MV_ALG_DEFAULT, bufferX);

        cublasDaxpy(cublasHandle, SIZE, &mone, b, 1, w, 1);  // w=w-b
        cublasDnrm2(cublasHandle, SIZE, w, 1, &beta);  // beta=norm(w,2)
        nrmr = beta;
        if (Iterations == 0)
            nrmr0 = beta;

        mOneOverBeta = -1.0 / beta;
        cublasDscal(cublasHandle, SIZE, &mOneOverBeta, w, 1);  // w=-w/beta
        cudaMemcpy(V, w, SIZE * sizeof(double), cudaMemcpyDeviceToDevice);
        memset(s, 0, sizeof(double) * (restart + 1));

        s[0] = beta;
        //        printf("-----2\n");

        for (size_t m = 0; m < restart; m++) {
            // v0=A*w
            cusparseCreateDnVec(&vecV0, SIZE, v0, CUDA_R_64F);
            cusparseStatus = cusparseSpMV_bufferSize(cusparseHandle, trans_A, &one, descrA, vecW, &zero, vecV0,
                     CUDA_R_64F, CUSPARSE_MV_ALG_DEFAULT, &bufferSize);
            cudaMalloc((void**)&bufferW, bufferSize);
            cusparseStatus = cusparseSpMV(cusparseHandle, trans_A, &one, descrA, vecW, &zero, vecV0, 
                     CUDA_R_64F, CUSPARSE_MV_ALG_DEFAULT, bufferW);

            cudaMemcpy(w, v0, SIZE * sizeof(double), cudaMemcpyDeviceToDevice);
            cublasDnrm2(cublasHandle, SIZE, w, 1, &temp);
            cudaDeviceSynchronize();
            //            printf("m=%d, temp=%f\n", m, temp);

            for (int k = 0; k < m; k++) {
                //  H(k,i) = <V(i+1),V(k)>
                cublasDdot(cublasHandle, SIZE, V + SIZE * k, 1, w, 1, &temp);
                cudaDeviceSynchronize();
                H[k * restart + m] = temp;
                mtemp = -temp;
                //                printf("%f ", temp);
                // V(i+1) -= H(k, i) * V(k)
                cublasDaxpy(cublasHandle, SIZE, &mtemp, V + SIZE * k, 1, w, 1);
                cudaDeviceSynchronize();
            }

            cublasDnrm2(cublasHandle, SIZE, w, 1, &Hnew);
            cudaDeviceSynchronize();
            H[(m + 1) * restart + m] = Hnew;  // H(i + 1, i)=norm(w,2)

            oneOverHnew = 1.0 / Hnew;
            // V(i+1) = V(i+1) / H(i+1, i)
            cublasDscal(cublasHandle, SIZE, &oneOverHnew, w, 1);
            cudaDeviceSynchronize();
            cudaMemcpy(V + (m + 1) * SIZE, w, sizeof(double) * SIZE, cudaMemcpyDeviceToDevice);
            PlaneRotation(H, cs, sn, s, (int)m, restart);
            res = abs(s[m + 1]);
        }

        //        printMatrix(H, restart + 1, restart);

        //        printf("-----3\n");

        // solve upper triangular system in place
        for (int j = restart - 1; j >= 0; j--) {
            s[j] /= H[j * restart + j];
            // S(0:j) = s(0:j) - s[j] H(0:j,j)
            for (int k = j - 1; k >= 0; k--) {
                s[k] -= H[k * restart + j] * s[j];
            }
        }

        // update the solution
        // copy s to gpu
        cudaMemcpy(sDev, s, (restart + 1), cudaMemcpyHostToDevice);
        cudaCheckError();

        // x= V(1:N,0:i)*s(0:i)+x
        for (size_t j = 0; j < restart; j++) {
            // x = x + s[j] * V(j)
            cublasDaxpy(cublasHandle, SIZE, s + j, V + j * SIZE, 1, x, 1);
            cudaDeviceSynchronize();
        }

        if (verbose)
            printf("Iterations=%d\t ||b-A*x||=%.4e\n", Iterations, nrmr);
        //        && Iterations > 3

        if (res < abs_res * 1e-14) {
            // There is not much that can be extract from the system
            if ((nrmr < rel_res * nrmr0 || nrmr < abs_res)) {
                solver_status = 1;
            } else {
                solver_status = 0;
            }
            residual = nrmr;
            break;
        }
        residual = nrmr;
    }

    //cusparseDestroySolveAnalysisInfo(info_l);
    //cusparseDestroySolveAnalysisInfo(info_u);
    cudaFree(w);
    cudaFree(v0);
    cudaFree(V);
    cudaFree(sDev);

    free(H);
    free(s);
    free(cs);
    free(sn);
#endif
}

}  // end namespace fsi
}  // namespace chrono
