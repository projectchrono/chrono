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
// Authors: Alessandro Tasora 
// =============================================================================

#ifndef CHKRYLOVSCHUREIG_H
#define CHKRYLOVSCHUREIG_H

#include "chrono_modal/ChApiModal.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChDirectSolverLScomplex.h"
#include <complex>

namespace chrono {
namespace modal {


/// Generic A*x callback.
/// Inherit from this class to define how to compute A*x
class ChApiModal callback_Ax {
public:
    // Inherit this. It must compute A*x. How A is stored (full, sparse, factorised, etc. ) is up to you.
    virtual void compute(ChVectorDynamic<std::complex<double>>& A_x,    ///< output: result of A*x. Assume already sized.
        const ChVectorDynamic<std::complex<double>>& x          ///< input:  x in A*x
    ) {};

    virtual ~callback_Ax() {};
};


//
// Ready-to-use callbacks for the most relevant cases: sparse matrices, generalized, shift&invert:
//

/// The callback to be used for "A*x"  where for shift&invert is: A = (As - sigma Bs)\Bs , 
/// so A*x = (As - sigma Bs)\(Bs*x), just like a linear system with coefficient matrix (As - sigma Bs) and known rhs  Bs*x
/// 
class ChApiModal callback_Ax_sparse_shiftinvert : public callback_Ax {
    public:
    callback_Ax_sparse_shiftinvert(
            const chrono::ChSparseMatrix& mA,
            const chrono::ChSparseMatrix& mB,
            double shift,
            ChDirectSolverLS* mlinear_solver = 0   ///< optional direct solver/factorization. Default is ChSolverSparseQR
        );

	~callback_Ax_sparse_shiftinvert();

	void compute(chrono::ChVectorDynamic<std::complex<double>>& A_x,     ///< output: result of A*x
		const chrono::ChVectorDynamic<std::complex<double>>& x  ///< input:  x in A*x
	) override;

	ChDirectSolverLS* linear_solver;
	chrono::ChSparseMatrix Bd;
	float default_solver;

    double sigma;
};

/// The callback to be used for "A*x"  where for shift&invert is: A = (As - sigma Bs)\Bs , with COMPLEX sigma shift,
/// so A*x = (As - sigma Bs)\(Bs*x), just like a linear system with coefficient matrix (As - sigma Bs) and known rhs  Bs*x
/// 
class ChApiModal callback_Ax_sparse_complexshiftinvert : public callback_Ax {
    public:
    callback_Ax_sparse_complexshiftinvert(
            const chrono::ChSparseMatrix& mA,
            const chrono::ChSparseMatrix& mB,
            std::complex<double> shift,
            ChDirectSolverLScomplex* mlinear_solver = 0  ///< optional direct solver/factorization. Default is ChSolverSparseComplexQR
        );

    ~callback_Ax_sparse_complexshiftinvert();

    void compute(   chrono::ChVectorDynamic<std::complex<double>>& A_x,     ///< output: result of A*x
                    const chrono::ChVectorDynamic<std::complex<double>>& x  ///< input:  x in A*x
    ) override;

	ChDirectSolverLScomplex* linear_solver;
    Eigen::SparseMatrix<std::complex<double>, Eigen::ColMajor> Bd;
	float default_solver;
    std::complex<double> sigma;
};



/// Compute (complex) eigenvalues and eigenvectors
/// using the Krylov-Schur algorithm. 
/// Adapted from Matlab code at  https://github.com/dingxiong/KrylovSchur
/// Use one of the chrono::modal::callback_Ax provided above depending on the type
/// of problem that you must solve.

class  ChApiModal ChKrylovSchurEig  {
public:
    
    // Construct and compute results, returned in v and eig.
    ChKrylovSchurEig(
        ChMatrixDynamic<std::complex<double>>& v,   ///< output matrix with eigenvectors as columns, will be resized 
        ChVectorDynamic<std::complex<double>>& eig, ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
        bool& isC,									///< 0 = k-th eigenvalue is real, 1= k-th and k-th+1 are complex conjugate pairs
        bool& flag,									///< 0 = has converged, 1 = hasn't converged 
        int& nc,									///< number of converged eigenvalues
        int& ni,									///< number of used iterations
        callback_Ax* Ax_function,					///< compute the A*x operation, assuming standard eigenvalue problem A*v=lambda*v. 
        ChVectorDynamic<std::complex<double>>& v1,  ///< initial approx of eigenvector, or random
        const int n,								///< size of A
        const int k,								///< number of needed eigenvalues
        const int m,								///< Krylov restart threshold (largest dimension of krylov subspace)
        const int maxIt,							///< max iteration number
        const double tol							///< tolerance
    );
};


}  // end namespace modal

}  // end namespace chrono

#endif
