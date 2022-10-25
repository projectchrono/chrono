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


#include "chrono_modal/ChKrylovSchurEig.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChDirectSolverLScomplex.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Eigenvalues>

#include <numeric>


using namespace Eigen;

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using SpMatrix = Eigen::SparseMatrix<double>;


namespace chrono {

namespace modal {

// CALLABACK ready-to-use:

callback_Ax_sparse_shiftinvert::callback_Ax_sparse_shiftinvert(
            const chrono::ChSparseMatrix& mA,
            const chrono::ChSparseMatrix& mB,
            double shift, 
            ChDirectSolverLS* mlinear_solver  ///< optional direct solver/factorization. Default is ChSolverSparseQR
        ) 
        : Bd(mB), sigma(shift), linear_solver(mlinear_solver) {

	if (!linear_solver) {
		linear_solver = new ChSolverSparseQR(); // NOTE! QR, not ChSolverSparseLU, as Eigen Sparse LU does not work well with RowMajor ordering!
		default_solver = true;
	}
	else {
		default_solver = false;
	}
			
	linear_solver->A() = (mA -shift * mB);
	linear_solver->SetupCurrent(); // factorize
}

callback_Ax_sparse_shiftinvert::~callback_Ax_sparse_shiftinvert() {
	if (default_solver)
		delete linear_solver;
}

void callback_Ax_sparse_shiftinvert::compute(chrono::ChVectorDynamic<std::complex<double>>& A_x,     ///< output: result of A*x
                const chrono::ChVectorDynamic<std::complex<double>>& x  ///< input:  x in A*x
                )  {

    std::complex<double> c1(0, 1.0);
	linear_solver->b() = Bd * x.real();
	linear_solver->SolveCurrent();
	A_x = linear_solver->x();
	linear_solver->b() = Bd * x.imag();
	linear_solver->SolveCurrent();
	A_x += c1*linear_solver->x();
};


//----------------

 callback_Ax_sparse_complexshiftinvert::callback_Ax_sparse_complexshiftinvert(
            const chrono::ChSparseMatrix& mA,
            const chrono::ChSparseMatrix& mB,
            std::complex<double> shift,
            ChDirectSolverLScomplex* mlinear_solver  ///< optional direct solver/factorization. Default is ChSolverSparseComplexQR
        ) 
        :  Bd(mB.cast<std::complex<double>>()), sigma(shift), linear_solver(mlinear_solver) {

	if (!linear_solver) {
		linear_solver = new ChSolverSparseComplexQR(); 
		default_solver = true;
	}
	else {
		default_solver = false;
	}
			
	linear_solver->A() = (mA.cast<std::complex<double>>() - (shift * mB.cast<std::complex<double>>()));
	linear_solver->Setup(); // factorize
}

callback_Ax_sparse_complexshiftinvert::~callback_Ax_sparse_complexshiftinvert() {
	if (default_solver)
		delete linear_solver;
}

void callback_Ax_sparse_complexshiftinvert::compute(chrono::ChVectorDynamic<std::complex<double>>& A_x,     ///< output: result of A*x
                const chrono::ChVectorDynamic<std::complex<double>>& x  ///< input:  x in A*x
                )  {
	linear_solver->Solve(Bd * x);
	A_x = linear_solver->x();
};






//-------------------------------------------------------------------------------------------------------------------
//
// TEST CODE HERE: ADAPT THE MATLAB CODE OF https://github.com/dingxiong/KrylovSchur
// 

 void ordschur(ChMatrixDynamic<std::complex<double>>& U, ChMatrixDynamic<std::complex<double>>& T, ChVectorDynamic<bool>& select)
{
    using std::swap;

    // build permutation vector
    ChVectorDynamic<int> permutation(select.size());
    Index ind = 0;
    for (Index j = 0; j < select.size(); j++) {
        if (select(j)) {
            permutation(j) = ind;
            ind++;
        }
    }
    for (Index j = 0; j < select.size(); j++) {
        if (!select(j)) {
            permutation(j) = ind;
            ind++;
        }
    }

    for (Index i = 0; i < permutation.size() - 1; i++) {
        Index j;
        for (j = i; j < permutation.size(); j++) {
            if (permutation(j) == i)
                break;
        }
        eigen_assert(permutation(j) == i);
        for (Index k = j - 1; k >= i; k--) {
            Eigen::JacobiRotation<std::complex<double>> rotation;
            rotation.makeGivens(T(k, k + 1), T(k + 1, k + 1) - T(k, k));
            T.applyOnTheLeft(k, k + 1, rotation.adjoint());
            T.applyOnTheRight(k, k + 1, rotation);
            U.applyOnTheRight(k, k + 1, rotation);
            swap(permutation.coeffRef(k), permutation.coeffRef(k + 1));
        }
    }
}


// test the convergence of the ith eigenvalue in Krylov-Schur iteration
int testConverge(
	ChMatrixDynamic<std::complex<double>>&H, 
	int k, 
	int i, 
	double tol) {
	/*
	% H is the hessenberg matrix after truncation
	% The test rule is 
	%    | b_i | < max( ||H(1:k, 1:k)||_F * epsilon, tol * | \lambda_i | ) 
	% Return:
	% flag - 1 : not converge
	% 1 : real eigenvalue converges
	% 2 : complex eigenvalue pair converges
	%
	% if i < k, we test whether the eigenvalue is real or complex
	% if i = k, we assume ith eigenvalue is real.
	*/

	int flag = -1;
	double delta;

    double epsilon = 2e-16;
	if (i < k) {
		delta = pow(H(i-1, i-1).real() - H(i, i).real(),2) + 4 * H(i-1, i-1).imag() * H(i, i).imag(); // (H(i, i) - H(i+1, i+1))^2 + 4 * H(i+1, i) * H(i, i+ 1);
	}
	else {
		delta = 1;
	}
    
	if (delta > 0) {                     // real case
		if (abs(H(k, i-1)) < std::max(H.norm() * epsilon, abs(H(i-1, i-1)) * tol)) {
			flag = 1;
		}
		else {
			flag = -1;
		}
	}
	else {                               // complex case
		std::complex<double> lambda(H(i - 1, i - 1).real() + H(i, i).real(), sqrt(-delta)); 
		lambda *= 0.5;
		if (abs(H(k, i-1)) < std::max(H.norm() * epsilon, abs(lambda) * tol)) {
			flag = 2;
		}
		else {
			flag = -1;
		}
	}
	return flag;
}


void truncateKrylov(
	ChMatrixDynamic<std::complex<double>>&Q, 
	ChMatrixDynamic<std::complex<double>>&H, 
	const int k, 
	const int m) {

	auto Qo = Q;
	auto Ho = H;
	Q.setZero(Qo.rows(), k + 1);
	Q << Qo(Eigen::all, seq(0, k - 1)), Qo(Eigen::all, m); //Q = [Q(:, 1 : k),   Q(:, m + 1)];
	
	H.setZero(k + 1, k);
	H << Ho(seq(0, k-1), seq(0, k-1)), Ho(m, seq(0,k-1));				 //H = [H(1:k, 1:k); H(m + 1, 1:k)]; 
}


// Perform Schur decompostion on A and put the needed k eigenvalues
// on the upper left block.
void sortSchur(
	ChMatrixDynamic<std::complex<double>>& US, 
	ChMatrixDynamic<std::complex<double>>& TS, 
	bool& isC, 
	const ChMatrixDynamic<std::complex<double>>& A, 
	const int k)
{
	// This function assumes that eigenvalues are reordered by their 
	// magnitudes. You can change this functions accordingly to obtain
	// eigenvalues that you need.
    
	//[U, T] = schur(A, 'real');
    //es = ordeig(T);     
    //[esp, ix] = sort(abs(es), 'descend');

	Eigen::ComplexSchur<ChMatrixDynamic<std::complex<double>>> mschur(A, true);
	const ChMatrixDynamic<std::complex<double>>& U = mschur.matrixU();
	const ChMatrixDynamic<std::complex<double>>& T = mschur.matrixT();
	//TODO: maybe call computeFromHessenberg?


	ChVectorDynamic<std::complex<double>> es = T.diagonal();


	std::vector<int> ix(es.size());
	std::iota(ix.begin(), ix.end(), 0);
	std::sort(ix.begin(), ix.end(), [&](int a, int b) {return std::abs(es[a]) > std::abs(es[b]);});
	//Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm;
	//perm.indices() = Eigen::Map<Eigen::ArrayXi>(ix.data(), ix.size());

    // judge the k-th and (k+1)-th eigenvalues are conjugate complex
    // pair or not.
    int k1 = ix[k];
    int k2 = ix[k+1];
    // here must use k2 not k1+1 because k1+1 maybe out of bounds
    // Also, there is a degeneracy problem.
	// Alex: since  we did a Eigen::ComplexSchur(), then all eigens are on the diagonal as complexes, not 2x2 diag blocks, so do instead: 
	double delta = pow(T(k1, k1).real() - T(k2, k2).real(),2) + 4 * T(k1, k1).imag() * T(k2, k2).imag();
	if ((k2 - k1 == 1) && (delta < 0)) {
		isC = 1; //DARIO: using ComplexSchur I assume we should never hit this branch
	}
	else {
		isC = 0;
	}

	// Modify the following reordering to pick different eigenvalues
	ChVectorDynamic<bool> select(es.size());
	select.setZero(es.size());		// select = zeros(length(es), 1);
	//for (int i=0; i<k+isC; ++i)
	//	select(ix[i]) = true;		// select(ix(1:k+isC)) = true;
     for (int i=0; i<k; ++i)
    	select(ix[i]) = true;		// select(ix(1:k+isC)) = true;
	US = U;
	TS = T;
	ordschur(US, TS, select);		// the ordshur() has no equivalent in EIGEN! Use own ordschur() 
	// TODO: can US and TS be reordered in-place?
}

// Expand Krylov subspace.
// The function contructs the sk + 1, sk + 2, ..., ek_th column of Q while Q(sk) is only read;
// It builds also H(0:ek, sk:ex-1)
void expandKrylov(
	ChMatrixDynamic<std::complex<double>>& Q,   ///< orthonormal matrix with dimension [n x k+1] or [n x k+2]
	ChMatrixDynamic<std::complex<double>>& H,   ///< `Hessenberg' matrix with dimension [k+1 x k] or [k+2 x k+1]
	callback_Ax* Ax_function, /// void (*Ax_function)(ChVectorDynamic<std::complex<double>>& A_x, const ChVectorDynamic<std::complex<double>>& x),  ///< compute the A*x operation, for standard eigenvalue problem A*v=lambda*v. 
	int sk, // start index
	int ek  // end index
) {

	//TODO: could we avoid resizing?
	Q.conservativeResize(NoChange, ek + 1);
    H.conservativeResizeLike(Eigen::MatrixXd::Zero(ek + 1, ek));


	for (int k = sk; k < ek; ++k) {
		ChVectorDynamic<std::complex<double>> v(Q.rows());
		Ax_function->compute(v, Q.col(k));  // v = Ax(Q(:, k));
		ChVectorDynamic<std::complex<double>> w = Q.leftCols(k+1).adjoint()  * v;  //w = Q(:, 1:k)' * v;
		v = v - Q.leftCols(k+1) * w;     // v = v - Q(:, 1 : k) * w;
		ChVectorDynamic<std::complex<double>> w2 = Q.leftCols(k+1).adjoint() * v; //  w2 = Q(:, 1 : k)' * v;            // double normalize
		v = v - Q.leftCols(k+1) * w2;    // v = v - Q(:, 1 : k) * w2;
		w = w + w2;
		double nv = v.norm();
		
		Q.col(k+1)  = v / nv;            // Q(:, k + 1) = v / nv;
		
		H(seq(0, k), k) = w;      // H(1:k+1, k) = [w; nv]; // right column of H
        H(k + 1, k) = nv;
	}
}

// Perform a Krylov-Schur decomposition to find invariant subspace of
// matrix A.
// A * Q(:, 1:k+isC) = Q(:, 1:k+isC) * H(1:k+isC, 1:k+isC)
void KrylovSchur(
	ChMatrixDynamic<std::complex<double>>& Q,   ///< orthonormal matrix with dimension [n x k+1] or [n x k+2]
	ChMatrixDynamic<std::complex<double>>& H,   ///< `Hessenberg' matrix with dimension [k+1 x k] or [k+2 x k+1]
	bool& isC,									///< 0 = k-th eigenvalue is real, 1= k-th and k-th+1 are complex conjugate pairs
	bool& flag,									///< 0 = has converged, 1 = hasn't converged 
	int& nc,									///< number of converged eigenvalues
	int& ni,									///< number of used iterations
	callback_Ax* Ax_function, /// void (*Ax_function)(ChVectorDynamic<std::complex<double>>& A_x, const ChVectorDynamic<std::complex<double>>& x),  ///< compute the A*x operation, for standard eigenvalue problem A*v=lambda*v. 
	const ChVectorDynamic<std::complex<double>>& v1,  ///< initial approx of eigenvector, or random
	const int n,								///< size of A
	const int k,								///< number of needed eigenvalues
	const int m,								///< Krylov restart threshold (largest dimension of krylov subspace)
	const int maxIt,							///< max iteration number
	const double tol							///< tolerance
	)
{
    Q.setZero(n, m+1); 
    H.setZero(m+1, m);
	Q.col(0) = v1.normalized();
    int p = 1;                         // converge test position
    isC = 0;                           // complex flag
	
    // initialize stage
    expandKrylov(Q, H, Ax_function, 0, k);
    
	// note on matrix slicing: 
	//   Eigen c++:                 equivalent to       Matlab:
	// P.block(i, j, rows, cols)                    P(i+1 : i+rows, j+1 : j+cols)
    // P.block(p-1, q-1, m-(p-1), n-(q-1))          P(p   : m,        q : n     )  
	// P(seq(p-1,m-1), seq(q-1,n-1))                P(p   : m,        q : n     )
	
    // iteration stage
    int i = 0;
	while ((i < maxIt) && (p <= k))
	{
		i = i + 1;

		// expand stage 
		expandKrylov(Q, H, Ax_function, k + isC, m);
		ChMatrixDynamic<std::complex<double>> U;
		ChMatrixDynamic<std::complex<double>> T;
		sortSchur(U, T, isC, H(seq(p - 1, m - 1), seq(p - 1, m - 1)), k - p + 1);  // matlab: H.block(p:m, p:m) 
		H(seq(p - 1, m - 1), seq(p - 1, m - 1)) = T;  // H(p:m, p:m) = T;
		H(seq(0, p - 1 - 1), seq(p - 1, m - 1)) = H(seq(0, p - 1 - 1), seq(p - 1, m - 1)) * U; // H(1:p-1, p:m) = H(1:p-1, p:m) * U;
		Q(Eigen::all, seq(p - 1, m - 1)) = Q(Eigen::all, seq(p - 1, m - 1)) * U; //Q(:, p:m) = Q(:, p:m) * U;
		H.row(m)(seq(p - 1, m - 1)) = H(m, m - 1) * U.bottomRows(1); //H(m+1, p:m) = H(m+1, m) * U(end, :);    
		//disp('err'); disp(Ax(Q(:, 1:m)) - Q(:, 1:m+1) * H(1:m+1, 1:m)); 
		//disp('Q'); disp(Q); disp('H'); disp(H);

		// truncate stage
		truncateKrylov(Q, H, k + isC, m);
		// disp(Ax(Q(:, 1:k+isC)) - Q(:, 1:k+1+isC) * H(1:k+1+isC, 1:k+isC));

		// test convergence
		bool check = true;
		while (check) {
			int result = testConverge(H, k + isC, p, tol);
			if (result == 1 || result == 2) {
				p = p + result;
				if (p > k) {
					check = false;
				}
			}
			else {
				check = false;
			}
		}

	}
    
    // return the convergence infomation
    ni = i;
	if (p > k) {
		flag = 0; // converges
		nc = k + isC;
	}
	else {
		flag = 1; // not converge
		nc = p - 1;
	}
}

// calculate (complex) eigenvalues and eigenvectors of matrix A.
// using the Krylov-Schur algorithm
ChKrylovSchurEig::ChKrylovSchurEig(
	ChMatrixDynamic<std::complex<double>>& v,   ///< output matrix with eigenvectors as columns, will be resized 
	ChVectorDynamic<std::complex<double>>& eig, ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
	bool& isC,									///< 0 = k-th eigenvalue is real, 1= k-th and k-th+1 are complex conjugate pairs
	bool& flag,									///< 0 = has converged, 1 = hasn't converged 
	int& nc,									///< number of converged eigenvalues
	int& ni,									///< number of used iterations
	callback_Ax* Ax_function, /// (*Ax_function)(ChVectorDynamic<std::complex<double>>& A_x, const ChVectorDynamic<std::complex<double>>& x),  ///< compute the A*x operation, for standard eigenvalue problem A*v=lambda*v. 
	ChVectorDynamic<std::complex<double>>& v1,  ///< initial approx of eigenvector, or random
	const int n,								///< size of A
	const int k,								///< number of needed eigenvalues
	const int m,								///< Krylov restart threshold (largest dimension of krylov subspace)
	const int maxIt,							///< max iteration number
	const double tol							///< tolerance
	)
{
	ChMatrixDynamic<std::complex<double>> Q; // orthonormal matrix with dimension [n x k+1] or [n x k+2]
	ChMatrixDynamic<std::complex<double>> H; // `Hessenberg' matrix with dimension [k+1 x k] or [k+2 x k+1]
	KrylovSchur(Q, H, isC, flag, nc, ni,  Ax_function, v1, n, k, m, maxIt, tol);
	Eigen::ComplexEigenSolver<ChMatrixDynamic<std::complex<double>>> es (H.topLeftCorner(k+isC, k+isC) ,true);
	eig = es.eigenvalues();
	v = Q.leftCols(k + isC) * es.eigenvectors();
}





}  // end namespace modal

}  // end namespace chrono
