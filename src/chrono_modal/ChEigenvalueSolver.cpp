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


#include "chrono_modal/ChEigenvalueSolver.h"
#include "chrono/core/ChMathematics.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Eigenvalues>

#include <numeric>

#include <Spectra/KrylovSchurGEigsSolver.h>
#include <Spectra/SymGEigsSolver.h>
#include <Spectra/SymGEigsShiftSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/MatOp/SparseGenMatProd.h>
#include <Spectra/MatOp/SparseRegularInverse.h>
#include <Spectra/GenEigsBase.h>

using namespace Spectra;
using namespace Eigen;

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using SpMatrix = Eigen::SparseMatrix<double>;


namespace chrono {

namespace modal {


// This is an helper class for using Krylov-Schur eigen solver also with the shift&invert mode, 
// because at the moment it is not yet available in Spectra.
template <typename OpType, typename BOpType>
class KrylovSchurGEigsShiftInvert :
    public KrylovSchurGEigsBase<SymGEigsShiftInvertOp<OpType, BOpType>, BOpType>
{
private:
    using Scalar = typename OpType::Scalar;
    using Index = Eigen::Index;
    using Array = Eigen::Array<Scalar, Eigen::Dynamic, 1>;

    using ModeMatOp = SymGEigsShiftInvertOp<OpType, BOpType>;
    using Base = KrylovSchurGEigsBase<ModeMatOp, BOpType>;
    using Base::m_nev;
    using Base::m_ritz_val;

    const Scalar m_sigma;

    // Set shift and forward
    static ModeMatOp set_shift_and_move(ModeMatOp&& op, const Scalar& sigma)
    {
        op.set_shift(sigma);
        return std::move(op);
    }

    // First transform back the Ritz values, and then sort
    void sort_ritzpair(SortRule sort_rule) override
    {
        // The eigenvalues we get from the iteration is nu = 1 / (lambda - sigma)
        // So the eigenvalues of the original problem is lambda = 1 / nu + sigma
        m_ritz_val.head(m_nev).array() = Scalar(1) / m_ritz_val.head(m_nev).array() + m_sigma;
        Base::sort_ritzpair(sort_rule);
    }

public:
    KrylovSchurGEigsShiftInvert(OpType& op, BOpType& Bop, Index nev, Index ncv, const Scalar& sigma) :
        Base(set_shift_and_move(ModeMatOp(op, Bop), sigma), Bop, nev, ncv),
        m_sigma(sigma)
    {}
};




// Assembly a sparse matrix by bordering square H with rectangular Cq.
//    HCQ = [ H  Cq' ]
//          [ Cq  0  ]
void sparse_assembly_2x2symm(Eigen::SparseMatrix<double, Eigen::ColMajor, int>& HCQ,       ///< resulting square sparse matrix (column major)
	const ChSparseMatrix& H,   ///< square sparse H matrix, n_v x n_v
	const ChSparseMatrix& Cq)  ///< rectangular  sparse Cq  n_c x n_v
{
	int n_v   = H.rows();
	int n_c   = Cq.rows();
	HCQ.resize(n_v + n_c, n_v + n_c);
	HCQ.reserve(H.nonZeros() + 2 * Cq.nonZeros());
    HCQ.setZero();

	for (int k=0; k<H.outerSize(); ++k)
		for (ChSparseMatrix::InnerIterator it(H,k); it; ++it) {
			HCQ.insert(it.row(),it.col()) = it.value();
        }

	for (int k=0; k<Cq.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(Cq, k); it; ++it) {
            HCQ.insert(it.row() + n_v, it.col()) = it.value(); // insert Cq
            HCQ.insert(it.col(), it.row() + n_v) = it.value(); // insert Cq'
        }

    // This seems necessary in Release mode
    HCQ.makeCompressed();

    //***NOTE*** 
    // for some reason the HCQ matrix created via .insert() or .elementRef() or triplet insert, is 
    // corrupt in Release mode, not in Debug mode. However, when doing a loop like the one below,
    // it repairs it. 
    // ***TODO*** avoid this bad hack and find the cause of the release/debug difference.
    for (int k = 0; k < HCQ.rows(); ++k) {
        for (int j = 0; j < HCQ.cols(); ++j) {
            auto foo = HCQ.coeffRef(k, j);
            //GetLog() << HCQ.coeffRef(k,j) << " ";
        }
    }
}



bool ChGeneralizedEigenvalueSolverKrylovSchur::Solve(const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
        const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
        const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
        ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
        ChEigenvalueSolverSettings settings  ///< optional: settings for the solver, or n. of desired lower eigenvalues. If =0, return all eigenvalues.
) const           
{
	// Assembly the A and B for the generalized constrained eigenvalue problem. 
	// Note that those sparse matrices must be column-major for better compatibility with Spectra.
	int n_vars   = M.rows();
	int n_constr = Cq.rows();

	// A  =  [ -K   -Cq' ]
	//       [ -Cq    0  ]
	Eigen::SparseMatrix<double> A(n_vars + n_constr, n_vars + n_constr);
	sparse_assembly_2x2symm(A, -K, -Cq);

	// B  =  [  M     0  ]
	//       [  0     0  ]
	Eigen::SparseMatrix<double> B(n_vars + n_constr, n_vars + n_constr);
	ChSparseMatrix O(n_constr, n_vars); O.setZero();
	sparse_assembly_2x2symm(B, M, O);

	int m = 2 * settings.n_modes >= 30 ? 2 * settings.n_modes : 30;  // minimum subspace size   //**TO DO*** make parametric?
	if (m > n_vars + n_constr-1)
		m = n_vars + n_constr-1;
	if (m <= settings.n_modes)
		m = settings.n_modes+1;

	// Construct matrix operation objects using the wrapper classes
	using OpType = SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
    using BOpType = SparseSymMatProd<double>;
    OpType op(A, B);
    BOpType Bop(B);

	// Dump data for test. ***TODO*** remove when well tested
	if (false)
	{
		ChStreamOutAsciiFile fileA("dump_modal_A.dat");
		fileA.SetNumFormat("%.12g");
		StreamOUTdenseMatlabFormat(ChMatrixDynamic<>(A), fileA);
		ChStreamOutAsciiFile fileB("dump_modal_B.dat");
		fileB.SetNumFormat("%.12g");
		StreamOUTdenseMatlabFormat(ChMatrixDynamic<>(B), fileB);
	}

	// The Krylov-Schur solver, using the shift and invert mode:
 	KrylovSchurGEigsShiftInvert<OpType, BOpType> eigen_solver(op, Bop, settings.n_modes, m, settings.sigma);  //*** OK EIGVECTS, WRONG EIGVALS REQUIRE eigen_values(i) = (1.0 / eigen_values(i)) + sigma;

	eigen_solver.init();

	int nconv = eigen_solver.compute(SortRule::LargestMagn, settings.max_iterations, settings.tolerance);

	if (settings.verbose) {
		if (eigen_solver.info() != CompInfo::Successful)
		{
			GetLog() << "KrylovSchurGEigsSolver FAILED. \n";
			if (eigen_solver.info() == CompInfo::NotComputed) GetLog() << " Error: not computed. \n";
			if (eigen_solver.info() == CompInfo::NotConverging) GetLog() << " Error: not converging. \n";
			if (eigen_solver.info() == CompInfo::NumericalIssue) GetLog() << " Error: numerical issue. \n";
			GetLog() << " nconv  = " << nconv << "\n";
			GetLog() << " niter  = " << eigen_solver.num_iterations() << "\n";
			GetLog() << " nops   = " << eigen_solver.num_operations() << "\n";
			return false;
		}
		else
		{
			GetLog() << "KrylovSchurGEigsSolver successfull. \n";
			GetLog() << " nconv   = " << nconv << "\n";
			GetLog() << " niter   = " << eigen_solver.num_iterations() << "\n";
			GetLog() << " nops    = " << eigen_solver.num_operations() << "\n";
			GetLog() << " n_modes = " << settings.n_modes  << "\n";
			GetLog() << " n_vars  = " << n_vars << "\n";
			GetLog() << " n_constr= " << n_constr << "\n";
		}
	}
	Eigen::VectorXcd eigen_values = eigen_solver.eigenvalues();
	Eigen::MatrixXcd eigen_vectors = eigen_solver.eigenvectors();

	// ***HACK***
	// Correct eigenvals for shift-invert because KrylovSchurGEigsShiftInvert does not take care of it.
	// This should be automatically done by KrylovSchurGEigsShiftInvert::sort_ritz_pairs() at the end of compute(), 
	// but at the moment such sort_ritz_pairs() is not called by the base KrylovSchurGEigsBase, differently from SymGEigsShiftSolver, for example.
	for (int i = 0; i < eigen_values.rows() ; ++i)
	{
		eigen_values(i) = (1.0 / eigen_values(i)) + settings.sigma;
	}

    // Return values
    V.setZero(M.rows(), settings.n_modes);
    eig.setZero(settings.n_modes);
	freq.setZero(settings.n_modes);

    for (int i = 0; i < settings.n_modes; i++) {
        V.col(i) = eigen_vectors.col(i).head(n_vars);  // store only displacement part of eigenvector, no constraint part
        V.col(i).normalize(); // check if necessary or already normalized
        eig(i) = eigen_values(i);
		freq(i) = (1.0 / CH_C_2PI) * sqrt(-eig(i).real());
    }

    return true;
}


bool ChGeneralizedEigenvalueSolverLanczos::Solve(const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
        const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
        const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
        ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
        ChEigenvalueSolverSettings settings ///< optional: settings for the solver, or n. of desired lower eigenvalues. If =0, return all eigenvalues.) 
)  const
{
	// Assembly the A and B for the generalized constrained eigenvalue problem. 
	// Note that those sparse matrices must be column-major for better compatibility with Spectra.
	int n_vars   = M.rows();
	int n_constr = Cq.rows();
 
	// A  =  [ -K   -Cq' ]
	//       [ -Cq    0  ]
	Eigen::SparseMatrix<double, Eigen::ColMajor, int> A(n_vars + n_constr, n_vars + n_constr);
	sparse_assembly_2x2symm(A, -K, -Cq);

	// B  =  [  M     0  ]
	//       [  0     0  ]
	Eigen::SparseMatrix<double, Eigen::ColMajor, int> B(n_vars + n_constr, n_vars + n_constr);
	ChSparseMatrix O(n_constr, n_vars); O.setZero();
	sparse_assembly_2x2symm(B, M, O);

	int m = 2 * settings.n_modes >= 20 ? 2 * settings.n_modes : 20;  // minimum subspace size   
	if (m > n_vars + n_constr -1)
		m = n_vars + n_constr -1;
	if (m <= settings.n_modes)
		m = settings.n_modes+1;

	// Construct matrix operation objects using the wrapper classes
    using OpType = SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
    using BOpType = SparseSymMatProd<double>;
    OpType op(A, B);
    BOpType Bop(B);
 
	// The Lanczos solver, using the shift and invert mode
    SymGEigsShiftSolver<OpType, BOpType, GEigsMode::ShiftInvert> eigen_solver(op, Bop, settings.n_modes, m, settings.sigma); 

	eigen_solver.init();

	int nconv = eigen_solver.compute(SortRule::LargestMagn, settings.max_iterations, settings.tolerance);

	if (settings.verbose) {
		if (eigen_solver.info() != CompInfo::Successful)
		{
			GetLog() << "Lanczos eigenvalue solver FAILED. \n";
			if (eigen_solver.info() == CompInfo::NotComputed) GetLog() << " Error: not computed. \n";
			if (eigen_solver.info() == CompInfo::NotConverging) GetLog() << " Error: not converging. \n";
			if (eigen_solver.info() == CompInfo::NumericalIssue) GetLog() << " Error: numerical issue. \n";
			GetLog() << " nconv  = " << nconv << "\n";
			GetLog() << " niter  = " << eigen_solver.num_iterations() << "\n";
			GetLog() << " nops   = " << eigen_solver.num_operations()  << "\n";
			return false;
		}
		else
		{
			GetLog() << "Lanczos eigenvalue solver successfull. \n";
			GetLog() << " nconv   = " << nconv << "\n";
			GetLog() << " niter   = " << eigen_solver.num_iterations() << "\n";
			GetLog() << " nops    = " << eigen_solver.num_operations() << "\n";
			GetLog() << " n_modes = " << settings.n_modes  << "\n";
			GetLog() << " n_vars  = " << n_vars << "\n";
			GetLog() << " n_constr= " << n_constr << "\n";
		}
	}
	Eigen::VectorXcd eigen_values = eigen_solver.eigenvalues();
	Eigen::MatrixXcd eigen_vectors = eigen_solver.eigenvectors();

	// Return values
    V.setZero(M.rows(), settings.n_modes);
    eig.setZero(settings.n_modes);
	freq.setZero(settings.n_modes);

    for (int i = 0; i < settings.n_modes; i++) {
        V.col(i) = eigen_vectors.col(i).head(n_vars);  // store only displacement part of eigenvector, no constraint part
        V.col(i).normalize(); // check if necessary or already normalized
        eig(i) = eigen_values(i);
		freq(i) = (1.0 / CH_C_2PI) * sqrt(-eig(i).real());
    }

    return true;
}



int ChModalSolveUndamped::Solve(
	const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
	const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
	const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
	ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
	ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
	ChVectorDynamic<double>& freq   ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
) const
{
	int found_eigs = 0;
	V.resize(0, 0);
	eig.resize(0);
	freq.resize(0);

	// for each freq_spans finds the closest modes to i-th input frequency:
	for (int i = 0; i < this->freq_spans.size(); ++i) {

		int nmodes_goal_i = this->freq_spans[i].nmodes;
		double sigma_i = -pow(this->freq_spans[i].freq * CH_C_2PI, 2); // sigma for shift&invert, as lowest eigenvalue, from Hz info

		ChMatrixDynamic<std::complex<double>> V_i;
		ChVectorDynamic<std::complex<double>> eig_i;
		ChVectorDynamic<double> freq_i;
		
		V_i.setZero(M.rows(), nmodes_goal_i);
		eig_i.setZero(nmodes_goal_i);
		freq_i.setZero(nmodes_goal_i);

		ChEigenvalueSolverSettings settings_i (nmodes_goal_i, this->max_iterations, this->tolerance, this->verbose, sigma_i);

		if (!this->msolver.Solve(M, K, Cq, V_i, eig_i, freq_i, settings_i))
			return found_eigs;

		// append to list of results

		int nmodes_out_i = eig_i.size();

		// Sort modes by frequencies if not exactly in increasing order. Some solver sometime fail at this.
		std::vector<int> order(nmodes_out_i);
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](int a, int b) {
			return freq_i[a] < freq_i[b];
		});
		Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm; 
		perm.indices() =  Eigen::Map<Eigen::ArrayXi>(order.data(), order.size()); 
		V_i = V_i * perm;
		eig_i = perm * eig_i;
		freq_i = perm * freq_i;

		// avoid overlap when multiple shifts were used, and too close.. If is it may happen that the lowest eigvals of 
		// some shift are smaller than the highest of the previous shift.
		int i_nodes_notoverlap = nmodes_out_i;
		if (freq.size() > 0) {
			double upper_freq = freq[freq.size()-1];
			for (int j = 0; j < nmodes_out_i; ++j)
				if (freq_i[j] < upper_freq)
					i_nodes_notoverlap--;
		}

		if (i_nodes_notoverlap) {
			V.conservativeResize(M.rows(), V.cols() + i_nodes_notoverlap);
			eig.conservativeResize(eig.size() + i_nodes_notoverlap);
			freq.conservativeResize(freq.size() + i_nodes_notoverlap);
			V.rightCols(i_nodes_notoverlap) = V_i;
			eig.tail(i_nodes_notoverlap) = eig_i;
			freq.tail(i_nodes_notoverlap) = freq_i;
			found_eigs = eig.size();
		}
	}
	return found_eigs;
}



//-------------------------------------------------------------------------------------------------------------------

bool ChQuadraticEigenvalueSolverNullspaceDirect::Solve(const ChSparseMatrix& M, const ChSparseMatrix& R, const ChSparseMatrix& K, const ChSparseMatrix& Cq, 
                                                        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix with eigenvectors as columns, will be resized
                                                        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
														ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
														ChVectorDynamic<double>& damping_ratio,  ///< output vector with n damping rations r=damping/critical_damping.
                                                        ChEigenvalueSolverSettings settings ) const
{
    // The folowing is adapted from the former implementation of Peng Chao

	// Compute the null space of the Cq matrix
    ChMatrixDynamic<> Cq_full = Cq; // because the fullPivLu is not available for sparse matrix in Eigen
	Eigen::MatrixXd Cq_null_space = Cq_full.fullPivLu().kernel();
	Eigen::MatrixXd M_hat = Cq_null_space.transpose() * M * Cq_null_space;
	Eigen::MatrixXd K_hat = Cq_null_space.transpose() * K * Cq_null_space;
	Eigen::MatrixXd R_hat = Cq_null_space.transpose() * R * Cq_null_space;
	
	// frequency-shift，Solve the matrix singular problem - it is not needed now, you can set it to 0
    double freq_shift = 0; // shift value, can take any value 
	Eigen::MatrixXd M_bar = M_hat;
	Eigen::MatrixXd K_bar = pow(freq_shift, 2) * M_hat + freq_shift * R_hat + K_hat;
	Eigen::MatrixXd R_bar = 2 * freq_shift * M_hat + R_hat;
	Eigen::MatrixXd M_bar_inv = M_bar.inverse();  // performance warning! dense inverse matrix! Only small sizes should be used.
	
	// Generate the A matrix of the state equation, whose eigenvalues ​​are the modal frequencies
	int dim = M_bar.rows();
	Eigen::MatrixXd A_tilde(2 * dim, 2 * dim);  
	A_tilde << Eigen::MatrixXd::Zero(dim, dim), Eigen::MatrixXd::Identity(dim, dim),
		       -M_bar_inv * K_bar             , -M_bar_inv * R_bar;
	
	// Call EIGEN3, dense matrix to directly solve the eigenvalues ​​and eigenvectors
	// NOTE：EIGEN3 has a very fast calculation speed in release mode, and a very slow calculation speed in debug mode
	Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(A_tilde);

	Eigen::VectorXcd eigen_values = eigen_solver.eigenvalues() + freq_shift;
	Eigen::MatrixXcd eigen_vectors = eigen_solver.eigenvectors();
 
    class eig_vect_and_val
    {
    public:
        Eigen::VectorXcd eigen_vect;
        std::complex<double> eigen_val;

        bool operator < (const eig_vect_and_val& str) const {
            return (eigen_val.imag() < str.eigen_val.imag());
        }
    };

	std::vector<eig_vect_and_val> all_eigen_values_and_vectors;
	for (int i = 0; i < eigen_values.size(); i++) {
        eig_vect_and_val vec_and_val{ eigen_vectors.col(i), eigen_values(i) };
        all_eigen_values_and_vectors.push_back(vec_and_val);
	}

	// sort
	std::sort(all_eigen_values_and_vectors.begin(), all_eigen_values_and_vectors.end());   // sort by imaginary part

	// organize and return results
	int middle_number = (int)(all_eigen_values_and_vectors.size() / 2);  // The eigenvalues ​​filtered out are conjugate complex roots, just take half
	int DOF_counts = (int)((all_eigen_values_and_vectors.at(0)).eigen_vect.rows()/2);  // The number of degrees of freedom in the model, used when extracting the mode shape. 
    
    // Return values

	int nmodes = settings.n_modes;

	// if nmodes==0, then compute all eigs: 
	if (nmodes == 0)
       nmodes = M.rows() - Cq.rows();

    // cannot use more modes than n. of dofs, if so, clamp
    nmodes = ChMin(nmodes, M.rows() - Cq.rows());

    V.setZero(M.rows(), nmodes);
    eig.setZero(nmodes);
	freq.setZero(nmodes);
	damping_ratio.setZero(nmodes);

    for (int i = 0; i < nmodes; i++) {
        int i_half = middle_number + i; // because the n.of eigenvalues is double (conjugate pairs), so just use the 2nd half after sorting
        auto mv = all_eigen_values_and_vectors.at(i_half).eigen_vect.head(DOF_counts);
        auto mvhns = Cq_null_space * mv;

        V.col(i) = mvhns; 
        V.col(i).normalize(); // check if necessary or already normalized
        eig(i) = all_eigen_values_and_vectors.at(i_half).eigen_val;
		freq(i) = (1.0 / CH_C_2PI) * (std::abs(eig(i))); // undamped freq.
		damping_ratio(i) = -eig(i).real() / std::abs(eig(i));
    }

    return true;
}



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
KrylovSchurEig::KrylovSchurEig(
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

// 
//-------------------------------------------------------------------------------------------------------------------


bool ChQuadraticEigenvalueSolverKrylovSchur::Solve(const ChSparseMatrix& M, const ChSparseMatrix& R, const ChSparseMatrix& K, const ChSparseMatrix& Cq,
	ChMatrixDynamic<std::complex<double>>& V,   ///< output matrix with eigenvectors as columns, will be resized
	ChVectorDynamic<std::complex<double>>& eig, ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
	ChVectorDynamic<double>& freq,				///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
	ChVectorDynamic<double>& damping_ratio,		///< output vector with n damping rations r=damping/critical_damping.
	ChEigenvalueSolverSettings settings) const
{
	//*****WARNING*****  THIS SHOULD NOT BE USED: KrylovSchurGEigsShiftInvert NOT YET WORKING FOR COMPLEX Re Im EIGENVALUES!
	throw (ChException("SOLVER ChQuadraticEigenvalueSolverKrylovSchur NOT YET WORKING. TO BE COMPLETED."));

	// Generate the A and B in state space
	int n_vars   = M.rows();
	int n_constr = Cq.rows();

	//***TO DO*** avoid Ad and Bd...
	// In fact here we create a temporary couple of _dense_ matrices Ad and Bd, just to exploit the easy << operator, 
	// then later copied to sparse matrices A and B. But sparse matrices A and B should be used from the beginning 
	// to avoid the dense Ad and Bd!!! (This is not done right now just because Eigen does not offer easy block-initialization for sparse matrices).
	ChMatrixDynamic<> Ad(2 * n_vars + n_constr, 2 * n_vars + n_constr);
	ChMatrixDynamic<> Bd(2 * n_vars + n_constr, 2 * n_vars + n_constr);
	ChMatrixDynamic<> Md = M;
	ChMatrixDynamic<> Rd = R;
	ChMatrixDynamic<> Kd = K;
	ChMatrixDynamic<> Cqd = Cq;

	// A  =  [  0     I     0 ]
	//       [ -K    -R  -Cq' ]
	//       [ -Cq    0     0 ]
	Ad << Eigen::MatrixXd::Zero    (n_vars,  n_vars), Eigen::MatrixXd::Identity(n_vars,  n_vars), Eigen::MatrixXd::Zero(n_vars,   n_constr),
		 -Kd                                        , -Rd                                       , -Cqd.transpose(),
		 -Cqd                                       , Eigen::MatrixXd::Zero(n_constr,n_vars)    , Eigen::MatrixXd::Zero(n_constr, n_constr);

	// B  =  [  I     0     0 ]
	//       [  0     M     0 ]
	//       [  0     0     0 ]
	Bd << Eigen::MatrixXd::Identity(n_vars,  n_vars), Eigen::MatrixXd::Zero(n_vars,  n_vars), Eigen::MatrixXd::Zero(n_vars,   n_constr),
		  Eigen::MatrixXd::Zero    (n_vars,  n_vars), Md                                    , Eigen::MatrixXd::Zero(n_vars,   n_constr),
		  Eigen::MatrixXd::Zero    (n_constr,n_vars), Eigen::MatrixXd::Zero(n_constr,n_vars), Eigen::MatrixXd::Zero(n_constr, n_constr);

	Eigen::SparseMatrix<double> A(2 * n_vars + n_constr, 2 * n_vars + n_constr);
	Eigen::SparseMatrix<double> B(2 * n_vars + n_constr, 2 * n_vars + n_constr);
	A = Ad.sparseView();
	B = Bd.sparseView();

	int n_computed_eigs = 2 * settings.n_modes;
	int m = 2 * n_computed_eigs >= 30 ? 2 * n_computed_eigs : 30;  // minimum subspace size   //**TO DO*** make parametric
	if (m > 2*n_vars + n_constr)
		m = 2*n_vars + n_constr;
	
	// Construct matrix operation objects using the wrapper classes
	//using OpType = SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
    //using BOpType = SparseSymMatProd<double>;
	using OpType = SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
    using BOpType = SparseSymMatProd<double>;
    OpType op(A, B);
    BOpType Bop(B);

	// The complex Krylov-Schur solver, using the shift and invert mode:
	KrylovSchurGEigsShiftInvert<OpType, BOpType> eigen_solver(op, Bop, n_computed_eigs, m, settings.sigma);   //***TODO**** wait that SPECTRA will have KrylovSchur implemented for the COMPLEX case
	
	eigen_solver.init();

	int nconv = eigen_solver.compute(SortRule::LargestMagn, settings.max_iterations, settings.tolerance);
    int niter = eigen_solver.num_iterations();
    int nops  = eigen_solver.num_operations();

	if (settings.verbose) {
		if (eigen_solver.info() != CompInfo::Successful)
		{
			GetLog() << "KrylovSchurGEigsShiftInvert FAILED. \n";
			if (eigen_solver.info() == CompInfo::NotComputed) GetLog() << " Error: not computed. \n";
			if (eigen_solver.info() == CompInfo::NotConverging) GetLog() << " Error: not converging. \n";
			if (eigen_solver.info() == CompInfo::NumericalIssue) GetLog() << " Error: numerical issue. \n";
			GetLog() << " nconv = " << nconv << "\n";
			GetLog() << " niter = " << niter << "\n";
			GetLog() << " nops  = " << nops  << "\n";
			
			return false;
		}
		else
		{
			GetLog() << "KrylovSchurGEigsShiftInvert successfull. \n";
			GetLog() << " nconv   = " << nconv << "\n";
			GetLog() << " niter   = " << niter << "\n";
			GetLog() << " nops    = " << nops  << "\n";
			GetLog() << " n_modes = " << settings.n_modes  << "\n";
			GetLog() << " n_eigs  = " << settings.n_modes*2 << "\n";
			GetLog() << " n_vars  = " << n_vars << "\n";
			GetLog() << " n_constr= " << n_constr << "\n";
		}
	}
	Eigen::VectorXcd eigen_values = eigen_solver.eigenvalues();
	Eigen::MatrixXcd eigen_vectors = eigen_solver.eigenvectors();
 
	// ***HACK***
	// Correct eigenvals for shift-invert because KrylovSchurGEigsShiftInvert does not take care of it.
	// This should be automatically done by KrylovSchurGEigsShiftInvert::sort_ritz_pairs() at the end of compute(), 
	// but at the moment such sort_ritz_pairs() is not called by the base KrylovSchurGEigsBase, differently from SymGEigsShiftSolver, for example.
	for (int i = 0; i < eigen_values.rows() ; ++i)
	{
        eigen_values(i) = (1.0 / eigen_values(i)) + settings.sigma;
	}

    class eig_vect_and_val
    {
    public:
        Eigen::VectorXcd eigen_vect;
        std::complex<double> eigen_val;

        bool operator < (const eig_vect_and_val& str) const {
            return (eigen_val.imag() < str.eigen_val.imag());
        }
    };

	std::vector<eig_vect_and_val> all_eigen_values_and_vectors;
	for (int i = 0; i < eigen_values.size(); i++) {
        eig_vect_and_val vec_and_val{ eigen_vectors.col(i), eigen_values(i) };
        all_eigen_values_and_vectors.push_back(vec_and_val);
		GetLog() << " Eig " << i << "= " << eigen_values(i).real() << "  " << eigen_values(i).imag() 
			<< "   freq= " << (1.0/CH_C_2PI)*std::abs(eigen_values(i)) << "\n";
	}

	// sort
	//std::sort(all_eigen_values_and_vectors.begin(), all_eigen_values_and_vectors.end());   // sort by imaginary part

	// organize and return results
	int middle_number = (int)(all_eigen_values_and_vectors.size() / 2);  // The eigenvalues ​​filtered out are conjugate complex roots, just take half

    // Return values
    V.setZero(M.rows(), settings.n_modes);
    eig.setZero(settings.n_modes);
	freq.setZero(settings.n_modes);
	damping_ratio.setZero(settings.n_modes);

    for (int i = 0; i < settings.n_modes; i++) {
        //int i_half = middle_number + i; // because the n.of eigenvalues is double (conjugate pairs), so just use the 2nd half after sorting
		int i_half = i;

        V.col(i) = all_eigen_values_and_vectors.at(i_half).eigen_vect.head(n_vars);  // store only displacement part of eigenvector, no speed part, no constraint part
        V.col(i).normalize(); // check if necessary or already normalized
        eig(i) = all_eigen_values_and_vectors.at(i_half).eigen_val;
		freq(i) = (1.0 / CH_C_2PI) * (std::abs(eig(i))); // undamped freq.
		damping_ratio(i) = -eig(i).real() / std::abs(eig(i));
    }

    return true;
}



//-------------------------------------------------------------------------------------


int ChModalSolveDamped::Solve(
	const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
	const ChSparseMatrix& R,  ///< input R matrix, n_v x n_v 
	const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
	const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
	ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
	ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
	ChVectorDynamic<double>& freq,   ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
	ChVectorDynamic<double>& damp_ratios         ///< output vector with n damping ratios, will be resized.
) const
{
	int found_eigs = 0;
	V.resize(0, 0);
	eig.resize(0);
	freq.resize(0);
	damp_ratios.resize(0);

	// for each freq_spans finds the closest modes to i-th input frequency:
	for (int i = 0; i < this->freq_spans.size(); ++i) {

		int nmodes_goal_i = this->freq_spans[i].nmodes;
		double sigma_i = -pow(this->freq_spans[i].freq * CH_C_2PI, 2); // sigma for shift&invert, as lowest eigenvalue, from Hz info
		//***TODO*** for the damped case, the sigma_i should be a *complex* shift, as w are on the imaginary axis.

		ChMatrixDynamic<std::complex<double>> V_i;
		ChVectorDynamic<std::complex<double>> eig_i;
		ChVectorDynamic<double> freq_i;
		ChVectorDynamic<double> damp_ratios_i;
		
		V_i.setZero(M.rows(), nmodes_goal_i);
		eig_i.setZero(nmodes_goal_i);
		freq_i.setZero(nmodes_goal_i);
		damp_ratios_i.setZero(nmodes_goal_i);

		ChEigenvalueSolverSettings settings_i (nmodes_goal_i, this->max_iterations, this->tolerance, this->verbose, sigma_i);

		if (!this->msolver.Solve(M, R, K, Cq, V_i, eig_i, freq_i, damp_ratios_i, settings_i))
			return found_eigs;

		// append to list of results
		
		int nmodes_out_i = eig_i.size();

		// Sort modes by eigenvalue imag part (or modulus(?)) if not exactly in increasing order. Some solver sometime fail at this.
		std::vector<int> order(nmodes_out_i);
		std::iota(order.begin(), order.end(), 0);
		std::sort(order.begin(), order.end(), [&](int a, int b) {
			//return std::abs(eig_i[a]) < std::abs(eig_i[b]);
			return eig_i[a].imag() < eig_i[b].imag();
		});
		Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm; 
		perm.indices() =  Eigen::Map<Eigen::ArrayXi>(order.data(), order.size()); 
		V_i = V_i * perm;
		eig_i = perm * eig_i;
		freq_i = perm * freq_i;
		damp_ratios_i = perm * damp_ratios_i;

		// avoid overlap when multiple shifts were used, and too close.. If is it may happen that the lowest eigvals of 
		// some shift are smaller than the highest of the previous shift.
		int i_nodes_notoverlap = nmodes_out_i;
		if (freq.size() > 0) {
			double upper_freq = freq[freq.size()-1];
			for (int j = 0; j < nmodes_out_i; ++j)
				if (freq_i[j] < upper_freq)
					i_nodes_notoverlap--;
		}

		if (i_nodes_notoverlap) {
			V.conservativeResize(M.rows(), V.cols() + i_nodes_notoverlap);
			eig.conservativeResize(eig.size() + i_nodes_notoverlap);
			freq.conservativeResize(freq.size() + i_nodes_notoverlap);
			damp_ratios.conservativeResize(damp_ratios.size() + i_nodes_notoverlap);
			V.rightCols(i_nodes_notoverlap) = V_i;
			eig.tail(i_nodes_notoverlap) = eig_i;
			freq.tail(i_nodes_notoverlap) = freq_i;
			damp_ratios.tail(i_nodes_notoverlap) = damp_ratios_i;
			found_eigs = eig.size();
		}
	}
	return found_eigs;
}





}  // end namespace modal

}  // end namespace chrono
