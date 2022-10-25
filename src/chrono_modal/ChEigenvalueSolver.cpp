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
#include "chrono_modal/ChKrylovSchurEig.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChDirectSolverLScomplex.h"

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




// 
//-------------------------------------------------------------------------------------------------------------------

ChQuadraticEigenvalueSolverKrylovSchur::ChQuadraticEigenvalueSolverKrylovSchur(ChDirectSolverLScomplex* mlinear_solver) {
	linear_solver = mlinear_solver;
}

bool ChQuadraticEigenvalueSolverKrylovSchur::Solve(const ChSparseMatrix& M, const ChSparseMatrix& R, const ChSparseMatrix& K, const ChSparseMatrix& Cq,
	ChMatrixDynamic<std::complex<double>>& V,   ///< output matrix with eigenvectors as columns, will be resized
	ChVectorDynamic<std::complex<double>>& eig, ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
	ChVectorDynamic<double>& freq,				///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
	ChVectorDynamic<double>& damping_ratio,		///< output vector with n damping rations r=damping/critical_damping.
	ChEigenvalueSolverSettings settings) const
{
	// Generate the A and B in state space
	int n_vars   = M.rows();
	int n_constr = Cq.rows();

	//***TO DO*** avoid Ad and Bd...
	// In fact here we create a temporary couple of _dense_ matrices Ad and Bd, just to exploit the easy << operator, 
	// then later copied to sparse matrices A and B. But sparse matrices A and B should be used from the beginning 
	// to avoid the dense Ad and Bd!!! (This is not done right now just because Eigen does not offer easy block-initialization for sparse matrices).
	// Hint: few lines above I implemented a sparse_assembly_2x2symm() helper function to do exactly this, so we need a sparse_assembly_3x3()...
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

	Eigen::SparseMatrix<double> As(2 * n_vars + n_constr, 2 * n_vars + n_constr);
	Eigen::SparseMatrix<double> Bs(2 * n_vars + n_constr, 2 * n_vars + n_constr);
	As = Ad.sparseView();
	Bs = Bd.sparseView();

	int n_computed_eigs = 2 * settings.n_modes;
	int m = 2 * n_computed_eigs >= 30 ? 2 * n_computed_eigs : 30;  // minimum subspace size   //**TO DO*** make parametric
	if (m > 2*n_vars + n_constr)
		m = 2*n_vars + n_constr;

	// Setup the Krylov Schur solver:
	ChVectorDynamic<std::complex<double>> eigen_values;
	ChMatrixDynamic<std::complex<double>> eigen_vectors;
	ChVectorDynamic<std::complex<double>> v1;
	v1.setRandom(Ad.cols()); // note: to make deterministic may be preceded by something like  std::srand((unsigned int)1234567);
	
	// because we need *complex* shift for focusing on some frequency, and settings.sigma is real, as in undamped case, so we do: 
	std::complex<double> complex_sigma(0, settings.sigma);

	// Setup the callback for matrix * vector
	callback_Ax_sparse_complexshiftinvert Ax_function3(
		As, 
		Bs, 
		complex_sigma, 
		this->linear_solver);

	bool isC, flag;
	int nconv, niter;
	ChKrylovSchurEig eigen_solver(
							eigen_vectors,   ///< output matrix with eigenvectors as columns, will be resized 
							eigen_values, ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
							isC,									///< 0 = k-th eigenvalue is real, 1= k-th and k-th+1 are complex conjugate pairs
							flag,									///< 0 = has converged, 1 = hasn't converged 
							nconv,									///< number of converged eigenvalues
							niter,									///< number of used iterations
							&Ax_function3,						///< callback for A*v
							v1,								///< initial approx of eigenvector, or random
							Ad.cols(),						///< size of A
							n_computed_eigs,				///< number of needed eigenvalues
							m,								///< Krylov restart threshold (largest dimension of krylov subspace)
							settings.max_iterations,		///< max iteration number
							settings.tolerance				///< tolerance
						);


	if (settings.verbose) {
		if (flag==1)
		{
			GetLog() << "KrylovSchurEig FAILED. \n";
			GetLog() << " shift   = (" << complex_sigma.real() << "," << complex_sigma.imag() << ")\n";
			GetLog() << " nconv = " << nconv << "\n";
			GetLog() << " niter = " << niter << "\n";	
			return false;
		}
		else
		{
			GetLog() << "KrylovSchurEig successfull. \n";
			GetLog() << " shift   = (" << complex_sigma.real() << "," << complex_sigma.imag() << ")\n";
			GetLog() << " nconv   = " << nconv << "\n";
			GetLog() << " niter   = " << niter << "\n";
		}
	}
 
	// Restore eigenvals, trasform back  from  shift-inverted problem to original problem:
	for (int i = 0; i < eigen_values.rows() ; ++i)
	{
        eigen_values(i) = (1.0 / eigen_values(i)) + complex_sigma;
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
	}

	// sort
	std::sort(all_eigen_values_and_vectors.begin(), all_eigen_values_and_vectors.end());   // sort by imaginary part

	if (settings.verbose)
		for (int i = 0; i < all_eigen_values_and_vectors.size(); i++) {
			GetLog() << "   sorted Eig " << i << "= " 
				<< all_eigen_values_and_vectors[i].eigen_val.real() << "  " 
				<< all_eigen_values_and_vectors[i].eigen_val.imag() 
				<< "   freq= " << (1.0/CH_C_2PI)*std::abs(all_eigen_values_and_vectors[i].eigen_val) << "\n";
		}

	// organize and return results
	int middle_number = (int)(all_eigen_values_and_vectors.size() / 2);  // The eigenvalues ​​filtered out are conjugate complex roots, just take half

    // Return values
    V.setZero(M.rows(), settings.n_modes);
    eig.setZero(settings.n_modes);
	freq.setZero(settings.n_modes);
	damping_ratio.setZero(settings.n_modes);

    for (int i = 0; i < settings.n_modes; i++) {
        int i_half = middle_number + i; // because the n.of eigenvalues is double (conjugate pairs), so just use the 2nd half after sorting

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
		double sigma_i = (this->freq_spans[i].freq * CH_C_2PI); // sigma for shift&invert, as lowest eigenvalue, from Hz info
		// Note, for the damped case, the sigma_i assumed as a *complex* shift, as w are on the imaginary axis.

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
