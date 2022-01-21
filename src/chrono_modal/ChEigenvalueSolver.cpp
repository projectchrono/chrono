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



#include <Spectra/KrylovSchurGEigsSolver.h>
#include <Spectra/SymGEigsSolver.h>
#include <Spectra/SymGEigsShiftSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/MatOp/SparseGenMatProd.h>
#include <Spectra/MatOp/SparseRegularInverse.h>
#include <Spectra/GenEigsBase.h>

using namespace Spectra;

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
        int n_modes)            ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.)
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

	int m = 2 * n_modes >= 30 ? 2 * n_modes : 30;  // minimum subspace size   //**TO DO*** make parametric?
	if (m > n_vars + n_constr-1)
		m = n_vars + n_constr-1;

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
 	KrylovSchurGEigsShiftInvert<OpType, BOpType> eigen_solver(op, Bop, n_modes, m, sigma);  //*** OK EIGVECTS, WRONG EIGVALS REQUIRE eigen_values(i) = (1.0 / eigen_values(i)) + sigma;

	eigen_solver.init();

	int nconv = eigen_solver.compute(SortRule::LargestMagn, max_iterations, tolerance);

	if (this->verbose) {
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
			GetLog() << " n_modes = " << n_modes  << "\n";
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
		eigen_values(i) = (1.0 / eigen_values(i)) + sigma;
	}

    // Return values
    V.setZero(M.rows(), n_modes);
    eig.setZero(n_modes);
	freq.setZero(n_modes);

    for (int i = 0; i < n_modes; i++) {
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
        int n_modes)            ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.)
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

	int m = 2 * n_modes >= 20 ? 2 * n_modes : 20;  // minimum subspace size   
	if (m > n_vars + n_constr)
		m = n_vars + n_constr;

	// Construct matrix operation objects using the wrapper classes
    using OpType = SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
    using BOpType = SparseSymMatProd<double>;
    OpType op(A, B);
    BOpType Bop(B);
 
	// The Lanczos solver, using the shift and invert mode
    SymGEigsShiftSolver<OpType, BOpType, GEigsMode::ShiftInvert> eigen_solver(op, Bop, n_modes, m, sigma); 

	eigen_solver.init();

	int nconv = eigen_solver.compute(SortRule::LargestMagn, max_iterations, tolerance);

	if (this->verbose) {
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
			GetLog() << " n_modes = " << n_modes  << "\n";
			GetLog() << " n_vars  = " << n_vars << "\n";
			GetLog() << " n_constr= " << n_constr << "\n";
		}
	}
	Eigen::VectorXcd eigen_values = eigen_solver.eigenvalues();
	Eigen::MatrixXcd eigen_vectors = eigen_solver.eigenvectors();

	// Return values
    V.setZero(M.rows(), n_modes);
    eig.setZero(n_modes);
	freq.setZero(n_modes);

    for (int i = 0; i < n_modes; i++) {
        V.col(i) = eigen_vectors.col(i).head(n_vars);  // store only displacement part of eigenvector, no constraint part
        V.col(i).normalize(); // check if necessary or already normalized
        eig(i) = eigen_values(i);
		freq(i) = (1.0 / CH_C_2PI) * sqrt(-eig(i).real());
    }

    return true;
}



//-------------------------------------------------------------------------------------------------------------------

bool ChQuadraticEigenvalueSolverNullspaceDirect::Solve(const ChSparseMatrix& M, const ChSparseMatrix& R, const ChSparseMatrix& K, const ChSparseMatrix& Cq, 
                                                        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix with eigenvectors as columns, will be resized
                                                        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
														ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
														ChVectorDynamic<double>& damping_ratio,  ///< output vector with n damping rations r=damping/critical_damping.
                                                        int n_modes)
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
    V.setZero(M.rows(), n_modes);
    eig.setZero(n_modes);
	freq.setZero(n_modes);
	damping_ratio.setZero(n_modes);

    for (int i = 0; i < n_modes; i++) {
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


bool ChQuadraticEigenvalueSolverKrylovSchur::Solve(const ChSparseMatrix& M, const ChSparseMatrix& R, const ChSparseMatrix& K, const ChSparseMatrix& Cq, 
                                                        ChMatrixDynamic<std::complex<double>>& V,   ///< output matrix with eigenvectors as columns, will be resized
                                                        ChVectorDynamic<std::complex<double>>& eig, ///< output vector with eigenvalues (real part not zero if some damping), will be resized 
														ChVectorDynamic<double>& freq,				///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
														ChVectorDynamic<double>& damping_ratio,		///< output vector with n damping rations r=damping/critical_damping.
                                                        int n_modes)
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

	int n_computed_eigs = 2 * n_modes;
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
	KrylovSchurGEigsShiftInvert<OpType, BOpType> eigen_solver(op, Bop, n_computed_eigs, m, sigma);   //***TODO**** wait that SPECTRA will have KrylovSchur implemented for the COMPLEX case
	
	eigen_solver.init();

	int nconv = eigen_solver.compute(SortRule::LargestMagn, this->max_iterations, this->tolerance);
    int niter = eigen_solver.num_iterations();
    int nops  = eigen_solver.num_operations();

	if (verbose) {
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
			GetLog() << " n_modes = " << n_modes  << "\n";
			GetLog() << " n_eigs  = " << n_modes*2 << "\n";
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
        eigen_values(i) = (1.0 / eigen_values(i)) + this->sigma;
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
    V.setZero(M.rows(), n_modes);
    eig.setZero(n_modes);
	freq.setZero(n_modes);
	damping_ratio.setZero(n_modes);

    for (int i = 0; i < n_modes; i++) {
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


}  // end namespace modal

}  // end namespace chrono
