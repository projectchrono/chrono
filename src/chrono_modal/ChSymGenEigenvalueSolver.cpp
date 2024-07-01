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

#include <numeric>
#include <iomanip>

#include "chrono_modal/ChSymGenEigenvalueSolver.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChDirectSolverLScomplex.h"
#include "chrono/utils/ChConstants.h"
#include "chrono/core/ChMatrix.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <Spectra/KrylovSchurGEigsSolver.h>
#include <Spectra/SymGEigsSolver.h>
#include <Spectra/SymGEigsShiftSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/MatOp/SparseGenMatProd.h>
#include <Spectra/MatOp/SparseRegularInverse.h>
#include <Spectra/GenEigsBase.h>

using namespace Spectra;
using namespace Eigen;

namespace chrono {

namespace modal {

// This is an helper class for using Krylov-Schur eigen solver also with the shift&invert mode,
// because at the moment it is not yet available in Spectra.
template <typename OpType, typename BOpType>
class KrylovSchurGEigsShiftInvert : public KrylovSchurGEigsBase<SymGEigsShiftInvertOp<OpType, BOpType>, BOpType> {
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
    static ModeMatOp set_shift_and_move(ModeMatOp&& op, const Scalar& sigma) {
        op.set_shift(sigma);
        return std::move(op);
    }

    // First transform back the Ritz values, and then sort
    void sort_ritzpair(SortRule sort_rule) override {
        // The eigenvalues we get from the iteration is nu = 1 / (lambda - sigma)
        // So the eigenvalues of the original problem is lambda = 1 / nu + sigma
        m_ritz_val.head(m_nev).array() = Scalar(1) / m_ritz_val.head(m_nev).array() + m_sigma;
        Base::sort_ritzpair(sort_rule);
    }

  public:
    KrylovSchurGEigsShiftInvert(OpType& op, BOpType& Bop, Index nev, Index ncv, const Scalar& sigma)
        : Base(set_shift_and_move(ModeMatOp(op, Bop), sigma), Bop, nev, ncv), m_sigma(sigma) {}
};

void ChSymGenEigenvalueSolver::GetNaturalFrequencies(const ChVectorDynamic<ScalarType>& eigvals,
                                                     ChVectorDynamic<double>& freq) {
    freq.resize(eigvals.size());

    for (auto i = 0; i < eigvals.size(); i++) {
        freq(i) = GetNaturalFrequency(eigvals(i));
    }
}

int ChSymGenEigenvalueSolverKrylovSchur::Solve(const ChSparseMatrix& A,
                                               const ChSparseMatrix& B,
                                               ChMatrixDynamic<ScalarType>& eigvects,
                                               ChVectorDynamic<ScalarType>& eigvals,
                                               int num_modes,
                                               ScalarType shift) const {
    m_timer_eigen_setup.start();

    int m = std::max(2 * num_modes, m_min_subspace_size);

    if (m > A.rows() - 1)
        m = A.rows() - 1;
    if (m <= num_modes)
        m = num_modes + 1;

    // Construct matrix operation objects using the wrapper classes
    using OpType = SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
    using BOpType = SparseSymMatProd<double>;
    OpType op(getColMajorSparseMatrix(A), getColMajorSparseMatrix(B));
    BOpType Bop(getColMajorSparseMatrix(B));

    // The Krylov-Schur solver, using the shift and invert mode:
    KrylovSchurGEigsShiftInvert<OpType, BOpType> eigen_solver(op, Bop, num_modes, m, shift);

    eigen_solver.init();

    m_timer_eigen_setup.start();
    m_timer_eigen_solver.start();

    int nconv = eigen_solver.compute(SortRule::LargestMagn, max_iterations, tolerance);
    m_timer_eigen_solver.stop();

    if (verbose) {
        if (eigen_solver.info() != CompInfo::Successful) {
            std::cout << "KrylovSchurGEigsSolver FAILED." << std::endl;
            if (eigen_solver.info() == CompInfo::NotComputed)
                std::cout << " Error: not computed." << std::endl;
            if (eigen_solver.info() == CompInfo::NotConverging)
                std::cout << " Error: not converging." << std::endl;
            if (eigen_solver.info() == CompInfo::NumericalIssue)
                std::cout << " Error: numerical issue." << std::endl;
            std::cout << " nconv  = " << nconv << std::endl;
            std::cout << " niter  = " << eigen_solver.num_iterations() << std::endl;
            std::cout << " nops   = " << eigen_solver.num_operations() << std::endl;
            return false;
        } else {
            std::cout << "KrylovSchurGEigsSolver successful." << std::endl;
            std::cout << " nconv   = " << nconv << std::endl;
            std::cout << " niter   = " << eigen_solver.num_iterations() << std::endl;
            std::cout << " nops    = " << eigen_solver.num_operations() << std::endl;
            std::cout << " n_modes = " << num_modes << std::endl;
            std::cout << " problem size  = " << A.rows() << std::endl;
        }
    }

    m_timer_solution_postprocessing.start();

    // TODO: pass the results without copying
    eigvals = eigen_solver.eigenvalues();
    eigvects = eigen_solver.eigenvectors();

    for (int i = 0; i < eigvals.rows(); ++i) {
        eigvals(i) = (1.0 / eigvals(i)) + shift;
    }

    if (sort_ritz_pairs)
        SortRitzPairs(eigvals, eigvects);

    m_timer_solution_postprocessing.stop();

    return nconv;
}

int ChSymGenEigenvalueSolverLanczos::Solve(const ChSparseMatrix& A,
                                           const ChSparseMatrix& B,
                                           ChMatrixDynamic<ScalarType>& eigvects,
                                           ChVectorDynamic<ScalarType>& eigvals,
                                           int num_modes,
                                           ScalarType shift) const {
    m_timer_eigen_setup.start();

    int m = 2 * num_modes >= 20 ? 2 * num_modes : 20;  // minimum subspace size
    if (m > A.rows() - 1)
        m = A.rows() - 1;
    if (m <= num_modes)
        m = num_modes + 1;

    // Construct matrix operation objects using the wrapper classes
    using OpType = SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
    using BOpType = SparseSymMatProd<double>;
    OpType op(getColMajorSparseMatrix(A), getColMajorSparseMatrix(B));
    BOpType Bop(getColMajorSparseMatrix(B));

    // Eigen::SparseMatrix<double> A_colMajor = A;
    // Eigen::SparseMatrix<double> B_colMajor = B;
    // OpType op(A_colMajor, B_colMajor);
    // BOpType Bop(B_colMajor);

    SymGEigsShiftSolver<OpType, BOpType, GEigsMode::ShiftInvert> eigen_solver(op, Bop, num_modes, m, shift);

    eigen_solver.init();
    m_timer_eigen_setup.stop();

    m_timer_eigen_solver.start();
    int nconv = eigen_solver.compute(SortRule::LargestMagn, max_iterations, tolerance);
    m_timer_eigen_solver.stop();

    m_timer_solution_postprocessing.start();

    if (sort_ritz_pairs) {
        auto perm = GetPermutationMatrix(nconv, [&](int a, int b) {
            return std::abs(eigen_solver.eigenvalues()[a]) < std::abs(eigen_solver.eigenvalues()[b]);
        });
        eigvects = eigen_solver.eigenvectors() * perm;
        eigvals = perm.transpose() * eigen_solver.eigenvalues();
    } else {
        eigvects = eigen_solver.eigenvectors();
        eigvals = eigen_solver.eigenvalues();
    }

    if (verbose) {
        if (eigen_solver.info() != CompInfo::Successful) {
            std::cout << "Lanczos eigenvalue solver FAILED." << std::endl;
            if (eigen_solver.info() == CompInfo::NotComputed)
                std::cout << " Error: not computed." << std::endl;
            if (eigen_solver.info() == CompInfo::NotConverging)
                std::cout << " Error: not converging." << std::endl;
            if (eigen_solver.info() == CompInfo::NumericalIssue)
                std::cout << " Error: numerical issue." << std::endl;
            std::cout << " nconv  = " << nconv << std::endl;
            std::cout << " niter  = " << eigen_solver.num_iterations() << std::endl;
            std::cout << " nops   = " << eigen_solver.num_operations() << std::endl;
            return false;
        } else {
            std::cout << "Lanczos eigenvalue solver successful." << std::endl;
            std::cout << " nconv   = " << nconv << std::endl;
            std::cout << " niter   = " << eigen_solver.num_iterations() << std::endl;
            std::cout << " nops    = " << eigen_solver.num_operations() << std::endl;
            std::cout << " n_modes = " << num_modes << std::endl;
        }

        ChVectorDynamic<ScalarType> resCallback(A.rows());
        op.set_shift(shift);

        for (int i = 0; i < nconv; i++) {
            ChVectorDynamic<ScalarType> Bx(eigvects.col(i).rows());
            ChVectorDynamic<ScalarType> AsigmaBinv_Bx(eigvects.col(i).rows());
            // it is necessary to copy the eigenvector instead of using eigvects.col(i).data()
            ChVectorDynamic<ScalarType> cur_eigvect = eigvects.col(i);
            Bop.perform_op(cur_eigvect.data(), Bx.data());
            op.perform_op(Bx.data(), AsigmaBinv_Bx.data());
            resCallback = AsigmaBinv_Bx - eigvects.col(i) / (eigvals(i) - shift);
            std::cout << "   Eig " << i << "= " << eigvals(i) << "; res: " << resCallback.lpNorm<Eigen::Infinity>()
                      << std::endl;
        }
    }

    m_timer_solution_postprocessing.stop();

    return nconv;
}

//-------------------------------------------------------------------------------------------------------------------

}  // end namespace modal

}  // end namespace chrono
