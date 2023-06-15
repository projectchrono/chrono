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
// Authors: Radu Serban
// =============================================================================
//
// Chrono solvers based on Eigen iterative linear solvers.
// All iterative linear solvers are implemented in a matrix-free context and
// rely on the system descriptor for the required SPMV operations.
// They can optionally use a diagonal preconditioner.
//
// Available solvers:
//   GMRES
//   BiCGSTAB
//   MINRES
//
// =============================================================================

#include "chrono/solver/ChIterativeSolverLS.h"

// =============================================================================

// Wrapper class for using the Eigen iterative solvers in a matrix-free context.
// See https://eigen.tuxfamily.org/dox/group__MatrixfreeSolverExample.html

namespace Eigen {
namespace internal {

// ChMatrixSPMV looks like a ChSparseMatrix, so we inherits its traits
template <>
struct traits<chrono::ChMatrixSPMV> : public Eigen::internal::traits<chrono::ChSparseMatrix> {};

}  // namespace internal
}  // namespace Eigen

namespace chrono {

CH_UPCASTING(ChIterativeSolverLS, ChIterativeSolver)
CH_UPCASTING(ChIterativeSolverLS, ChSolverLS)

// Matrix-free wrapper from a user type to Eigen's compatible type.
// We defer to the system descriptor to perform the SPMV operation.
class ChMatrixSPMV : public Eigen::EigenBase<ChMatrixSPMV> {
  public:
    // Required typedefs, constants, and method
    typedef double Scalar;
    typedef double RealScalar;
    typedef int StorageIndex;
    enum { ColsAtCompileTime = Eigen::Dynamic, MaxColsAtCompileTime = Eigen::Dynamic, IsRowMajor = true };

    Index rows() const { return m_N; }
    Index cols() const { return m_N; }

    template <typename Rhs>
    Eigen::Product<ChMatrixSPMV, Rhs, Eigen::AliasFreeProduct> operator*(const Eigen::MatrixBase<Rhs>& x) const {
        return Eigen::Product<ChMatrixSPMV, Rhs, Eigen::AliasFreeProduct>(*this, x.derived());
    }

    // Custom API
    ChMatrixSPMV() : m_N(0), m_sysd(nullptr) {}
    void Setup(Index N, chrono::ChSystemDescriptor& sysd) {
        m_N = N;
        m_sysd = &sysd;
        m_vect.resize(m_N);
    }
    chrono::ChSystemDescriptor* sysd() { return m_sysd; }
    chrono::ChVectorDynamic<>& vect() { return m_vect; }

  private:
    Index m_N;                           // problem dimension
    chrono::ChSystemDescriptor* m_sysd;  // pointer to system descriptor
    chrono::ChVectorDynamic<> m_vect;    // workspace for the result of the SPMV operation
};

/// Simple diagonal preconditioner
class ChDiagonalPreconditioner {
    typedef double Scalar;

  public:
    typedef int StorageIndex;
    enum { ColsAtCompileTime = Eigen::Dynamic, MaxColsAtCompileTime = Eigen::Dynamic };

    ChDiagonalPreconditioner() : m_N(0), m_diag_precond(false), m_invdiag(nullptr) {}

    void Setup(Eigen::Index N, const ChVectorDynamic<>& invdiag) {
        m_N = N;
        m_invdiag = &invdiag;
        m_diag_precond = (invdiag.size() > 0);
    }

    Eigen::Index rows() const { return m_N; }
    Eigen::Index cols() const { return m_N; }

    template <typename MatType>
    ChDiagonalPreconditioner& analyzePattern(const MatType&) {
        return *this;
    }
    template <typename MatType>
    ChDiagonalPreconditioner& factorize(const MatType& mat) {
        return *this;
    }
    template <typename MatType>
    ChDiagonalPreconditioner& compute(const MatType& mat) {
        return *this;
    }

    template <typename Rhs, typename Dest>
    void _solve_impl(const Rhs& b, Dest& x) const {
        if (m_diag_precond) {
            x = m_invdiag->array() * b.array();
        } else {
            x = b;
        }
    }

    template <typename Rhs>
    inline const Eigen::Solve<ChDiagonalPreconditioner, Rhs> solve(const Eigen::MatrixBase<Rhs>& b) const {
        return Eigen::Solve<ChDiagonalPreconditioner, Rhs>(*this, b.derived());
    }

    Eigen::ComputationInfo info() { return Eigen::Success; }

  protected:
    Eigen::Index m_N;                    // problem dimension
    const ChVectorDynamic<>* m_invdiag;  // pointer to (invcerse) diagonal entries
    bool m_diag_precond;                 // if false, no preconditioning
};

}  // namespace chrono

namespace Eigen {
namespace internal {

// Implementation of ChMatrixSPMV * Eigen::DenseVector through a specialization of internal::generic_product_impl
template <typename Rhs>
struct generic_product_impl<chrono::ChMatrixSPMV, Rhs, SparseShape, DenseShape, GemvProduct>
    : generic_product_impl_base<chrono::ChMatrixSPMV, Rhs, generic_product_impl<chrono::ChMatrixSPMV, Rhs> > {
    typedef typename Product<chrono::ChMatrixSPMV, Rhs>::Scalar Scalar;

    // This method should implement "dst += alpha * lhs * rhs" inplace,
    // However, for iterative solvers, alpha is always equal to 1, so we ignore it.
    template <typename Dest>
    static void scaleAndAddTo(Dest& dst, const chrono::ChMatrixSPMV& lhs, const Rhs& rhs, const Scalar& alpha) {
        assert(alpha == Scalar(1) && "scaling is not implemented");
        EIGEN_ONLY_USED_FOR_DEBUG(alpha);

        // Hack to allow calling ChSystemDescriptor::SystemProduct
        auto lhs_ = const_cast<chrono::ChMatrixSPMV&>(lhs);

        lhs_.sysd()->SystemProduct(lhs_.vect(), rhs);
        dst += lhs_.vect();
    }
};

}  // namespace internal
}  // namespace Eigen

// =============================================================================

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverGMRES)
CH_FACTORY_REGISTER(ChSolverBiCGSTAB)
CH_FACTORY_REGISTER(ChSolverMINRES)

ChIterativeSolverLS::ChIterativeSolverLS() : ChIterativeSolver(-1, -1.0, true, false) {
    m_spmv = new ChMatrixSPMV();
}

ChIterativeSolverLS::~ChIterativeSolverLS() {
    delete m_spmv;
}

bool ChIterativeSolverLS::Setup(ChSystemDescriptor& sysd) {
    // Calculate problem size
    int dim = sysd.CountActiveVariables() + sysd.CountActiveConstraints();

    // Set up the SPMV wrapper
    m_spmv->Setup(dim, sysd);

    // If needed, evaluate the inverse diagonal entries
    if (m_use_precond) {
        m_invdiag.resize(dim);
        sysd.BuildDiagonalVector(m_invdiag);
        for (int i = 0; i < dim; i++) {
            if (std::abs(m_invdiag(i)) > 1e-9)
                m_invdiag(i) = 1.0 / m_invdiag(i);
            else
                m_invdiag(i) = 1.0;
        }
    }

    // If needed, evaluate the initial guess
    if (m_warm_start) {
        m_initguess.resize(dim);
        sysd.FromUnknownsToVector(m_initguess);
    }

    // Let the concrete solver initialize itself
    bool result = SetupProblem();

    //// ---- DEBUGGING
    ////SaveMatrix(sysd);
    //// ---- DEBUGGING

    return result;
}

double ChIterativeSolverLS::Solve(ChSystemDescriptor& sysd) {
    // Assemble the problem right-hand side vector
    sysd.ConvertToMatrixForm(nullptr, &m_rhs);

    // Let the concrete solver compute the solution (in m_sol)
    bool result = SolveProblem();

    if (verbose) {
        // Calculate exact residual and report its norm
        ChVectorDynamic<> Ax(m_sol.size());
        sysd.SystemProduct(Ax, m_sol);
        std::cout << "  ||Ax-b|| = " << (Ax - m_rhs).norm() << std::endl;
    }

    //// DEBUGGING
    ////SaveMatrix(sysd);
    ////CheckSolution(sysd, m_sol);
    //// DEBUGGING

    // Scatter solution vector to the system descriptor
    sysd.FromVectorToUnknowns(m_sol);

    return result;
}

// ---------------------------------------------------------------------------

ChSolverGMRES::ChSolverGMRES() {
    m_engine = new Eigen::GMRES<ChMatrixSPMV, ChDiagonalPreconditioner>();
}

ChSolverGMRES::~ChSolverGMRES() {
    delete m_engine;
}

bool ChSolverGMRES::SetupProblem() {
    m_engine->preconditioner().Setup(m_rhs.size(), m_invdiag);
    m_engine->compute(*m_spmv);
    return (m_engine->info() == Eigen::Success);
}

bool ChSolverGMRES::SolveProblem() {
    if (m_max_iterations > 0) {
        m_engine->setMaxIterations(m_max_iterations);
    }
    if (m_tolerance > 0) {
        m_engine->setTolerance(m_tolerance);
    }

    if (m_warm_start) {
        m_sol = m_engine->solveWithGuess(m_rhs, m_initguess);
    } else {
        m_sol = m_engine->solve(m_rhs);
    }

    if (verbose) {
        std::cout << "  GMRES iterations: " << m_engine->iterations() << " error: " << m_engine->error() << std::endl;
    }

    return (m_engine->info() == Eigen::Success);
}

int ChSolverGMRES::GetIterations() const {
    return (int)m_engine->iterations();
}

double ChSolverGMRES::GetError() const {
    return m_engine->error();
}

// ---------------------------------------------------------------------------

ChSolverBiCGSTAB::ChSolverBiCGSTAB() {
    m_engine = new Eigen::BiCGSTAB<ChMatrixSPMV, ChDiagonalPreconditioner>();
}

ChSolverBiCGSTAB::~ChSolverBiCGSTAB() {
    delete m_engine;
}

bool ChSolverBiCGSTAB::SetupProblem() {
    m_engine->preconditioner().Setup(m_rhs.size(), m_invdiag);
    m_engine->compute(*m_spmv);
    return (m_engine->info() == Eigen::Success);
}

bool ChSolverBiCGSTAB::SolveProblem() {
    if (m_max_iterations > 0) {
        m_engine->setMaxIterations(m_max_iterations);
    }
    if (m_tolerance > 0) {
        m_engine->setTolerance(m_tolerance);
    }

    if (m_warm_start) {
        m_sol = m_engine->solveWithGuess(m_rhs, m_initguess);
    } else {
        m_sol = m_engine->solve(m_rhs);
    }

    if (verbose) {
        std::cout << "  BiCGSTAB iterations: " << m_engine->iterations() << " error: " << m_engine->error()
                  << std::endl;
    }

    return (m_engine->info() == Eigen::Success);
}

int ChSolverBiCGSTAB::GetIterations() const {
    return (int)m_engine->iterations();
}

double ChSolverBiCGSTAB::GetError() const {
    return m_engine->error();
}

// ---------------------------------------------------------------------------

ChSolverMINRES::ChSolverMINRES() {
    m_engine = new Eigen::MINRES<ChMatrixSPMV, Eigen::Lower | Eigen::Upper, ChDiagonalPreconditioner>();
}

ChSolverMINRES::~ChSolverMINRES() {
    delete m_engine;
}

bool ChSolverMINRES::SetupProblem() {
    m_engine->preconditioner().Setup(m_rhs.size(), m_invdiag);
    m_engine->compute(*m_spmv);
    return (m_engine->info() == Eigen::Success);
}

bool ChSolverMINRES::SolveProblem() {
    if (m_max_iterations > 0) {
        m_engine->setMaxIterations(m_max_iterations);
    }
    if (m_tolerance > 0) {
        m_engine->setTolerance(m_tolerance);
    }

    if (m_warm_start) {
        m_sol = m_engine->solveWithGuess(m_rhs, m_initguess);
    } else {
        m_sol = m_engine->solve(m_rhs);
    }

    if (verbose) {
        std::cout << "  MINRES iterations: " << m_engine->iterations() << " error: " << m_engine->error() << std::endl;
    }

    return (m_engine->info() == Eigen::Success);
}

int ChSolverMINRES::GetIterations() const {
    return (int)m_engine->iterations();
}

double ChSolverMINRES::GetError() const {
    return m_engine->error();
}

}  // end namespace chrono
