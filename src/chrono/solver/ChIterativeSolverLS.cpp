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
// Chrono solvers based on Eigen iterative linear solvers
//
// =============================================================================

#include "chrono/solver/ChIterativeSolverLS.h"

// =============================================================================

// Wrapper class for using the Eigen iterative solvers in a matrix-free context.
// See https://eigen.tuxfamily.org/dox/group__MatrixfreeSolverExample.html

namespace Eigen {
namespace internal {

// ChMatrixSPMV looks-like a ChSparseMatrix, so we inherits its traits
template <>
struct traits<chrono::ChMatrixSPMV> : public Eigen::internal::traits<chrono::ChSparseMatrix> {};

}  // namespace internal
}  // namespace Eigen

namespace chrono {

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
    void Setup(int N, chrono::ChSystemDescriptor& sysd) {
        m_N = N;
        m_sysd = &sysd;
        m_vect.resize(m_N);
    }
    chrono::ChSystemDescriptor* sysd() { return m_sysd; }
    chrono::ChVectorDynamic<>& vect() { return m_vect; }

  private:
    int m_N;                             // problem dimension
    chrono::ChSystemDescriptor* m_sysd;  // pointer to system descriptor
    chrono::ChVectorDynamic<> m_vect;    // workspace for the result of the SPMV operation
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

ChIterativeSolverLS::ChIterativeSolverLS() : m_max_iterations(-1), m_tolerance(-1) {
    m_spmv = new ChMatrixSPMV();
}

ChIterativeSolverLS::~ChIterativeSolverLS() {
    delete m_spmv;
}

bool ChIterativeSolverLS::Setup(ChSystemDescriptor& sysd) {
    // Calculate problem size
    int dim = sysd.CountActiveVariables() + sysd.CountActiveConstraints();

    // Setup the SPMV wrapper and associate it with the Eigen solver
    m_spmv->Setup(dim, sysd);
    bool result = FactorizeMatrix();

    //// DEBUGGING
    bool dump = false;
    if (dump) {
        ChSparseMatrix Z;
        sysd.ConvertToMatrixForm(&Z, nullptr);

        const char* numformat = "%.12g";
        char filename[300];
        sprintf(filename, "%s%s", "matrix_", "Z.dat");
        ChStreamOutAsciiFile file_Z(filename);
        file_Z.SetNumFormat(numformat);
        StreamOUTsparseMatlabFormat(Z, file_Z);
    }
    //// DEBUGGING

    return result;
}

double ChIterativeSolverLS::Solve(ChSystemDescriptor& sysd) {
    // Assemble the problem right-hand side vector
    sysd.ConvertToMatrixForm(nullptr, &m_rhs);

    // Let the concrete solver compute the solution
    bool result = SolveSystem();

    // Scatter solution vector to the system descriptor
    sysd.FromVectorToUnknowns(m_sol);

    return result;
}

// ---------------------------------------------------------------------------

ChSolverGMRES::ChSolverGMRES() {
    m_engine = new Eigen::GMRES<ChMatrixSPMV, Eigen::IdentityPreconditioner>();
    ////m_engine = new Eigen::GMRES<ChMatrixSPMV, Eigen::DiagonalPreconditioner<double>>();

    Eigen::DGMRES<ChMatrixSPMV, Eigen::IdentityPreconditioner> gmres;
}

ChSolverGMRES::~ChSolverGMRES() {
    delete m_engine;
}

bool ChSolverGMRES::FactorizeMatrix() {
    m_engine->compute(*m_spmv);
    return (m_engine->info() == Eigen::Success);
}

bool ChSolverGMRES::SolveSystem() {
    if (m_max_iterations > 0) {
        m_engine->setMaxIterations(m_max_iterations);
    }
    if (m_tolerance > 0) {
        m_engine->setTolerance(m_tolerance);
    }

    m_sol = m_engine->solve(m_rhs);

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
    m_engine = new Eigen::BiCGSTAB<ChMatrixSPMV, Eigen::IdentityPreconditioner>();
    ////m_engine = new Eigen::BiCGSTAB<ChMatrixSPMV, Eigen::DiagonalPreconditioner<double>>();
}

ChSolverBiCGSTAB::~ChSolverBiCGSTAB() {
    delete m_engine;
}

bool ChSolverBiCGSTAB::FactorizeMatrix() {
    m_engine->compute(*m_spmv);
    return (m_engine->info() == Eigen::Success);
}

bool ChSolverBiCGSTAB::SolveSystem() {
    if (m_max_iterations > 0) {
        m_engine->setMaxIterations(m_max_iterations);
    }
    if (m_tolerance > 0) {
        m_engine->setTolerance(m_tolerance);
    }

    m_sol = m_engine->solve(m_rhs);

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
    m_engine = new Eigen::MINRES<ChMatrixSPMV, Eigen::Lower | Eigen::Upper, Eigen::IdentityPreconditioner>();
    ////m_engine = new Eigen::MINRES<ChMatrixSPMV, Eigen::Lower | Eigen::Upper, Eigen::DiagonalPreconditioner<double>>();
}

ChSolverMINRES::~ChSolverMINRES() {
    delete m_engine;
}

bool ChSolverMINRES::FactorizeMatrix() {
    m_engine->compute(*m_spmv);
    return (m_engine->info() == Eigen::Success);
}

bool ChSolverMINRES::SolveSystem() {
    if (m_max_iterations > 0) {
        m_engine->setMaxIterations(m_max_iterations);
    }
    if (m_tolerance > 0) {
        m_engine->setTolerance(m_tolerance);
    }

    m_sol = m_engine->solve(m_rhs);

    if (verbose) {
        std::cout << "  MINRES iterations: " << m_engine->iterations() << " error: " << m_engine->error()
                  << std::endl;
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
