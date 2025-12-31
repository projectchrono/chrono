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
// Authors: Alessandro Tasora, Dario Mangoni
// =============================================================================

#ifndef CHMODALSOLVERDAMPED_H
#define CHMODALSOLVERDAMPED_H

#include "chrono_modal/ChApiModal.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono_modal/ChModalSolver.h"
#include "chrono_modal/ChUnsymGenEigenvalueSolver.h"

#include <complex>

namespace chrono {

// Forward references
class ChUnsymGenEigenvalueSolver;

namespace modal {

/// @addtogroup modal
/// @{

/// Modal solver for damped systems of the form (-w^2*M + i*w*R + K)*x = 0  s.t. Cq*x = 0.
/// with complex w (where w.length() = undamped nat.freq)
class ChApiModal ChModalSolverDamped : public ChModalSolver {
  public:
    /// Creates a modal solver for the damped case.
    /// \a n_lower_modes number of lowest modes to be found.
    /// \a base_freq frequency around which the modes will be found; higher values can help finding eigenvalues for
    /// ill-conditioned problems.
    /// \a scaleCq if true, the Cq matrix is scaled to improve conditioning.
    /// \a verbose if true, additional information is printed during the solution process.
    /// \a solver the inner eigensolver to be used.
    ChModalSolverDamped(int n_lower_modes,
                        double base_freq,
                        bool scaleCq = true,
                        bool verbose = false,
                        std::shared_ptr<ChUnsymGenEigenvalueSolver> solver =
                            chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>())
        : ChModalSolver(n_lower_modes, base_freq, scaleCq, verbose), m_solver(solver){};

    /// Creates a modal solver for the damped case.
    /// \a freq_spans pair of number of modes and frequency around which the modes will be found.
    /// ill-conditioned problems.
    /// \a scaleCq if true, the Cq matrix is scaled to improve conditioning.
    /// \a verbose if true, additional information is printed during the solution process.
    /// \a solver the inner eigensolver to be used.
    ChModalSolverDamped(std::vector<ChFreqSpan> freq_spans,
                        bool scaleCq = true,
                        bool verbose = false,
                        std::shared_ptr<ChUnsymGenEigenvalueSolver> solver =
                            chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>())
        : ChModalSolver(freq_spans, scaleCq, verbose), m_solver(solver){};

    virtual ~ChModalSolverDamped(){};

    /// Solve the constrained eigenvalue problem retrieving it from the ChAssembly.
    /// Only the position part of the eigenvectors is returned, unless SetClipPositionCoords(false) is called.
    virtual int Solve(const ChAssembly& assembly,
                      ChMatrixDynamic<std::complex<double>>& eigvects,
                      ChVectorDynamic<std::complex<double>>& eigvals,
                      ChVectorDynamic<double>& freq,
                      ChVectorDynamic<double>& damp_ratios) const;

    /// Solve the constrained eigenvalue problem setting it up from individual matrices.
    /// Only the position part of the eigenvectors is returned, unless SetClipPositionCoords(false) is called.
    virtual int Solve(const ChSparseMatrix& K,
                      const ChSparseMatrix& R,
                      const ChSparseMatrix& M,
                      const ChSparseMatrix& Cq,
                      ChMatrixDynamic<std::complex<double>>& eigvects,
                      ChVectorDynamic<std::complex<double>>& eigvals,
                      ChVectorDynamic<double>& freq,
                      ChVectorDynamic<double>& damp_ratios) const;

    /// Get the inner eigensolver.
    std::shared_ptr<ChUnsymGenEigenvalueSolver> GetEigenSolver() const { return m_solver; }

  protected:
    std::shared_ptr<ChUnsymGenEigenvalueSolver> m_solver;
};

/// @} modal

}  // end namespace modal
}  // end namespace chrono

#endif
