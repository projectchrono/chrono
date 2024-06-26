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

#ifndef CHMODALSOLVER_H
#define CHMODALSOLVER_H

#include "chrono_modal/ChApiModal.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChAssembly.h"

#include <complex>

namespace chrono {

namespace modal {

/// Base class for modal solvers.
/// Contrary to eigensolvers, these classes deals directly with Chrono data structures, creating the appropriate setup
/// for running the inner eigensolver.
/// Multiple requests spanning different frequency ranges can be specified.
class ChApiModal ChModalSolver {
  public:
    struct ChFreqSpan {
        int nmodes;
        double freq;
    };

    /// Creates a modal solver.
    /// \a n_lower_modes number of lowest modes to be found.
    /// \a base_freq frequency around which the modes will be found; higher values can help finding eigenvalues for
    /// ill-conditioned problems.
    /// \a scaleCq if true, the Cq matrix is scaled to improve conditioning.
    /// \a verbose if true, additional information is printed during the solution process.
    ChModalSolver(int n_lower_modes, double base_freq, bool scaleCq, bool verbose)
        : m_freq_spans({{n_lower_modes, base_freq}}),
          m_clip_position_coords(true),
          m_scaleCq(scaleCq),
          m_verbose(verbose){};

    /// Creates a modal solver.
    /// \a freq_spans pair of number of modes and frequency around which the modes will be found.
    /// ill-conditioned problems.
    /// \a scaleCq if true, the Cq matrix is scaled to improve conditioning.
    /// \a verbose if true, additional information is printed during the solution process.
    ChModalSolver(std::vector<ChFreqSpan> freq_spans, bool scaleCq, bool verbose)
        : m_freq_spans(freq_spans), m_clip_position_coords(true), m_scaleCq(scaleCq), m_verbose(verbose){};

    virtual ~ChModalSolver(){};

    /// Get the total number of requested modes.
    int GetNumRequestedModes() const;

    /// Clip the eigenvectors to only the position coordinates.
    void SetClipPositionCoords(bool val) { m_clip_position_coords = val; }

    /// Get the set of frequency spans for which modes are requested.
    const std::vector<ChFreqSpan>& GetFrequencySpans() const { return m_freq_spans; }

    /// Get cumulative time for matrix assembly.
    double GetTimeMatrixAssembly() const { return m_timer_matrix_assembly(); }

    /// Get cumulative time eigensolver solution.
    double GetTimeEigenSolver() const { return m_timer_eigen_solver(); }

    /// Get cumulative time for post-solver solution postprocessing.
    double GetTimeSolutionPostProcessing() const { return m_timer_solution_postprocessing(); }

  protected:
    mutable ChTimer m_timer_matrix_assembly;          ///< timer for matrix assembly
    mutable ChTimer m_timer_eigen_solver;             ///< timer for eigensolver solution
    mutable ChTimer m_timer_solution_postprocessing;  ///< timer for conversion of eigensolver solution

    std::vector<ChFreqSpan> m_freq_spans;  ///< frequency spans for which modes are requested

    bool m_clip_position_coords =
        true;                ///< store only the part of each eigenvector that refers to the position coordinates
    bool m_scaleCq = true;   ///< if true, the Cq matrix is scaled to improve conditioning
    bool m_verbose = false;  ///< if true, additional information is printed during the solution process
};

}  // end namespace modal

}  // end namespace chrono

#endif
