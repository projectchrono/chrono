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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_SOLVER_H
#define CH_SOLVER_H

#include <vector>

#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

class ChIterativeSolver;
class ChDirectSolverLS;

/// @addtogroup chrono_solver
/// @{

/// Base class for all Chrono solvers (for linear problems or complementarity problems). \n
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.
class ChApi ChSolver {
  public:
    /// Available types of solvers.
    enum class Type {
        // Iterative VI solvers
        PSOR,             ///< Projected SOR (Successive Over-Relaxation)
        PSSOR,            ///< Projected symmetric SOR
        PJACOBI,          ///< Projected Jacobi
        PMINRES,          ///< Projected MINRES
        BARZILAIBORWEIN,  ///< Barzilai-Borwein
        APGD,             ///< Accelerated Projected Gradient Descent
        ADMM,             ///< Alternating Direction Method of Multipliers
        // Direct linear solvers
        SPARSE_LU,    ///< Sparse supernodal LU factorization
        SPARSE_QR,    ///< Sparse left-looking rank-revealing QR factorization
        PARDISO_MKL,  ///< Pardiso MKL (super-nodal sparse direct solver)
        MUMPS,        ///< Mumps (MUltifrontal Massively Parallel sparse direct Solver)
        // Iterative linear solvers
        GMRES,     ///< Generalized Minimal RESidual Algorithm
        MINRES,    ///< MINimum RESidual method
        BICGSTAB,  ///< Bi-conjugate gradient stabilized
        // Other
        CUSTOM
    };

    virtual ~ChSolver() {}

    /// Return type of the solver.
    virtual Type GetType() const { return Type::CUSTOM; }

    /// Return true if iterative solver.
    virtual bool IsIterative() const = 0;

    /// Return true if direct solver.
    virtual bool IsDirect() const = 0;

    /// Downcast to ChIterativeSolver.
    virtual ChIterativeSolver* AsIterative() { return nullptr; }

    /// Downcast to ChDirectSolver.
    virtual ChDirectSolverLS* AsDirect() { return nullptr; }

    /// Indicate whether or not the Solve() phase requires an up-to-date problem matrix.
    /// Typically, direct solvers only need the matrix for the Setup() phase. However, iterative solvers likely require
    /// the matrix to perform the necessary matrix-vector operations.
    virtual bool SolveRequiresMatrix() const = 0;

    /// Perform setup operations for the solver.
    /// This function prepares the solver for subsequent calls to the solve function. The system descriptor contains the
    /// constraints and variables.
    /// This function is called only as frequently it is determined that it is appropriate to perform the setup phase.
    /// The argument `analyze` indicates if a full analysis of the system matrix is required. This is true when a
    /// structural change in the system was detected (e.g., when a physical component was added to or removed from the
    /// Chrono system).
    /// This function must return true if successfull and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd, bool analyze) { return true; }

    /// Solve the linear system.
    /// The system descriptor contains the constraints and variables.
    /// The return value is specific to a derived solver class.
    virtual double Solve(ChSystemDescriptor& sysd) = 0;

    /// Set verbose output from solver.
    void SetVerbose(bool mv) { verbose = mv; }

    /// Enable/disable debug output of matrix, RHS, and solution vector.
    void EnableWrite(bool val, const std::string& frame, const std::string& out_dir = ".");

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

    /// Return the solver type as a string.
    static std::string GetTypeAsString(Type type); 

  protected:
    ChSolver() : verbose(false) {}

    bool verbose;
    bool write_matrix;
    std::string output_dir;
    std::string frame_id;
};

/// @} chrono_solver

}  // end namespace chrono

#endif