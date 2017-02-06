// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHSOLVER_H
#define CHSOLVER_H

#include <vector>

#include "chrono/solver/ChConstraint.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChVariables.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// Base class for solvers aimed at solving complementarity problems arising from QP optimization
/// problems. This is an abstract class and specific solution methods are implemented in derived
/// classes (e.g., SOR, APGD, etc.)
/// The problem is described by a variational inequality VI(Z*x-d,K):
///
///  | M -Cq'|*|q|- | f|= |0| , l \in Y, C \in Ny, normal cone to Y
///  | Cq -E | |l|  |-b|  |c|
///
/// Also Z symmetric by flipping sign of l_i: |M  Cq'|*| q|-| f|=|0|
///                                           |Cq  E | |-l| |-b| |c|
/// * case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
/// * case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
/// * case CCP: Y_i are friction cones

class ChApi ChSolver {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSolver)

  public:
      /// Available types of solvers.
      enum class Type {
          SOR = 0,
          SYMMSOR,
          JACOBI,
          SOR_MULTITHREAD,
          PMINRES,
          BARZILAIBORWEIN,
          PCG,
          APGD,
          MINRES,
          SOLVER_DEM,
          CUSTOM,
      };

    ChSolver() : verbose(false) {}

    virtual ~ChSolver() {}

    /// Return type of the solver.
    /// Default is CUSTOM. Derived classes should override this function.
    virtual Type GetType() const { return Type::CUSTOM; }

    /// Indicate whether or not the Solve() phase requires an up-to-date problem matrix.
    /// Typically, direct solvers only need the matrix for the Setup() phase. However,
    /// iterative solvers likely require the matrix to perform the necessary matrix-vector
    /// operations.
    virtual bool SolveRequiresMatrix() const = 0;

    /// Performs the solution of the problem.
    /// This function MUST be implemented in children classes, with specialized
    /// methods such as iterative or direct solvers.
    /// Returns true if it successfully solves the problem and false otherwise.
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) = 0;

    /// This function does the setup operations for the solver.
    /// The purpose of this function is to prepare the solver for subsequent calls to the
    /// solve function.  This function is called only as frequently it is determined that
    /// it is appropriate to perform the setup phase.
    virtual bool Setup(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                       ) {
        return true;
    }

    /// Set verbose output from solver.
    void SetVerbose(bool mv) { verbose = mv; }

    // Return whether or not verbose output is enabled.
    bool GetVerbose() const { return verbose; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

  protected:
    bool verbose;
};

/// @} chrono_solver

}  // end namespace chrono

#endif