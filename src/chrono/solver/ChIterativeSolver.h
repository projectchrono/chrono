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

#ifndef CHITERATIVESOLVER_H
#define CHITERATIVESOLVER_H

#include "chrono/solver/ChSolver.h"

namespace chrono {

/// Base class for ITERATIVE solvers aimed at solving complementarity problems arising
/// from QP optimization problems.
/// This class does nothing: it is up to derived clases to implement specific solution
/// methods.
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

class ChApi ChIterativeSolver : public ChSolver {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChIterativeSolver)

  protected:
    int max_iterations;  ///< maximum allowed iterations
    int tot_iterations;  ///< total number of iterations performed by the solver
    bool warm_start;     ///< indicate whether or not to use warm starting
    double tolerance;    ///< tolerance for termination criteria
    double omega;        ///< over-relaxation factor
    double shlambda;     ///< sharpness factor

    bool record_violation_history;
    std::vector<double> violation_history;
    std::vector<double> dlambda_history;

  public:
    ChIterativeSolver(int mmax_iters = 50,       ///< max.number of iterations
                      bool mwarm_start = false,  ///< uses warm start?
                      double mtolerance = 0.0,   ///< tolerance for termination criterion
                      double momega = 1.0,       ///< overrelaxation, if any
                      double mshlambda = 1.0     ///< sharpness, if any
                      )
        : max_iterations(mmax_iters),
          tot_iterations(0),
          warm_start(mwarm_start),
          tolerance(mtolerance),
          omega(momega),
          shlambda(mshlambda),
          record_violation_history(false) {}

    virtual ~ChIterativeSolver() {}

    /// Indicate whether ot not the Solve() phase requires an up-to-date problem matrix.
    /// Typically, this is the case for iterative solvers (as the matrix is needed for
    /// the matrix-vector operations).
    virtual bool SolveRequiresMatrix() const override { return true; }

    /// Set the maximum number of iterations.
    /// If the solver exceed this limit, it should stop even if the required tolerance
    /// isn't yet reached. Default limit: 50 iterations.
    void SetMaxIterations(int mval) { max_iterations = mval; }

    /// Get the current value for the maximum allowed number of iterations.
    int GetMaxIterations() const { return max_iterations; }

    /// Get the number of total iterations taken by the solver.
    int GetTotalIterations() const { return tot_iterations; }

    /// Set the overrelaxation factor.
    /// This factor may be used by SOR-like methods. Default: 1.
    void SetOmega(double mval) {
        if (mval > 0.)
            omega = mval;
    }

    /// Return the current value of the overrelaxation factor.
    double GetOmega() const { return omega; }

    /// Set the sharpness factor.
    /// This factor may be used by SOR-like methods with projection (see Mangasarian LCP method).
    /// Usualy in the range [0,1]. Default: 1. 
    virtual void SetSharpnessLambda(double mval) {
        if (mval > 0.)
            shlambda = mval;
    }

    /// Return the current value of the sharpness factor.
    virtual double GetSharpnessLambda() const { return shlambda; }

    /// Enable/disable 'warm start'.
    /// If enabled, the initial guess is set to the current values of the unknowns.
    ///	Useful if the variables are already near to the solution. In most iterative schemes,
    /// this is often a good option.
    void SetWarmStart(bool mval) { warm_start = mval; }

    /// Return a flag indicating whether or not warm start is enabled.
    bool GetWarmStart() const { return warm_start; }

    /// Set the tolerance for stopping criterion.
    /// The iteration is stopped when the constraint feasability error is below this value.
    /// Default: 0.0
    void SetTolerance(double mval) { tolerance = mval; }

    /// Return the current value of the solver tolerance.
    double GetTolerance() const { return tolerance; }

    /// Enable/disable recording of the constraint violation history.
    /// If enabled, the maximum constraint violation at the end of each iteration is
    /// stored in a vector (see GetViolationHistory).
    void SetRecordViolation(bool mval) { record_violation_history = mval; }

    /// Return a flag indicating whether or not constraint violations are recorded.
    bool GetRecordViolation() const { return record_violation_history; }

    /// Access the vector of constraint violation history.
    /// Note that collection of constraint violations must be enabled through SetRecordViolation.
    const std::vector<double>& GetViolationHistory() const { return violation_history; }

    /// Access the vector with history of maximum change in Lagrange multipliers
    /// Note that collection of constraint violations must be enabled through SetRecordViolation.
    const std::vector<double>& GetDeltalambdaHistory() const { return dlambda_history; };

  protected:
    /// This method MUST be called by all iterative methods INSIDE their iteration loops
    /// (at the end). If history recording is enabled, this function will store the
    /// current values as passed as arguments.
    /// Note: 'iternum' starts at 0 for the first iteration.
    void AtIterationEnd(double mmaxviolation, double mdeltalambda, unsigned int iternum) {
        if (!record_violation_history)
            return;
        if (iternum != violation_history.size()) {
            violation_history.clear();
            violation_history.resize(iternum);
        }
        if (iternum != dlambda_history.size()) {
            dlambda_history.clear();
            dlambda_history.resize(iternum);
        }
        violation_history.push_back(mmaxviolation);
        dlambda_history.push_back(mdeltalambda);
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChIterativeSolver>();
        // serialize parent class
        ChSolver::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(max_iterations);
        marchive << CHNVP(tot_iterations);
        marchive << CHNVP(warm_start);
        marchive << CHNVP(tolerance);
        marchive << CHNVP(omega);
        marchive << CHNVP(shlambda);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChIterativeSolver>();
        // deserialize parent class
        ChSolver::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(max_iterations);
        marchive >> CHNVP(tot_iterations);
        marchive >> CHNVP(warm_start);
        marchive >> CHNVP(tolerance);
        marchive >> CHNVP(omega);
        marchive >> CHNVP(shlambda);
    }
};

}  // end namespace chrono

#endif
