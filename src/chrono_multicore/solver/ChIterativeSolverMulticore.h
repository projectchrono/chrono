// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This class calls the multicore solver, used as an intermediate
// between chrono's solver interface and the multicore solver interface.
//
// =============================================================================

#pragma once

#include "chrono/solver/ChIterativeSolverVI.h"

#include "chrono_multicore/ChMulticoreDefines.h"
#include "chrono_multicore/ChDataManager.h"

#include "chrono_multicore/physics/Ch3DOFContainer.h"

#include "chrono/multicore_math/ChMulticoreMath.h"
#include "chrono_multicore/solver/ChSolverMulticore.h"

namespace chrono {

/// @addtogroup multicore_solver
/// @{

/// Base class for all iterative solvers.
class CH_MULTICORE_API ChIterativeSolverMulticore : public ChIterativeSolverVI {
  public:
    virtual ~ChIterativeSolverMulticore();

    /// Each child class must define its own solve method.
    virtual double Solve(ChSystemDescriptor& sysd) override { return 0; }
    /// Similarly, the run timestep function needs to be defined.
    virtual void RunTimeStep() = 0;
    /// This function computes the new velocities based on the lagrange multipliers.
    virtual void ComputeImpulses() = 0;

    /// Compute the inverse mass matrix and the term v+M_inv*hf.
    void ComputeInvMassMatrix();
    /// Compute mass matrix.
    void ComputeMassMatrix();
    /// Solves just the bilaterals so that they can be warm started.
    void PerformStabilization();

    real GetResidual() const;
    virtual double GetError() const override { return (double)GetResidual(); }

    ChMulticoreDataManager* data_manager;
    ChSolverMulticore* solver;
    ChSolverMulticore* bilateral_solver;

  protected:
    ChIterativeSolverMulticore(ChMulticoreDataManager* dc);

    ChSchurProductBilateral SchurProductBilateral;
    ChProjectNone ProjectNone;
};

/// Wrapper class for all complementarity solvers.
class CH_MULTICORE_API ChIterativeSolverMulticoreNSC : public ChIterativeSolverMulticore {
  public:
    ChIterativeSolverMulticoreNSC(ChMulticoreDataManager* dc) : ChIterativeSolverMulticore(dc) {}

    virtual void RunTimeStep();
    virtual void ComputeImpulses();

    /// Compute the constraint Jacobian matrix.
    void ComputeD();
    /// Compute the compliance matrix.
    void ComputeE();
    /// Compute the RHS vector. This will not change depending on the solve.
    void ComputeR();
    /// Compute the Schur matrix N.
    void ComputeN();
    /// Set the RHS vector depending on the local solver mode.
    void SetR();
    /// This function computes an initial guess for each contact.
    void PreSolve();
    /// This function is used to change the solver algorithm.
    void ChangeSolverType(SolverType type);

  private:
    ChSchurProduct SchurProductFull;
    ChProjectConstraints ProjectFull;
};

/// Iterative solver for SMC (penalty-based) problems.
class CH_MULTICORE_API ChIterativeSolverMulticoreSMC : public ChIterativeSolverMulticore {
  public:
    ChIterativeSolverMulticoreSMC(ChMulticoreDataManager* dc) : ChIterativeSolverMulticore(dc) {}

    virtual void RunTimeStep();
    virtual void ComputeImpulses();

    /// Compute the constraint Jacobian matrix.
    void ComputeD();
    /// Compute the compliance matrix.
    void ComputeE();
    /// Compute the RHS vector.
    void ComputeR();

    void ProcessContacts();

  private:
    void host_CalcContactForces(custom_vector<int>& ct_bid,
                                custom_vector<real3>& ct_force,
                                custom_vector<real3>& ct_torque,
                                custom_vector<vec2>& shape_pairs,
                                custom_vector<char>& shear_touch);

    void host_AddContactForces(uint ct_body_count, const custom_vector<int>& ct_body_id);

    void host_SetContactForcesMap(uint ct_body_count, const custom_vector<int>& ct_body_id);
};

/// @} multicore_solver

}  // end namespace chrono
