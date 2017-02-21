// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This class calls the parallel solver, used as an intermediate
// between chrono's solver interface and the parallel solver interface.
// =============================================================================

#pragma once

#include "chrono/solver/ChIterativeSolver.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/ChIntegratorParallel.h"

#include "chrono_parallel/physics/Ch3DOFContainer.h"

#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class CH_PARALLEL_API ChIterativeSolverParallel : public ChIterativeSolver {
  public:
    virtual ~ChIterativeSolverParallel();

    ///< Each child class must define its own solve method
    virtual double Solve(ChSystemDescriptor& sysd) { return 0; }
    ///< Similarly, the run timestep function needs to be defined
    virtual void RunTimeStep() = 0;
    ///< This function computes the new velocities based on the lagrange multipliers
    virtual void ComputeImpulses() = 0;

    ///< Compute the inverse mass matrix and the term v+M_inv*hf
    void ComputeInvMassMatrix();
    // Compute mass matrix
    void ComputeMassMatrix();
    ///< Solves just the bilaterals so that they can be warm started
    void PerformStabilization();

    real GetResidual();

    ChParallelDataManager* data_manager;
    ChSolverParallel* solver;
    ChSolverParallel* bilateral_solver;

  protected:
    ChIterativeSolverParallel(ChParallelDataManager* dc);

    ChShurProductBilateral ShurProductBilateral;
    ChShurProductFEM ShurProductFEM;
    ChProjectNone ProjectNone;
};

class CH_PARALLEL_API ChIterativeSolverParallelDVI : public ChIterativeSolverParallel {
  public:
    ChIterativeSolverParallelDVI(ChParallelDataManager* dc) : ChIterativeSolverParallel(dc) {}

    virtual void RunTimeStep();
    virtual void ComputeImpulses();

    ///< Compute the constraint Jacobian matrix.
    void ComputeD();
    ///< Compute the compliance matrix.
    void ComputeE();
    ///< Compute the RHS vector. This will not change depending on the solve
    void ComputeR();
    ///< Compute the Shur matrix N.
    void ComputeN();
    ///< Set the RHS vector depending on the local solver mode
    void SetR();
    ///< This function computes an initial guess for each contact
    void PreSolve();
    ///< This function is used to change the solver algorithm.
    void ChangeSolverType(SolverType type);

  private:
    ChShurProduct ShurProductFull;
    ChProjectConstraints ProjectFull;
};

class CH_PARALLEL_API ChIterativeSolverParallelDEM : public ChIterativeSolverParallel {
  public:
    ChIterativeSolverParallelDEM(ChParallelDataManager* dc) : ChIterativeSolverParallel(dc) {}

    virtual void RunTimeStep();
    virtual void ComputeImpulses();

    ///< Compute the constraint Jacobian matrix.
    void ComputeD();
    ///< Compute the compliance matrix.
    void ComputeE();
    ///< Compute the RHS vector.
    void ComputeR();

    void ProcessContacts();

  private:
    void host_CalcContactForces(custom_vector<int>& ext_body_id,
                                custom_vector<real3>& ext_body_force,
                                custom_vector<real3>& ext_body_torque,
                                custom_vector<vec2>& shape_pairs,
                                custom_vector<char>& shear_touch);

    void host_AddContactForces(uint ct_body_count, const custom_vector<int>& ct_body_id);

    void host_SetContactForcesMap(uint ct_body_count, const custom_vector<int>& ct_body_id);
};

}  // end namespace chrono
