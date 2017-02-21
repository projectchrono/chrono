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

#include <cstdio>

#include "chrono/parallel/ChThreadsSync.h"
#include "chrono/solver/ChConstraintTwoTuplesFrictionT.h"
#include "chrono/solver/ChConstraintTwoTuplesRollingN.h"
#include "chrono/solver/ChConstraintTwoTuplesRollingT.h"
#include "chrono/solver/ChSolverSORmultithread.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSolverSORmultithread)

// Each thread will own an instance of the following data:

struct thread_data {
    ChSolverSORmultithread* solver;  // reference to solver
    ChMutexSpinlock* mutex;          // this will be used to avoid race condition when writing to shared memory.

    enum solver_stage { STAGE1_PREPARE = 0, STAGE2_ADDFORCES, STAGE3_LOOPCONSTRAINTS };

    solver_stage stage;

    // the range of scanned multipliers (for loops on multipliers)
    unsigned int constr_from;
    unsigned int constr_to;

    // the range of scanned 'variables' objects (for loops on variables, aka rigid bodies)
    unsigned int var_from;
    unsigned int var_to;

    std::vector<ChConstraint*>* mconstraints;
    std::vector<ChVariables*>* mvariables;
};

// Don't create local store memory, just return 0

void* SolverMemoryFunc() {
    return 0;
}

// The following is the function which will be executed by
// each thread, when threads are launched at each Solve()

void SolverThreadFunc(void* userPtr, void* lsMemory) {
    double maxviolation = 0.;
    double maxdeltalambda = 0.;
    int i_friction_comp = 0;
    double old_lambda_friction[3];

    thread_data* tdata = (thread_data*)userPtr;

    std::vector<ChConstraint*>* mconstraints = tdata->mconstraints;
    std::vector<ChVariables*>* mvariables = tdata->mvariables;

    switch (tdata->stage) {
        case thread_data::STAGE1_PREPARE: {
            //    Update auxiliary data in all constraints before starting,
            //    that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
            //
            for (unsigned int ic = tdata->constr_from; ic < tdata->constr_to; ic++)
                (*mconstraints)[ic]->Update_auxiliary();

            //    Average all g_i for the triplet of contact constraints n,u,v.
            //
            int j_friction_comp = 0;
            double gi_values[3];
            for (unsigned int ic = tdata->constr_from; ic < tdata->constr_to; ic++) {
                if ((*mconstraints)[ic]->GetMode() == CONSTRAINT_FRIC) {
                    gi_values[j_friction_comp] = (*mconstraints)[ic]->Get_g_i();
                    j_friction_comp++;
                    if (j_friction_comp == 3) {
                        double average_g_i = (gi_values[0] + gi_values[1] + gi_values[2]) / 3.0;
                        (*mconstraints)[ic - 2]->Set_g_i(average_g_i);
                        (*mconstraints)[ic - 1]->Set_g_i(average_g_i);
                        (*mconstraints)[ic - 0]->Set_g_i(average_g_i);
                        j_friction_comp = 0;
                    }
                }
            }

            break;  // end stage
        }

        case thread_data::STAGE2_ADDFORCES: {
            //     Compute, for all items with variables, the initial guess for
            //     still unconstrained system:
            //
            for (unsigned int iv = tdata->var_from; iv < tdata->var_to; iv++)
                if ((*mvariables)[iv]->IsActive())
                    (*mvariables)[iv]->Compute_invMb_v((*mvariables)[iv]->Get_qb(),
                                                       (*mvariables)[iv]->Get_fb());  // q = [M]'*fb

            break;  // end stage
        }

        case thread_data::STAGE3_LOOPCONSTRAINTS: {
            //     For all items with variables, add the effect of initial (guessed)
            //     lagrangian reactions of contraints, if a warm start is desired.
            //     Otherwise, if no warm start, simply resets initial lagrangians to zero.
            //
            if (tdata->solver->GetWarmStart()) {
                for (unsigned int ic = tdata->constr_from; ic < tdata->constr_to; ic++)
                    if ((*mconstraints)[ic]->IsActive()) {
                        //	tdata->mutex->Lock();   // this avoids double writing on shared q vector
                        (*mconstraints)[ic]->Increment_q((*mconstraints)[ic]->Get_l_i());
                        //	tdata->mutex->Unlock(); // end critical section
                    }
            } else {
                for (unsigned int ic = tdata->constr_from; ic < tdata->constr_to; ic++)
                    (*mconstraints)[ic]->Set_l_i(0.);
            }

            //    Perform the solver iteration loops
            //
            for (int iter = 0; iter < tdata->solver->GetMaxIterations(); iter++) {
                // The iteration on all constraints
                //

                maxviolation = 0;
                maxdeltalambda = 0;
                i_friction_comp = 0;

                for (unsigned int ic = tdata->constr_from; ic < tdata->constr_to; ic++) {
                    // skip computations if constraint not active.
                    if ((*mconstraints)[ic]->IsActive()) {
                        // compute residual  c_i = [Cq_i]*q + b_i + cfm_i*l_i
                        double mresidual = (*mconstraints)[ic]->Compute_Cq_q() + (*mconstraints)[ic]->Get_b_i() +
                                           (*mconstraints)[ic]->Get_cfm_i() * (*mconstraints)[ic]->Get_l_i();

                        // true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
                        double candidate_violation = fabs((*mconstraints)[ic]->Violation(mresidual));

                        // compute:  delta_lambda = -(omega/g_i) * ([Cq_i]*q + b_i + cfm_i*l_i )
                        double deltal = (tdata->solver->GetOmega() / (*mconstraints)[ic]->Get_g_i()) * (-mresidual);

                        if ((*mconstraints)[ic]->GetMode() == CONSTRAINT_FRIC) {
                            candidate_violation = 0;
                            
                            // update:   lambda += delta_lambda;
                            old_lambda_friction[i_friction_comp] = (*mconstraints)[ic]->Get_l_i();
                            (*mconstraints)[ic]->Set_l_i(old_lambda_friction[i_friction_comp] + deltal);
                            i_friction_comp++;

                            if (i_friction_comp == 1)
                                candidate_violation = fabs(ChMin(0.0, mresidual));

                            if (i_friction_comp == 3) {
                                (*mconstraints)[ic - 2]->Project();  // the N normal component will take care of N,U,V
                                double new_lambda_0 = (*mconstraints)[ic - 2]->Get_l_i();
                                double new_lambda_1 = (*mconstraints)[ic - 1]->Get_l_i();
                                double new_lambda_2 = (*mconstraints)[ic - 0]->Get_l_i();
                                // Apply the smoothing: lambda= sharpness*lambda_new_projected +
                                // (1-sharpness)*lambda_old
                                if (tdata->solver->GetSharpnessLambda() != 1.0) {
                                    double shlambda = tdata->solver->GetSharpnessLambda();
                                    new_lambda_0 = shlambda * new_lambda_0 + (1.0 - shlambda) * old_lambda_friction[0];
                                    new_lambda_1 = shlambda * new_lambda_1 + (1.0 - shlambda) * old_lambda_friction[1];
                                    new_lambda_2 = shlambda * new_lambda_2 + (1.0 - shlambda) * old_lambda_friction[2];
                                    (*mconstraints)[ic - 2]->Set_l_i(new_lambda_0);
                                    (*mconstraints)[ic - 1]->Set_l_i(new_lambda_1);
                                    (*mconstraints)[ic - 0]->Set_l_i(new_lambda_2);
                                }
                                double true_delta_0 = new_lambda_0 - old_lambda_friction[0];
                                double true_delta_1 = new_lambda_1 - old_lambda_friction[1];
                                double true_delta_2 = new_lambda_2 - old_lambda_friction[2];
                                //	tdata->mutex->Lock();   // this avoids double writing on shared q vector
                                (*mconstraints)[ic - 2]->Increment_q(true_delta_0);
                                (*mconstraints)[ic - 1]->Increment_q(true_delta_1);
                                (*mconstraints)[ic - 0]->Increment_q(true_delta_2);
                                //	tdata->mutex->Unlock(); // end critical section
                                /*
								if (this->record_violation_history)
								{
									maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_0));
									maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_1));
									maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_2));
								}
								*/  //***TO DO***
                                i_friction_comp = 0;
                            }
                        } else {
                            // update:   lambda += delta_lambda;
                            double old_lambda = (*mconstraints)[ic]->Get_l_i();
                            (*mconstraints)[ic]->Set_l_i(old_lambda + deltal);

                            // If new lagrangian multiplier does not satisfy inequalities, project
                            // it into an admissible orthant (or, in general, onto an admissible set)
                            (*mconstraints)[ic]->Project();

                            // After projection, the lambda may have changed a bit..
                            double new_lambda = (*mconstraints)[ic]->Get_l_i();

                            // Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
                            if (tdata->solver->GetSharpnessLambda() != 1.0) {
                                double shlambda = tdata->solver->GetSharpnessLambda();
                                new_lambda = shlambda * new_lambda + (1.0 - shlambda) * old_lambda;
                                (*mconstraints)[ic]->Set_l_i(new_lambda);
                            }

                            double true_delta = new_lambda - old_lambda;

                            // For all items with variables, add the effect of incremented
                            // (and projected) lagrangian reactions:
                            tdata->mutex->Lock();  // this avoids double writing on shared q vector
                            (*mconstraints)[ic]->Increment_q(true_delta);
                            tdata->mutex->Unlock();  // end critical section
                            /*
							if (this->record_violation_history)
								maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta)); 
							*/                       //***TO DO***
                        }

                        maxviolation = ChMax(maxviolation, fabs(candidate_violation));

                    }  // end IsActive()

                }  // end loop on constraints

                // For recording into violaiton history, if debugging
                // if (this->record_violation_history)
                //	AtIterationEnd(maxviolation, maxdeltalambda, iter);  //***TO DO***

                // Terminate the loop if violation in constraints has been succesfully limited.
                if (maxviolation < tdata->solver->GetTolerance())
                    break;

            }  // end iteration loop

            break;
        }  // end stage

        default: { break; }
    }  // end stage  switching
}

// When the solver object is created, threads are also
// created and initialized, in 'wait' mode.

ChSolverSORmultithread::ChSolverSORmultithread(const char* uniquename,
                                               int nthreads,
                                               int mmax_iters,
                                               bool mwarm_start,
                                               double mtolerance,
                                               double momega)
    : ChIterativeSolver(mmax_iters, mwarm_start, mtolerance, momega) {
    ChThreadConstructionInfo create_args(uniquename, SolverThreadFunc, SolverMemoryFunc, nthreads);

    solver_threads = new ChThreads(create_args);
}

ChSolverSORmultithread::~ChSolverSORmultithread() {
    if (solver_threads) {
        solver_threads->flush();
        delete (solver_threads);
        solver_threads = 0;
    }
}

// The SOR solver process has been modified so that some
// parallelizable code has been moved to the SolverThreadFunc().
// So, the N threads must be started (each executing SolverThreadFunc() )
// and, after waiting for all them to be completed, with flush(), the
// solution is done.

double ChSolverSORmultithread::Solve(
    ChSystemDescriptor& sysd  ///< system description with constraints and variables
    ) {
    std::vector<ChConstraint*>& mconstraints = sysd.GetConstraintsList();
    std::vector<ChVariables*>& mvariables = sysd.GetVariablesList();

    double maxviolation = 0.;
    double maxdeltalambda = 0.;
    int i_friction_comp = 0;

    /////////////////////////////////////////////
    /// THE PARALLEL SOLVER, PERFORMED IN STAGES

    ChMutexSpinlock spinlock;

    // --0--  preparation:
    //        subdivide the workload to the threads and prepare their 'thread_data':

    int numthreads = this->solver_threads->getNumberOfThreads();
    std::vector<thread_data> mdataN(numthreads);

    int var_slice = 0;
    int constr_slice = 0;
    for (int nth = 0; nth < numthreads; nth++) {
        unsigned int var_from = var_slice;
        unsigned int var_to = var_from + ((unsigned int)mvariables.size() - var_from) / (numthreads - nth);
        unsigned int constr_from = constr_slice;
        unsigned int constr_to = constr_from + ((unsigned int)mconstraints.size() - constr_from) / (numthreads - nth);
        if (constr_to < mconstraints.size())  // do not slice the three contact multipliers (or six in case of rolling)
        {
            if (dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[constr_to]))
                constr_to++;
            if (dynamic_cast<ChConstraintTwoTuplesFrictionTall*>(mconstraints[constr_to]))
                constr_to++;
            if (constr_to < mconstraints.size()) {
                if (dynamic_cast<ChConstraintTwoTuplesRollingNall*>(mconstraints[constr_to]))
                    constr_to++;
                if (dynamic_cast<ChConstraintTwoTuplesRollingTall*>(mconstraints[constr_to]))
                    constr_to++;
                if (dynamic_cast<ChConstraintTwoTuplesRollingTall*>(mconstraints[constr_to]))
                    constr_to++;
            }
        }
        mdataN[nth].solver = this;
        mdataN[nth].mutex = &spinlock;
        mdataN[nth].constr_from = constr_from;
        mdataN[nth].constr_to = constr_to;
        mdataN[nth].var_from = var_from;
        mdataN[nth].var_to = var_to;
        mdataN[nth].mconstraints = &mconstraints;
        mdataN[nth].mvariables = &mvariables;

        var_slice = var_to;
        constr_slice = constr_to;
    }

    // LAUNCH THE PARALLEL COMPUTATION ON THREADS !!!!

    // --1--  stage:
    //        precompute aux variables in constraints.
    for (int nth = 0; nth < numthreads; nth++) {
        mdataN[nth].stage = thread_data::STAGE1_PREPARE;
        solver_threads->sendRequest(1, &mdataN[nth], nth);
    }
    //... must wait that the all the threads finished their stage!!!
    solver_threads->flush();

    // --2--  stage:
    //        add external forces and mass effects, on variables.
    for (int nth = 0; nth < numthreads; nth++) {
        mdataN[nth].stage = thread_data::STAGE2_ADDFORCES;
        solver_threads->sendRequest(1, &mdataN[nth], nth);
    }
    //... must wait that the all the threads finished their stage!!!
    solver_threads->flush();

    // --3--  stage:
    //        loop on constraints.
    for (int nth = 0; nth < numthreads; nth++) {
        mdataN[nth].stage = thread_data::STAGE3_LOOPCONSTRAINTS;
        solver_threads->sendRequest(1, &mdataN[nth], nth);
    }
    //... must wait that the all the threads finished their stage!!!
    solver_threads->flush();

    return 0;
}

void ChSolverSORmultithread::ChangeNumberOfThreads(int mthreads) {
    if (mthreads < 1)
        mthreads = 1;

    char mname[100];
    strncpy(mname, solver_threads->getUniqueName().c_str(), sizeof(mname)-1);

    solver_threads->flush();
    delete (solver_threads);
    solver_threads = 0;

    ChThreadConstructionInfo create_args(mname, SolverThreadFunc, SolverMemoryFunc, mthreads);

    solver_threads = new ChThreads(create_args);
}

} // end namespace chrono
