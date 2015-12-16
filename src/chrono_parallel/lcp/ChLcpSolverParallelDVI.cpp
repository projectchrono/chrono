#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

#include "chrono_parallel/solver/ChSolverAPGD.h"
#include "chrono_parallel/solver/ChSolverAPGDREF.h"

using namespace chrono;

#define CLEAR_RESERVE_RESIZE(M, nnz, rows, cols) \
    clear(M);                                    \
    M.reserve(nnz);                              \
    M.resize(rows, cols, false);

void ChLcpSolverParallelDVI::RunTimeStep() {
    LOG(INFO) << "ChLcpSolverParallelDVI::RunTimeStep";
    // Compute the offsets and number of constrains depending on the solver mode
    if (data_manager->settings.solver.solver_mode == NORMAL) {
        rigid_rigid.offset = 1;
        data_manager->num_unilaterals = 1 * data_manager->num_rigid_contacts;
    } else if (data_manager->settings.solver.solver_mode == SLIDING) {
        rigid_rigid.offset = 3;
        data_manager->num_unilaterals = 3 * data_manager->num_rigid_contacts;
    } else if (data_manager->settings.solver.solver_mode == SPINNING) {
        rigid_rigid.offset = 6;
        data_manager->num_unilaterals = 6 * data_manager->num_rigid_contacts;
    }

    uint num_rigid_fluid = data_manager->num_rigid_fluid_contacts * 3;
    uint num_fluid_fluid = data_manager->num_fluid_contacts * 3;

    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        num_fluid_fluid = data_manager->num_fluid_bodies;  // + ;
        if (data_manager->settings.fluid.enable_viscosity) {
            num_fluid_fluid += data_manager->num_fluid_bodies * 3;
        }
    }

    // This is the total number of constraints
    data_manager->num_constraints =
        data_manager->num_unilaterals + data_manager->num_bilaterals + num_rigid_fluid + num_fluid_fluid;

    // Generate the mass matrix and compute M_inv_k
    ComputeInvMassMatrix();
    ComputeMassMatrix();

    data_manager->host_data.gamma.resize(data_manager->num_constraints);
    data_manager->host_data.gamma.reset();

    // Perform any setup tasks for all constraint types
    rigid_rigid.Setup(data_manager);
    bilateral.Setup(data_manager);
    rigid_fluid.Setup(data_manager);
    fluid_fluid.Setup(data_manager);
    // Clear and reset solver history data and counters
    solver->current_iteration = 0;
    data_manager->measures.solver.total_iteration = 0;
    data_manager->measures.solver.maxd_hist.clear();
    data_manager->measures.solver.maxdeltalambda_hist.clear();
    // Set pointers to constraint objects and perform setup actions for solver
    solver->rigid_rigid = &rigid_rigid;
    solver->bilateral = &bilateral;
    solver->rigid_fluid = &rigid_fluid;
    solver->fluid_fluid = &fluid_fluid;
    solver->Setup(data_manager);

    ComputeD();
    ComputeE();
    ComputeR();

    ComputeN();

    // PreSolve();

    data_manager->system_timer.start("ChLcpSolverParallel_Solve");

    //  if (data_manager->settings.solver.max_iteration_bilateral > 0) {
    //    solver->SetMaxIterations(data_manager->settings.solver.max_iteration_bilateral);
    //    data_manager->settings.solver.local_solver_mode = BILATERAL;
    //    SetR();
    //    solver->Solve();
    //  }

    PerformStabilization();

    if (data_manager->settings.solver.solver_mode == NORMAL || data_manager->settings.solver.solver_mode == SLIDING ||
        data_manager->settings.solver.solver_mode == SPINNING) {
        if (data_manager->settings.solver.max_iteration_normal > 0) {
            solver->SetMaxIterations(data_manager->settings.solver.max_iteration_normal);
            data_manager->settings.solver.local_solver_mode = NORMAL;
            SetR();
            LOG(INFO) << "ChLcpSolverParallelDVI::RunTimeStep - Solve Normal";
            solver->Solve();
        }
    }
    if (data_manager->settings.solver.solver_mode == SLIDING || data_manager->settings.solver.solver_mode == SPINNING) {
        if (data_manager->settings.solver.max_iteration_sliding > 0) {
            solver->SetMaxIterations(data_manager->settings.solver.max_iteration_sliding);
            data_manager->settings.solver.local_solver_mode = SLIDING;
            SetR();
            LOG(INFO) << "ChLcpSolverParallelDVI::RunTimeStep - Solve Sliding";
            solver->Solve();
        }
    }
    if (data_manager->settings.solver.solver_mode == SPINNING) {
        if (data_manager->settings.solver.max_iteration_spinning > 0) {
            solver->SetMaxIterations(data_manager->settings.solver.max_iteration_spinning);
            data_manager->settings.solver.local_solver_mode = SPINNING;
            SetR();
            LOG(INFO) << "ChLcpSolverParallelDVI::RunTimeStep - Solve Spinning";
            solver->Solve();
        }
    }

    //    DynamicVector<real> temp(data_manager->num_rigid_bodies * 6, 0.0);
    //    DynamicVector<real> output(data_manager->num_rigid_contacts * 3, 0.0);
    //
    //    // DynamicVector<real> temp(data_manager->num_fluid_bodies * 3, 0.0);
    //    // DynamicVector<real> output(data_manager->num_fluid_bodies, 0.0);
    //
    //    /////
    //    double t1 = 0;
    //    {
    //        ChTimer<> timer;
    //        timer.start();
    //        rigid_rigid.Dx(data_manager->host_data.gamma, temp);
    //        rigid_rigid.D_Tx(temp, output);
    //        // fluid_fluid.Dx(data_manager->host_data.gamma, temp);
    //        // fluid_fluid.D_Tx(temp, output);
    //        timer.stop();
    //        t1 = timer();
    //    }
    //    ChTimer<> timer;
    //    timer.start();
    //    DynamicVector<real> compare =
    //        data_manager->host_data.D_T * data_manager->host_data.D * data_manager->host_data.gamma;
    //    timer.stop();
    //    std::cout << "time1: " << t1 << " time2: " << timer() << std::endl;
    //    /////

    data_manager->Fc_current = false;
    data_manager->system_timer.stop("ChLcpSolverParallel_Solve");
    fluid_fluid.ArtificialPressure();
    ComputeImpulses();

    for (int i = 0; i < data_manager->measures.solver.maxd_hist.size(); i++) {
        AtIterationEnd(data_manager->measures.solver.maxd_hist[i], data_manager->measures.solver.maxdeltalambda_hist[i],
                       i);
    }
    tot_iterations = data_manager->measures.solver.maxd_hist.size();

    LOG(TRACE) << "Solve Done: " << residual;
}

void ChLcpSolverParallelDVI::ComputeD() {
    LOG(INFO) << "ChLcpSolverParallelDVI::ComputeD()";
    data_manager->system_timer.start("ChLcpSolverParallel_D");
    uint num_constraints = data_manager->num_constraints;
    if (num_constraints <= 0) {
        return;
    }

    uint num_shafts = data_manager->num_shafts;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_dof = data_manager->num_dof;
    uint num_rigid_contacts = data_manager->num_rigid_contacts;
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    uint num_fluid_contacts = data_manager->num_fluid_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;
    uint nnz_bilaterals = data_manager->nnz_bilaterals;

    int nnz_normal = 6 * 2 * num_rigid_contacts;
    int nnz_tangential = 6 * 4 * num_rigid_contacts;
    int nnz_spinning = 6 * 3 * num_rigid_contacts;

    int num_normal = 1 * num_rigid_contacts;
    int num_tangential = 2 * num_rigid_contacts;
    int num_spinning = 3 * num_rigid_contacts;

    int nnz_rigid_fluid = 9 * 3 * num_rigid_fluid_contacts;
    int nnz_fluid_fluid = 6 * num_fluid_contacts;

    uint num_rigid_fluid = num_rigid_fluid_contacts * 3;
    uint num_fluid_fluid = num_fluid_contacts * 3;

    if (data_manager->settings.fluid.fluid_is_rigid == false) {
        int max_interactions = data_manager->settings.fluid.max_interactions;
        nnz_fluid_fluid = num_fluid_bodies * 6 * max_interactions;  // + num_fluid_bodies * 18 * max_interactions;
        num_fluid_fluid = num_fluid_bodies;                         // + num_fluid_bodies * 3;

        if (data_manager->settings.fluid.enable_viscosity) {
            nnz_fluid_fluid += data_manager->num_fluid_bodies * 18 * max_interactions;
            num_fluid_fluid += num_fluid_bodies * 3;
        }
    }

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    CompressedMatrix<real>& D = data_manager->host_data.D;
    CompressedMatrix<real>& M_invD = data_manager->host_data.M_invD;

    const CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;

    int nnz_total = nnz_bilaterals + nnz_rigid_fluid + nnz_fluid_fluid;
    int num_rows = num_bilaterals + num_rigid_fluid + num_fluid_fluid;

    switch (data_manager->settings.solver.solver_mode) {
        case NORMAL:
            nnz_total += nnz_normal;
            num_rows += num_normal;
            break;
        case SLIDING:
            nnz_total += nnz_normal + nnz_tangential;
            num_rows += num_normal + num_tangential;

            break;
        case SPINNING:
            nnz_total += nnz_normal + nnz_tangential + nnz_spinning;
            num_rows += num_normal + num_tangential + num_spinning;
            break;
    }

    CLEAR_RESERVE_RESIZE(D_T, nnz_total, num_rows, num_dof)
    CLEAR_RESERVE_RESIZE(D, nnz_total, num_dof, num_rows)
    CLEAR_RESERVE_RESIZE(M_invD, nnz_total, num_dof, num_rows)

    rigid_rigid.GenerateSparsity();
    bilateral.GenerateSparsity();
    rigid_fluid.GenerateSparsity();
    fluid_fluid.GenerateSparsity();
    rigid_rigid.Build_D();
    bilateral.Build_D();
    rigid_fluid.Build_D();
    fluid_fluid.Build_D();
    LOG(INFO) << "ChLcpSolverParallelDVI::ComputeD - D";

    D = trans(D_T);
    LOG(INFO) << "ChLcpSolverParallelDVI::ComputeD - M_invD";

    M_invD = M_inv * D;

    data_manager->system_timer.stop("ChLcpSolverParallel_D");
}

void ChLcpSolverParallelDVI::ComputeE() {
    LOG(INFO) << "ChLcpSolverParallelDVI::ComputeE()";
    data_manager->system_timer.start("ChLcpSolverParallel_E");
    if (data_manager->num_constraints <= 0) {
        return;
    }

    data_manager->host_data.E.resize(data_manager->num_constraints);
    reset(data_manager->host_data.E);

    rigid_rigid.Build_E();
    bilateral.Build_E();
    rigid_fluid.Build_E();
    data_manager->system_timer.stop("ChLcpSolverParallel_E");
}

void ChLcpSolverParallelDVI::ComputeR() {
    LOG(INFO) << "ChLcpSolverParallelDVI::ComputeR()";
    data_manager->system_timer.start("ChLcpSolverParallel_R");
    if (data_manager->num_constraints <= 0) {
        return;
    }

    const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
    const CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    DynamicVector<real>& R = data_manager->host_data.R_full;
    DynamicVector<real>& b = data_manager->host_data.b;

    b.resize(data_manager->num_constraints);
    reset(b);

    R.resize(data_manager->num_constraints);
    reset(R);

    rigid_rigid.Build_b();
    bilateral.Build_b();
    rigid_fluid.Build_b();
    fluid_fluid.Build_b();
    R = -b - D_T * M_invk;

    data_manager->system_timer.stop("ChLcpSolverParallel_R");
}

void ChLcpSolverParallelDVI::ComputeN() {
    if (data_manager->settings.solver.compute_N == false) {
        return;
    }

    LOG(INFO) << "ChLcpSolverParallelDVI::ComputeN";
    data_manager->system_timer.start("ChLcpSolverParallel_N");
    const CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    CompressedMatrix<real>& Nshur = data_manager->host_data.Nshur;
    const CompressedMatrix<real>& M_invD = data_manager->host_data.M_invD;
    const CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    Nshur = D_T * M_invD;
    data_manager->system_timer.stop("ChLcpSolverParallel_N");
}

void ChLcpSolverParallelDVI::SetR() {
    LOG(INFO) << "ChLcpSolverParallelDVI::SetR()";
    if (data_manager->num_constraints <= 0) {
        return;
    }

    DynamicVector<real>& R = data_manager->host_data.R;
    const DynamicVector<real>& R_full = data_manager->host_data.R_full;

    uint num_rigid_contacts = data_manager->num_rigid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    uint num_rigid_fluid = data_manager->num_rigid_fluid_contacts * 3;
    uint num_fluid_fluid = data_manager->num_fluid_contacts;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    R.resize(data_manager->num_constraints);
    reset(R);

    if (data_manager->settings.solver.local_solver_mode == data_manager->settings.solver.solver_mode) {
        R = R_full;
    } else {
        subvector(R, num_unilaterals, num_bilaterals) = subvector(R_full, num_unilaterals, num_bilaterals);
        subvector(R, num_unilaterals + num_bilaterals, num_rigid_fluid) =
            subvector(R_full, num_unilaterals + num_bilaterals, num_rigid_fluid);

        if (data_manager->settings.fluid.fluid_is_rigid == false) {
            subvector(R, num_unilaterals + num_bilaterals + num_rigid_fluid, num_fluid_bodies) =
                subvector(R_full, num_unilaterals + num_bilaterals + num_rigid_fluid, num_fluid_bodies);

        } else {
            subvector(R, num_unilaterals + num_bilaterals + num_rigid_fluid, num_fluid_fluid) =
                subvector(R_full, num_unilaterals + num_bilaterals + num_rigid_fluid, num_fluid_fluid);
        }

        switch (data_manager->settings.solver.local_solver_mode) {
            case BILATERAL: {
            } break;

            case NORMAL: {
                subvector(R, 0, num_rigid_contacts) = subvector(R_full, 0, num_rigid_contacts);
            } break;

            case SLIDING: {
                subvector(R, 0, num_rigid_contacts) = subvector(R_full, 0, num_rigid_contacts);
                subvector(R, num_rigid_contacts, num_rigid_contacts * 2) =
                    subvector(R_full, num_rigid_contacts, num_rigid_contacts * 2);
            } break;

            case SPINNING: {
                subvector(R, 0, num_rigid_contacts) = subvector(R_full, 0, num_rigid_contacts);
                subvector(R, num_rigid_contacts, num_rigid_contacts * 2) =
                    subvector(R_full, num_rigid_contacts, num_rigid_contacts * 2);
                subvector(R, num_rigid_contacts * 3, num_rigid_contacts * 3) =
                    subvector(R_full, num_rigid_contacts * 3, num_rigid_contacts * 3);
            } break;
        }
    }
}

void ChLcpSolverParallelDVI::ComputeImpulses() {
    LOG(INFO) << "ChLcpSolverParallelDVI::ComputeImpulses()";
    const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
    const DynamicVector<real>& gamma = data_manager->host_data.gamma;
    const CompressedMatrix<real>& M_invD = data_manager->host_data.M_invD;

    DynamicVector<real>& v = data_manager->host_data.v;

    if (data_manager->num_constraints > 0) {
        // Compute new velocity based on the lagrange multipliers
        v = M_invk + M_invD * gamma;
    } else {
        // When there are no constraints we need to still apply gravity and other
        // body forces!
        v = M_invk;
    }
}

void ChLcpSolverParallelDVI::PreSolve() {
    // Currently not supported, might be added back in the future
}

void ChLcpSolverParallelDVI::ChangeSolverType(SOLVERTYPE type) {
    data_manager->settings.solver.solver_type = type;

    if (this->solver) {
        delete (this->solver);
    }
    switch (type) {
        case APGD:
            solver = new ChSolverAPGD();
            break;
        case APGDREF:
            solver = new ChSolverAPGDREF();
            break;
    }
}
