#include "chrono_parallel/solver/ChSolverParallel.h"

using namespace chrono;

ChSolverParallel::ChSolverParallel() {
    current_iteration = 0;
    rigid_rigid = NULL;
    three_dof = NULL;
    fem = NULL;
    bilateral = NULL;
}

void ChSolverParallel::Project(real* gamma) {
    data_manager->system_timer.start("ChSolverParallel_Project");
    rigid_rigid->Project(gamma);
    three_dof->Project(gamma);
    fem->Project(gamma);
    data_manager->system_timer.stop("ChSolverParallel_Project");
}

void ChSolverParallel::Project_Single(int index, real* gamma) {
    data_manager->system_timer.start("ChSolverParallel_Project");
    rigid_rigid->Project_Single(index, gamma);
    data_manager->system_timer.stop("ChSolverParallel_Project");
}
//=================================================================================================================================

void ChSolverParallel::ComputeSRhs(custom_vector<real>& gamma,
                                   const custom_vector<real>& rhs,
                                   custom_vector<real3>& vel_data,
                                   custom_vector<real3>& omg_data,
                                   custom_vector<real>& b) {
    // TODO change SHRS to use blaze
    // ComputeImpulses(gamma, vel_data, omg_data);
    // rigid_rigid->ComputeS(rhs, vel_data, omg_data, b);
}

bool init_eigen_vec = 0;

real ChSolverParallel::LargestEigenValue(ChShurProduct& ShurProduct, DynamicVector<real>& temp, real lambda) {
    eigen_vec.resize(temp.size());
    if (init_eigen_vec == 0) {
        eigen_vec = 1;
        init_eigen_vec = 1;
    }

    if (lambda != 0) {
        ShurProduct(eigen_vec, temp);
        eigen_vec = 1.0 / lambda * temp;
    }
    real lambda_old;

    for (int i = 0; i < data_manager->settings.solver.max_power_iteration; i++) {
        ShurProduct(eigen_vec, temp);
        lambda = Sqrt((temp, temp));
        if (lambda == 0) {
            return 1;
        }
        printf("Lambda: %.20f \n", lambda);
        if (Abs(lambda_old - lambda) < data_manager->settings.solver.power_iter_tolerance) {
            break;
        }
        eigen_vec = 1.0 / lambda * temp;
        lambda_old = lambda;
    }
    return lambda;
}

void ChSolverParallel::InnerSolve() {
    data_manager->mpm_container->InnerSolve();
}
