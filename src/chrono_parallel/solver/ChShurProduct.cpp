#include "chrono_parallel/solver/ChSolverParallel.h"

using namespace chrono;

ChShurProduct::ChShurProduct() {
    data_manager = 0;
}
void ChShurProduct::operator()(const DynamicVector<real>& x, DynamicVector<real>& output) {
    data_manager->system_timer.start("ShurProduct");

    const DynamicVector<real>& E = data_manager->host_data.E;

    uint num_rigid_contacts = data_manager->num_rigid_contacts;
    uint num_rigid_fluid_contacts = data_manager->num_rigid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    output.reset();

    const CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    const CompressedMatrix<real>& Nshur = data_manager->host_data.Nshur;

    if (data_manager->settings.solver.local_solver_mode == data_manager->settings.solver.solver_mode) {
        if (data_manager->settings.solver.compute_N) {
            output = Nshur * x + E * x;
        } else {
            output = D_T * data_manager->host_data.M_invD * x + E * x;
        }

    } else {
        const SubMatrixType& D_n_T = _DNT_;
        const SubMatrixType& D_b_T = _DBT_;
        const SubMatrixType& M_invD_n = _MINVDN_;
        const SubMatrixType& M_invD_b = _MINVDB_;

        SubVectorType o_b = subvector(output, num_unilaterals, num_bilaterals);
        ConstSubVectorType x_b = subvector(x, num_unilaterals, num_bilaterals);
        ConstSubVectorType E_b = subvector(E, num_unilaterals, num_bilaterals);

        SubVectorType o_n = subvector(output, 0, num_rigid_contacts);
        ConstSubVectorType x_n = subvector(x, 0, num_rigid_contacts);
        ConstSubVectorType E_n = subvector(E, 0, num_rigid_contacts);

        switch (data_manager->settings.solver.local_solver_mode) {
            case SolverMode::BILATERAL: {
                o_b = D_b_T * (M_invD_b * x_b) + E_b * x_b;
            } break;

            case SolverMode::NORMAL: {
                blaze::DynamicVector<real> tmp = M_invD_b * x_b + M_invD_n * x_n;
                o_b = D_b_T * tmp + E_b * x_b;
                o_n = D_n_T * tmp + E_n * x_n;
            } break;

            case SolverMode::SLIDING: {
                const SubMatrixType& D_t_T = _DTT_;
                const SubMatrixType& M_invD_t = _MINVDT_;
                SubVectorType o_t = subvector(output, num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType x_t = subvector(x, num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType E_t = subvector(E, num_rigid_contacts, num_rigid_contacts * 2);

                blaze::DynamicVector<real> tmp = M_invD_b * x_b + M_invD_n * x_n + M_invD_t * x_t;
                o_b = D_b_T * tmp + E_b * x_b;
                o_n = D_n_T * tmp + E_n * x_n;
                o_t = D_t_T * tmp + E_t * x_t;

            } break;

            case SolverMode::SPINNING: {
                const SubMatrixType& D_t_T = _DTT_;
                const SubMatrixType& D_s_T = _DST_;
                const SubMatrixType& M_invD_t = _MINVDT_;
                const SubMatrixType& M_invD_s = _MINVDS_;
                SubVectorType o_t = subvector(output, num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType x_t = subvector(x, num_rigid_contacts, num_rigid_contacts * 2);
                ConstSubVectorType E_t = subvector(E, num_rigid_contacts, num_rigid_contacts * 2);

                SubVectorType o_s = subvector(output, num_rigid_contacts * 3, num_rigid_contacts * 3);
                ConstSubVectorType x_s = subvector(x, num_rigid_contacts * 3, num_rigid_contacts * 3);
                ConstSubVectorType E_s = subvector(E, num_rigid_contacts * 3, num_rigid_contacts * 3);

                blaze::DynamicVector<real> tmp = M_invD_b * x_b + M_invD_n * x_n + M_invD_t * x_t + M_invD_s * x_s;
                o_b = D_b_T * tmp + E_b * x_b;
                o_n = D_n_T * tmp + E_n * x_n;
                o_t = D_t_T * tmp + E_t * x_t;
                o_s = D_s_T * tmp + E_s * x_s;

            } break;
        }
    }
    data_manager->system_timer.stop("ShurProduct");
}

void ChShurProductBilateral::Setup(ChParallelDataManager* data_container_) {
    ChShurProduct::Setup(data_container_);
    if (data_manager->num_bilaterals == 0) {
        return;
    }
    NshurB = _DBT_ * _MINVDB_;
}

void ChShurProductBilateral::operator()(const DynamicVector<real>& x, DynamicVector<real>& output) {
    output = NshurB * x;
}

void ChShurProductFEM::Setup(ChParallelDataManager* data_container_) {
    ChShurProduct::Setup(data_container_);
    //    if (data_manager->num_fea_tets == 0) {
    //        return;
    //    }
    //    // start row, start column
    //    // num rows, num columns
    //
    //    uint num_3dof_3dof = data_manager->node_container->GetNumConstraints();
    //    uint start_tet = data_manager->num_unilaterals + data_manager->num_bilaterals + num_3dof_3dof;
    //    int num_constraints = data_manager->num_fea_tets * (6 + 1);
    //    uint start_nodes =
    //        data_manager->num_rigid_bodies * 6 + data_manager->num_shafts + data_manager->num_fluid_bodies * 3;
    //    NshurB = submatrix(data_manager->host_data.D_T, start_tet, start_nodes, num_constraints,
    //                       data_manager->num_fea_nodes * 3) *
    //             submatrix(data_manager->host_data.M_invD, start_nodes, start_tet, data_manager->num_fea_nodes * 3,
    //                       num_constraints);
}

void ChShurProductFEM::operator()(const DynamicVector<real>& x, DynamicVector<real>& output) {
    uint num_3dof_3dof = data_manager->node_container->GetNumConstraints();
    uint start_tet = data_manager->num_unilaterals + data_manager->num_bilaterals + num_3dof_3dof;
    int num_constraints = data_manager->num_fea_tets * (6 + 1);
    uint start_nodes =
        data_manager->num_rigid_bodies * 6 + data_manager->num_shafts + data_manager->num_fluid_bodies * 3;
    output = submatrix(data_manager->host_data.D_T, start_tet, start_nodes, num_constraints,
                       data_manager->num_fea_nodes * 3) *
                 submatrix(data_manager->host_data.M_invD, start_nodes, start_tet, data_manager->num_fea_nodes * 3,
                           num_constraints) *
                 x +
             blaze::subvector(data_manager->host_data.E, start_tet, num_constraints) * x;
}
