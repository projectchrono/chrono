#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#include "chrono/physics/ChBody.h"

using namespace chrono;

ChIterativeSolverParallel::ChIterativeSolverParallel(ChParallelDataManager* dc) : data_manager(dc) {
    tolerance = 1e-7;
    record_violation_history = true;
    warm_start = false;
    solver = new ChSolverParallelAPGD();
    bilateral_solver = new ChSolverParallelMinRes();
    data_manager->rigid_rigid = new ChConstraintRigidRigid();
    data_manager->bilateral = new ChConstraintBilateral();
}

ChIterativeSolverParallel::~ChIterativeSolverParallel() {
    delete solver;
    delete bilateral_solver;
}

void ChIterativeSolverParallel::ComputeInvMassMatrix() {
    LOG(INFO) << "ChIterativeSolverParallel::ComputeInvMassMatrix()";
    uint num_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_fea_nodes = data_manager->num_fea_nodes;
    uint num_dof = data_manager->num_dof;
    bool use_full_inertia_tensor = data_manager->settings.solver.use_full_inertia_tensor;
    const custom_vector<real>& shaft_inr = data_manager->host_data.shaft_inr;

    std::vector<std::shared_ptr<ChBody> >* body_list = data_manager->body_list;
    std::vector<std::shared_ptr<ChLink> >* link_list = data_manager->link_list;
    std::vector<std::shared_ptr<ChPhysicsItem> >* other_physics_list = data_manager->other_physics_list;

    const DynamicVector<real>& hf = data_manager->host_data.hf;
    const DynamicVector<real>& v = data_manager->host_data.v;

    DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;

    if (M_inv.capacity() > 0) {
        clear(M_inv);
    }

    // Each rigid object has 3 mass entries and 9 inertia entries
    // Each shaft has one inertia entry
    M_inv.reserve(num_bodies * 12 + num_shafts * 1 + num_fluid_bodies * 3 + num_fea_nodes * 3);
    // The mass matrix is square and each rigid body has 6 DOF
    // Shafts have one DOF
    M_inv.resize(num_dof, num_dof);

    for (int i = 0; i < (signed)num_bodies; i++) {
        if (data_manager->host_data.active_rigid[i]) {
            real inv_mass = 1.0 / body_list->at(i)->GetMass();
            ChMatrix33<>& body_inv_inr = body_list->at(i)->VariablesBody().GetBodyInvInertia();

            M_inv.append(i * 6 + 0, i * 6 + 0, inv_mass);
            M_inv.finalize(i * 6 + 0);
            M_inv.append(i * 6 + 1, i * 6 + 1, inv_mass);
            M_inv.finalize(i * 6 + 1);
            M_inv.append(i * 6 + 2, i * 6 + 2, inv_mass);
            M_inv.finalize(i * 6 + 2);

            M_inv.append(i * 6 + 3, i * 6 + 3, body_inv_inr.GetElement(0, 0));
            if (use_full_inertia_tensor) {
                M_inv.append(i * 6 + 3, i * 6 + 4, body_inv_inr.GetElement(0, 1));
                M_inv.append(i * 6 + 3, i * 6 + 5, body_inv_inr.GetElement(0, 2));
            }
            M_inv.finalize(i * 6 + 3);
            if (use_full_inertia_tensor) {
                M_inv.append(i * 6 + 4, i * 6 + 3, body_inv_inr.GetElement(1, 0));
            }
            M_inv.append(i * 6 + 4, i * 6 + 4, body_inv_inr.GetElement(1, 1));
            if (use_full_inertia_tensor) {
                M_inv.append(i * 6 + 4, i * 6 + 5, body_inv_inr.GetElement(1, 2));
            }
            M_inv.finalize(i * 6 + 4);
            if (use_full_inertia_tensor) {
                M_inv.append(i * 6 + 5, i * 6 + 3, body_inv_inr.GetElement(2, 0));
                M_inv.append(i * 6 + 5, i * 6 + 4, body_inv_inr.GetElement(2, 1));
            }
            M_inv.append(i * 6 + 5, i * 6 + 5, body_inv_inr.GetElement(2, 2));
            M_inv.finalize(i * 6 + 5);
        } else {
            M_inv.finalize(i * 6 + 0);
            M_inv.finalize(i * 6 + 1);
            M_inv.finalize(i * 6 + 2);
            M_inv.finalize(i * 6 + 3);
            M_inv.finalize(i * 6 + 4);
            M_inv.finalize(i * 6 + 5);
        }
    }

    for (int i = 0; i < (signed)num_shafts; i++) {
        M_inv.append(num_bodies * 6 + i, num_bodies * 6 + i, shaft_inr[i]);
        M_inv.finalize(num_bodies * 6 + i);
    }

    int offset = num_bodies * 6 + num_shafts;
    data_manager->node_container->ComputeInvMass(offset);
    data_manager->fea_container->ComputeInvMass(offset + num_fluid_bodies * 3);

    M_invk = v + M_inv * hf;
}

void ChIterativeSolverParallel::ComputeMassMatrix() {
    LOG(INFO) << "ChIterativeSolverParallel::ComputeMassMatrix()";
    uint num_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_fea_nodes = data_manager->num_fea_nodes;
    uint num_dof = data_manager->num_dof;
    bool use_full_inertia_tensor = data_manager->settings.solver.use_full_inertia_tensor;
    const custom_vector<real>& shaft_inr = data_manager->host_data.shaft_inr;

    std::vector<std::shared_ptr<ChBody> >* body_list = data_manager->body_list;
    std::vector<std::shared_ptr<ChLink> >* link_list = data_manager->link_list;
    std::vector<std::shared_ptr<ChPhysicsItem> >* other_physics_list = data_manager->other_physics_list;

    CompressedMatrix<real>& M = data_manager->host_data.M;

    if (M.capacity() > 0) {
        clear(M);
    }

    // Each rigid object has 3 mass entries and 9 inertia entries
    // Each shaft has one inertia entry
    M.reserve(num_bodies * 12 + num_shafts * 1 + num_fluid_bodies * 3 + num_fea_nodes * 3);
    // The mass matrix is square and each rigid body has 6 DOF
    // Shafts have one DOF
    M.resize(num_dof, num_dof);

    for (int i = 0; i < (signed)num_bodies; i++) {
        if (data_manager->host_data.active_rigid[i]) {
            real mass = body_list->at(i)->GetMass();
            ChMatrix33<>& body_inr = body_list->at(i)->VariablesBody().GetBodyInertia();

            M.append(i * 6 + 0, i * 6 + 0, mass);
            M.finalize(i * 6 + 0);
            M.append(i * 6 + 1, i * 6 + 1, mass);
            M.finalize(i * 6 + 1);
            M.append(i * 6 + 2, i * 6 + 2, mass);
            M.finalize(i * 6 + 2);

            M.append(i * 6 + 3, i * 6 + 3, body_inr.GetElement(0, 0));
            if (use_full_inertia_tensor) {
                M.append(i * 6 + 3, i * 6 + 4, body_inr.GetElement(0, 1));
                M.append(i * 6 + 3, i * 6 + 5, body_inr.GetElement(0, 2));
            }
            M.finalize(i * 6 + 3);
            if (use_full_inertia_tensor) {
                M.append(i * 6 + 4, i * 6 + 3, body_inr.GetElement(1, 0));
            }
            M.append(i * 6 + 4, i * 6 + 4, body_inr.GetElement(1, 1));
            if (use_full_inertia_tensor) {
                M.append(i * 6 + 4, i * 6 + 5, body_inr.GetElement(1, 2));
            }
            M.finalize(i * 6 + 4);
            if (use_full_inertia_tensor) {
                M.append(i * 6 + 5, i * 6 + 3, body_inr.GetElement(2, 0));
                M.append(i * 6 + 5, i * 6 + 4, body_inr.GetElement(2, 1));
            }
            M.append(i * 6 + 5, i * 6 + 5, body_inr.GetElement(2, 2));
            M.finalize(i * 6 + 5);
        } else {
            M.finalize(i * 6 + 0);
            M.finalize(i * 6 + 1);
            M.finalize(i * 6 + 2);
            M.finalize(i * 6 + 3);
            M.finalize(i * 6 + 4);
            M.finalize(i * 6 + 5);
        }
    }

    for (int i = 0; i < (signed)num_shafts; i++) {
        M.append(num_bodies * 6 + i, num_bodies * 6 + i, 1.0 / shaft_inr[i]);
        M.finalize(num_bodies * 6 + i);
    }

    int offset = num_bodies * 6 + num_shafts;
    data_manager->node_container->ComputeMass(offset);
    data_manager->fea_container->ComputeMass(offset + num_fluid_bodies * 3);
}

void ChIterativeSolverParallel::PerformStabilization() {
    LOG(INFO) << "ChIterativeSolverParallel::PerformStabilization";
    const DynamicVector<real>& R_full = data_manager->host_data.R_full;
    DynamicVector<real>& gamma = data_manager->host_data.gamma;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;

    if (data_manager->settings.solver.max_iteration_bilateral > 0 && num_bilaterals > 0) {
        const DynamicVector<real> R_b = blaze::subvector(R_full, num_unilaterals, num_bilaterals);
        DynamicVector<real> gamma_b = blaze::subvector(gamma, num_unilaterals, num_bilaterals);

        data_manager->system_timer.start("ChIterativeSolverParallel_Stab");

        data_manager->measures.solver.total_iteration +=
            bilateral_solver->Solve(ShurProductBilateral,                                   //
                                    ProjectNone,                                            //
                                    data_manager->settings.solver.max_iteration_bilateral,  //
                                    num_bilaterals,                                         //
                                    R_b,                                                    //
                                    gamma_b);                                               //
        blaze::subvector(gamma, num_unilaterals, num_bilaterals) = gamma_b;
    }
    if (data_manager->settings.solver.max_iteration_fem > 0 && data_manager->num_fea_tets > 0) {
        uint num_3dof_3dof = data_manager->node_container->GetNumConstraints();
        uint start_tet = data_manager->num_unilaterals + data_manager->num_bilaterals + num_3dof_3dof;
        int num_constraints = data_manager->num_fea_tets * (6 + 1);
        uint start_nodes =
            data_manager->num_rigid_bodies * 6 + data_manager->num_shafts + data_manager->num_fluid_bodies * 3;

        const DynamicVector<real> R_fem = blaze::subvector(R_full, start_tet, num_constraints);
        DynamicVector<real> gamma_fem = blaze::subvector(gamma, start_tet, num_constraints);

        data_manager->measures.solver.total_iteration +=
            bilateral_solver->Solve(ShurProductFEM,                                   //
                                    ProjectNone,                                      //
                                    data_manager->settings.solver.max_iteration_fem,  //
                                    num_constraints,                                  //
                                    R_fem,                                            //
                                    gamma_fem);                                       //
        blaze::subvector(gamma, start_tet, num_constraints) = gamma_fem;
    }
    data_manager->system_timer.stop("ChIterativeSolverParallel_Stab");
}

real ChIterativeSolverParallel::GetResidual() {
    return data_manager->measures.solver.maxd_hist.size() > 0 ? data_manager->measures.solver.maxd_hist.back() : 0.0;
}