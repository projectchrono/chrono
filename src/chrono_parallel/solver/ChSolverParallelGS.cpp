#include "chrono_parallel/solver/ChSolverParallel.h"

#if BLAZE_MAJOR_VERSION == 2
#include <blaze/math/SparseRow.h>
#endif

#include <blaze/math/CompressedMatrix.h>

using namespace chrono;

uint ChSolverParallelGS::Solve(ChShurProduct& ShurProduct,
                               ChProjectConstraints& Project,
                               const uint max_iter,
                               const uint size,
                               const DynamicVector<real>& r,
                               DynamicVector<real>& gamma) {
    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;

    ml = gamma;
    ml_old = gamma;
    Project(ml.data());

    uint num_constraints = data_manager->num_constraints;
    uint num_contacts = data_manager->num_rigid_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;

    CompressedMatrix<real> Nshur = data_manager->host_data.D_T * data_manager->host_data.M_invD;
    DynamicVector<real> D;
    D.resize(num_constraints, false);

    for (int index = 0; index < (signed)data_manager->num_rigid_contacts; index++) {
        D[index] = Nshur(index, index) + Nshur(num_contacts + index * 2 + 0, num_contacts + index * 2 + 0) +
                   Nshur(num_contacts + index * 2 + 1, num_contacts + index * 2 + 1);
        D[index] = 3.0 / D[index];
        D[num_contacts + index * 2 + 0] = D[index];
        D[num_contacts + index * 2 + 1] = D[index];
    }
    uint offset = data_manager->num_unilaterals;

    for (size_t i = 0; i < num_bilaterals; i++) {
        D[offset + i] = 1.0 / Nshur(offset + i, offset + i);
    }

    if (data_manager->num_fluid_bodies) {
        offset += data_manager->num_bilaterals;

        for (size_t i = 0; i < data_manager->num_fluid_bodies; i++) {
            D[offset + i] = 1.0 / Nshur(offset + i, offset + i);
        }

        offset += data_manager->num_fluid_bodies;

        for (size_t i = 0; i < data_manager->num_rigid_fluid_contacts; i++) {
            if (data_manager->node_container->contact_mu == 0) {
                D[offset + i] = Nshur(offset + i, offset + i);
                D[offset + i] = 3.0 / D[offset + i];
            } else {
                D[offset + i] = Nshur(offset + i, offset + i) + Nshur(offset + i * 2 + 0, offset + i * 2 + 0) +
                                Nshur(offset + i * 2 + 1, offset + i * 2 + 1);
                D[offset + i] = 3.0 / D[offset + i];
                D[offset + i * 2 + 0] = D[offset + i];
                D[offset + i * 2 + 1] = D[offset + i];
            }
        }
    }
    int nc = data_manager->num_rigid_contacts;
    int nfc = data_manager->num_rigid_fluid_contacts;
    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        real omega = .2;
        offset = 0;

        for (int i = 0; i < (signed)data_manager->num_rigid_contacts; i++) {
            ml[offset + i] = ml[offset + i] -
                             omega * D[offset + i] * ((row(Nshur, offset + i * 1 + 0),
                                                       blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                      r[offset + i]);
            ml[nc + i * 2 + 0] =
                ml[nc + i * 2 + 0] -
                omega * D[nc + i * 2 + 0] *
                    ((row(Nshur, nc + i * 2 + 0), blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                     r[nc + i * 2 + 0]);
            ml[nc + i * 2 + 1] =
                ml[nc + i * 2 + 1] -
                omega * D[nc + i * 2 + 1] *
                    ((row(Nshur, nc + i * 2 + 1), blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                     r[nc + i * 2 + 1]);

            data_manager->rigid_rigid->Project_Single(i, ml.data());
        }

        offset += data_manager->num_unilaterals;

        for (size_t i = 0; i < num_bilaterals; i++) {
            ml[offset + i] = ml[offset + i] -
                             omega * D[offset + i] * ((row(Nshur, offset + i * 1 + 0),
                                                       blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                      r[offset + i]);
        }
        if (data_manager->num_fluid_bodies) {
            offset += data_manager->num_bilaterals;

            for (size_t i = 0; i < data_manager->num_fluid_bodies; i++) {
                ml[offset + i] = ml[offset + i] -
                                 omega * D[offset + i] * ((row(Nshur, offset + i * 1 + 0),
                                                           blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                          r[offset + i]);
            }

            offset += data_manager->num_fluid_bodies;

            for (size_t i = 0; i < data_manager->num_rigid_fluid_contacts; i++) {
                ml[offset + i] = ml[offset + i] -
                                 omega * D[offset + i] * ((row(Nshur, offset + i * 1 + 0),
                                                           blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                          r[offset + i]);
                if (data_manager->node_container->contact_mu != 0) {
                    ml[offset + nfc + i * 2 + 0] = ml[offset + nfc + i * 2 + 0] -
                                                   omega * D[offset + nfc + i * 2 + 0] *
                                                       ((row(Nshur, offset + nfc + i * 2 + 0),
                                                         blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                        r[offset + nfc + i * 2 + 0]);
                    ml[offset + nfc + i * 2 + 1] = ml[offset + nfc + i * 2 + 1] -
                                                   omega * D[offset + nfc + i * 2 + 1] *
                                                       ((row(Nshur, offset + nfc + i * 2 + 1),
                                                         blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) -
                                                        r[offset + nfc + i * 2 + 1]);
                }
                data_manager->node_container->Project(ml.data());
            }
        }

        gamma = ml;
        real gdiff = 1.0 / pow(data_manager->num_constraints, 2.0);
        ShurProduct(gamma, ml_old);

        objective_value = (ml, (0.5 * ml_old - r));

        ml_old = ml_old - r;
        ml_old = gamma - gdiff * (ml_old);
        Project(ml_old.data());
        ml_old = (1.0 / gdiff) * (gamma - ml_old);

        residual = Sqrt((double)(ml_old, ml_old));

        AtIterationEnd(residual, objective_value);

        if (data_manager->settings.solver.test_objective) {
            if (objective_value <= data_manager->settings.solver.tolerance_objective) {
                break;
            }
        } else {
            if (residual < data_manager->settings.solver.tol_speed) {
                break;
            }
        }
    }

    return current_iteration;
}
