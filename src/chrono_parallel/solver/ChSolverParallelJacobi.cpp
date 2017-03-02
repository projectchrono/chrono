#include "chrono_parallel/solver/ChSolverParallel.h"

#if BLAZE_MAJOR_VERSION == 2
#include <blaze/math/SparseRow.h>
#endif
#include <blaze/math/CompressedMatrix.h>

using namespace chrono;

uint ChSolverParallelJacobi::Solve(ChShurProduct& ShurProduct,
                                   ChProjectConstraints& Project,
                                   const uint max_iter,
                                   const uint size,
                                   const DynamicVector<real>& r,
                                   DynamicVector<real>& gamma) {
    if (size == 0) {
        return 0;
    }

    real& residual = data_manager->measures.solver.residual;
    real& objective_value = data_manager->measures.solver.objective_value;

    uint num_constraints = data_manager->num_constraints;
    uint num_contacts = data_manager->num_rigid_contacts;
    uint num_bilaterals = data_manager->num_bilaterals;
    ml = gamma;
    ml_old = gamma;
    DynamicVector<real> temp;
    temp.resize(size);
    DynamicVector<real> deltal;
    deltal.resize(size);
    CompressedMatrix<real> Nshur = data_manager->host_data.D_T * data_manager->host_data.M_invD;
    DynamicVector<real> D;
    D.resize(num_constraints, false);
    real theta = 1;
    real thetaNew = theta;
    real Beta = 0;
    // real eignenval = LargestEigenValue(ShurProduct, temp);
    // Rigid contacts
    // Bilaterals
    // num fluid bodies
    // rigid fluid norm
    // rigid fluid tan

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

    for (current_iteration = 0; current_iteration < (signed)max_iter; current_iteration++) {
        real omega = .2;  // 2.0 / eignenval;//1.0 / 3.0;
        ml = ml_old - omega * D * (Nshur * ml_old - r);

        Project(ml.data());
        gamma = ml;

        ml_old = ml;

        real gdiff = 1.0 / pow(data_manager->num_constraints, 2.0);
        ShurProduct(gamma, temp);

        objective_value = (gamma, (0.5 * temp - r));

        temp = temp - r;
        temp = gamma - gdiff * (temp);
        Project(temp.data());
        temp = (1.0 / gdiff) * (gamma - temp);

        residual = Sqrt((double)(temp, temp));

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