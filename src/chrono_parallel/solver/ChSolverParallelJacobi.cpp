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
	real& residual = data_manager->measures.solver.residual;
	real& objective_value = data_manager->measures.solver.objective_value;

	uint num_contacts = data_manager->num_constraints;
	ml = gamma;
	ml_old = gamma;
	CompressedMatrix<real> Nshur = data_manager->host_data.D_T * data_manager->host_data.M_invD;
	DynamicVector<real> D;
	D.resize(num_contacts, false);

	for (size_t i = 0; i < num_contacts; ++i) {
		D[i] = 1.0 / (Nshur(i, i) + data_manager->host_data.E[i]);
		Nshur(i, i) = 0;
	}
	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		real omega = 1.0 / 3.0;
		ml = omega * D*(r - Nshur*ml) + (1 - omega) * ml;
		Project(ml.data());


		real gdiff = 1.0 / pow(data_manager->num_constraints, 2.0);
		ShurProduct(gamma, ml_old);

		objective_value = (ml, (0.5*ml_old - r));

		ml_old = ml_old - r;
		ml_old = gamma - gdiff * (ml_old);
		Project(ml_old.data());
		ml_old = (1.0 / gdiff) * (gamma - ml_old);

		residual =  Sqrt((double)(ml_old, ml_old));

		AtIterationEnd(residual, objective_value);

		if (data_manager->settings.solver.test_objective) {
			if (objective_value <= data_manager->settings.solver.tolerance_objective) {
				break;
			}
		}
		else {
			if (residual < data_manager->settings.solver.tol_speed) {
				break;
			}
		}

	}
	gamma = ml;
	return current_iteration;
}
