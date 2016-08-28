#include "chrono_parallel/solver/ChSolverParallel.h"
#include <blaze/math/SparseRow.h>
#include <blaze/math/CompressedVector.h>
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
	Project(ml.data());

	uint num_contacts = data_manager->num_constraints;
	CompressedMatrix<real> Nshur = data_manager->host_data.D_T * data_manager->host_data.M_invD;
	DynamicVector<real> D;
	D.resize(num_contacts, false);

	for (size_t i = 0; i < num_contacts; ++i) {
		Nshur(i, i) += data_manager->host_data.E[i];
		D[i] = 1.0 / (Nshur(i, i));
	}


	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		real omega = 1.0;

		for (int i = 0; i < data_manager->num_constraints; ++i) {
			ml[i] = ml[i] -
				omega * D[i] * ((row(Nshur, i * 1 + 0), blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) - r[i]);
		}
		Project(ml.data());
		gamma = ml;
		real gdiff = 1.0 / pow(data_manager->num_constraints, 2.0);
		ShurProduct(gamma, ml_old);

		objective_value = (ml, (0.5*ml_old - r));

		ml_old = ml_old - r;
		ml_old = gamma - gdiff * (ml_old);
		Project(ml_old.data());
		ml_old = (1.0 / gdiff) * (gamma - ml_old);

		residual = Sqrt((double)(ml_old, ml_old));

		AtIterationEnd(residual, objective_value);
	}

	return current_iteration;
}
