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
	CompressedMatrix<real> Nshur = data_manager->host_data.D_T * data_manager->host_data.M_invD;
	for (current_iteration = 0; current_iteration < max_iter; current_iteration++) {
		real omega = 1.0;

		for (int i = 0; i < data_manager->num_constraints; ++i) {
			ml[i] = ml[i] -
				omega * 1.0 / Nshur(i, i) * ((row(Nshur, i * 1 + 0), blaze::subvector(ml, 0, 1 * data_manager->num_constraints)) +
					data_manager->host_data.E[i] * ml[i] - r[i]);
			
		}
		Project(ml.data());
		gamma = ml;
		residual = 0;         // Res4Blaze(ml, mb);
		objective_value = 0;  // GetObjective(ml, mb);
	}

	return current_iteration;
}
