#ifndef CHBASEGPU_H
#define CHBASEGPU_H

#include "ChParallelDefines.h"
#include "ChDataManager.h"
#include "math/ChParallelMath.h"
#include "math/ChThrustLinearAlgebra.h"

namespace chrono {

class ChApiGPU ChBaseParallel {
	public:
		ChBaseParallel() {
		}
		~ChBaseParallel() {
		}
		void Initialize();
	protected:
		ChParallelDataManager *data_container;
		int number_of_rigid;
		int number_of_rigid_rigid;
		int number_of_bilaterals;
		int number_of_constraints;
		int number_of_updates;

		real step_size;

		real inv_hpa;
		real inv_hhpa;

		real alpha;
		real contact_recovery_speed;
};

}

#endif
