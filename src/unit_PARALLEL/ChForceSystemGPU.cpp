#include "ChForceSystemGPU.h"

namespace chrono {

ChForceSystemGPU::ChForceSystemGPU() {
	//solver = new ChForceSolverGPU();
	param=0;
}

void ChForceSystemGPU::Init() {
}

void ChForceSystemGPU::Solve() {
	solver->ComputeForces();

}
}
