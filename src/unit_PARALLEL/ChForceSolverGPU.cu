#include "ChForceSolverGPU.h"

using namespace chrono;
__constant__ unsigned int number_of_bodies_const;
__constant__ float3 gravity_const;

__global__ void KernelForces(float3* aux, float3* forces) {
	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i < number_of_bodies_const) {
		forces[i]=gravity_const/aux[i].z;
	}
}

ChForceSolverGPU::~ChForceSolverGPU() {
}
void ChForceSolverGPU::CD() {
}

void ChForceSolverGPU::ComputeForces() {

	int number_of_bodies = data_container->number_of_objects;
	COPY_TO_CONST_MEM(number_of_bodies);
	COPY_TO_CONST_MEM(gravity);
KernelForces<<< BLOCKS(number_of_bodies), THREADS >>>(
		CASTF3(data_container->device_aux_data),
		CASTF3(data_container->device_frc_data));
}

