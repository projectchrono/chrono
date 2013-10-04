#include "ChIntegratorGPU.h"
using namespace chrono;
__constant__ uint number_of_objects_const;
__constant__ real step_size_const;
__host__ __device__ void function_Integrate_Timestep_Semi_Implicit(uint& index, real& step_size, bool* active, real3* acc, real4* rot, real3* vel, real3* omega, real3* pos, real3* lim) {
	real3 velocity = vel[index];

	if (active[index] == 0) {
		return;
	}

	// Do 1st order integration of quaternion position as q[t+dt] = qw_abs^(dt) * q[dt] = q[dt] * qw_local^(dt)
	// where qw^(dt) is the quaternion { cos(0.5|w|), wx/|w| sin(0.5|w|), wy/|w| sin(0.5|w|), wz/|w| sin(0.5|w|)}^dt
	// that is argument of sine and cosines are multiplied by dt.
	real3 omg = omega[index];
	real3 limits = lim[index];
	real wlen = length(omg);
//    if (limits.x == 1) {
//        real w = 2.0 * wlen;
//
//        if (w > limits.z) {
//            omg = omg * limits.z / w;
//            wlen = sqrt(dot3(omg, omg));
//        }
//
//        real v = length(velocity);
//
//        if (v > limits.y) {
//            velocity = velocity * limits.y / v;
//        }
//
//        vel[index] = velocity;
//        omega[index] = omg;
//    }
	//printf("%f, %f, %f ", pos[index].x,pos[index].y,pos[index].z);
	pos[index] = pos[index] + velocity * step_size; // Do 1st order integration of linear speeds

	real3 newwel_abs = MatMult(AMat(rot[index]),omg);
	real mangle = length(newwel_abs)*step_size;
	newwel_abs=normalize(newwel_abs);
	real4 mdeltarot = Q_from_AngAxis(mangle, newwel_abs);

	//printf("%f, %f, %f\n", pos[index].x,pos[index].y,pos[index].z);
	//real4 Rw = (abs(wlen) > 10e-10) ? Q_from_AngAxis(step_size * wlen, omg / wlen) : R4(1, 0, 0, 0); // to avoid singularity for near zero angular speed
	//Rw = normalize(Rw);
	real4 mq = mult2(mdeltarot,rot[index]);
	//mq = mq /sqrt(dot(mq, mq));
	rot[index] = mq;
	acc[index] = (velocity - acc[index]) / step_size;
}
__global__ void device_Integrate_Timestep_Semi_Implicit(bool* active, real3* acc, real4* rot, real3* vel, real3* omega, real3* pos, real3* lim) {
	INIT_CHECK_THREAD_BOUNDED(INDEX1D, number_of_objects_const);
	function_Integrate_Timestep_Semi_Implicit(index, step_size_const, active, acc, rot, vel, omega, pos, lim);
}
void ChIntegratorGPU::host_Integrate_Timestep_Semi_Implicit(bool* active, real3* acc, real4* rot, real3* vel, real3* omega, real3* pos, real3* lim) {
#pragma omp parallel for schedule(guided)

	for (uint index = 0; index < number_of_objects; index++) {
		function_Integrate_Timestep_Semi_Implicit(index, step_size, active, acc, rot, vel, omega, pos, lim);
	}
}
void ChIntegratorGPU::IntegrateSemiImplicit(real step, device_container& gpu_data_) {
	gpu_data = &gpu_data_;
	step_size = step;
	number_of_objects = gpu_data->number_of_objects;

#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(number_of_objects);
	COPY_TO_CONST_MEM(step_size);

	cudaFuncSetCacheConfig(device_Integrate_Timestep_Semi_Implicit, cudaFuncCachePreferL1);

	device_Integrate_Timestep_Semi_Implicit CUDA_KERNEL_DIM(BLOCKS(number_of_objects), THREADS)(
			CASTB1(gpu_data->device_active_data),
			CASTR3(gpu_data->device_acc_data),
			CASTR4(gpu_data->device_rot_data),
			CASTR3(gpu_data->device_vel_data),
			CASTR3(gpu_data->device_omg_data),
			CASTR3(gpu_data->device_pos_data),
			CASTR3(gpu_data->device_lim_data));
#else
	host_Integrate_Timestep_Semi_Implicit(
			gpu_data->device_active_data.data(),
			gpu_data->device_acc_data.data(),
			gpu_data->device_rot_data.data(),
			gpu_data->device_vel_data.data(),
			gpu_data->device_omg_data.data(),
			gpu_data->device_pos_data.data(),
			gpu_data->device_lim_data.data());
#endif

}
