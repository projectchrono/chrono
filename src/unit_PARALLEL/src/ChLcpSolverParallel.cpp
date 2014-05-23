#include "ChLcpSolverParallel.h"
#include "math/ChThrustLinearAlgebra.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Kernel for adding invmass*force*step_size_const to body speed vector.
// This kernel must be applied to the stream of the body buffer.
// -----------------------------------------------------------------------------
__host__ __device__
void function_addForces(
    int&   index,
    bool*  active,
    real*  mass,
    real3* inertia,
    real3* forces,
    real3* torques,
    real3* vel,
    real3* omega)
{
  if (active[index] != 0) {
    // v += m_inv * h * f
    vel[index] += forces[index] * mass[index];
    // w += J_inv * h * c
    omega[index] += torques[index] * inertia[index];
  }
}

void ChLcpSolverParallel::host_addForces(
    bool* active,
    real* mass,
    real3* inertia,
    real3* forces,
    real3* torques,
    real3* vel,
    real3* omega)
{
#pragma omp parallel for
  for (int index = 0; index < number_of_objects; index++) {
    function_addForces(index, active, mass, inertia, forces, torques, vel, omega);
  }
}

__host__ __device__
void function_ComputeGyro(
    int& index,
    real3* omega,
    real3* inertia,
    real3* gyro,
    real3* torque)
{
  real3 body_inertia = inertia[index];
  body_inertia = R3(1.0 / body_inertia.x, 1.0 / body_inertia.y, 1.0 / body_inertia.z);
  real3 body_omega = omega[index];
  real3 gyr = cross(body_omega, body_inertia * body_omega);
  gyro[index] = gyr;
}

void ChLcpSolverParallel::host_ComputeGyro(
    real3* omega,
    real3* inertia,
    real3* gyro,
    real3* torque)
{
#pragma omp parallel for
  for (int index = 0; index < number_of_objects; index++) {
    function_ComputeGyro(index, omega, inertia, gyro, torque);
  }
}

void ChLcpSolverParallel::Preprocess() {
  data_container->number_of_updates = 0;
  data_container->host_data.gyr_data.resize(number_of_objects);

#ifdef SIM_ENABLE_GPU_MODE

  COPY_TO_CONST_MEM(number_of_rigid);

  device_ComputeGyro CUDA_KERNEL_DIM(BLOCKS(number_of_rigid), THREADS)(
      CASTR3(data_container->device_data.device_omg_data),
      CASTR3(data_container->device_data.device_inr_data),
      CASTR3(data_container->device_data.device_gyr_data),
      CASTR3(data_container->device_data.device_trq_data));
  device_addForces CUDA_KERNEL_DIM(BLOCKS(number_of_rigid), THREADS)(
      CASTB1(data_container->device_data.device_active_data),
      CASTR1(data_container->device_data.device_mass_data),
      CASTR3(data_container->device_data.device_inr_data),
      CASTR3(data_container->device_data.device_frc_data),
      CASTR3(data_container->device_data.device_trq_data),
      CASTR3(data_container->device_data.device_vel_data),
      CASTR3(data_container->device_data.device_omg_data));
#else
  //	host_ComputeGyro(
  //			data_container->host_data.omg_data.data(),
  //			data_container->host_data.inr_data.data(),
  //			data_container->host_data.gyr_data.data(),
  //			data_container->host_data.trq_data.data());

  host_addForces(data_container->host_data.active_data.data(),
                 data_container->host_data.mass_data.data(),
                 data_container->host_data.inr_data.data(),
                 data_container->host_data.frc_data.data(),
                 data_container->host_data.trq_data.data(),
                 data_container->host_data.vel_data.data(),
                 data_container->host_data.omg_data.data());
#endif
}
