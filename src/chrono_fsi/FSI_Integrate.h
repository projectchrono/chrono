/*
 * FSI_Integrate.h
 *
 *  Created on: Nov 5, 2015
 *      Author: arman
 */

#ifndef FSI_INTEGRATE_H_
#define FSI_INTEGRATE_H_

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
//#include "chrono_fsi/custom_cutil_math.h"
//#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/MyStructs.cuh"  //just for SimParams

//#include "chrono_parallel/physics/ChSystemParallel.h"
//#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"
//#include "chrono_utils/ChUtilsCreators.h"  //Arman: why is this
//#include "chrono_utils/ChUtilsInputOutput.h" //Arman: Why is this
//#include "utils/ChUtilsGenerators.h"

#include "chrono_fsi/VehicleExtraProperties.h"
//#include "chrono_vehicle/ChVehicleModelData.h"

void InitializeChronoGraphics(chrono::ChSystemParallelDVI& mphysicalSystem);

void DoStepDynamics_FSI(chrono::ChSystemParallelDVI& mphysicalSystem,
		chrono::vehicle::ChWheeledVehicleAssembly* mVehicle,
		thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real3>& vel_XSPH_D,
		thrust::device_vector<Real4>& rhoPresMuD,

		thrust::device_vector<Real3>& posRadD2,
		thrust::device_vector<Real3>& velMasD2,
		thrust::device_vector<Real4>& rhoPresMuD2,

		thrust::device_vector<Real4>& derivVelRhoD,
		thrust::device_vector<uint>& rigidIdentifierD,
		thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D,

		thrust::device_vector<Real3>& posRigid_fsiBodies_D,
		thrust::device_vector<Real4>& velMassRigid_fsiBodies_D,
		thrust::device_vector<Real3>& accRigid_fsiBodies_D,
		thrust::device_vector<Real4>& q_fsiBodies_D,
		thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,
		thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,


		thrust::device_vector<Real3>& posRigid_fsiBodies_D2,
		thrust::device_vector<Real4>& velMassRigid_fsiBodies_D2,
		thrust::device_vector<Real3>& accRigid_fsiBodies_D2,
		thrust::device_vector<Real4>& q_fsiBodies_D2,
		thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D2,
		thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D2,

		thrust::host_vector<Real3>& pos_ChSystemBackupH,
		thrust::host_vector<Real3>& vel_ChSystemBackupH,
		thrust::host_vector<Real3>& acc_ChSystemBackupH,
		thrust::host_vector<Real4>& quat_ChSystemBackupH,
		thrust::host_vector<Real3>& omegaVelGRF_ChSystemBackupH,
		thrust::host_vector<Real3>& omegaAccGRF_ChSystemBackupH,

		thrust::host_vector<Real3>& posRigid_fsiBodies_dummyH,
		thrust::host_vector<Real4>& velMassRigid_fsiBodies_dummyH,
		thrust::host_vector<Real3>& accRigid_fsiBodies_dummyH,
		thrust::host_vector<Real4>& q_fsiBodies_dummyH,
		thrust::host_vector<Real3>& omegaVelLRF_fsiBodies_dummyH,
		thrust::host_vector<Real3>& omegaAccLRF_fsiBodies_dummyH,

		thrust::device_vector<Real3>& rigid_FSI_ForcesD,
		thrust::device_vector<Real3>& rigid_FSI_TorquesD,

		thrust::device_vector<uint>& bodyIndexD,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		const thrust::host_vector<int4>& referenceArray,
		const NumberOfObjects& numObjects, const SimParams& paramsH,
		double mTime, double time_hold_vehicle, int tStep,
		bool haveVehicle);

void DoStepDynamics_ChronoRK2(chrono::ChSystemParallelDVI& mphysicalSystem,
		chrono::vehicle::ChWheeledVehicleAssembly* mVehicle,

		thrust::host_vector<Real3>& pos_ChSystemBackupH,
		thrust::host_vector<Real3>& vel_ChSystemBackupH,
		thrust::host_vector<Real3>& acc_ChSystemBackupH,
		thrust::host_vector<Real4>& quat_ChSystemBackupH,
		thrust::host_vector<Real3>& omegaVelGRF_ChSystemBackupH,
		thrust::host_vector<Real3>& omegaAccGRF_ChSystemBackupH,

		const SimParams& paramsH, double mTime, double time_hold_vehicle,
		bool haveVehicle);

#endif /* FSI_INTEGRATE_H_ */
