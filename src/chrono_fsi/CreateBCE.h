/*
 * CreateBCE.h
 *
 *  Created on: Nov 12, 2015
 *      Author: arman
 */

#ifndef CREATEBCE_H_
#define CREATEBCE_H_

#include <thrust/host_vector.h>
#include "chrono_fsi/custom_cutil_math.h"
#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/MyStructs.cuh"  //just for SimParams
#include "chrono_parallel/physics/ChSystemParallel.h"


Real CreateOne3DRigidCylinder(thrust::host_vector<Real3>& posRadH,
                              thrust::host_vector<Real4>& velMasH,
                              thrust::host_vector<Real4>& rhoPresMuH,
                              chrono::ChBody* body,
                              Real cyl_rad,
                              Real cyl_h,
                              Real rigidMass,
                              Real sphMarkerMass,
                              int type,
                              const SimParams& paramsH);

Real CreateOne3DRigidSphere(thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH, chrono::ChBody* body, Real cyl_rad,
		Real rigidMass, Real sphMarkerMass, int type,
		const SimParams& paramsH);

// Function to create BCE markers on the surface of a box and a few layers (paramsH.NUM_BOUNDARY_LAYERS) below that
// input argument 'face' determines which face: 12: xy positive, -12: xy negative, 13: xz+, -13: xz-, 23: yz +, -23: yz-
void CreateBCE_On_Box(
    thrust::host_vector<Real3>& posRadBCE,
    thrust::host_vector<Real4>& velMasBCE,
    thrust::host_vector<Real4>& rhoPresMuBCE,
    const SimParams& paramsH,
    const Real& sphMarkerMass,
    const chrono::ChVector<>& size,
    const chrono::ChVector<>& pos,
    const chrono::ChQuaternion<>& rot,
    int face);

void LoadBCE_fromFile(
    thrust::host_vector<Real3>& posRadBCE,  // do not set the size here since you are using push back later
    std::string fileName);

#endif /* CREATEBCE_H_ */
