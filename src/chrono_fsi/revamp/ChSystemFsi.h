// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki
// =============================================================================
//
// Base class for fsi system.//
// =============================================================================

#ifndef CH_SYSTEM_FSI_H_
#define CH_SYSTEM_FSI_H_

namespace fsi {

class CH_FSI_API ChSystemFsi {
	FsiDataContainer* fsiData;

void InitSystem();

void ForceSPH();
void DensityReinitialization();
void IntegrateSPH();
void UpdateFluid();
void Copy_SortedVelXSPH_To_VelXSPH();
void UpdateBoundary();
void ApplyBoundarySPH_Markers();
void ProjectDensityPressureToBCandBCE();
void ReCalcDensity();
void CalcBCE_Stresses();
void RecalcVelocity_XSPH();

void MapSPH_ToGrid();
void CalcCartesianData();


void MakeRigidIdentifier();
void Populate_RigidSPH_MeshPos_LRF();
void Rigid_Forces_Torques();
void UpdateRigidMarkersPositionVelocity1();
void UpdateRigidMarkersPositionVelocity2();
void CalcBceAcceleration();

};
}

#endif /* CH_SYSTEM_FSI_H_ */
