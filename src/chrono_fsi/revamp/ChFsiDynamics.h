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
// Class for performing handling fsi system.
// =============================================================================

#ifndef CH_SYSTEMFSI_H_
#define CH_SYSTEMFSI_H_

namespace fsi {

class CH_FSI_API ChSystemFsi : public ChFsiGeneral{

public:
	ChSystemFsi();
	~ChSystemFsi();

	DoStepDynamics_FSI();
	CopyDeviceDataToHalfStep();

private:
	DoStepChronoSystem(Real dT,
		double mTime, double time_hold_vehicle, bool haveVehicle);

	ChFsiDataManager* fsiData;
	ChFluidDynamics* ChFluidDynamics;
	ChFsiInterface* ChFsiInterface;
	ChBce* bceWorker;

	chrono::ChSystemParallelDVI * mphysicalSystem;
	chrono::vehicle::ChWheeledVehicleAssembly* mVehicle;
};
}
#endif