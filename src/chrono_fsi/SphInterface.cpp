/*
 * SphInterface.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: arman
 */


#include "SphInterface.h"

//**********************************************
void SetupParamsH(SimParams & paramsH) {
	paramsH.sizeScale = 1; //don't change it.
	paramsH.HSML = 0.2;
	paramsH.MULT_INITSPACE = 1.0;
		paramsH.NUM_BOUNDARY_LAYERS = 3;
		paramsH.toleranceZone = paramsH.NUM_BOUNDARY_LAYERS * (paramsH.HSML * paramsH.MULT_INITSPACE);
	paramsH.BASEPRES = 0;//10;
		paramsH.LARGE_PRES = 10000;//paramsH.BASEPRES;//10000;
		paramsH.deltaPress; //** modified below
		paramsH.multViscosity_FSI = 5.0;
	paramsH.gravity = mR3(0, -9.81, 0);//mR3(0);//mR3(0, -9.81, 0);
	paramsH.bodyForce3 = mR3(0,0,0);//mR4(3.2e-3,0,0,0);// mR4(0);;// /*Re = 100 */ //mR4(3.2e-4, 0, 0, 0);/*Re = 100 */
		paramsH.rho0 = 1000;
		paramsH.mu0 = .001;
	paramsH.v_Max = 10;//50e-3;//18e-3;//1.5;//2e-1; /*0.2 for Re = 100 */ //2e-3;
		paramsH.EPS_XSPH = .5f;
	paramsH.dT = 0.0005;//0.1;//.001; //sph alone: .01 for Re 10;
		paramsH.tFinal = 1000;//20 * paramsH.dT; //400
		paramsH.timePause = 0;//.0001 * paramsH.tFinal;//.0001 * paramsH.tFinal; 	// time before applying any bodyforce. Particles move only due to initialization. keep it as small as possible. the time step will be 1/10 * dT.
		paramsH.kdT = 5; // I don't know what is kdT
		paramsH.gammaBB = 0.5;
	// ************
		paramsH.binSize0; // will be changed
		paramsH.rigidRadius; //will be changed
	paramsH.densityReinit = 0; //0: no re-initialization, 1: with initialization
	//****************************************************************************************
	//*** initialize straight channel
	paramsH.straightChannelBoundaryMin = mR3(0, 0, 0); //3D channel
	paramsH.straightChannelBoundaryMax = mR3(3, 2, 3) * paramsH.sizeScale;
	//********************************************************************************************************
	//**  reminiscent of the past******************************************************************************
//	paramsH.cMin = mR3(-paramsH.toleranceZone, -paramsH.toleranceZone, -paramsH.toleranceZone);						// 3D channel
//	paramsH.cMax = mR3( 3  + paramsH.toleranceZone, 2 + paramsH.toleranceZone,  3 + paramsH.toleranceZone);
	paramsH.cMin = mR3(0, 0, 0);						// 3D channel
	paramsH.cMax = mR3( 3, 2,  3 );
	//****************************************************************************************
	//printf("a1  paramsH.cMax.x, y, z %f %f %f,  binSize %f\n", paramsH.cMax.x, paramsH.cMax.y, paramsH.cMax.z, 2 * paramsH.HSML);
	int3 side0 = mI3(
			floor((paramsH.cMax.x - paramsH.cMin.x) / (2 * paramsH.HSML)),
			floor((paramsH.cMax.y - paramsH.cMin.y) / (2 * paramsH.HSML)),
			floor((paramsH.cMax.z - paramsH.cMin.z) / (2 * paramsH.HSML)));
	Real3 binSize3 = mR3((paramsH.cMax.x - paramsH.cMin.x) / side0.x,
			(paramsH.cMax.y - paramsH.cMin.y) / side0.y,
			(paramsH.cMax.z - paramsH.cMin.z) / side0.z);
	paramsH.binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
//	paramsH.binSize0 = (paramsH.binSize0 > binSize3.z) ? paramsH.binSize0 : binSize3.z;
	paramsH.binSize0 = binSize3.x; //for effect of distance. Periodic BC in x direction. we do not care about paramsH.cMax y and z.
	paramsH.cMax = paramsH.cMin + paramsH.binSize0 * mR3(side0);
	paramsH.boxDims = paramsH.cMax - paramsH.cMin;
	//************************** modify pressure ***************************
//		paramsH.deltaPress = paramsH.rho0 * paramsH.boxDims * paramsH.bodyForce3;  //did not work as I expected
	paramsH.deltaPress = 0.9 * paramsH.boxDims * paramsH.bodyForce3;

	// modify bin size stuff
	//****************************** bin size adjustement and contact detection stuff *****************************
	int3 SIDE = mI3(int((paramsH.cMax.x - paramsH.cMin.x) / paramsH.binSize0 + .1), int((paramsH.cMax.y - paramsH.cMin.y) / paramsH.binSize0 + .1),
			int((paramsH.cMax.z - paramsH.cMin.z) / paramsH.binSize0 + .1));
	Real mBinSize = paramsH.binSize0; //Best solution in that case may be to change cMax or cMin such that periodic sides be a multiple of binSize
	//**********************************************************************************************************
	paramsH.gridSize = SIDE;
	//paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = paramsH.cMin;
	paramsH.cellSize = mR3(mBinSize, mBinSize, mBinSize);

	//***** print numbers
	printf("********************\n paramsH.sizeScale: %f\n paramsH.HSML: %f\n paramsH.bodyForce3: %f %f %f\n paramsH.gravity: %f %f %f\n paramsH.rho0: %e\n paramsH.mu0: %f\n paramsH.v_Max: %f\n paramsH.dT: %e\n paramsH.tFinal: %f\n  paramsH.timePause: %f\n  paramsH.timePauseRigidFlex: %f\n paramsH.densityReinit: %d\n",
			paramsH.sizeScale, paramsH.HSML, paramsH.bodyForce3.x, paramsH.bodyForce3.y, paramsH.bodyForce3.z, paramsH.gravity.x, paramsH.gravity.y, paramsH.gravity.z,
			paramsH.rho0, paramsH.mu0, paramsH.v_Max, paramsH.dT, paramsH.tFinal, paramsH.timePause, paramsH.timePauseRigidFlex, paramsH.densityReinit);
	printf(" paramsH.cMin: %f %f %f, paramsH.cMax: %f %f %f\n binSize: %f\n",
			paramsH.cMin.x, paramsH.cMin.y, paramsH.cMin.z, paramsH.cMax.x,
			paramsH.cMax.y, paramsH.cMax.z, paramsH.binSize0);
	printf(" paramsH.MULT_INITSPACE: %f\n", paramsH.MULT_INITSPACE);
	printf(" paramsH.NUM_BOUNDARY_LAYERS: %d\n paramsH.toleranceZone: %f\n paramsH.NUM_BCE_LAYERS: %d\n paramsH.solidSurfaceAdjust: %f\n", paramsH.NUM_BOUNDARY_LAYERS, paramsH.toleranceZone, paramsH.NUM_BCE_LAYERS, paramsH.solidSurfaceAdjust);
	printf(" paramsH.BASEPRES: %f\n paramsH.LARGE_PRES: %f\n paramsH.deltaPress: %f %f %f\n", paramsH.BASEPRES, paramsH.LARGE_PRES, paramsH.deltaPress.x, paramsH.deltaPress.y, paramsH.deltaPress.z);
	printf(" paramsH.nPeriod: %d\n paramsH.EPS_XSPH: %f\n paramsH.multViscosity_FSI: %f\n paramsH.rigidRadius: %f\n", paramsH.nPeriod, paramsH.EPS_XSPH, paramsH.multViscosity_FSI, paramsH.rigidRadius);
	printf("boxDims: %f, %f, %f\n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z);
	printf("SIDE: %d, %d, %d\n", paramsH.gridSize.x, paramsH.gridSize.y, paramsH.gridSize.z);
}
//------------------------------------------------------------------------------------
void AddSphDataToChSystem(
		ChSystemParallelDVI& mphysicalSystem,
		int & startIndexSph,
		const thrust::host_vector<Real3> & posRadH,
		const thrust::host_vector<Real4> & velMasH,
		const SimParams & paramsH,
		const NumberOfObjects & numObjects) {

	Real rad = 0.5 * paramsH.MULT_INITSPACE * paramsH.HSML;
	// NOTE: mass properties and shapes are all for sphere
	double volume = utils::CalcSphereVolume(rad);
	ChVector<> gyration = utils::CalcSphereGyration(rad).Get_Diag();
	double density = paramsH.rho0;
	double mass = density * volume;
	double muFriction = 0;

	//int fId = 0; //fluid id

	// Create a common material
	ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
	mat_g->SetFriction(muFriction);
	mat_g->SetCohesion(0);
	mat_g->SetCompliance(0.0);
	mat_g->SetComplianceT(0.0);
	mat_g->SetDampingF(0.2);

	const ChQuaternion<> rot = ChQuaternion<>(1, 0, 0, 0);


	startIndexSph = mphysicalSystem.Get_bodylist()->size();
	// openmp does not work here
	for (int i = 0; i < numObjects.numAllMarkers; i++) {
		Real3 p3 = posRadH[i];
		Real4 vM4 = velMasH[i];
		ChVector<> pos = ChVector<>(p3.x, p3.y, p3.z);
		ChVector<> vel = ChVector<>(vM4.x, vM4.y, vM4.z);
		ChSharedBodyPtr body;
		body = ChSharedBodyPtr(new  ChBody(new ChCollisionModelParallel));
		body->SetMaterialSurface(mat_g);
		//body->SetIdentifier(fId);
		body->SetPos(pos);
		body->SetRot(rot);
		body->SetCollide(true);
		body->SetBodyFixed(false);
	    body->SetMass(mass);
	    body->SetInertiaXX(mass * gyration);

		body->GetCollisionModel()->ClearModel();

		// add collision geometry
	//	body->GetCollisionModel()->AddEllipsoid(size.x, size.y, size.z, pos, rot);
	//
	//	// add asset (for visualization)
	//	ChSharedPtr<ChEllipsoidShape> ellipsoid(new ChEllipsoidShape);
	//	ellipsoid->GetEllipsoidGeometry().rad = size;
	//	ellipsoid->Pos = pos;
	//	ellipsoid->Rot = rot;
	//
	//	body->GetAssets().push_back(ellipsoid);

	//	utils::AddCapsuleGeometry(body.get_ptr(), size.x, size.y);		// X
	//	utils::AddCylinderGeometry(body.get_ptr(), size.x, size.y);		// O
	//	utils::AddConeGeometry(body.get_ptr(), size.x, size.y); 		// X
	//	utils::AddBoxGeometry(body.get_ptr(), size);					// O
		utils::AddSphereGeometry(body.get_ptr(), rad);				// O
	//	utils::AddEllipsoidGeometry(body.get_ptr(), size);					// X

		body->GetCollisionModel()->SetFamily(100);
		body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(100);

		body->GetCollisionModel()->BuildModel();
		mphysicalSystem.AddBody(body);
	}
}
//------------------------------------------------------------------------------------
void UpdateSphDataInChSystem(
		ChSystemParallelDVI& mphysicalSystem,
		const thrust::host_vector<Real3> & posRadH,
		const thrust::host_vector<Real4> & velMasH,
		const NumberOfObjects & numObjects,
		int  startIndexSph) {

	#pragma omp parallel for
	for (int i = 0; i < numObjects.numAllMarkers; i++) {
		Real3 p3 = posRadH[i];
		Real4 vM4 = velMasH[i];
		ChVector<> pos = ChVector<>(p3.x, p3.y, p3.z);
		ChVector<> vel = ChVector<>(vM4.x, vM4.y, vM4.z);

		int chSystemBodyId = startIndexSph + i;
		std::vector<ChBody*>::iterator ibody = mphysicalSystem.Get_bodylist()->begin() + chSystemBodyId;
		(*ibody)->SetPos(pos);
		(*ibody)->SetPos_dt(vel);
	}
}
//------------------------------------------------------------------------------------
void AddChSystemForcesToSphForces(
		thrust::host_vector<Real4>  & derivVelRhoChronoH,
		const thrust::host_vector<Real4> & velMasH2,
		ChSystemParallelDVI& mphysicalSystem,
		const NumberOfObjects & numObjects,
		int startIndexSph,
		Real dT) {
	std::vector<ChBody*>::iterator bodyIter = mphysicalSystem.Get_bodylist()->begin() + startIndexSph;
#pragma omp parallel for
	for (int i = 0; i < numObjects.numAllMarkers; i++) {
//		std::vector<ChBody*>::iterator bodyIterB = bodyIter + i;
		ChVector<> v = ((ChBody*)(*(bodyIter + i)))->GetPos_dt();
		Real3 a3 = (mR3(v.x, v.y, v.z) - mR3(velMasH2[i])) / dT; // f = m * a
		derivVelRhoChronoH[i] += mR4(a3,0);
	}
}
//------------------------------------------------------------------------------------

void ClearArraysH(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH) {

	posRadH.clear();
	velMasH.clear();
	rhoPresMuH.clear();
}
//------------------------------------------------------------------------------------

void ClearArraysH(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH,
	thrust::host_vector<uint> & bodyIndex,
	thrust::host_vector<int3> & referenceArray) {

	ClearArraysH(posRadH, velMasH, rhoPresMuH);
	bodyIndex.clear();
	referenceArray.clear();
}
//------------------------------------------------------------------------------------

void CopyD2H(
	thrust::host_vector<Real4> & derivVelRhoChronoH,
	const thrust::device_vector<Real4> & derivVelRhoD) {
	assert(derivVelRhoChronoH.size() == derivVelRhoD.size() && "Error! size mismatch host and device" );
	thrust::copy(derivVelRhoD.begin(), derivVelRhoD.end(), derivVelRhoChronoH.begin());
}
//------------------------------------------------------------------------------------

void CopyH2D(
	thrust::device_vector<Real4> & derivVelRhoD,
	const thrust::host_vector<Real4> & derivVelRhoChronoH) {
	assert(derivVelRhoChronoH.size() == derivVelRhoD.size() && "Error! size mismatch host and device" );
	thrust::copy(derivVelRhoChronoH.begin(), derivVelRhoChronoH.end(), derivVelRhoD.begin());
}
//------------------------------------------------------------------------------------

void CopySys2D(
	thrust::device_vector<Real3> & posRadD,
	ChSystemParallelDVI& mphysicalSystem,
	const NumberOfObjects & numObjects,
	int startIndexSph) {
	thrust::host_vector<Real3> posRadH(numObjects.numAllMarkers);
	std::vector<ChBody*>::iterator bodyIter = mphysicalSystem.Get_bodylist()->begin() + startIndexSph;
#pragma omp parallel for
	for (int i = 0; i < numObjects.numAllMarkers; i++) {
		ChVector<> p = ((ChBody*)(*(bodyIter + i)))->GetPos();
		posRadH[i] += mR3(p.x, p.y, p.z);
	}
	thrust::copy(posRadH.begin(), posRadH.end(), posRadD.begin());
}
//------------------------------------------------------------------------------------

void CopyD2H(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH,
	const thrust::device_vector<Real3> & posRadD,
	const thrust::device_vector<Real4> & velMasD,
	const thrust::device_vector<Real4> & rhoPresMuD) {
	assert(posRadH.size() == posRadD.size() && "Error! size mismatch host and device" );
	thrust::copy(posRadD.begin(), posRadD.end(), posRadH.begin());
	thrust::copy(velMasD.begin(), velMasD.end(), velMasH.begin());
	thrust::copy(rhoPresMuD.begin(), rhoPresMuD.end(), rhoPresMuH.begin());
}


