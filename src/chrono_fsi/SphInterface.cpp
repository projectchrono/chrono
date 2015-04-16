/*
 * SphInterface.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: arman
 */


#include "SphInterface.h"

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
		derivVelRhoChronoH[i] += mR4(a3,0);  // note, gravity force is also coming from rigid system
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


