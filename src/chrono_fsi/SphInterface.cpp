/*
 * SphInterface.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: arman
 */

#include "SphInterface.h"

//------------------------------------------------------------------------------------
void AddSphDataToChSystem(chrono::ChSystemParallelDVI& mphysicalSystem,
                          int& startIndexSph,
                          const thrust::host_vector<Real3>& posRadH,
                          const thrust::host_vector<Real4>& velMasH,
                          const SimParams& paramsH,
                          const NumberOfObjects& numObjects,
                          int collisionFamilly,
                          Real sphMarkerMass) {
  Real rad = 0.5 * paramsH.MULT_INITSPACE * paramsH.HSML;
  // NOTE: mass properties and shapes are all for sphere
  double volume = chrono::utils::CalcSphereVolume(rad);
  chrono::ChVector<> gyration = chrono::utils::CalcSphereGyration(rad).Get_Diag();
  double density = paramsH.rho0;
  double mass = sphMarkerMass;//density * volume;
  double muFriction = 0;

  // int fId = 0; //fluid id

  // Create a common material
  chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_g(new chrono::ChMaterialSurface);
  mat_g->SetFriction(muFriction);
  mat_g->SetCohesion(0);
  mat_g->SetCompliance(0.0);
  mat_g->SetComplianceT(0.0);
  mat_g->SetDampingF(0.2);

  const chrono::ChQuaternion<> rot = chrono::ChQuaternion<>(1, 0, 0, 0);

  startIndexSph = mphysicalSystem.Get_bodylist()->size();
  // openmp does not work here
  for (int i = 0; i < numObjects.numFluidMarkers; i++) {
    Real3 p3 = posRadH[i];
    Real4 vM4 = velMasH[i];
    chrono::ChVector<> pos = chrono::ChVector<>(p3.x, p3.y, p3.z);
    chrono::ChVector<> vel = chrono::ChVector<>(vM4.x, vM4.y, vM4.z);
    chrono::ChSharedBodyPtr body;
    body = chrono::ChSharedBodyPtr(new chrono::ChBody(new chrono::collision::ChCollisionModelParallel));
    body->SetMaterialSurface(mat_g);
    // body->SetIdentifier(fId);
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

    //	chrono::utils::AddCapsuleGeometry(body.get_ptr(), size.x, size.y);		// X
    //	chrono::utils::AddCylinderGeometry(body.get_ptr(), size.x, size.y);		// O
    //	chrono::utils::AddConeGeometry(body.get_ptr(), size.x, size.y); 		// X
    //	chrono::utils::AddBoxGeometry(body.get_ptr(), size);					// O
    chrono::utils::AddSphereGeometry(body.get_ptr(), rad);  // O
                                                    //	chrono::utils::AddEllipsoidGeometry(body.get_ptr(), size);					// X

    body->GetCollisionModel()->SetFamily(collisionFamilly);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(collisionFamilly);

    body->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(body);
  }
}
//------------------------------------------------------------------------------------
void AddHydroForce(chrono::ChSystemParallelDVI& mphysicalSystem,
                          int& startIndexSph,
                          const NumberOfObjects& numObjects) {
  // openmp does not work here
	std::vector<chrono::ChBody*>::iterator bodyIter = mphysicalSystem.Get_bodylist()->begin() + startIndexSph;
#pragma omp parallel for
	for (int i = 0; i < numObjects.numFluidMarkers; i++) {
		char forceTag[] = "hydrodynamics_force";
		chrono::ChSharedPtr<chrono::ChForce> hydroForce = (*(bodyIter + i))->SearchForce(forceTag);
		if (hydroForce.IsNull()) {
			hydroForce = chrono::ChSharedPtr<chrono::ChForce>(new chrono::ChForce);
			hydroForce->SetMode(
			  FTYPE_FORCE);  // no need for this. It is the default option.
			(*(bodyIter + i))->AddForce(hydroForce);
			// ** or: hydroForce = ChSharedPtr<ChForce>(new ChForce());
			hydroForce->SetName(forceTag);
		}
	}
}
//------------------------------------------------------------------------------------
void UpdateSphDataInChSystem(chrono::ChSystemParallelDVI& mphysicalSystem,
                             const thrust::host_vector<Real3>& posRadH,
                             const thrust::host_vector<Real4>& velMasH,
                             const NumberOfObjects& numObjects,
                             int startIndexSph) {
#pragma omp parallel for
  for (int i = 0; i < numObjects.numFluidMarkers; i++) {
    Real3 p3 = posRadH[i];
    Real4 vM4 = velMasH[i];
    chrono::ChVector<> pos = chrono::ChVector<>(p3.x, p3.y, p3.z);
    chrono::ChVector<> vel = chrono::ChVector<>(vM4.x, vM4.y, vM4.z);

    int chSystemBodyId = startIndexSph + i;
    std::vector<chrono::ChBody*>::iterator ibody = mphysicalSystem.Get_bodylist()->begin() + chSystemBodyId;
    (*ibody)->SetPos(pos);
    (*ibody)->SetPos_dt(vel);
  }
}
//------------------------------------------------------------------------------------
void AddChSystemForcesToSphForces(thrust::host_vector<Real4>& derivVelRhoChronoH,
                                  const thrust::host_vector<Real4>& velMasH2,
                                  chrono::ChSystemParallelDVI& mphysicalSystem,
                                  const NumberOfObjects& numObjects,
                                  int startIndexSph,
                                  Real dT) {
  std::vector<chrono::ChBody*>::iterator bodyIter = mphysicalSystem.Get_bodylist()->begin() + startIndexSph;
#pragma omp parallel for
  for (int i = 0; i < numObjects.numFluidMarkers; i++) {
    chrono::ChVector<> v = ((chrono::ChBody*)(*(bodyIter + i)))->GetPos_dt();
    Real3 a3 = (mR3(v.x, v.y, v.z) - mR3(velMasH2[i])) / dT;  // f = m * a
    derivVelRhoChronoH[i] += mR4(a3, 0);                      // note, gravity force is also coming from rigid system
  }
}
//------------------------------------------------------------------------------------

void ClearArraysH(thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
                  thrust::host_vector<Real4>& velMasH,
                  thrust::host_vector<Real4>& rhoPresMuH) {
  posRadH.clear();
  velMasH.clear();
  rhoPresMuH.clear();
}
//------------------------------------------------------------------------------------

void ClearArraysH(thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
                  thrust::host_vector<Real4>& velMasH,
                  thrust::host_vector<Real4>& rhoPresMuH,
                  thrust::host_vector<uint>& bodyIndex,
                  thrust::host_vector<::int3>& referenceArray) {
  ClearArraysH(posRadH, velMasH, rhoPresMuH);
  bodyIndex.clear();
  referenceArray.clear();
}
//------------------------------------------------------------------------------------

void CopyD2H(thrust::host_vector<Real4>& derivVelRhoChronoH, const thrust::device_vector<Real4>& derivVelRhoD) {
  assert(derivVelRhoChronoH.size() == derivVelRhoD.size() && "Error! size mismatch host and device");
  thrust::copy(derivVelRhoD.begin(), derivVelRhoD.end(), derivVelRhoChronoH.begin());
}

//------------------------------------------------------------------------------------
void CountNumContactsPerSph(thrust::host_vector<short int>& numContactsOnAllSph,
                                  const chrono::ChSystemParallelDVI& mphysicalSystem,
                                  const NumberOfObjects& numObjects,
                                  int startIndexSph) {
	int numContacts = mphysicalSystem.data_manager->host_data.bids_rigid_rigid.size();
#pragma omp parallel for
	for (int i = 0; i < numContacts; i ++) {
		chrono::int2 ids = mphysicalSystem.data_manager->host_data.bids_rigid_rigid[i];
		if (ids.x > startIndexSph)
			numContactsOnAllSph[ids.x - startIndexSph] += 1;
		if (ids.y > startIndexSph)
			numContactsOnAllSph[ids.y - startIndexSph] += 1;
	}
}

//------------------------------------------------------------------------------------
void CopyForceSphToChSystem(chrono::ChSystemParallelDVI& mphysicalSystem,
		const NumberOfObjects& numObjects,
		int startIndexSph,
		const thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<short int>& numContactsOnAllSph,
		Real sphMass) {

	std::vector<chrono::ChBody*>::iterator bodyIter = mphysicalSystem.Get_bodylist()->begin() + startIndexSph;
#pragma omp parallel for
	for (int i = 0; i < numObjects.numFluidMarkers; i++) {
		char forceTag[] = "hydrodynamics_force";
		chrono::ChSharedPtr<chrono::ChForce> hydroForce = (*(bodyIter + i))->SearchForce(forceTag);
//		if (!hydroForce.IsNull())
//			hydroForce->SetMforce(0);
//
//		if (numContactsOnAllSph[i] == 0) continue;
		assert(!hydroForce.IsNull() && "Error! sph marker does not have hyroforce tag in ChSystem");

		Real4 mDerivVelRho= derivVelRhoD[i];
		Real3 forceSphMarker = mR3(mDerivVelRho) * sphMass;
		chrono::ChVector<> f3 = chrono::ChVector<>(forceSphMarker.x, forceSphMarker.y, forceSphMarker.z);
		hydroForce->SetMforce(f3.Length());
		f3.Normalize();
		hydroForce->SetDir(f3);
	}
}

//------------------------------------------------------------------------------------

void CopyCustomChSystemPosVel2HostThrust(
        thrust::host_vector<Real3>& posRadH,
        thrust::host_vector<Real4>& velMasH,
		chrono::ChSystemParallelDVI& mphysicalSystem,
		const NumberOfObjects& numObjects,
		int startIndexSph,
		const thrust::host_vector<short int>& numContactsOnAllSph) {

	std::vector<chrono::ChBody*>::iterator bodyIter = mphysicalSystem.Get_bodylist()->begin() + startIndexSph;
#pragma omp parallel for
	for (int i = 0; i < numObjects.numFluidMarkers; i++) {
		if (numContactsOnAllSph[i] == 0) continue;
		chrono::ChVector<> pos = (*(bodyIter + i))->GetPos();
		posRadH[i] = mR3(pos.x, pos.y, pos.z);
		chrono::ChVector<> vel = (*(bodyIter + i))->GetPos_dt();
		Real mass = velMasH[i].w;
		velMasH[i] = mR4(vel.x, vel.y, vel.z, mass);
	}
}
//------------------------------------------------------------------------------------

void CopyH2DPosVel(
		thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real4>& velMasD,
		const thrust::host_vector<Real3>& posRadH,
		const thrust::host_vector<Real4>& velMasH) {
  assert(posRadH.size() == posRadD.size() && "Error! size mismatch host and device");
  thrust::copy(posRadH.begin(), posRadH.end(), posRadD.begin());
  thrust::copy(velMasH.begin(), velMasH.end(), velMasD.begin());
}

//------------------------------------------------------------------------------------

void CopyD2HPosVel(
		thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		const thrust::host_vector<Real3>& posRadD,
		const thrust::host_vector<Real4>& velMasD) {
  assert(posRadH.size() == posRadD.size() && "Error! size mismatch host and device");
  thrust::copy(posRadD.begin(), posRadD.end(), posRadH.begin());
  thrust::copy(velMasD.begin(), velMasD.end(), velMasH.begin());
}

//------------------------------------------------------------------------------------

void CopyH2D(thrust::device_vector<Real4>& derivVelRhoD, const thrust::host_vector<Real4>& derivVelRhoChronoH) {
  assert(derivVelRhoChronoH.size() == derivVelRhoD.size() && "Error! size mismatch host and device");
  thrust::copy(derivVelRhoChronoH.begin(), derivVelRhoChronoH.end(), derivVelRhoD.begin());
}
//------------------------------------------------------------------------------------

void CopySys2D(thrust::device_vector<Real3>& posRadD,
               chrono::ChSystemParallelDVI& mphysicalSystem,
               const NumberOfObjects& numObjects,
               int startIndexSph) {
  thrust::host_vector<Real3> posRadH(numObjects.numFluidMarkers);
  std::vector<chrono::ChBody*>::iterator bodyIter = mphysicalSystem.Get_bodylist()->begin() + startIndexSph;
#pragma omp parallel for
  for (int i = 0; i < numObjects.numFluidMarkers; i++) {
    chrono::ChVector<> p = ((chrono::ChBody*)(*(bodyIter + i)))->GetPos();
    posRadH[i] += mR3(p.x, p.y, p.z);
  }
  thrust::copy(posRadH.begin(), posRadH.end(), posRadD.begin());
}
//------------------------------------------------------------------------------------

void CopyD2H(thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
             thrust::host_vector<Real4>& velMasH,
             thrust::host_vector<Real4>& rhoPresMuH,
             const thrust::device_vector<Real3>& posRadD,
             const thrust::device_vector<Real4>& velMasD,
             const thrust::device_vector<Real4>& rhoPresMuD) {
  assert(posRadH.size() == posRadD.size() && "Error! size mismatch host and device");
  thrust::copy(posRadD.begin(), posRadD.end(), posRadH.begin());
  thrust::copy(velMasD.begin(), velMasD.end(), velMasH.begin());
  thrust::copy(rhoPresMuD.begin(), rhoPresMuD.end(), rhoPresMuH.begin());
}
