#include "contactForces.cuh"

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Add_ContactForces_SerpentineD(real3 * totalAccRigid3, real3 * posRigidD, real4 * velMassRigidD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}
	real3 posRigidA = posRigidD[rigidSphereA];
	real4 dummyVelMasA = velMassRigidD[rigidSphereA];
	real3 n3;

	real3 force3 = R3(0);
	real_ rad = paramsD.rigidRadius.x + paramsD.HSML;
	real_ penDist = ContactWith_YPlanes(n3, posRigidA, rad);
	if (penDist < 0) {
		force3 += DEM_Force(-penDist, n3, rad, 20 * rad, dummyVelMasA, R4(0));
	}

	penDist = ContactWithSerpentineCurve(n3, posRigidA, rad);
	if (penDist < 0) {
		force3 += DEM_Force(-penDist, n3, rad, 20 * rad, dummyVelMasA, R4(0)); //approximate the curve with straight line
	}

	ContactWithOtherSpheres(force3, n3, rigidSphereA, posRigidA, dummyVelMasA, posRigidD, velMassRigidD, rad);

	real3 totalAcc = totalAccRigid3[rigidSphereA];
	totalAcc += force3 / dummyVelMasA.w;
	totalAccRigid3[rigidSphereA] = totalAcc;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Add_ContactForces_StraightChD(real3 * totalAccRigid3, real3 * posRigidD, real4 * velMassRigidD) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}
	real3 posRigidA = posRigidD[rigidSphereA];
	real4 dummyVelMasA = velMassRigidD[rigidSphereA];
	real3 n3;

	real3 force3 = R3(0);
	real_ rad = paramsD.rigidRadius.x + paramsD.HSML;
	real_ penDist = ContactWith_YPlanes(n3, posRigidA, rad);
	if (penDist < 0) {
		force3 += DEM_Force(-penDist, n3, rad, 20 * rad, dummyVelMasA, R4(0));
	}
	penDist = ContactWith_ZPlanes(n3, posRigidA, rad);
	if (penDist < 0) {
		force3 += DEM_Force(-penDist, n3, rad, 20 * rad, dummyVelMasA, R4(0));
	}

	ContactWithOtherSpheres(force3, n3, rigidSphereA, posRigidA, dummyVelMasA, posRigidD, velMassRigidD, rad);

	real3 totalAcc = totalAccRigid3[rigidSphereA];
	totalAcc += force3 / dummyVelMasA.w;
	totalAccRigid3[rigidSphereA] = totalAcc;
}
//--------------------------------------------------------------------------------------------------------------------------------
void setParameters2(SimParams *hostParams, NumberOfObjects *numObjects) {
	// copy parameters to constant memory
	cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, hostParams, sizeof(SimParams)));
	cutilSafeCall( cudaMemcpyToSymbolAsync(numObjectsD, numObjects, sizeof(NumberOfObjects)));
}
//--------------------------------------------------------------------------------------------------------------------------------
void Add_ContactForces_Serpentine(
		real3* totalAccRigid3,
		real3* posRigidD,
		real4* velMassRigidD) {
	NumberOfObjects numObjectsH;
	cudaMemcpyFromSymbolAsync(&numObjectsH, numObjectsD, sizeof(NumberOfObjects));

	//**********************************************************************
	SerpentineParams serpGeom;
	serpGeom.mm = .001;
	serpGeom.r1_2 = R2(1.351, 1.750) * serpGeom.mm;
	serpGeom.r2_2 = R2(1.341, 1.754) * serpGeom.mm;
	serpGeom.r3_2 = R2(2.413, 3.532) * serpGeom.mm;
	serpGeom.r4_2 = R2(0.279, 0.413) * serpGeom.mm;

	serpGeom.r5_2 = R2(1.675, 1.235) * serpGeom.mm; //r5_2 = R2(1.727, 1.235);  	//the smaller one
	serpGeom.r6_2 = R2(2.747, 4.272) * serpGeom.mm; //the larger one
	serpGeom.x_FirstChannel = 8 * serpGeom.mm;
	serpGeom.sPeriod = 5.384 * serpGeom.mm; //serpentine period
	serpGeom.x_SecondChannel = 2 * serpGeom.mm;

	cutilSafeCall( cudaMemcpyToSymbolAsync(serpGeomD, &serpGeom, sizeof(SerpentineParams)));
	//**********************************************************************

	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numObjectsH.numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);

	Add_ContactForces_SerpentineD<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(totalAccRigid3, posRigidD, velMassRigidD);

}
//--------------------------------------------------------------------------------------------------------------------------------
void Add_ContactForces_StraightCh(
		real3* totalAccRigid3,
		real3* posRigidD,
		real4* velMassRigidD) {
	NumberOfObjects numObjectsH;
	cudaMemcpyFromSymbolAsync(&numObjectsH, numObjectsD, sizeof(NumberOfObjects));
	//**********************************************************************

	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numObjectsH.numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);

	Add_ContactForces_StraightChD<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(totalAccRigid3, posRigidD, velMassRigidD);

}

