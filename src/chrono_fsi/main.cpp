///////////////////////////////////////////////////////////////////////////////
//	main.cpp
//	Reads the initializes the particles, either from file or inside the code
//	
//	Related Files: collideSphereSphere.cu, collideSphereSphere.cuh
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description 
//					reads the number of particles first. The each line provides the 
//					properties of one SPH particl: 
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu, particle_type(rigid or fluid)
//
//	Created by Arman Pazouki
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <vector>
#include <cstdlib> //for RAND_MAX


//for memory leak detection, apparently does not work in conjunction with cuda
//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>//just for min

#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include <thrust/host_vector.h>
#include <thrust/scan.h>
#include "collideSphereSphere.cuh"
#include "SDKCollisionSystem.cuh" //just for HSML and BASEPRES
#include <algorithm>

using namespace std;
//typedef unsigned int uint;

const real2 r1_2 = R2(1.351, 1.750) * sizeScale;
const real2 r2_2 = R2(1.341, 1.754) * sizeScale;
const real2 r3_2 = R2(2.413, 3.532) * sizeScale;
const real2 r4_2 = R2(0.279, 0.413) * sizeScale;

const real2 r5_2 = R2(1.675, 1.235) * sizeScale; //r5_2 = R2(1.727, 1.235);  	//the smaller one
const real2 r6_2 = R2(2.747, 4.272) * sizeScale;												//the larger one
const real_ x_FirstChannel = 8 * sizeScale;
const real_ x_SecondChannel = 2 * sizeScale;

const real_ channelRadius = 50.0 * sizeScale; //5.6 * sizeScale; //1.0 * sizeScale; //tube
const real2 channelCenterYZ = R2(50, 50);

const real_ sPeriod = 5.384 * sizeScale;		//serpentine period
const real_ toleranceZone = 5 * HSML;
real3 straightChannelBoundaryMin;
real3 straightChannelBoundaryMax;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
struct Rotation {
		real_ a00, a01, a02, a10, a11, a12, a20, a21, a22;
};
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CalcQuat2RotationMatrix (Rotation & rotMat, const real4 & q) {
	rotMat.a00 = 2.0 * (0.5f - q.z * q.z - q.w * q.w);
	rotMat.a01 = 2.0 * (q.y * q.z - q.x * q.w);
	rotMat.a02 = 2.0 * (q.y * q.w + q.x * q.z);

	rotMat.a10 = 2 * (q.y * q.z + q.x * q.w);
	rotMat.a11 = 2 * (0.5f - q.y * q.y - q.w * q.w);
	rotMat.a12 = 2 * (q.z * q.w - q.x * q.y);

	rotMat.a20 = 2 *  (q.y * q.w - q.x * q.z);
	rotMat.a21 = 2 * (q.z * q.w + q.x * q.y);
	rotMat.a22 = 2 * (0.5f - q.y * q.y - q.z * q.z);
};
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void ConvertQuatArray2RotArray (thrust::host_vector<Rotation> & rotArray, const thrust::host_vector<real4> & quat) {
	for (int i = 0; i < quat.size(); i++) {
		Rotation rot;
		CalcQuat2RotationMatrix(rot, quat[i]);
		rotArray[i] = rot;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ Min(real_ a, real_ b) {
	return (a < b) ? a : b;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateMassMomentEllipsoid(
		real_ & mass,
		real3 & j1,
		real3 & j2,
		real_ r1, real_ r2, real_ r3,
		const real_ rhoRigid) {
	mass = 4.0 / 3 * PI * r1 * r2 * r3 * rhoRigid;			//for sphere
	j1 = .2 * mass * R3(r2 * r2 + r3 * r3, 0.0f, 0.0f);
	j2 = .2 * mass * R3(r1 * r1 + r3 * r3, 0.0f, r1 * r1 + r2 * r2);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateMassMomentCylinder(
		real_ & mass,
		real3 & j1,
		real3 & j2,
		real_ r1,
		const real3 cMin,
		const real3 cMax,
		const real_ rhoRigid) {
		mass = PI * pow(r1, 2) * (cMax.y - cMin.y) * rhoRigid;	//for cylinder
		j1 = R3(1.0 / 12.0  * mass * (3 * pow(r1, 2) + pow(cMax.y - cMin.y, 2)), 0, 0);
		j2 = R3(.5 * mass * pow(r1, 2), 0, 1.0 / 12.0  * mass * (3 * pow(r1, 2) + pow(cMax.y - cMin.y, 2)));
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CalcInvJ(const real3 j1, const real3 j2, real3 & invJ1, real3 & invJ2) {
	//normalize J to deal with machine precision
	real3 maxJ3 = rmaxr(j1, j2);
	real_ maxComp = max(maxJ3.x, maxJ3.y);
	maxComp = max(maxComp, maxJ3.z);
	//********************************************
	real3 nJ1 = j1 / maxComp;	//nJ1 is normalJ1
	real3 nJ2 = j2 / maxComp;	//nJ2 is normalJ2

	real_ detJ = 2 * nJ1.z * nJ1.y * nJ2.y - nJ1.z * nJ1.z * nJ2.x - nJ1.y * nJ1.y * nJ2.z + nJ1.x * nJ2.x * nJ2.z - nJ1.x * nJ2.y * nJ2.y;
	invJ1 = R3(nJ2.x * nJ2.z - nJ2.y * nJ2.y, -nJ1.y * nJ2.z + nJ1.z * nJ2.y, nJ1.y * nJ2.y - nJ1.z * nJ2.x);
	invJ2 = R3(-nJ1.z * nJ1.z + nJ1.x * nJ2.z, -nJ1.x * nJ2.y + nJ1.z * nJ1.y, -nJ1.y * nJ1.y + nJ1.x * nJ2.x);

	//printf("invJ %f %f %f %f %f %f\n", aa.x, aa.y, aa.z, bb.x, bb.y, bb.z);
	//printf("invJ %f %f %f %f %f %f\n", 1e12 * j1.x, 1e12 *  j1.y, 1e12 *  j1.z,  1e12 * j2.x,  1e12 * j2.y, 1e12 *  j2.z);
	//printf("detJ %e\n", detJ * maxComp);
	// j = maxComp * nJ, therefore, jInv = maxComp * nJ_Inv
	invJ1 = invJ1 / detJ / maxComp;
	invJ2 = invJ2 / detJ / maxComp;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesRandom(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & spheresVelMas,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const real3 referenceR,
		const real_ rhoRigid,
		real3 cMin,
		real3 cMax,
		int numSpheres) {

	srand ( time(NULL) );
	real_ num;
	vector<real_> randLinVec(0);
	fstream inRandomFile("../randomLinear", ios::in);
	inRandomFile >> num;
	while (!inRandomFile.eof()) {
		randLinVec.push_back(num);
		inRandomFile >> num;
	}
	real_ maxR = max(referenceR.x, referenceR.y);
	maxR = max(maxR, referenceR.z);

	real_ xSpace = (cMax.x - cMin.x) / (numSpheres + 1);
	for (int i = 0; i < numSpheres; i++) {
//		int index = (int)(randLinVec.size() - 1) * real_ (rand()) / RAND_MAX;
//		real_ r = (4.5 * sizeScale) * randLinVec[index];



		real_ r = (channelRadius - maxR - 2 * HSML) * real_ (rand()) / RAND_MAX;
//						real_ r = (channelRadius - maxR - 2 * HSML) * (.2);

		printf("sizeRandomLinear %d\n", randLinVec.size() );	//4.5 comes from channelRadius
		real_ teta = 2 * PI * real_ (rand()) / RAND_MAX;
//						real_ teta = 2 * PI * real_ (.2);
		real3 pos = R3(cMin.x, channelCenterYZ.x, channelCenterYZ.y) + R3( (i + 1.0) * xSpace, real_(r  * cos(teta)), real_(r *  sin(teta)) );
		rigidPos.push_back(pos);

		real4 dumQuat = R4(1 - 2.0 * real_ (rand()) / RAND_MAX, 1 - 2.0 * real_ (rand()) / RAND_MAX, 1 - 2.0 * real_ (rand()) / RAND_MAX, 1 - 2.0 * real_ (rand()) / RAND_MAX); //generate random quaternion
//						real4 dumQuat = R4(1,0,0,0);

		dumQuat *= 1.0 / length(dumQuat);
		mQuatRot.push_back(dumQuat);

		 ellipsoidRadii.push_back(referenceR);
		 real_ mass;
		 real3 j1, j2;
		 CreateMassMomentEllipsoid(mass, j1, j2, referenceR.x, referenceR.y, referenceR.z, rhoRigid);						//create Ellipsoid

		spheresVelMas.push_back(R4(0, 0, 0, real_(mass)));
		rigidBodyOmega.push_back(R3(0, 0, 0));
		rigidBody_J1.push_back(j1);
		rigidBody_J2.push_back(j2);

		real3 invJ1, invJ2;
		CalcInvJ(j1, j2, invJ1, invJ2);
		rigidBody_InvJ1.push_back(invJ1);
		rigidBody_InvJ2.push_back(invJ2);
	}
	randLinVec.clear();
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesFromFile(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & spheresVelMas,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const real3 cMin,
		const real3 cMax,
		const string fileNameRigids,
		const real_ rhoRigid) {

	fstream ifileSpheres(fileNameRigids.c_str(), ios::in);
	//rRigidBody = .08 * sizeScale;//.06 * sizeScale;//.125 * sizeScale; // .25 * sizeScale; //.06 * sizeScale;//.08 * sizeScale;//.179 * sizeScale;

	real_ x, y, z;
	char ch;

	real_ dumRRigidBody1, dumRRigidBody2, dumRRigidBody3;
	ifileSpheres >> x >> ch >> y >> ch >> z >> ch >> dumRRigidBody1 >> ch >> dumRRigidBody2 >> ch >> dumRRigidBody3;
	int counterRigid = 0;
	while (!ifileSpheres.eof()) {
		//real_ r = rRigidBody * (.75 + .75 * real_(rand())/RAND_MAX);
		for (int period = 0; period < nPeriod; period++) {
			rigidPos.push_back(R3(x * sizeScale + period * sPeriod, y * sizeScale, z * sizeScale));
			 mQuatRot.push_back(R4(1, 0, 0, 0));
			//real_ r1 = .8 * rRigidBody, r2 = 1.2 * rRigidBody, r3 = 3 * rRigidBody;
			//real_ r1 = rRigidBody, r2 = rRigidBody, r3 = rRigidBody;
			real_ r1 = dumRRigidBody1 * sizeScale;
			real_ r2 = dumRRigidBody2 * sizeScale;
			real_ r3 = dumRRigidBody3 * sizeScale;
			ellipsoidRadii.push_back(R3(r1, r2, r3));
			real_ mass;
			real3 j1, j2;

			CreateMassMomentEllipsoid(mass, j1, j2, r1, r2, r3, rhoRigid);						//create Ellipsoid
			//CreateMassMomentCylinder(mass, j1, j2, r1, cMin, cMax, rhoRigid);			//create Cylinder

			spheresVelMas.push_back(R4(0, 0, 0, real_(mass)));
			rigidBodyOmega.push_back(R3(0, 0, 0));
			rigidBody_J1.push_back(j1);
			rigidBody_J2.push_back(j2);

			real3 invJ1, invJ2;
			CalcInvJ(j1, j2, invJ1, invJ2);
			rigidBody_InvJ1.push_back(invJ1);
			rigidBody_InvJ2.push_back(invJ2);
		}
		ifileSpheres >> x >> ch >> y >> ch >> z >> ch >> dumRRigidBody1 >> ch >> dumRRigidBody2 >> ch >> dumRRigidBody3;
		counterRigid ++;
	}
	ifileSpheres.close();
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPattern(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & spheresVelMas,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const real3 referenceR,
		const real_ rhoRigid,
		int3 stride) {

	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	//printf("cMin %f %f %f, cMax %f %f %f\n", straightChannelBoundaryMin.x, straightChannelBoundaryMin.y, straightChannelBoundaryMin.z, straightChannelBoundaryMax.x, straightChannelBoundaryMax.y, straightChannelBoundaryMax.z);
	real3 spaceRigids = 2 * (referenceR + 0.6 * R3(HSML));
	real3 n3Rigids = (straightChannelBoundaryMax - straightChannelBoundaryMin) / spaceRigids;
	for (int i = 1; i < n3Rigids.x - 1; i += stride.x) {
		for  (int j = 1; j < n3Rigids.y - 1; j += stride.y) {
			 for (int k = 1; k < n3Rigids.z - 1; k += stride.z) {
				 real3 pos = straightChannelBoundaryMin + R3(i, j, k) * spaceRigids;
				 //printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				 rigidPos.push_back(pos);
				 mQuatRot.push_back(R4(1, 0, 0, 0));
				 ellipsoidRadii.push_back(referenceR);
				 real_ mass;
				 real3 j1, j2;
				 CreateMassMomentEllipsoid(mass, j1, j2, referenceR.x, referenceR.y, referenceR.z, rhoRigid);						//create Ellipsoid

				spheresVelMas.push_back(R4(0, 0, 0, real_(mass)));
				rigidBodyOmega.push_back(R3(0, 0, 0));
				rigidBody_J1.push_back(j1);
				rigidBody_J2.push_back(j2);

				real3 invJ1, invJ2;
				CalcInvJ(j1, j2, invJ1, invJ2);
				rigidBody_InvJ1.push_back(invJ1);
				rigidBody_InvJ2.push_back(invJ2);
			 }
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPatternPipe(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & spheresVelMas,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const real3 referenceR,
		const real_ rhoRigid,
		real3 cMin,
		real3 cMax) {

	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	//printf("cMin %f %f %f, cMax %f %f %f\n", straightChannelBoundaryMin.x, straightChannelBoundaryMin.y, straightChannelBoundaryMin.z, straightChannelBoundaryMax.x, straightChannelBoundaryMax.y, straightChannelBoundaryMax.z);
	real3 spaceRigids = 2 * (referenceR + 1.1 * R3(HSML));
	real3 n3Rigids = (cMax - cMin) / spaceRigids;
	for (int i = 1; i < n3Rigids.x - 1; i++) {
		for  (real_ r = spaceRigids.x; r < channelRadius - spaceRigids.x; r += spaceRigids.x) {
			real_ dTeta = spaceRigids.x / r;
			 for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				 real3 pos = R3(cMin.x, channelCenterYZ.x, channelCenterYZ.y) +  R3(i * spaceRigids.x, real_(r  * cos(teta)), real_(r *  sin(teta)) );

				 //printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				 rigidPos.push_back(pos);
				 mQuatRot.push_back(R4(1, 0, 0, 0));
				 ellipsoidRadii.push_back(referenceR);
				 real_ mass;
				 real3 j1, j2;
				 CreateMassMomentEllipsoid(mass, j1, j2, referenceR.x, referenceR.y, referenceR.z, rhoRigid);						//create Ellipsoid

				spheresVelMas.push_back(R4(0, 0, 0, real_(mass)));
				rigidBodyOmega.push_back(R3(0, 0, 0));
				rigidBody_J1.push_back(j1);
				rigidBody_J2.push_back(j2);

				real3 invJ1, invJ2;
				CalcInvJ(j1, j2, invJ1, invJ2);
				rigidBody_InvJ1.push_back(invJ1);
				rigidBody_InvJ2.push_back(invJ2);
			 }
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPatternStepPipe(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & spheresVelMas,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const real3 referenceR,
		const real_ rhoRigid,
		real3 cMin,
		real3 cMax) {

	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	//printf("cMin %f %f %f, cMax %f %f %f\n", straightChannelBoundaryMin.x, straightChannelBoundaryMin.y, straightChannelBoundaryMin.z, straightChannelBoundaryMax.x, straightChannelBoundaryMax.y, straightChannelBoundaryMax.z);
	real3 spaceRigids = 2 * (referenceR + 2.0 * R3(HSML));

	real_ tubeLength = cMax.x - cMin.x;
	real_ r1 = channelRadius;
	real_ r2 = 0.5 * channelRadius;
	real_ d1 = cMin.x +  .25 * tubeLength;
	real_ d2 = cMin.x +  .75 * tubeLength;

	real_ space = max( max(spaceRigids.x, spaceRigids.y) , spaceRigids.z);
	real_ nX = (d2 - d1) / space;
	for (int i = 1; i < nX - 1; i++) {
		for  (real_ r = space; r < r1 -space; r += space) {
			real_ dTeta = space / r;
			 for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				 real3 pos = R3(d1, channelCenterYZ.x, channelCenterYZ.y) +  R3(i * space, real_(r  * cos(teta)), real_(r *  sin(teta)) );

				 //printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				 rigidPos.push_back(pos);
				 mQuatRot.push_back(R4(1, 0, 0, 0));
				 ellipsoidRadii.push_back(referenceR);
				 real_ mass;
				 real3 j1, j2;
				 CreateMassMomentEllipsoid(mass, j1, j2, referenceR.x, referenceR.y, referenceR.z, rhoRigid);						//create Ellipsoid

				spheresVelMas.push_back(R4(0, 0, 0, real_(mass)));
				rigidBodyOmega.push_back(R3(0, 0, 0));
				rigidBody_J1.push_back(j1);
				rigidBody_J2.push_back(j2);

				real3 invJ1, invJ2;
				CalcInvJ(j1, j2, invJ1, invJ2);
				rigidBody_InvJ1.push_back(invJ1);
				rigidBody_InvJ2.push_back(invJ2);
			 }
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPatternPipe_KindaRandom(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & spheresVelMas,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const real3 referenceR,
		const real_ rhoRigid,
		real3 cMin,
		real3 cMax,
		int numParticles) {

	srand ( time(NULL) );
	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	//printf("cMin %f %f %f, cMax %f %f %f\n", straightChannelBoundaryMin.x, straightChannelBoundaryMin.y, straightChannelBoundaryMin.z, straightChannelBoundaryMax.x, straightChannelBoundaryMax.y, straightChannelBoundaryMax.z);
//	real3 spaceRigids = 2 * (referenceR + 2 * R3(HSML));
	real3 spaceRigids = 2 * (referenceR + 1.1 * R3(HSML));
	real3 n3Rigids = (cMax - cMin) / spaceRigids;
	int totalNumberPossibleParticles = 0;
	for (int i = 1; i < n3Rigids.x ; i++) {
		real_ shiftR = real_(i % 4) / 4 * 2 * referenceR.x;
		for  (real_ r = 0.5 * spaceRigids.x + shiftR; r < channelRadius - 0.5 * spaceRigids.x; r += spaceRigids.x) {
			if (i == 1) printf("r %f\n", r);
			real_ dTeta = spaceRigids.x / r;
			 for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				 totalNumberPossibleParticles ++;
			 }
		}
	}
	int skipCount = totalNumberPossibleParticles / numParticles;

	printf("totalNumberPossibleParticles %d  skipCount %d\n", totalNumberPossibleParticles, skipCount);
	int particleCounter = 0;
	int loopCounter = -1;

	for (int i = 1; i < n3Rigids.x ; i++) {
		for  (real_ r = 0.5 * spaceRigids.x; r < channelRadius - 0.5 * spaceRigids.x; r += spaceRigids.x) {
			real_ dTeta = spaceRigids.x / r;
			 for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				 loopCounter ++;
				 int randomSeed = int(skipCount * real_(rand() - 1) / RAND_MAX);
				 bool goRandom = true;
				 if (loopCounter < 0.8 * totalNumberPossibleParticles) {
					 goRandom = true;
				 } else {
					 goRandom = false;
				 }
				 bool meetSeed;
				 if (goRandom) {
					// printf("loopCounter %d skipCount %d randomSeed %d\n", loopCounter, skipCount, randomSeed);
					 meetSeed = (loopCounter % skipCount == randomSeed);
				 } else {
					 meetSeed = true;
				 }
//				 printf("meetSeed: %s\n",(meetSeed)?"true":"false");
				 if (!meetSeed) continue;
//				 printf("ha\n");
				 if (particleCounter >= numParticles) continue;
				 particleCounter ++;
				 real3 pos = R3(cMin.x, channelCenterYZ.x, channelCenterYZ.y) +  R3(i * spaceRigids.x, real_(r  * cos(teta)), real_(r *  sin(teta)) );

				 //printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				 rigidPos.push_back(pos);
				 mQuatRot.push_back(R4(1, 0, 0, 0));
				 ellipsoidRadii.push_back(referenceR);
				 real_ mass;
				 real3 j1, j2;
				 CreateMassMomentEllipsoid(mass, j1, j2, referenceR.x, referenceR.y, referenceR.z, rhoRigid);						//create Ellipsoid

				spheresVelMas.push_back(R4(0, 0, 0, real_(mass)));
				rigidBodyOmega.push_back(R3(0, 0, 0));
				rigidBody_J1.push_back(j1);
				rigidBody_J2.push_back(j2);

				real3 invJ1, invJ2;
				CalcInvJ(j1, j2, invJ1, invJ2);
				rigidBody_InvJ1.push_back(invJ1);
				rigidBody_InvJ2.push_back(invJ2);
			 }
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideSphere(real3 sphParPos, real4 spherePosRad, real_ clearance) {
	real3 dist3 = sphParPos - R3(spherePosRad);
	if (length(dist3) < spherePosRad.w + clearance) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideEllipsoid(real3 sphParPos, real3 rigidPos, Rotation rot, real3 radii, real_ clearance) {
	real3 dist3GF = sphParPos - rigidPos;
	real3 dist3 = dist3GF.x * R3(rot.a00, rot.a01, rot.a02) + dist3GF.y * R3(rot.a10, rot.a11, rot.a12) + dist3GF.z * R3(rot.a20, rot.a21, rot.a22);
	real3 mappedDist = dist3 / (radii + R3(clearance));
	if (length(mappedDist) < 1) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideCylinder_XZ(real3 sphParPos, real3 rigidPos, real3 radii, real_ clearance) {
	real3 dist3 = sphParPos - rigidPos;
	dist3.y = 0;
	//if (length(dist3) < spherePosRad.w + 2 * (HSML)) {
	if (length(dist3) < radii.x + clearance) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInEllipse(real2 pos, real2 radii) {
//	printf(" pos %f %f  r2 %f %f\n", pos.x, pos.y, radii.x, radii.y);
//	real2 kk  = pos / radii;
//	printf("kk.x kk.y   %f %f \n", kk.x, kk.y);
//	printf("lengthkk %f and the other length %f \n", length(kk), length(pos / radii));
	return length( pos / radii );
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsOutBoundaryEllipsoid(real2 coord, real2 cent2, real2 r2) {
	real2 relDist2 = coord - cent2;
//	printf("**** relDist %f %f  r2 %f %f\n", relDist2.x, relDist2.y, r2.x, r2.y);
	real_ criteria = IsInEllipse(relDist2, r2);
	if (criteria < 1) {
//		printf("yeap my friend\n");
		real_ x = relDist2.x / criteria;
		return -1 * length(relDist2 - R2(x, relDist2.y / relDist2.x * x));
	} else {
//		printf("creiteria %f\n", criteria);
		return 1; //a positive number implying it is outside
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
real_ IsInBoundaryEllipsoid(real2 coord, real2 cent2, real2 r2) {
	real2 relDist2 = coord - cent2;
//	printf("**** relDist %f %f  r2 %f %f\n", relDist2.x, relDist2.y, r2.x, r2.y);
	real_ criteria = IsInEllipse(relDist2, r2);
	if (criteria > 1) {
//		printf("neap my friend\n");
		real_ x = relDist2.x / criteria;
		return -1 * length(relDist2 - R2(x, relDist2.y / relDist2.x * x));
	} else {
//		printf("creiteria %f\n", criteria);
		return 1; //a positive number implying it is outside
	}
}
////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//real_ IsInsideCurveOfSerpentine(real3 posRad) {
//	const real_ sphR = HSML;
//	real_ x, y;
//	real_ distFromWall = 0;//2 * sphR;//0;//2 * sphR;
//	real_ penDist = 0;
//	real_ largePenet = -5*sphR;//like a large number. Should be negative (assume large initial penetration)
//	real_ penDist2 = 0;
//	bool isOut = false;
//
//	if (posRad.y < -toleranceZone || posRad.y > 1.0 * sizeScale + toleranceZone) {
//		return largePenet;
//	}
//	else if (posRad.y < 0) {
//		penDist2 = posRad.y;
//		isOut = true;
//	}
//	else if ( posRad.y > 1.0 * sizeScale) {
//		penDist2 = (1.0 * sizeScale - posRad.y);
//		isOut = true;
//	}
//	//serpentine
//	real_ r1 = 1.3 * sizeScale, r2 = 1.0 * sizeScale, r3=2.0 * sizeScale, r4 = 0.3 * sizeScale;
//	x = fmod(posRad.x, sPeriod); //posRad.x - int(posRad.x / sPeriod) * sPeriod; //fmod
//	y = posRad.z;
//	if (x >= 0 && x < 1.3 * sizeScale) {
//		if (y < -3 * toleranceZone) return largePenet;
//		if (y < 0) return (x - 1.3 * sizeScale);
//		penDist = IsOutBoundaryCircle(R2(x, y), R2(0, 0), r1); if (penDist < 0) return penDist;
//		penDist = IsInBoundaryCircle(R2(x, y), R2(0, 1.0 * sizeScale), r3); if (penDist < 0) return penDist;
//	} else if (x >= 1.3 * sizeScale && x < 2.0 * sizeScale) {
//		if (y > 1.0 * sizeScale) { penDist = IsInBoundaryCircle(R2(x, y), R2(0, 1.0 * sizeScale), r3); if (penDist < 0) return penDist; }
//		else if (y < 0) { penDist = IsInBoundaryCircle(R2(x, y), R2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist; }
//	} else if (x >= 2.0 * sizeScale && x < 2.6 * sizeScale) {
//		if (y < .55 * sizeScale) {
//			penDist = IsInBoundaryCircle(R2(x, y), R2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist;
//			penDist = IsOutBoundaryCircle(R2(x, y), R2(2.3 * sizeScale, .55 * sizeScale), r4); if (penDist < 0) return penDist; }
//		else if (y < 2 * sizeScale) { penDist = IsOutBoundaryCircle(R2(x, y), R2(2.3 * sizeScale, y), r4); if (penDist < 0) return penDist; }
//		else return largePenet;
//	} else if (x >= 2.6 * sizeScale && x < 3.3 * sizeScale) {
//		if (y > 1.0 * sizeScale) { penDist = IsInBoundaryCircle(R2(x, y), R2(4.6 * sizeScale, 1.0 * sizeScale), r3); if (penDist < 0) return penDist; }
//		else if (y < 0) { penDist = IsInBoundaryCircle(R2(x, y), R2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist; }
//	} else if (x >= 3.3 * sizeScale && x < 4.6 * sizeScale) {
//		if (y < -3 * toleranceZone) return largePenet;
//		if (y < 0) return 3.3 * sizeScale - x;
//		penDist = IsOutBoundaryCircle(R2(x, y), R2(4.6 * sizeScale, 0), r1); if (penDist < 0) return penDist;
//		penDist = IsInBoundaryCircle(R2(x, y), R2(4.6 * sizeScale, 1.0 * sizeScale), r3); if (penDist < 0) return penDist;
//	}
//	if (!isOut)
//		return -largePenet;
//	return penDist2;
//}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideCurveOfSerpentineBeta(real3 posRad) {
	const real_ sphR = HSML;
	real_ x, y;
	real_ distFromWall = 0; //2 * sphR;//0;//2 * sphR;
	real_ penDist = 0;
	real_ largePenet = -5*sphR;//like a large number. Should be negative (assume large initial penetration)
	real_ penDist2 = 0;
	bool isOut = false;

	if (posRad.y < -toleranceZone || posRad.y > 1.0 * sizeScale + toleranceZone) {
		return largePenet;
	}
	else if (posRad.y < 0) {
		penDist2 = posRad.y;
		isOut = true;
	}
	else if ( posRad.y > 1.0 * sizeScale) {
		penDist2 = (1.0 * sizeScale - posRad.y);
		isOut = true;
	}
	//serpentine
	//real_ r1 = 1.3 * sizeScale, r2 = 1.0 * sizeScale, r3=2.0 * sizeScale, r4 = 0.3 * sizeScale;

	x = fmod(posRad.x, sPeriod); //posRad.x - int(posRad.x / sPeriod) * sPeriod; //fmod
	y = posRad.z;
	if (y > 0) {
		if (x >= 0 && x < r2_2.x) {
			penDist = IsOutBoundaryEllipsoid(R2(x, y), R2(0, 0), r2_2); if (penDist < 0) return penDist;
		}
		if (x >=0 && x< r3_2.x + toleranceZone) {
			penDist = IsInBoundaryEllipsoid(R2(x, y), R2(0, 0), r3_2); if (penDist < 0) return penDist;
		}
		if (x >= r3_2.x + toleranceZone && x < r3_2.x + 2 * r4_2.x - toleranceZone) {
			return largePenet;
		}
		if (x > r3_2.x + 2 * r4_2.x - toleranceZone && x < 2 * r3_2.x + 2 * r4_2.x) {
			penDist = IsInBoundaryEllipsoid(R2(x, y), R2(2 * r3_2.x + 2 * r4_2.x, 0), r3_2); if (penDist < 0) return penDist;
		}
		if (x > r2_2.x + 2 * r1_2.x && x < 2 * r3_2.x + 2 * r4_2.x) {
			penDist = IsOutBoundaryEllipsoid(R2(x, y), R2(2 * r3_2.x + 2 * r4_2.x, 0), r2_2); if (penDist < 0) return penDist;
		}
	} else {
		if (x > r3_2.x && x < r3_2.x + 2 * r4_2.x) {
			penDist = IsOutBoundaryEllipsoid(R2(x, y), R2(r3_2.x + r4_2.x, 0), r4_2); if (penDist < 0) return penDist;
		}
		if (x > r2_2.x - toleranceZone && x < r2_2.x + 2 * r1_2.x + toleranceZone) {
			penDist = IsInBoundaryEllipsoid(R2(x, y), R2(r2_2.x + r1_2.x, 0), r1_2); if (penDist < 0) return penDist;
		} else {
			return largePenet;
		}
	}
	if (!isOut)
		return -largePenet;
	return penDist2;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideSerpentine(real3 posRad) {
	const real_ sphR = HSML;
	real_ x, y;
	real_ distFromWall = 0;//2 * sphR;//0;//2 * sphR;
	real_ penDist = 0;
	real_ largePenet = -5*sphR;//like a large number. Should be negative (assume large initial penetration)
	real_ penDist2 = 0;
	bool isOut = false;

	if (posRad.y < -toleranceZone || posRad.y > 1.0 * sizeScale + toleranceZone) {
		return largePenet;
	}
	else if (posRad.y < 0) {
		penDist2 = posRad.y;
		isOut = true;
	}
	else if ( posRad.y > 1.0 * sizeScale) {
		penDist2 = (1.0 * sizeScale - posRad.y);
		isOut = true;
	}
	//serpentine
	if (posRad.x < nPeriod * sPeriod) {
		return IsInsideCurveOfSerpentineBeta(posRad);
	}
	else {
		//straight channel
		x = posRad.x - nPeriod * sPeriod;
		y = posRad.z;

		if (y < 0) {
			if (x < r2_2.x - toleranceZone) {
				return largePenet;
			}
			if (x >= r2_2.x -  toleranceZone && x < r2_2.x + 2 * r1_2.x + toleranceZone) {
				penDist = IsInBoundaryEllipsoid(R2(x, y), R2(r2_2.x + r1_2.x, 0), r1_2); if (penDist < 0) return penDist;
			}
			if (x >= r3_2.x && x < r3_2.x + 2 * r4_2.x) {
				penDist = IsOutBoundaryEllipsoid(R2(x, y), R2(r3_2.x + r4_2.x, 0), r4_2); if (penDist < 0) return penDist;
			}
			if (x >= r2_2.x + 2 * r1_2.x + toleranceZone) {
				return largePenet;
			}
		} else {
			if (x >= 0 && x < r2_2.x) {
				penDist = IsOutBoundaryEllipsoid(R2(x, y), R2(0, 0), r2_2); if (penDist < 0) return penDist;
			}
			if (x >=0 && x< r3_2.x + toleranceZone) {
				penDist = IsInBoundaryEllipsoid(R2(x, y), R2(0, 0), r3_2); if (penDist < 0) return penDist;
			}
			if (x >= r3_2.x + toleranceZone && x < r3_2.x + 2 * r4_2.x - toleranceZone) {
				return largePenet;
			}
			if (x >= r3_2.x + 2 * r4_2.x - toleranceZone && x <  r3_2.x + 2 * r4_2.x + r6_2.x) {
				penDist = IsInBoundaryEllipsoid(R2(x, y), R2( r3_2.x + 2 * r4_2.x + r6_2.x, 0), r6_2); if (penDist < 0) return penDist;
			}
			if (x >= r2_2.x + 2 * r1_2.x && x < r2_2.x + 2 * r1_2.x + r5_2.x) {
				penDist = IsOutBoundaryEllipsoid(R2(x, y), R2( r2_2.x + 2 * r1_2.x + r5_2.x, 0), r5_2); if (penDist < 0) return penDist;
			}
			//****** horizontal walls
			x = x - (r3_2.x + 2 * r4_2.x + r6_2.x);
			if (x > 0) {
				real2 y2_slimHor = R2(2.314, 3.314)  * sizeScale;
				real2 y2_endHor = R2(r2_2.y, r3_2.y);
				if (x < x_FirstChannel) {
					penDist = y - r5_2.y; if (penDist < 0) return penDist;  //note that y is negative, fabs(y) = -y
					penDist = -y + r6_2.y; if (penDist < 0) return penDist;
				}
				if (x >= x_FirstChannel + toleranceZone &&  x < x_FirstChannel + x_SecondChannel - toleranceZone) {
					penDist = y - y2_slimHor.x;  if (penDist < 0) return penDist;
					penDist = -y + y2_slimHor.y; if (penDist < 0) return penDist;
				}
				if (x >= x_FirstChannel + x_SecondChannel){
					penDist = y - y2_endHor.x;  if (penDist < 0) return penDist;
					penDist = -y + y2_endHor.y; if (penDist < 0) return penDist;
				}
				//****** vertical walls
				if (x >= x_FirstChannel && x < x_FirstChannel + toleranceZone) {
					if (y > r6_2.y + toleranceZone || y < r5_2.y - toleranceZone) {return largePenet;}
					if (y < y2_slimHor.x || y > y2_slimHor.y) {return x_FirstChannel - x;}
				}
				if (x >= x_FirstChannel + x_SecondChannel - toleranceZone && x < x_FirstChannel + x_SecondChannel) {
					if (y > y2_endHor.y + toleranceZone || y < y2_endHor.x - toleranceZone) {return largePenet;}
					if (y < y2_slimHor.x || y > y2_slimHor.y) {return x - (x_FirstChannel + x_SecondChannel);}
				}
			}
		}
	}
	if (!isOut)
		return -largePenet;
	return penDist2;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideStraightChannel(real3 posRad) {
	const real_ sphR = HSML;
	real_ penDist1 = 0;
	real_ penDist2 = 0;
	//const real_ toleranceZone = 2 * sphR;
	real_ largePenet = -5 * sphR;		//like a large number. Should be negative (assume large initial penetration)

	if (posRad.z > straightChannelBoundaryMax.z) {
		penDist1 = straightChannelBoundaryMax.z - posRad.z;
	}
	if (posRad.z < straightChannelBoundaryMin.z) {
		penDist1 = posRad.z - straightChannelBoundaryMin.z;
	}

	if (posRad.y < straightChannelBoundaryMin.y) {
		penDist2 = posRad.y - straightChannelBoundaryMin.y;
	}
	if (posRad.y > straightChannelBoundaryMax.y) {
		penDist2 = straightChannelBoundaryMax.y - posRad.y;
	}
	if (penDist1 < 0 && penDist2 < 0) {
		return Min(penDist1, penDist2);
	}
	if (penDist1 < 0) return penDist1;
	if (penDist2 < 0) return penDist2;
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideStraightChannel_XZ(real3 posRad) {
	const real_ sphR = HSML;
	real_ penDist1 = 0;
	//const real_ toleranceZone = 2 * sphR;
	real_ largePenet = -5 * sphR;		//like a large number. Should be negative (assume large initial penetration)

	//if (posRad.z > 3.0 * sizeScale) {penDist1 = 3.0 * sizeScale - posRad.z;}
	if (posRad.z > 2.0 * sizeScale) {
		penDist1 = 2.0 * sizeScale - posRad.z;
	}
	if (posRad.z < 1.0 * sizeScale) {
		penDist1 = posRad.z - 1.0 * sizeScale;
	}

	if (penDist1 < 0) return penDist1;
	//printf("hey \n");
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideTube(real3 posRad, real3 cMax, real3 cMin) {
	const real_ sphR = HSML;
	real_ penDist1 = 0;
	//const real_ toleranceZone = 2 * sphR;
	real_ largePenet = -5 * sphR; //like a large number. Should be negative (assume large initial penetration)
	real2 centerLine = R2(channelCenterYZ.x, channelCenterYZ.y);
	real_ r = length(R2(posRad.y, posRad.z) - centerLine);
//printf("ch R %f\n", channelRadius);
	//if (posRad.z > 3.0 * sizeScale) {penDist1 = 3.0 * sizeScale - posRad.z;}

	if (r > channelRadius) {
		penDist1 = channelRadius - r;
	}
	if (penDist1 < 0) return penDist1;
	//printf("hey \n");
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideStepTube(real3 posRad, real3 cMax, real3 cMin) {
	const real_ sphR = HSML;
	real_ penDist1 = 0;
	real_ penDist2 = 0;
	//const real_ toleranceZone = 2 * sphR;
	real_ largePenet = -5 * sphR; //like a large number. Should be negative (assume large initial penetration)
	real2 centerLine = R2(channelCenterYZ.x, channelCenterYZ.y);
	real_ r = length(R2(posRad.y, posRad.z) - centerLine);
//printf("ch R %f\n", channelRadius);
	//if (posRad.z > 3.0 * sizeScale) {penDist1 = 3.0 * sizeScale - posRad.z;}

	real_ tubeLength = cMax.x - cMin.x;
	real_ r1 = channelRadius;
	real_ r2 = 0.5 * channelRadius;
	real_ d1 = cMin.x +  .25 * tubeLength;
	real_ d2 = cMin.x +  .75 * tubeLength;
	real_ d3 = cMin.x +  tubeLength;
	if (posRad.x < d1) {
		if (r > r2) {
			penDist1 = r2 - r;
		}
	} else if (posRad.x < d2) {
		if (r > r1) {
			penDist1 = r1 - r;
		}
	} else {
		if (r > r2) {
			penDist1 = r2 - r;
		}
	}
	// vertical wall
	if (r < r1 + toleranceZone) {
		if (posRad.x < d1 ) {
			penDist2 = posRad.x - d1;
		}
		if (posRad.x > d2) {
			penDist2 = d2 - posRad.x;
		}
	}

	if (penDist1 < 0 && penDist2 < 0) return max(penDist1, penDist2); //note: both are negative. max is closer to boundary
	if (penDist1 < 0) return penDist1; //penDist2 is always either zero or negative
	//printf("hey \n");
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int2 CreateFluidParticles(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		thrust::host_vector<int3> & mType,
		thrust::host_vector<real3> & mPosRadBoundary,
		thrust::host_vector<real4> & mVelMasBoundary,
		thrust::host_vector<real4> & mRhoPresMuBoundary,
		thrust::host_vector<int3> & mTypeBoundary,
		real_ & sphParticleMass,
		const thrust::host_vector<real3> & rigidPos,
		const thrust::host_vector<Rotation> rigidRotMatrix,
		const thrust::host_vector<real3> & ellipsoidRadii,
		real_ sphR,
		real3 cMax,
		real3 cMin,
		real_ rho,
		real_ pres,
		real_ mu) {
	//real2 rad2 = .5 * R2(cMax.y - cMin.y, cMax.z - cMin.z);
	//channelRadius = (rad2.x < rad2.y) ? rad2.x : rad2.y;
	int num_FluidParticles = 0;
	int num_BoundaryParticles = 0;
	srand(964);
	//real_ initSpace0 = 0.9 * sphR; //1.1 * sphR;//1.1 * sphR;//pow(4.0 / 3 * PI, 1.0 / 3) * sphR;
	real_ multInitSpace = MULT_INITSPACE;//0.9;//0.9;
	real_ initSpace0 = multInitSpace * sphR;
	printf("initSpaceFluid = %f * sphR\n", multInitSpace);
	int nFX = ceil((cMax.x - cMin.x) / (initSpace0));
	real_ initSpaceX = (cMax.x - cMin.x) / nFX;
	//printf("orig nFx and nFx %f %f\n", (cMax.x - cMin.x) / initSpace, ceil ((cMax.x - cMin.x) / (initSpace)));
	int nFY = ceil((cMax.y - cMin.y) / (initSpace0));
	real_ initSpaceY = (cMax.y - cMin.y) / nFY;
	int nFZ = ceil((cMax.z - cMin.z) / (initSpace0));
	real_ initSpaceZ = (cMax.z - cMin.z) / nFZ;
	//printf("&&&&& %f   %f %f %f \n", 1.1 * sphR, initSpaceX, initSpaceY, initSpaceZ);
	printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ, (nFY - 1) * initSpaceY, initSpaceY);
	sphParticleMass = (initSpaceX * initSpaceY * initSpaceZ) * rho;
	//printf("sphParticleMass * 1e12 %f\n", (initSpaceX * initSpaceY * initSpaceZ) * rho*1e12);

	//printf("** nFX, nFX, nFX %d %d %d\n", nFX, nFY, nFZ);
	//printf("cMin.z + initSpaceZ * (nFZ - 1) %f cMax.z %f initSpaceZ %f initSpace0 %f\n", cMin.z + initSpaceZ * (nFZ - 1), cMax.z, initSpaceZ, initSpace0);
	//printf("dist&*&*&* %f %f\n", (cMax.x - cMin.x) - initSpace * (nFX-1) - 2 * sphR, sphR);

	//for(int i=.5 * nFX - 1; i < .5 * nFX; i+=2) {
	//	for (int j=0; j < nFY; j++) {
	//	//for (int j=.5 * nFY - 1; j < .5 * nFY; j++) {
	//		for (int k=.5 * nFZ - 1; k < .5 * nFZ; k+=2) {
	//			if (j != 0 && j != nFY - 1) continue;

	for (int i = 0; i < nFX; i++) {
		for (int j = 0; j < nFY; j++) {
			for (int k = 0; k < nFZ; k++) {
				real3 posRad;
//					printf("initSpace X, Y, Z %f %f %f \n", initSpaceX, initSpaceY, initSpaceZ);
				posRad = cMin + R3(i * initSpaceX, j * initSpaceY, k * initSpaceZ)
								+ R3(.5 * initSpace0)/* + R3(sphR) + initSpace * .05 * (real_(rand()) / RAND_MAX)*/;
				real_ penDist = 0;
				bool flag = true;
				///penDist = IsInsideCurveOfSerpentineBeta(posRad);
				///penDist = IsInsideSerpentine(posRad);
				///penDist = IsInsideStraightChannel(posRad);
				///penDist = IsInsideStraightChannel_XZ(posRad);
				penDist = IsInsideTube(posRad, cMax, cMin);
				///penDist = IsInsideStepTube(posRad, cMax, cMin);

				if (penDist < -toleranceZone) flag = false;
				if (flag) {
					for (int rigidSpheres = 0; rigidSpheres < rigidPos.size(); rigidSpheres++) {
						if (IsInsideEllipsoid(posRad, rigidPos[rigidSpheres], rigidRotMatrix[rigidSpheres], ellipsoidRadii[rigidSpheres], initSpace0)) { flag = false; }
//						if ( IsInsideCylinder_XZ(posRad, rigidPos[rigidSpheres], ellipsoidRadii[rigidSpheres], initSpace0 ) ) { flag = false;}
					}
				}
				if (flag) {
					if (penDist > 0) {
						if (k < nFZ) {
							num_FluidParticles++;
							mPosRad.push_back(posRad);
							mVelMas.push_back(R4(0, 0, 0, sphParticleMass));
							mRhoPresMu.push_back(R4(rho, pres, mu, -1)); //rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
																		//just note that the type, i.e. mRhoPresMu.w is real_.
																		//viscosity of the water is .0018

																		//if (posRad.x < .98 * cMax.x && posRad.x > .96 * cMax.x && posRad.y > .48 * sizeScale &&  posRad.y < .52 * sizeScale
																		//	&& posRad.z > 1.7 * sizeScale &&  posRad.y < 1.75 * sizeScale) {
																		//	if (num_FluidParticles < 1) {
																		//		num_FluidParticles ++;
																		//		mPosRad.push_back(posRad);
																		//		mVelMas.push_back( R4(0, 0, 0, pow(initSpace, 3) * rho) );
																		//		mRhoPresMu.push_back(R4(rho, pres, mu, -1));		//rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
																		//															//just note that the type, i.e. mRhoPresMu.w is real_.
																		//															//viscosity of the water is .0018
																		//	}
							mType.push_back(I3(-1, 0, 0));											//}

						}
					} else {
						num_BoundaryParticles++;
						mPosRadBoundary.push_back(posRad);
						mVelMasBoundary.push_back(R4(0, 0, 0, sphParticleMass));
						mRhoPresMuBoundary.push_back(R4(rho, pres, mu, 0));	//rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
						//just note that the type, i.e. mRhoPresMu.w is real_.
						mTypeBoundary.push_back(I3(0, 0, 0));
						//viscosity of the water is .0018
					}
				}
			}
		}
	}
	return I2(num_FluidParticles, num_BoundaryParticles);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int CreateEllipsoidParticles(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		thrust::host_vector<int3> & mType,
		real3 rigidPos,
		Rotation rigidRotMatrix,
		real3 ellipsoidRadii,
		real4 sphereVelMas,
		real3 rigidBodyOmega,
		real_ sphR,
		real_ sphParticleMass,
		real_ rho,
		real_ pres,
		real_ mu,
		real3 cMin,
		real3 cMax,
		int type) {
	int num_rigidBodyParticles = 0;
	//real_ spacing = .9 * sphR;
	real_ multInitSpace = MULT_INITSPACE;//0.9;//1.0;//0.9;
	real_ spacing = multInitSpace * sphR;
	//printf("initSpaceEllipsoid = %f * sphR\n", multInitSpace);
	for (int k = 0; k < 2; k++) {
		real3 r3 = ellipsoidRadii - R3(k * spacing);
		//printf("r, rigidR, k*spacing %f %f %f\n", r * 1000000, spherePosRad.w * 1000000, k * spacing * 1000000);
		real_ minR = r3.x;
		minR = (minR < r3.y) ? minR : r3.y;
		minR = (minR < r3.z) ? minR : r3.z;
		if ( minR > 0) {
			real_ deltaTeta0 = spacing / r3.z;
			real_ teta = 0.1 * deltaTeta0;
			while (teta < PI - .1 * deltaTeta0) {
				real_ deltaPhi0 = spacing / (r3.z * sin(teta));
				real_ phi = 0.1 * deltaPhi0;
				real_ currentR = 1 / length(R3(sin(teta) * cos(phi), sin(teta) * sin(phi), cos(teta)) / r3);
				real_ nextR = 1 / length(R3(sin(teta + spacing / currentR) * cos(phi), sin(teta + spacing / currentR) * sin(phi), cos(teta + spacing / currentR)) / r3);
				real_ deltaTeta = spacing / max(currentR, nextR);
				while (phi < 2 * PI + deltaPhi0) {
					real3 mult3 = R3(sin(teta) * cos(phi), sin(teta) * sin(phi), cos(teta)) / r3;
					real_ r = 1 / length(mult3);
					real3 posRadRigid_sphParticleLocal = R3(r * sin(teta) * cos(phi), r * sin(teta) * sin(phi), r * cos(teta));
					real3 posRadRigid_sphParticleLocalRotate = posRadRigid_sphParticleLocal.x * R3(rigidRotMatrix.a00, rigidRotMatrix.a10, rigidRotMatrix.a20) +
							posRadRigid_sphParticleLocal.y * R3(rigidRotMatrix.a01, rigidRotMatrix.a11, rigidRotMatrix.a21) +
							posRadRigid_sphParticleLocal.z * R3(rigidRotMatrix.a02, rigidRotMatrix.a12, rigidRotMatrix.a22);

					real3 posRadRigid_sphParticle = posRadRigid_sphParticleLocalRotate + rigidPos;

					mPosRad.push_back(posRadRigid_sphParticle);
					real_ deltaPhiDum = spacing / (r * sin(teta));
					real_ rNewDum = 1 / length(R3(sin(teta) * cos(phi + deltaPhiDum), sin(teta) * sin(phi + deltaPhiDum), cos(teta)) / r3);
					real_ maxR = max(rNewDum, r);
					phi += spacing / (maxR * sin(teta));

					real3 vel = R3(sphereVelMas) + cross(rigidBodyOmega, posRadRigid_sphParticle - rigidPos); //assuming LRF is the same as GRF at time zero (rigibodyOmega is in LRF, the second term of the cross is in GRF)
					//printf("veloc %f %f %f\n", vel.x, vel.y, vel.z);
					mVelMas.push_back(R4(vel, sphParticleMass));
					real_ representedArea = spacing * spacing;
					mRhoPresMu.push_back(R4(rho, pres, mu, type));					// for rigid body particle, rho represents the represented area
					mType.push_back(I3(1, type, 0));
					//																						// for the rigid particles, the fluid properties are those of the contacting fluid particles
					num_rigidBodyParticles++;
					//printf("num_rigidBodyParticles %d\n", num_rigidBodyParticles);
					//printf("y %f\n", y);


					//////reCalc deltaTeta and deltaPhi
				}
				teta += deltaTeta;
			}
		}
	}
	//printf("num_rigidBodyParticles %f\n", num_rigidBodyParticles);
	return num_rigidBodyParticles;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int CreateCylinderParticles_XZ(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		thrust::host_vector<int3> & mType,
		real3 rigidPos,
		real3 ellipsoidRadii,
		real4 sphereVelMas,
		real3 rigidBodyOmega,
		real_ sphR,
		real_ sphParticleMass,
		real_ rho,
		real_ pres,
		real_ mu,
		real3 cMin,
		real3 cMax,
		int type) {
	int num_rigidBodyParticles = 0;
	real_ multInitSpace = MULT_INITSPACE;//0.9;
	real_ spacing = multInitSpace * sphR;
	printf("initSpaceCylinder = %f * sphR\n", multInitSpace);
	for (int k = 0; k < 3; k++) {
		real_ r = ellipsoidRadii.x - k * spacing;
		if (r > 0) {
			real_ deltaTeta = spacing / r;
			for (real_ teta = .1 * deltaTeta; teta < 2 * PI - .1 * deltaTeta; teta += deltaTeta) {
				//printf("gher %f, %f\n", deltaTeta / 2 / PI, deltaTeta);
				for (real_ y = cMin.y; y < cMax.y - .1 * spacing; y += spacing) {
					//printf("aha\n");
					real3 posRadRigid_sphParticle = R3(r * cos(teta), y, r * sin(teta)) + R3(rigidPos.x, 0, rigidPos.z);

					mPosRad.push_back(posRadRigid_sphParticle);
					real3 vel = R3(sphereVelMas) + cross(rigidBodyOmega, posRadRigid_sphParticle - rigidPos); //assuming LRF is the same as GRF at time zero (rigibodyOmega is in LRF, the second term of the cross is in GRF)
					mVelMas.push_back(R4(vel, sphParticleMass));
					real_ representedArea = spacing * spacing;
					mRhoPresMu.push_back(R4(rho, pres, mu, type));					// for rigid body particle, rho represents the represented area
					mType.push_back(I3(1, type, 0));
					//																						// for the rigid particles, the fluid properties are those of the contacting fluid particles 
					num_rigidBodyParticles++;
					//printf("num_rigidBodyParticles %d\n", num_rigidBodyParticles);
					//printf("y %f\n", y);
				}

			}
		}
	}
	//printf("num_rigidBodyParticles %f\n", num_rigidBodyParticles);
	return num_rigidBodyParticles;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int main() {
	//****************************************************************************************
	time_t rawtime;
	struct tm * timeinfo;

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	printf ( "Job was submittet at date/time is: %s\n", asctime (timeinfo) );
	//****************************************************************************************
	//(void) cudaSetDevice(0);
	int numAllParticles = 0;
	//********************************************************************************************************
	//** Initialization
	real_ r = HSML;	//.02;

	//real3 cMin = R3(0, -0.2, -1.2) * sizeScale; 							//for channel and serpentine
	//real3 cMin = R3(0, -0.2, -2) * sizeScale; 							//for channel and serpentine
//	real3 cMin = R3(0, -2, -2) * sizeScale;							//for tube

	real3 cMin = R3(0, -20, -20) * sizeScale;							//for tube

	//real3 cMax = R3( nPeriod * 4.6 + 0, 1.5,  4.0) * sizeScale;  //for only CurvedSerpentine (w/out straight part)
		///real3 cMax = R3( nPeriod * sPeriod + 8 * sizeScale, 1.5 * sizeScale,  4.0 * sizeScale);  //for old serpentine
		///real3 cMax = R3( nPeriod * sPeriod + r3_2.x + 2 * r4_2.x + r6_2.x + x_FirstChannel + 2 * x_SecondChannel, 1.5 * sizeScale,  r6_2.y + 2 * toleranceZone);  //for serpentine
	///real3 cMax = R3( nPeriod * sPeriod, 1.5 * sizeScale,  4.0 * sizeScale);  //for serpentine

	//real3 cMax = R3( nPeriod * 1.0 + 0, 1.5,  4.0) * sizeScale;  //for  straight channel
//	real3 cMax = R3( nPeriod * 20.0 + 0, 11.2 + 2,  11.2 + 2) * sizeScale;  //for  tube
	fstream inp("dist.txt", ios::in);
	real_ distance;
	inp >> distance;
	inp.close();
	printf("distance, %f\n", distance);
	real3 cMax = R3( nPeriod * distance + 0, 102 + 20,  102 + 20) * sizeScale;  //for  tube

				//	real3 cMax = R3(nPeriod * 1.0 + 0, .5,  3.5) * sizeScale;  //for straight channel, sphere
				//	real3 cMin = R3(0, -0.1, 0.5) * sizeScale;
				//	real3 cMax = R3(nPeriod * 1.0 + 0, 1.5, 1.5) * sizeScale;  //for tube channel, sphere
				//	real3 cMin = R3(0, -0.5, -0.5) * sizeScale;

	//printf("a1  cMax.x, y, z %f %f %f,  binSize %f\n", cMax.x, cMax.y, cMax.z, 2 * HSML);
	int3 side0 = I3(floor((cMax.x - cMin.x) / (2 * HSML)), floor((cMax.y - cMin.y) / (2 * HSML)), floor((cMax.z - cMin.z) / (2 * HSML)));
	real3 binSize3 = R3((cMax.x - cMin.x) / side0.x, (cMax.y - cMin.y) / side0.y, (cMax.z - cMin.z) / side0.z);
	real_ binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
//	binSize0 = (binSize0 > binSize3.z) ? binSize0 : binSize3.z;
	binSize0 = binSize3.x;  //for effect of distance. Periodic BC in x direction. we do not care about cMax y and z.
	cMax = cMin + binSize0 * R3(side0);
	straightChannelBoundaryMin = R3(cMin.x, 0.0 * sizeScale, 0.0 * sizeScale);
	straightChannelBoundaryMax = R3(cMax.x, 11.2 * sizeScale, 11.2 * sizeScale);
	printf("cMin.x, y, z %f %f %f cMax.x, y, z %f %f %f,  binSize %f\n", cMin.x, cMin.y, cMin.z, cMax.x, cMax.y, cMax.z, binSize0);
	printf("HSML %f\n", HSML);
	//printf("side0 %d %d %d \n", side0.x, side0.y, side0.z);

	real_ delT = dT_EXPLICIT;

	bool readFromFile = false;  //true;		//true: initializes from file. False: initializes inside the code

	real_ pres = BASEPRES;  //0;//100000;
	real_ mu = mu0;
	real_ rhoRigid;
	//1---------------------------------------------- Initialization ----------------------------------------
	//2------------------------------------------- Generating Random Data -----------------------------------
	// This section can be used to generate ellipsoids' specifications and save them into data.txt	
	//num_FluidParticles = 40000;//5000;//10000;//4096;//16384;//65536;//8192;//32768;;//262144;//1024;//262144;//1024; //262144;//1000000;//16 * 1024;//262144;//1024;//262144;//1024;//262144;//1024 * 16;//262144;//4194304;//262144;//1024;//4194304; //2097152;//262144;//262144;//1024;//65536;//262144;	// The number of bodies

	//--------------------buffer initialization ---------------------------
	thrust::host_vector<int2> referenceArray;
//	thrust::host_vector<int3> referenceArray_Types;
	thrust::host_vector<int2> flexIdentifier;
	thrust::host_vector<real3> mPosRad;			//do not set the size here since you are using push back later
	thrust::host_vector<real4> mVelMas;
	thrust::host_vector<real4> mRhoPresMu;
	thrust::host_vector<int3> mType;

	//thrust::host_vector<real4> spheresPosRad;
	thrust::host_vector<real3> rigidPos;
	thrust::host_vector<real4> mQuatRot;
	thrust::host_vector<real3> ellipsoidRadii;
	thrust::host_vector<real4> spheresVelMas;
	thrust::host_vector<real3> rigidBodyOmega;
	thrust::host_vector<real3> rigidBody_J1;
	thrust::host_vector<real3> rigidBody_J2;
	thrust::host_vector<real3> rigidBody_InvJ1;
	thrust::host_vector<real3> rigidBody_InvJ2;
	real_ sphParticleMass;

	////***** here: define rigid bodies
	string fileNameRigids("spheresPos.dat");
	rhoRigid = 1.0 * rho0;//1.5 * rho0; // rho0;//1180;//1000; //1050; //originally .079 //.179 for cylinder

	//real_ rr = .4 * (real_(rand()) / RAND_MAX + 1);
	real3 r3Ellipsoid = R3(0.5, 0.5, 0.5) * sizeScale;//R3(0.4 * sizeScale); //R3(0.8 * sizeScale); //real3 r3Ellipsoid = R3(.03 * sizeScale); //R3(.05, .03, .02) * sizeScale; //R3(.03 * sizeScale);

	//**
//	int3 stride = I3(5, 5, 5);
//	CreateRigidBodiesPattern(rigidPos, mQuatRot, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, r3Ellipsoid, rhoRigid, stride);
	//**
	CreateRigidBodiesFromFile(rigidPos, mQuatRot, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, cMin, cMax, fileNameRigids, rhoRigid);
	//**
//	//channelRadius = 1.0 * sizeScale;
//	CreateRigidBodiesPatternPipe(rigidPos, mQuatRot, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, r3Ellipsoid, rhoRigid, cMin, cMax);
//	CreateRigidBodiesPatternStepPipe(rigidPos, mQuatRot, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, r3Ellipsoid, rhoRigid, cMin, cMax);

	//**
//	CreateRigidBodiesRandom(rigidPos, mQuatRot, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, r3Ellipsoid, rhoRigid, cMin, cMax, 2); //changed 2 to 4
//	CreateRigidBodiesPatternPipe_KindaRandom(rigidPos, mQuatRot, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, r3Ellipsoid, rhoRigid, cMin, cMax, 128);
	printf("numRigids %d\n", rigidPos.size());
	printf("rigid Radii %f %f %f\n", r3Ellipsoid.x, r3Ellipsoid.y, r3Ellipsoid.z);

	thrust::host_vector<Rotation> rigidRotMatrix(mQuatRot.size());
	ConvertQuatArray2RotArray(rigidRotMatrix, mQuatRot);

	printf("size rigids %d\n", rigidPos.size());
	//---------------------------------------------------------------------
	// initialize fluid particles
	if (readFromFile) {
		int num_FluidParticles = 0;
		fstream inp("initializer.txt", ios::in);
		inp >> num_FluidParticles;
		for (int i = 0; i < num_FluidParticles; i++) {
			char dummyCh;
			real3 posRad = R3(0);
			real4 velMas = R4(0);
			real4 rhoPresMu = R4(0);
			int type1;
			inp >> posRad.x >> dummyCh >> posRad.y >> dummyCh >> posRad.z >> dummyCh >> velMas.x >> dummyCh >> velMas.y
					>> dummyCh >> velMas.z >> dummyCh >> velMas.w >> dummyCh >> rhoPresMu.x >> dummyCh >> rhoPresMu.y >> dummyCh >> rhoPresMu.z
					>> dummyCh >> type1;

			rhoPresMu.z = mu;
			rhoPresMu.w = type1;

			mPosRad.push_back(posRad);
			mVelMas.push_back(velMas);
			mRhoPresMu.push_back(rhoPresMu);
			mType.push_back(I3(type1, 0, 0));
		}
		//num_FluidParticles *= 2;
		referenceArray.push_back(I2(0, num_FluidParticles)); //map fluid -1
//		referenceArray_Types.push_back(I3(-1, 0, 0));
	} else {
		thrust::host_vector<real3> mPosRadBoundary;			//do not set the size here since you are using push back later
		thrust::host_vector<real4> mVelMasBoundary;
		thrust::host_vector<real4> mRhoPresMuBoundary;
		thrust::host_vector<int3> mTypeBoundary;

		int2 num_fluidOrBoundaryParticles = CreateFluidParticles(mPosRad, mVelMas, mRhoPresMu, mType, mPosRadBoundary, mVelMasBoundary, mRhoPresMuBoundary, mTypeBoundary, sphParticleMass,
				rigidPos, rigidRotMatrix, ellipsoidRadii, r, cMax, cMin, rho0, pres, mu);
		referenceArray.push_back(I2(0, num_fluidOrBoundaryParticles.x));  //map fluid -1
//		referenceArray_Types.push_back((I3(-1, 0, 0)));
		numAllParticles += num_fluidOrBoundaryParticles.x;
		printf("num_FluidParticles: %d\n", num_fluidOrBoundaryParticles.x);

		mPosRad.resize(num_fluidOrBoundaryParticles.x + num_fluidOrBoundaryParticles.y);
		mVelMas.resize(num_fluidOrBoundaryParticles.x + num_fluidOrBoundaryParticles.y);
		mRhoPresMu.resize(num_fluidOrBoundaryParticles.x + num_fluidOrBoundaryParticles.y);
		////boundary: type = 0
		int num_BoundaryParticles = 0;
		//printf("size1 %d, %d , numpart %d, numFluid %d \n", mPosRadBoundary.end() - mPosRadBoundary.begin(), mPosRadBoundary.size(), num_fluidOrBoundaryParticles.y, num_fluidOrBoundaryParticles.x);
		thrust::copy(mPosRadBoundary.begin(), mPosRadBoundary.end(), mPosRad.begin() + num_fluidOrBoundaryParticles.x);
		thrust::copy(mVelMasBoundary.begin(), mVelMasBoundary.end(), mVelMas.begin() + num_fluidOrBoundaryParticles.x);
		thrust::copy(mRhoPresMuBoundary.begin(), mRhoPresMuBoundary.end(), mRhoPresMu.begin() + num_fluidOrBoundaryParticles.x);
		thrust::copy(mTypeBoundary.begin(), mTypeBoundary.end(), mType.begin() + num_fluidOrBoundaryParticles.x);
		mPosRadBoundary.clear();
		mVelMasBoundary.clear();
		mRhoPresMuBoundary.clear();
		mTypeBoundary.clear();

		referenceArray.push_back(I2(numAllParticles, numAllParticles + num_fluidOrBoundaryParticles.y));  //map bc 0
//		referenceArray_Types.push_back(I3(0, 0, 0));
		numAllParticles += num_fluidOrBoundaryParticles.y;
		printf("num_BoundaryParticles: %d\n", num_fluidOrBoundaryParticles.y);

		//rigid body: type = 1, 2, 3, ...
		printf("num_RigidBodyParticles: \n");
		for (int rigidSpheres = 0; rigidSpheres < rigidPos.size(); rigidSpheres++) {
			int num_RigidBodyParticles = CreateEllipsoidParticles(mPosRad, mVelMas, mRhoPresMu, mType, rigidPos[rigidSpheres], rigidRotMatrix[rigidSpheres], ellipsoidRadii[rigidSpheres], spheresVelMas[rigidSpheres],
					rigidBodyOmega[rigidSpheres], r, sphParticleMass, rho0, pres, mu, cMin, cMax, rigidSpheres + 1);		//as type
//			int num_RigidBodyParticles = CreateCylinderParticles_XZ(mPosRad, mVelMas, mRhoPresMu, mType,
//													rigidPos[rigidSpheres], ellipsoidRadii[rigidSpheres],
//													 spheresVelMas[rigidSpheres],
//													 rigidBodyOmega[rigidSpheres],
//													 r,
//													sphParticleMass,
//													 rho0, pres, mu,
//													 cMin, cMax,
//													 rigidSpheres + 1);		//as type
			referenceArray.push_back(I2(numAllParticles, numAllParticles + num_RigidBodyParticles));  //map bc : rigidSpheres + 1
//			referenceArray_Types.push_back(I3(1, rigidSpheres, 0));
			numAllParticles += num_RigidBodyParticles;
			//printf(" %d \n", num_RigidBodyParticles);
		}
		printf("\n");
	}

	//@@@@@@@@ rigid body

	thrust::host_vector<uint> bodyIndex(numAllParticles);
	thrust::fill(bodyIndex.begin(), bodyIndex.end(), 1);
	thrust::exclusive_scan(bodyIndex.begin(), bodyIndex.end(), bodyIndex.begin());
	printf("numAllParticles %d\n", numAllParticles);

	if (numAllParticles != 0) {
		cudaCollisions(mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray, flexIdentifier, numAllParticles, cMax, cMin, delT, rigidPos, mQuatRot, spheresVelMas,
				rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, binSize0, channelRadius, channelCenterYZ);
	}
	mPosRad.clear();
	mVelMas.clear();
	mRhoPresMu.clear();
	mType.clear();
	bodyIndex.clear();
	referenceArray.clear();
//	referenceArray_Types.clear();
	flexIdentifier.clear();
	rigidPos.clear();
	mQuatRot.clear();
	rigidRotMatrix.clear();
	ellipsoidRadii.clear();
	spheresVelMas.clear();
	rigidBodyOmega.clear();
	rigidBody_J1.clear();
	rigidBody_J2.clear();
	rigidBody_InvJ1.clear();
	rigidBody_InvJ2.clear();
	return 0;
}
