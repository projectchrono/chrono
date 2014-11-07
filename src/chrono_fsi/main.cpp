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
#include "SDKCollisionSystem.cuh" //just for SimParams
#include "collideSphereSphere.cuh"
#include <algorithm>
#include "FlexibleBodies.cuh"
#include "checkPoint.cuh"


using namespace std;

SimParams paramsH;
//typedef unsigned int uint;

////************ note: paramsH is zero here. These expressions are wrong
const real_ mm = .001;
//// the very original
//const real2 r1_2 = R2(1.351, 1.750) * mm;
//const real2 r2_2 = R2(1.341, 1.754) * mm;
//const real2 r3_2 = R2(2.413, 3.532) * mm;
//const real2 r4_2 = R2(0.279, 0.413) * mm;

//the best one
const real2 r1_2 = R2(1.351, 3.750) * mm;
const real2 r2_2 = R2(1.341, 1.754) * mm;
const real2 r3_2 = R2(2.413, 4.272) * mm;
const real2 r4_2 = R2(0.279, 0.413) * mm;

//const real2 r1_2 = R2(1.351, 1.750) * mm;
//const real2 r2_2 = R2(1.341, 1.2) * mm;
//const real2 r3_2 = R2(2.413, 4.272) * mm;
//const real2 r4_2 = R2(0.279, 0.413) * mm;

const real2 r5_2 = R2(1.675, 1.235) * mm; //r5_2 = R2(1.727, 1.235);  	//the smaller one
const real2 r6_2 = R2(2.747, 4.272) * mm; //the larger one
const real_ x_FirstChannel = 8 * mm;
const real_ sPeriod = 5.384 * mm; //serpentine period
const real_ x_SecondChannel = 2 * mm;
////*************
real_ channelRadius; //5.6 * paramsH.sizeScale; //1.0 * paramsH.sizeScale; //tube
real2 channelCenterYZ;

//&&& some other definitions for boundary and such

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
struct CylinderGeometry {
	real3 pa3;
	real3 pb3;
	real3 center;
	real_ r;
	real_ h;
};
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
const bool initParabolic = false;
real_ vMaxParabolic = .00561;
real_ CalcChannelParabolicVel(real3 posRad) {
	real_ y = posRad.y - paramsH.straightChannelBoundaryMin.y;
	real_ z = posRad.z - paramsH.straightChannelBoundaryMin.z;
	real_ hy = paramsH.straightChannelBoundaryMax.y - paramsH.straightChannelBoundaryMin.y;
	real_ hz = paramsH.straightChannelBoundaryMax.z - paramsH. straightChannelBoundaryMin.z;

	real_ alpha = (hy < hz) ? hy / hz : hz / hy;    //Aspect ratio of the domain
	real_ m = 1.7 + 0.5*pow(alpha,-1.4); 				//Some constants
	real_ n = 2;
	if (alpha > 1.0/3.0) {
	    n = 2 + 0.3 * (alpha - 1.0/3.0);
	}

	// Reference:
	// Shah and London, "Laminar Flow Forced Convection in Ducts", 1978, Eqn. 335-338
	real_ vAve = 0.5341 * vMaxParabolic; // according to the Matlab file
	real_ vx = vAve * ((m+1)/m) * ((n+1)/n) * (1 - pow(fabs((y-0.5*hy)/(0.5*hy)), n)) * (1 - pow(fabs((z-0.5*hz)/(0.5*hz)), m));
	return vx;


//	real_ vx1 = vMaxParabolic * (1 - 4.0 / (hy*hy) * pow(z - 0.5* hy,2));
//	real_ vx2 = vMaxParabolic * (1 - 4.0 / (hy*hy) * pow( hz - z - 0.5* hy,2));
//	real_ vx3 = vMaxParabolic * (1 - 4.0 / (hy*hy) * pow(posRad.y - 0.5* hy,2));
//	if (z < (paramsH.straightChannelBoundaryMin.z + 0.5 * hy)) {
//		return min(vx1, vx3);
//	} else if (z > (paramsH.straightChannelBoundaryMax.z - 0.5 * hy)) {
//		return min(vx2, vx3);
//	} else {
//		return vx3;
//	}
//	return 0;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void ConvertQuatArray2RotArray(thrust::host_vector<Rotation> & rotArray,
		const thrust::host_vector<real4> & quat) {
	for (int i = 0; i < quat.size(); i++) {
		Rotation rot;
		CalcQuat2RotationMatrix(rot, quat[i]);
		rotArray[i] = rot;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
Rotation UnitMatrix() {
	Rotation I;
	I.a01 = I.a02 = I.a10 = I.a12 = I.a20 = I.a21 = 0;
	I.a00 = I.a11 = I.a22 = 1;
	return I;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ myRand() {
	return real_(rand()) / RAND_MAX;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ Min(real_ a, real_ b) {
	return (a < b) ? a : b;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ Max(real_ a, real_ b) {
	return (a < b) ? b : a;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateMassMomentEllipsoid(real_ & mass, real3 & j1, real3 & j2, real_ r1,
		real_ r2, real_ r3, const real_ rhoRigid) {
	mass = 4.0 / 3 * PI * r1 * r2 * r3 * rhoRigid; //for sphere
	j1 = .2 * mass * R3(r2 * r2 + r3 * r3, 0.0f, 0.0f);
	j2 = .2 * mass * R3(r1 * r1 + r3 * r3, 0.0f, r1 * r1 + r2 * r2);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateMassMomentCylinder2D_XZ(real_ & mass, real3 & j1, real3 & j2, real_ r1,
		const real_ rhoRigid) {
	mass = PI * pow(r1, 2) * (paramsH.cMax.y - paramsH.cMin.y) * rhoRigid; //for cylinder
	j1 = R3(1.0 / 12.0 * mass * (
			3 * pow(r1, 2) + pow(paramsH.cMax.y - paramsH.cMin.y, 2)
			), 0, 0);
	j2 = R3(.5 * mass * pow(r1, 2), 0,
					1.0 / 12.0 * mass * (
							3 * pow(r1, 2)	+ pow(paramsH.cMax.y - paramsH.cMin.y, 2))
							);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateMassMomentCylinder3D_AlongZ(real_ & mass, real3 & j1, real3 & j2, real_ r1, real_ h, const real_ rhoRigid) {
	mass = PI * pow(r1, 2) * h * rhoRigid; //for cylinder
	j1 = R3(1.0 / 12.0 * mass * (
				3 * pow(r1, 2) + pow(h, 2)
				), 0, 0);
	j2 = R3(1.0 / 12.0 * mass * (
			3 * pow(r1, 2) + pow(h, 2)
			), 0, 0.5 * mass * pow(r1, 2));
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CalcInvJ(const real3 j1, const real3 j2, real3 & invJ1, real3 & invJ2) {
	//normalize J to deal with machine precision
	real3 maxJ3 = rmaxr(j1, j2);
	real_ maxComp = max(maxJ3.x, maxJ3.y);
	maxComp = max(maxComp, maxJ3.z);
	//********************************************
	real3 nJ1 = j1 / maxComp; //nJ1 is normalJ1
	real3 nJ2 = j2 / maxComp; //nJ2 is normalJ2

	real_ detJ = 2 * nJ1.z * nJ1.y * nJ2.y - nJ1.z * nJ1.z * nJ2.x
			- nJ1.y * nJ1.y * nJ2.z + nJ1.x * nJ2.x * nJ2.z
			- nJ1.x * nJ2.y * nJ2.y;
	invJ1 = R3(nJ2.x * nJ2.z - nJ2.y * nJ2.y, -nJ1.y * nJ2.z + nJ1.z * nJ2.y,
			nJ1.y * nJ2.y - nJ1.z * nJ2.x);
	invJ2 = R3(-nJ1.z * nJ1.z + nJ1.x * nJ2.z, -nJ1.x * nJ2.y + nJ1.z * nJ1.y,
			-nJ1.y * nJ1.y + nJ1.x * nJ2.x);

	//printf("invJ %f %f %f %f %f %f\n", aa.x, aa.y, aa.z, bb.x, bb.y, bb.z);
	//printf("invJ %f %f %f %f %f %f\n", 1e12 * j1.x, 1e12 *  j1.y, 1e12 *  j1.z,  1e12 * j2.x,  1e12 * j2.y, 1e12 *  j2.z);
	//printf("detJ %e\n", detJ * maxComp);
	// j = maxComp * nJ, therefore, jInv = maxComp * nJ_Inv
	invJ1 = invJ1 / detJ / maxComp;
	invJ2 = invJ2 / detJ / maxComp;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void AddPosVelMass(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		const real3 & pos,
		const real4 & q,
		const real_ & mass,
		const real3 & j1,
		const real3 & j2)
{
	rigidPos.push_back(pos);
	mQuatRot.push_back(q);
//	static int i = 0;
//	real4 velMas = R4(0.004 * pow(-1,i), 0, 0, real_(mass));
	//	real4 velMas = R4(0.004, 0, 0, real_(mass));
	real3 v3 = R3(0);
	//********************************************
	if (initParabolic) {
		real_ vx = CalcChannelParabolicVel(pos);
		v3 = R3(vx, 0, 0);
	}
	//********************************************
	real4 velMas = R4(v3, real_(mass));
	velMassRigidH.push_back(velMas);
	rigidBodyOmega.push_back(R3(0, 0, 0));
	rigidBody_J1.push_back(j1);
	rigidBody_J2.push_back(j2);

	real3 invJ1, invJ2;
	CalcInvJ(j1, j2, invJ1, invJ2);
	rigidBody_InvJ1.push_back(invJ1);
	rigidBody_InvJ2.push_back(invJ2);
//	i++;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void Add_Ellipsoid_To_Data(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const real3 & pos,
		const real4 & q,
		const real3 referenceR,
		const real_ rhoRigid)
{
	real_ mass;
	real3 j1, j2;
	CreateMassMomentEllipsoid(mass, j1, j2, referenceR.x,
			referenceR.y, referenceR.z, rhoRigid); //create Ellipsoid
	ellipsoidRadii.push_back(referenceR);

	AddPosVelMass(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
			rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2,
			pos, q, mass, j1, j2);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void Add_2DCylinder_To_Data(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const real3 & pos,
		const real4 & q,
		const real3 referenceR,
		const real_ rhoRigid)
{
	real_ mass;
	real3 j1, j2;
	CreateMassMomentCylinder2D_XZ(mass, j1, j2, referenceR.x, rhoRigid);			//create Cylinder
	ellipsoidRadii.push_back(referenceR);

	AddPosVelMass(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
			rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2,
			pos, q, mass, j1, j2);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void Add_3DCylinder_To_Data(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		const real3 & pos,
		const real4 & q,
		const CylinderGeometry & cylGeom,
		const real_ rhoRigid)
{
	real_ mass;
	real3 j1, j2;
	CreateMassMomentCylinder3D_AlongZ(mass, j1, j2, cylGeom.r, cylGeom.h, rhoRigid);			//create Cylinder

	AddPosVelMass(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
			rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2,
			pos, q, mass, j1, j2);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesRandom(thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii, const real3 referenceR,
		const real_ rhoRigid, int numSpheres) {

	srand(time(NULL));
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

	real_ xSpace = (paramsH.cMax.x - paramsH.cMin.x) / (numSpheres + 1);
	for (int i = 0; i < numSpheres; i++) {
//		int index = (int)(randLinVec.size() - 1) * real_ (rand()) / RAND_MAX;
//		real_ r = (4.5 * paramsH.sizeScale) * randLinVec[index];

		real_ r = (channelRadius - maxR - 2 * paramsH.HSML)
				* real_(rand()) / RAND_MAX;
//						real_ r = (channelRadius - maxR - 2 * paramsH.HSML) * (.2);

		printf("sizeRandomLinear %d\n", randLinVec.size()); //4.5 comes from channelRadius
		real_ teta = 2 * PI * real_(rand()) / RAND_MAX;
//						real_ teta = 2 * PI * real_ (.2);
		real3 pos = R3(paramsH.cMin.x, channelCenterYZ.x, channelCenterYZ.y)
				+ R3((i + 1.0) * xSpace, real_(r * cos(teta)),
						real_(r * sin(teta)));

		real4 dumQuat = R4(1 - 2.0 * real_(rand()) / RAND_MAX,
				1 - 2.0 * real_(rand()) / RAND_MAX,
				1 - 2.0 * real_(rand()) / RAND_MAX,
				1 - 2.0 * real_(rand()) / RAND_MAX); //generate random quaternion
//						real4 dumQuat = R4(1,0,0,0);
		dumQuat *= 1.0 / length(dumQuat);

		Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
				rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
				pos, dumQuat, referenceR, rhoRigid);
	}
	randLinVec.clear();
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// note: spheres radius should be similar
void CreateRigidBodiesFromFile(thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const string fileNameRigids, const real_ rhoRigid,
		SimParams & paramsH) {

	fstream ifileSpheres(fileNameRigids.c_str(), ios::in);

	real_ x, y, z;
	char ch;

	real_ dumRRigidBody1, dumRRigidBody2, dumRRigidBody3;
	ifileSpheres >> x >> ch >> y >> ch >> z >> ch >> dumRRigidBody1 >> ch
			>> dumRRigidBody2 >> ch >> dumRRigidBody3;
	int counterRigid = 0;
	while (!ifileSpheres.eof()) {
		//real_ r = rRigidBody * (.75 + .75 * real_(rand())/RAND_MAX);

		real3 pos = R3(x, y, z);
		real4 q = R4(1, 0, 0, 0);
		real3 referenceR = 	R3(dumRRigidBody1, dumRRigidBody2, dumRRigidBody3);

		Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
						rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
						pos, q, referenceR, rhoRigid);
//			Add_2DCylinder_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
//							rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
//							pos, q, referenceR, rhoRigid);

		ifileSpheres >> x >> ch >> y >> ch >> z >> ch >> dumRRigidBody1 >> ch
				>> dumRRigidBody2 >> ch >> dumRRigidBody3;
		counterRigid++;
		paramsH.rigidRadius = R3(dumRRigidBody1, dumRRigidBody2, dumRRigidBody3);
	}
	ifileSpheres.close();
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// note: paramsH.rigidRadius is not initialized here
void CreateRigidBodiesFromFileSerpentinePeriodic(thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii,
		const string fileNameRigids, const real_ rhoRigid) {

	fstream ifileSpheres(fileNameRigids.c_str(), ios::in);

	real_ x, y, z;
	char ch;

	real_ dumRRigidBody1, dumRRigidBody2, dumRRigidBody3;
	ifileSpheres >> x >> ch >> y >> ch >> z >> ch >> dumRRigidBody1 >> ch
			>> dumRRigidBody2 >> ch >> dumRRigidBody3;
	int counterRigid = 0;
	while (!ifileSpheres.eof()) {
		//real_ r = rRigidBody * (.75 + .75 * real_(rand())/RAND_MAX);
		for (int period = 0; period < paramsH.nPeriod; period++) {

			real3 pos = R3(x + period * sPeriod, y , z);
			real4 q = R4(1, 0, 0, 0);
			real3 referenceR = 	R3(dumRRigidBody1, dumRRigidBody2, dumRRigidBody3);

			Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
							rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
							pos, q, referenceR, rhoRigid);

		}
		ifileSpheres >> x >> ch >> y >> ch >> z >> ch >> dumRRigidBody1 >> ch
				>> dumRRigidBody2 >> ch >> dumRRigidBody3;
		counterRigid++;
	}
	ifileSpheres.close();
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateOne3DRigidCylinder(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<CylinderGeometry> & cylinderGeom,
		const string fileNameRigids, const real_ rhoRigid,
		const real2 cylinderR_H) {

	CylinderGeometry cylGeom;
	cylGeom.r = cylinderR_H.x;
	cylGeom.h = cylinderR_H.y;
	real3 pa3 = R3(.2, .4, .4);
	real3 pb3 = pa3 + R3(0, cylGeom.h * cos(PI/6), cylGeom.h * sin(PI/6));
	cylGeom.pa3 = pa3;
	cylGeom.pb3 = pb3;
	cylGeom.center = 0.5 * (cylGeom.pa3 + cylGeom.pb3);
	cylinderGeom.push_back(cylGeom);

	real3 pos = 0.5 * (pa3 + pb3);
	real3 slope3 = normalize(pb3 - pa3);
	real4 q;
	QuaternionFromAxisVector(q, slope3);
	//real4 q = R4(1,0,0,0);
	Add_3DCylinder_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
								rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2,
								pos, q, cylGeom, rhoRigid);
}
////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPattern(thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii, const real3 referenceR,
		const real_ rhoRigid, int3 stride) {

	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	//printf("paramsH.cMin %f %f %f, paramsH.cMax %f %f %f\n", paramsH.straightChannelBoundaryMin.x, paramsH.straightChannelBoundaryMin.y, paramsH.straightChannelBoundaryMin.z, paramsH.straightChannelBoundaryMax.x, paramsH.straightChannelBoundaryMax.y, paramsH.straightChannelBoundaryMax.z);
	real3 spaceRigids = 2 * (referenceR + 0.6 * R3(paramsH.HSML));
	real3 n3Rigids = (paramsH.straightChannelBoundaryMax - paramsH.straightChannelBoundaryMin)
			/ spaceRigids;
	for (int i = 1; i < n3Rigids.x - 1; i += stride.x) {
		for (int j = 1; j < n3Rigids.y - 1; j += stride.y) {
			for (int k = 1; k < n3Rigids.z - 1; k += stride.z) {
				real3 pos = paramsH.straightChannelBoundaryMin
						+ R3(i, j, k) * spaceRigids;
				//printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				real4 q = R4(1, 0, 0, 0);
				Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
						rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
						pos, q, referenceR, rhoRigid);
			}
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPatternWithinBeams(thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii, const real3 referenceR,
		const real_ rhoRigid, int3 stride,
		const ANCF_Params & flexParams) {

	//*********** this should be the same as the flex
	real_ margin = 4 * paramsH.HSML;// 2 * paramsH.HSML;
	real_ spaceFlex = 2 * (flexParams.r + margin);
	//************************************************

	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	//printf("paramsH.cMin %f %f %f, paramsH.cMax %f %f %f\n", paramsH.straightChannelBoundaryMin.x, paramsH.straightChannelBoundaryMin.y, paramsH.straightChannelBoundaryMin.z, paramsH.straightChannelBoundaryMax.x, paramsH.straightChannelBoundaryMax.y, paramsH.straightChannelBoundaryMax.z);
	real3 spaceRigids = 2 * (referenceR + .6 * R3(paramsH.HSML));
	real3 n3Rigids = (paramsH.straightChannelBoundaryMax - paramsH.straightChannelBoundaryMin)
			/ spaceRigids;
	for (int i = 1; i < n3Rigids.x - 1; i += stride.x) {
		for (int j = 1; j < n3Rigids.y - 1; j += stride.y) {
			for (int k = 1; k < n3Rigids.z - 1; k += stride.z) {
				real3 pos = paramsH.straightChannelBoundaryMin
						+ R3(i, j, k) * spaceRigids;
				real2 distFromBeam = R2(fmod(pos.x, spaceFlex), fmod(pos.y, spaceFlex));
				distFromBeam.x = min(distFromBeam.x, spaceFlex - distFromBeam.x);
				distFromBeam.y = min(distFromBeam.y, spaceFlex - distFromBeam.y);

//				if (distFromBeam.x < referenceR.x + flexParams.r + 1.2 * paramsH.HSML) continue;
//				if (spaceFlex - distFromBeam.x < referenceR.x + flexParams.r + 1.2 * paramsH.HSML) continue;
//				if (distFromBeam.y < referenceR.y + flexParams.r + 1.2 * paramsH.HSML) continue;
//				if (spaceFlex - distFromBeam.y < referenceR.y + flexParams.r + 1.2 * paramsH.HSML) continue;
				if (length(distFromBeam) < referenceR.x + flexParams.r + 1.2 * paramsH.HSML) continue;

				//printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				real4 q = R4(1, 0, 0, 0);
				Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
						rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
						pos, q, referenceR, rhoRigid);
			}
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPatternPipe(thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii, const real3 referenceR,
		const real_ rhoRigid) {

	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	real3 spaceRigids = 2 * (referenceR + 1.1 * R3(paramsH.HSML));
	real3 n3Rigids = (paramsH.cMax - paramsH.cMin) / spaceRigids;
	for (int i = 1; i < n3Rigids.x - 1; i++) {
		for (real_ r = spaceRigids.x; r < channelRadius - spaceRigids.x; r +=
				spaceRigids.x) {
			real_ dTeta = spaceRigids.x / r;
			for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				real3 pos = R3(paramsH.cMin.x, channelCenterYZ.x,
						channelCenterYZ.y)
						+ R3(i * spaceRigids.x, real_(r * cos(teta)),
								real_(r * sin(teta)));

				//printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				real4 q = R4(1, 0, 0, 0);
				Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
						rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
						pos, q, referenceR, rhoRigid);
			}
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPatternStepPipe(thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii, const real3 referenceR,
		const real_ rhoRigid) {

	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	real3 spaceRigids = 2 * (referenceR + 2.0 * R3(paramsH.HSML));

	real_ tubeLength = paramsH.cMax.x - paramsH.cMin.x;
	real_ r1 = channelRadius;
	real_ r2 = 0.5 * channelRadius;
	real_ d1 = paramsH.cMin.x + .25 * tubeLength;
	real_ d2 = paramsH.cMin.x + .75 * tubeLength;

	real_ space = max(max(spaceRigids.x, spaceRigids.y), spaceRigids.z);
	real_ nX = (d2 - d1) / space;
	for (int i = 1; i < nX - 1; i++) {
		for (real_ r = space; r < r1 - space; r += space) {
			real_ dTeta = space / r;
			for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				real3 pos = R3(d1, channelCenterYZ.x, channelCenterYZ.y)
						+ R3(i * space, real_(r * cos(teta)),
								real_(r * sin(teta)));

				//printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				real4 q = R4(1, 0, 0, 0);
				Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
						rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
						pos, q, referenceR, rhoRigid);
			}
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesPatternPipe_KindaRandom(
		thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii, const real3 referenceR,
		const real_ rhoRigid, int numRigidParticles) {

	srand(time(NULL));
	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
//	real3 spaceRigids = 2 * (referenceR + 2 * R3(paramsH.HSML));
	real3 spaceRigids = 2 * (referenceR + 1.1 * R3(paramsH.HSML));
	real3 n3Rigids = (paramsH.cMax - paramsH.cMin) / spaceRigids;
	int totalNumberPossibleParticles = 0;
	for (int i = 1; i < n3Rigids.x; i++) {
		real_ shiftR = real_(i % 4) / 4 * 2 * referenceR.x;
		for (real_ r = 0.5 * spaceRigids.x + shiftR;
				r < channelRadius - 0.5 * spaceRigids.x; r += spaceRigids.x) {
			if (i == 1)
				printf("r %f\n", r);
			real_ dTeta = spaceRigids.x / r;
			for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				totalNumberPossibleParticles++;
			}
		}
	}
	int skipCount = totalNumberPossibleParticles / numRigidParticles;

	printf("totalNumberPossibleParticles %d  skipCount %d\n",
			totalNumberPossibleParticles, skipCount);
	int particleCounter = 0;
	int loopCounter = -1;

	for (int i = 1; i < n3Rigids.x; i++) {
		for (real_ r = 0.5 * spaceRigids.x;
				r < channelRadius - 0.5 * spaceRigids.x; r += spaceRigids.x) {
			real_ dTeta = spaceRigids.x / r;
			for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				loopCounter++;
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
				if (!meetSeed)
					continue;
//				 printf("ha\n");
				if (particleCounter >= numRigidParticles)
					continue;
				particleCounter++;
				real3 pos = R3(paramsH.cMin.x, channelCenterYZ.x,
						channelCenterYZ.y)
						+ R3(i * spaceRigids.x, real_(r * cos(teta)),
								real_(r * sin(teta)));

				//printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				real4 q = R4(1, 0, 0, 0);
				Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
						rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
						pos, q, referenceR, rhoRigid);
			}
		}
	}
}
////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesChannelRandom(thrust::host_vector<real3> & rigidPos,
		thrust::host_vector<real4> & mQuatRot,
		thrust::host_vector<real4> & velMassRigidH,
		thrust::host_vector<real3> & rigidBodyOmega,
		thrust::host_vector<real3> & rigidBody_J1,
		thrust::host_vector<real3> & rigidBody_J2,
		thrust::host_vector<real3> & rigidBody_InvJ1,
		thrust::host_vector<real3> & rigidBody_InvJ2,
		thrust::host_vector<real3> & ellipsoidRadii, const real3 referenceR,
		const real_ rhoRigid, int numRigids) {

	for (int i = 0; i < numRigids; i++) {
		real_ maxR = Max(referenceR.x, referenceR.y);
		maxR = Max(maxR, referenceR.z);
		real3 channelMin = paramsH.straightChannelBoundaryMin + R3(maxR);
		real3 channelBox = paramsH.straightChannelBoundaryMax - paramsH.straightChannelBoundaryMin - 2 * R3(maxR);
		real3 pos = R3(myRand(), myRand(), myRand()) *  channelBox + channelMin;
		real4 q = R4(2 * myRand() - 1, 2 * myRand() - 1, 2 * myRand() - 1, 2 * myRand() - 1);
		q = normalize(q);
		Add_Ellipsoid_To_Data(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
				rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii,
				pos, q, referenceR, rhoRigid);
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateOneFlexBody(thrust::host_vector<real3> & ANCF_Nodes,
		thrust::host_vector<real3> & ANCF_Slopes,
		thrust::host_vector<real3> & ANCF_NodesVel,
		thrust::host_vector<real3> & ANCF_SlopesVel,
		thrust::host_vector<real_> & ANCF_Beam_Length,
		thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		thrust::host_vector<bool> & ANCF_IsCantilever,
		real_ pipeRadius, real_ pipeLength, real3 pipeInPoint3,
		const real_ beamLength,
		const ANCF_Params & flexParams
		) {
	//TODO create mass property of the beams

	int numElementsPerBeam = flexParams.ne;
	real_ myMargin = paramsH.MULT_INITSPACE * (paramsH.NUM_BCE_LAYERS + 1)
			* paramsH.HSML;
//
//	//hanging beam
//	real3 pa3 = R3(.1, .25, .8);
//	real3 pb3 = pa3 + R3(0,0,beamLength);

	//beam vs rigid
//	real3 pa3 = R3(.2, .4, .4);
//	real3 pb3 = pa3 + R3(0, beamLength * cos(PI/6), beamLength * sin(PI/6));

//	//vibration
//	real3 pa3 = R3(.1, .25, .8);
//	real3 pb3 = pa3 + R3(0,0,beamLength);
//	real3 pa3 = R3(.1, .5, .5);
//	real3 pb3 = pa3 + R3(beamLength,0,0);

	//validation
	real3 pa3 = R3(.1, .2, .4);
	real3 pb3 = pa3 + R3(beamLength * cos(PI/6),0,beamLength * sin(PI/6));

	if (pb3.x > paramsH.cMax.x) {
		perror("beam end out of bound\n");
		exit(0);

	}

	real3 slope3 = pb3 - pa3;
	if (fabs(length(slope3) - beamLength) > 1e-8 ) {
		perror("Error! something wrong: beam lenght\n");
		return;
		exit(0);
	}
	ANCF_Beam_Length.push_back(beamLength);
	slope3 /= beamLength;
	for (int m = 0; m < numElementsPerBeam + 1; m++) {
		ANCF_Nodes.push_back(
				pa3 + m * (beamLength / numElementsPerBeam) * slope3);
		ANCF_Slopes.push_back(slope3);
		ANCF_NodesVel.push_back(R3(0));
		ANCF_SlopesVel.push_back(R3(0));
	}
	ANCF_ReferenceArrayNodesOnBeams.push_back(I2(0, numElementsPerBeam + 1));
	ANCF_IsCantilever.push_back(false);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateSomeFlexBodies(thrust::host_vector<real3> & ANCF_Nodes,
		thrust::host_vector<real3> & ANCF_Slopes,
		thrust::host_vector<real3> & ANCF_NodesVel,
		thrust::host_vector<real3> & ANCF_SlopesVel,
		thrust::host_vector<real_> & ANCF_Beam_Length,
		thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		thrust::host_vector<bool> & ANCF_IsCantilever,
		real_ pipeRadius, real_ pipeLength, real3 pipeInPoint3,
		const ANCF_Params & flexParams
		) {
	//TODO create mass property of the beams

	int numBeams = 2;
	int numElementsPerBeam = flexParams.ne;
	real_ myMargin = paramsH.MULT_INITSPACE * (paramsH.NUM_BCE_LAYERS + 1)
			* paramsH.HSML;
	real_ sectionLenght = (pipeLength - 2 * myMargin) / numBeams;
	for (int i = 0; i < numBeams; i++) {
		real_ x = (i + myRand()) * sectionLenght + (pipeInPoint3.x + myMargin);
		real_ r = myRand() * (pipeRadius - myMargin);
		real_ theta = myRand() * 2 * PI;
		real3 pa3 = R3(x, r * cos(theta) + pipeInPoint3.y,
				r * sin(theta) + pipeInPoint3.z);

		x = (i + myRand()) * sectionLenght + (pipeInPoint3.x + myMargin);
		r = myRand() * (pipeRadius - myMargin);
		theta = myRand() * 2 * PI;
		real3 pb3 = R3(x, r * cos(theta) + pipeInPoint3.y,
				r * sin(theta) + pipeInPoint3.z);

		real3 slope3 = pb3 - pa3;
		real_ beamLength = length(slope3);
		ANCF_Beam_Length.push_back(beamLength);
		slope3 /= beamLength;
		for (int m = 0; m < numElementsPerBeam + 1; m++) {
			ANCF_Nodes.push_back(
					pa3 + m * (beamLength / numElementsPerBeam) * slope3);
			ANCF_Slopes.push_back(slope3);
			ANCF_NodesVel.push_back(R3(0));
			ANCF_SlopesVel.push_back(R3(0));
		}
		if (i == 0) {
			ANCF_ReferenceArrayNodesOnBeams.push_back(
					I2(0, numElementsPerBeam + 1));
		} else {
			int nodesSofar = ANCF_ReferenceArrayNodesOnBeams[i - 1].y;
			ANCF_ReferenceArrayNodesOnBeams.push_back(
					I2(nodesSofar, nodesSofar + numElementsPerBeam + 1));
		}
		ANCF_IsCantilever.push_back(false);

		// mass and stiffness properties of beams
//		real3 invJ1, invJ2;
//		CalcInvJ(j1, j2, invJ1, invJ2);
//		rigidBody_InvJ1.push_back(invJ1);
//		rigidBody_InvJ2.push_back(invJ2);

	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateManyFlexBodiesPipe(thrust::host_vector<real3> & ANCF_Nodes,
		thrust::host_vector<real3> & ANCF_Slopes,
		thrust::host_vector<real3> & ANCF_NodesVel,
		thrust::host_vector<real3> & ANCF_SlopesVel,
		thrust::host_vector<real_> & ANCF_Beam_Length,
		thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		thrust::host_vector<bool> & ANCF_IsCantilever,
		real_ pipeRadius, real_ pipeLength, real3 pipeInPoint3,
		const real_ referenceR,
		const ANCF_Params & flexParams
		) {
	//TODO create mass property of the beams
	int numElementsPerBeam = flexParams.ne;
	real_ spaceRigids = 2 * (referenceR + 1.1 * paramsH.HSML);
	int nX = int( (paramsH.cMax.x - paramsH.cMin.x) / spaceRigids );
	int beamCounter = 0;
	for (int ix = 1; ix < nX - 1; ix++) {
		for (real_ r = spaceRigids; r < pipeRadius - spaceRigids; r += spaceRigids) {
			real_ dTeta = spaceRigids / r;
			for (real_ teta = 0; teta < 2 * PI - dTeta; teta += dTeta) {
				real3 pos = R3(paramsH.cMin.x, channelCenterYZ.x,
						channelCenterYZ.y)
						+ R3(ix * spaceRigids, real_(r * cos(teta)),
								real_(r * sin(teta)));


				real_ halfLBeam = referenceR;
				real_ thetaBeam = myRand() * PI;
				real_ phiBeam = myRand() * 2 * PI;
				real3 pa3 = pos + halfLBeam * R3(cos(thetaBeam) * cos(phiBeam), cos(thetaBeam) * sin(phiBeam), sin(thetaBeam));
				real3 pb3 = pos - halfLBeam * R3(cos(thetaBeam) * cos(phiBeam), cos(thetaBeam) * sin(phiBeam), sin(thetaBeam));

				real3 slope3 = pb3 - pa3;
				real_ beamLength = length(slope3);
				ANCF_Beam_Length.push_back(beamLength);
				slope3 /= beamLength;
				for (int m = 0; m < numElementsPerBeam + 1; m++) {
					ANCF_Nodes.push_back(
							pa3 + m * (beamLength / numElementsPerBeam) * slope3);
					ANCF_Slopes.push_back(slope3);
					ANCF_NodesVel.push_back(R3(0));
					ANCF_SlopesVel.push_back(R3(0));
				}
				if (beamCounter == 0) {
					ANCF_ReferenceArrayNodesOnBeams.push_back(
							I2(0, numElementsPerBeam + 1));
				} else {
					int nodesSofar = ANCF_ReferenceArrayNodesOnBeams[beamCounter - 1].y;
					ANCF_ReferenceArrayNodesOnBeams.push_back(
							I2(nodesSofar, nodesSofar + numElementsPerBeam + 1));
				}
				ANCF_IsCantilever.push_back(false);

				beamCounter ++;
			}
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateManyFlexBodiesChannelGoodWithRigids(thrust::host_vector<real3> & ANCF_Nodes,
		thrust::host_vector<real3> & ANCF_Slopes,
		thrust::host_vector<real3> & ANCF_NodesVel,
		thrust::host_vector<real3> & ANCF_SlopesVel,
		thrust::host_vector<real_> & ANCF_Beam_Length,
		thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		thrust::host_vector<bool> & ANCF_IsCantilever,
		const ANCF_Params & flexParams)
{
	//TODO create mass property of the beams
	//*********** this should be the same as the flex
	real_ margin = 1.2 * paramsH.HSML;// 2 * paramsH.HSML;
	real_ spaceFlex = 2 * (flexParams.r + margin);
	//************************************************
	int numElementsPerBeam = flexParams.ne;
	real_ beamLength = (paramsH.straightChannelBoundaryMax.z - paramsH.straightChannelBoundaryMin.z) - 6 * margin;//paramsH.HSML * 40;
	printf("beamLength %f\n", beamLength);
	real_ beamLengthWithSpacing = beamLength + margin;

	real3 n3Flex = R3( (paramsH.straightChannelBoundaryMax.x - paramsH.straightChannelBoundaryMin.x) / spaceFlex, (paramsH.straightChannelBoundaryMax.y - paramsH.straightChannelBoundaryMin.y) / spaceFlex,
			(paramsH.straightChannelBoundaryMax.z - paramsH.straightChannelBoundaryMin.z) / beamLengthWithSpacing );
	printf("\nn3Flex %f %f %f\n", n3Flex.x, n3Flex.y, n3Flex.z);
	int beamCounter = 0;
	for (int i = 1; i < n3Flex.x - 1; i += 1) {
		for (int j = 1; j < n3Flex.y - 1; j += 1) {
			for (int k = 0; k < n3Flex.z - 1; k += 1) {
				real3 pa3 = paramsH.straightChannelBoundaryMin
						+ R3(i * spaceFlex, j * spaceFlex, k * beamLengthWithSpacing + margin);
				real3 pb3 = pa3 + R3(0, 0, beamLength);

				real3 slope3 = pb3 - pa3;
				ANCF_Beam_Length.push_back(beamLength);
				slope3 /= beamLength;
				for (int m = 0; m < numElementsPerBeam + 1; m++) {
					ANCF_Nodes.push_back(
							pa3 + m * (beamLength / numElementsPerBeam) * slope3);
					ANCF_Slopes.push_back(slope3);
					ANCF_NodesVel.push_back(R3(0));
					ANCF_SlopesVel.push_back(R3(0));
				}
				if (beamCounter == 0) {
					ANCF_ReferenceArrayNodesOnBeams.push_back(
							I2(0, numElementsPerBeam + 1));
				} else {
					int nodesSofar = ANCF_ReferenceArrayNodesOnBeams[beamCounter - 1].y;
					ANCF_ReferenceArrayNodesOnBeams.push_back(
							I2(nodesSofar, nodesSofar + numElementsPerBeam + 1));
				}
				ANCF_IsCantilever.push_back(true);

				beamCounter ++;

			}
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideEllipsoid(real3 sphParPos, real3 rigidPos, Rotation rot,
		real3 radii, real_ clearance) {
	real3 dist3GF = sphParPos - rigidPos;
	real3 dist3 = dist3GF.x * R3(rot.a00, rot.a01, rot.a02)
			+ dist3GF.y * R3(rot.a10, rot.a11, rot.a12)
			+ dist3GF.z * R3(rot.a20, rot.a21, rot.a22);
	real3 mappedDist = dist3 / (radii + R3(clearance));
	if (length(mappedDist) < 1) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideCylinder_XZ(real3 sphParPos, real3 rigidPos, real3 radii,
		real_ clearance) {
	real3 dist3 = sphParPos - rigidPos;
	dist3.y = 0;
	//if (length(dist3) < spherePosRad.w + 2 * (paramsH.HSML)) {
	if (length(dist3) < radii.x + clearance) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideCylinder_3D(real3 sphParPos, real3 pa3, real3 pb3, real_ rad,
		real_ clearance) {
	real3 n3 = pb3 - pa3;
	real3 n3Normal = n3 / length(n3);
	real3 pa3Cleared = pa3 - clearance * n3Normal;
	real3 pb3Cleared = pb3 + clearance * n3Normal;
	real3 r = sphParPos - pa3Cleared;
	real_ s = dot(r, n3Normal);
	if (s < 0 || s > length(pb3Cleared - pa3Cleared)) {
		return false;
	}
	real3 s3 = s * n3Normal;
	if (length(r - s3) < rad + clearance) {
		return true;
	} else {
		return false;
	}
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideStraightFlex(real3 sphParPos, real3 pa3, real3 pb3, real_ rad,
		real_ clearance) {
	return IsInsideCylinder_3D(sphParPos, pa3, pb3, rad, clearance);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideStraightFlexWithBob(real3 sphParPos, real3 pa3, real3 pb3, real_ rad, real_ bobRad,
		real_ clearance) {
	if (IsInsideStraightFlex(sphParPos, pa3, pb3, rad, clearance)) {return true;}
	Rotation rot = UnitMatrix();
	if (IsInsideEllipsoid(sphParPos, pb3, rot, R3(bobRad), clearance)) {return true;}
	return false;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInEllipse(real2 pos, real2 radii) {
//	printf(" pos %f %f  r2 %f %f\n", pos.x, pos.y, radii.x, radii.y);
//	real2 kk  = pos / radii;
//	printf("kk.x kk.y   %f %f \n", kk.x, kk.y);
//	printf("lengthkk %f and the other length %f \n", length(kk), length(pos / radii));
	return length(pos / radii);
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
//	const real_ sphR = paramsH.HSML;
//	real_ x, y;
//	real_ distFromWall = 0;//2 * sphR;//0;//2 * sphR;
//	real_ penDist = 0;
//	real_ largePenet = -5*sphR;//like a large number. Should be negative (assume large initial penetration)
//	real_ penDist2 = 0;
//	bool isOut = false;
//
//	if (posRad.y < -paramsH.toleranceZone || posRad.y > 1.0 * mm + paramsH.toleranceZone) {
//		return largePenet;
//	}
//	else if (posRad.y < 0) {
//		penDist2 = posRad.y;
//		isOut = true;
//	}
//	else if ( posRad.y > 1.0 * mm) {
//		penDist2 = (1.0 * mm - posRad.y);
//		isOut = true;
//	}
//	//serpentine
//	real_ r1 = 1.3 * mm, r2 = 1.0 * mm, r3=2.0 * mm, r4 = 0.3 * mm;
//	x = fmod(posRad.x, sPeriod); //posRad.x - int(posRad.x / sPeriod) * sPeriod; //fmod
//	y = posRad.z;
//	if (x >= 0 && x < 1.3 * mm) {
//		if (y < -3 * paramsH.toleranceZone) return largePenet;
//		if (y < 0) return (x - 1.3 * mm);
//		penDist = IsOutBoundaryCircle(R2(x, y), R2(0, 0), r1); if (penDist < 0) return penDist;
//		penDist = IsInBoundaryCircle(R2(x, y), R2(0, 1.0 * mm), r3); if (penDist < 0) return penDist;
//	} else if (x >= 1.3 * mm && x < 2.0 * mm) {
//		if (y > 1.0 * mm) { penDist = IsInBoundaryCircle(R2(x, y), R2(0, 1.0 * mm), r3); if (penDist < 0) return penDist; }
//		else if (y < 0) { penDist = IsInBoundaryCircle(R2(x, y), R2(2.3 * mm, 0), r2); if (penDist < 0) return penDist; }
//	} else if (x >= 2.0 * mm && x < 2.6 * mm) {
//		if (y < .55 * mm) {
//			penDist = IsInBoundaryCircle(R2(x, y), R2(2.3 * mm, 0), r2); if (penDist < 0) return penDist;
//			penDist = IsOutBoundaryCircle(R2(x, y), R2(2.3 * mm, .55 * mm), r4); if (penDist < 0) return penDist; }
//		else if (y < 2 * mm) { penDist = IsOutBoundaryCircle(R2(x, y), R2(2.3 * mm, y), r4); if (penDist < 0) return penDist; }
//		else return largePenet;
//	} else if (x >= 2.6 * mm && x < 3.3 * mm) {
//		if (y > 1.0 * mm) { penDist = IsInBoundaryCircle(R2(x, y), R2(4.6 * mm, 1.0 * mm), r3); if (penDist < 0) return penDist; }
//		else if (y < 0) { penDist = IsInBoundaryCircle(R2(x, y), R2(2.3 * mm, 0), r2); if (penDist < 0) return penDist; }
//	} else if (x >= 3.3 * mm && x < 4.6 * mm) {
//		if (y < -3 * paramsH.toleranceZone) return largePenet;
//		if (y < 0) return 3.3 * mm - x;
//		penDist = IsOutBoundaryCircle(R2(x, y), R2(4.6 * mm, 0), r1); if (penDist < 0) return penDist;
//		penDist = IsInBoundaryCircle(R2(x, y), R2(4.6 * mm, 1.0 * mm), r3); if (penDist < 0) return penDist;
//	}
//	if (!isOut)
//		return -largePenet;
//	return penDist2;
//}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideCurveOfSerpentineBeta(real3 posRad) {
	real_ x, y;
	real_ distFromWall = 0; //2 * sphR;//0;//2 * sphR;
	real_ penDist = 0;
	real_ largePenet = -paramsH.MULT_INITSPACE * paramsH.HSML * paramsH.NUM_BCE_LAYERS * 2; //like a large number. Should be negative (assume large initial penetration)
	real_ penDist2 = 0;
	bool isOut = false;

	if (posRad.y < -paramsH.toleranceZone
			|| posRad.y > 1.0 * mm + paramsH.toleranceZone) {
		return largePenet;
	} else if (posRad.y < 0) {
		penDist2 = posRad.y;
		isOut = true;
	} else if (posRad.y > 1.0 * mm) {
		penDist2 = (1.0 * mm - posRad.y);
		isOut = true;
	}
	//serpentine
	//real_ r1 = 1.3 * mm, r2 = 1.0 * mm, r3=2.0 * mm, r4 = 0.3 * mm;

	x = fmod(posRad.x, sPeriod); //posRad.x - int(posRad.x / sPeriod) * sPeriod; //fmod
	y = posRad.z;
	if (y > 0) {
		if (x >= 0 && x < r2_2.x) {
			penDist = IsOutBoundaryEllipsoid(R2(x, y), R2(0, 0), r2_2);
			if (penDist < 0)
				return penDist;
		}
		real_ penDist3 = 1, penDist4 = 1;
		if (x >= 0 && x < r3_2.x + paramsH.toleranceZone) {
			penDist3 = IsInBoundaryEllipsoid(R2(x, y), R2(0, 0), r3_2);
		}
		if (x > r3_2.x + 2 * r4_2.x - paramsH.toleranceZone
				&& x < 2 * r3_2.x + 2 * r4_2.x) {
			penDist4 = IsInBoundaryEllipsoid(R2(x, y),
					R2(2 * r3_2.x + 2 * r4_2.x, 0), r3_2);
		}
		if (penDist3 < 0 && penDist4 < 0) {
			return max(penDist3, penDist4);
		}
		if (penDist3 < 0) {
			return penDist3;
		}
		if (penDist4 < 0) {
			return penDist4;
		}
		if (x >= r3_2.x + paramsH.toleranceZone
				&& x < r3_2.x + 2 * r4_2.x - paramsH.toleranceZone) {
			return largePenet;
		}
//		if (x >= 0 && x < r3_2.x + paramsH.toleranceZone) {
//			penDist = IsInBoundaryEllipsoid(R2(x, y), R2(0, 0), r3_2);
//			if (penDist < 0)
//				return penDist;
//		}
//		if (x >= r3_2.x + paramsH.toleranceZone
//				&& x < r3_2.x + 2 * r4_2.x - paramsH.toleranceZone) {
//			return largePenet;
//		}
//		if (x > r3_2.x + 2 * r4_2.x - paramsH.toleranceZone
//				&& x < 2 * r3_2.x + 2 * r4_2.x) {
//			penDist = IsInBoundaryEllipsoid(R2(x, y),
//					R2(2 * r3_2.x + 2 * r4_2.x, 0), r3_2);
//			if (penDist < 0)
//				return penDist;
//		}
		if (x > r2_2.x + 2 * r1_2.x && x < 2 * r3_2.x + 2 * r4_2.x) {
			penDist = IsOutBoundaryEllipsoid(R2(x, y),
					R2(2 * r3_2.x + 2 * r4_2.x, 0), r2_2);
			if (penDist < 0)
				return penDist;
		}
	} else {
		if (x > r3_2.x && x < r3_2.x + 2 * r4_2.x) {
			penDist = IsOutBoundaryEllipsoid(R2(x, y), R2(r3_2.x + r4_2.x, 0),
					r4_2);
			if (penDist < 0)
				return penDist;
		}
		if (x > r2_2.x - paramsH.toleranceZone
				&& x < r2_2.x + 2 * r1_2.x + paramsH.toleranceZone) {
			penDist = IsInBoundaryEllipsoid(R2(x, y), R2(r2_2.x + r1_2.x, 0),
					r1_2);
			if (penDist < 0)
				return penDist;
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
	real_ x, y;
	real_ distFromWall = 0; //2 * sphR;//0;//2 * sphR;
	real_ penDist = 0;
	real_ largePenet = -paramsH.MULT_INITSPACE * paramsH.HSML * paramsH.NUM_BCE_LAYERS * 2; //like a large number. Should be negative (assume large initial penetration)
	real_ penDist2 = 0;
	bool isOut = false;

	if (posRad.y < -paramsH.toleranceZone
			|| posRad.y > 1.0 * mm + paramsH.toleranceZone) {
		return largePenet;
	} else if (posRad.y < 0) {
		penDist2 = posRad.y;
		isOut = true;
	} else if (posRad.y > 1.0 * mm) {
		penDist2 = (1.0 * mm - posRad.y);
		isOut = true;
	}
	//serpentine
	if (posRad.x < paramsH.nPeriod * sPeriod) {
		return IsInsideCurveOfSerpentineBeta(posRad);
	} else {
		//straight channel
		x = posRad.x - paramsH.nPeriod * sPeriod;
		y = posRad.z;

		if (y < 0) {
			if (x < r2_2.x - paramsH.toleranceZone) {
				return largePenet;
			}
			if (x >= r2_2.x - paramsH.toleranceZone
					&& x < r2_2.x + 2 * r1_2.x + paramsH.toleranceZone) {
				penDist = IsInBoundaryEllipsoid(R2(x, y),
						R2(r2_2.x + r1_2.x, 0), r1_2);
				if (penDist < 0)
					return penDist;
			}
			if (x >= r3_2.x && x < r3_2.x + 2 * r4_2.x) {
				penDist = IsOutBoundaryEllipsoid(R2(x, y),
						R2(r3_2.x + r4_2.x, 0), r4_2);
				if (penDist < 0)
					return penDist;
			}
			if (x >= r2_2.x + 2 * r1_2.x + paramsH.toleranceZone) {
				return largePenet;
			}
		} else {
			if (x >= 0 && x < r2_2.x) {
				penDist = IsOutBoundaryEllipsoid(R2(x, y), R2(0, 0), r2_2);
				if (penDist < 0)
					return penDist;
			}


			real_ penDist3=1, penDist4=1;
			if (x >= 0 && x < r3_2.x + paramsH.toleranceZone) {
				penDist3 = IsInBoundaryEllipsoid(R2(x, y), R2(0, 0), r3_2);
//						if (penDist1 < 0)
//							return penDist1;
			}
			if (x >= r3_2.x + 2 * r4_2.x - paramsH.toleranceZone
					&& x < r3_2.x + 2 * r4_2.x + r6_2.x) {
				penDist4 = IsInBoundaryEllipsoid(R2(x, y),
						R2(r3_2.x + 2 * r4_2.x + r6_2.x, 0), r6_2);
//								if (penDist2 < 0)
//									return penDist2;
			}
			if (penDist3 < 0 && penDist4 < 0) {
							return max(penDist3, penDist4);
			}
			if (penDist3 < 0) {
				return penDist3;
			}
			if (penDist4 < 0) {
				return penDist4;
			}
			if (x >= r3_2.x + paramsH.toleranceZone
					&& x < r3_2.x + 2 * r4_2.x - paramsH.toleranceZone) {
				return largePenet;
			}



//			if (x >= 0 && x < r3_2.x + paramsH.toleranceZone) {
//				penDist = IsInBoundaryEllipsoid(R2(x, y), R2(0, 0), r3_2);
//				if (penDist < 0)
//					return penDist;
//			}
//			if (x >= r3_2.x + paramsH.toleranceZone
//					&& x < r3_2.x + 2 * r4_2.x - paramsH.toleranceZone) {
//				return largePenet;
//			}
//			if (x >= r3_2.x + 2 * r4_2.x - paramsH.toleranceZone
//					&& x < r3_2.x + 2 * r4_2.x + r6_2.x) {
//				penDist = IsInBoundaryEllipsoid(R2(x, y),
//						R2(r3_2.x + 2 * r4_2.x + r6_2.x, 0), r6_2);
//				if (penDist < 0)
//					return penDist;
//			}
			if (x >= r2_2.x + 2 * r1_2.x && x < r2_2.x + 2 * r1_2.x + r5_2.x) {
				penDist = IsOutBoundaryEllipsoid(R2(x, y),
						R2(r2_2.x + 2 * r1_2.x + r5_2.x, 0), r5_2);
				if (penDist < 0)
					return penDist;
			}
			//****** horizontal walls
			x = x - (r3_2.x + 2 * r4_2.x + r6_2.x);
			if (x > 0) {
				real2 y2_slimHor = R2(2.314, 3.314) * mm;
				real2 y2_endHor = R2(r2_2.y, r3_2.y);
				if (x < x_FirstChannel) {
					penDist = y - r5_2.y;
					if (penDist < 0)
						return penDist; //note that y is negative, fabs(y) = -y
					penDist = -y + r6_2.y;
					if (penDist < 0)
						return penDist;
				}
				if (x >= x_FirstChannel + paramsH.toleranceZone
						&& x
								< x_FirstChannel + x_SecondChannel
										- paramsH.toleranceZone) {
					penDist = y - y2_slimHor.x;
					if (penDist < 0)
						return penDist;
					penDist = -y + y2_slimHor.y;
					if (penDist < 0)
						return penDist;
				}
				if (x >= x_FirstChannel + x_SecondChannel) {
					penDist = y - y2_endHor.x;
					if (penDist < 0)
						return penDist;
					penDist = -y + y2_endHor.y;
					if (penDist < 0)
						return penDist;
				}
				//****** vertical walls
				if (x >= x_FirstChannel && x < x_FirstChannel + paramsH.toleranceZone) {
					if (y > r6_2.y + paramsH.toleranceZone
							|| y < r5_2.y - paramsH.toleranceZone) {
						return largePenet;
					}
					if (y < y2_slimHor.x || y > y2_slimHor.y) {
						return x_FirstChannel - x;
					}
				}
				if (x >= x_FirstChannel + x_SecondChannel - paramsH.toleranceZone
						&& x < x_FirstChannel + x_SecondChannel) {
					if (y > y2_endHor.y + paramsH.toleranceZone
							|| y < y2_endHor.x - paramsH.toleranceZone) {
						return largePenet;
					}
					if (y < y2_slimHor.x || y > y2_slimHor.y) {
						return x - (x_FirstChannel + x_SecondChannel);
					}
				}
			}
		}
	}
	if (!isOut)
		return -largePenet;
	return penDist2;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//*** paramsH.straightChannelBoundaryMin   should be taken care of
//*** paramsH.straightChannelBoundaryMax   should be taken care of
real_ IsInsideStraightChannel(real3 posRad) {
	const real_ sphR = paramsH.HSML;
	real_ penDist1 = 0;
	real_ penDist2 = 0;
	real_ largePenet = -5 * sphR; //like a large number. Should be negative (assume large initial penetration)

	if (posRad.z > paramsH.straightChannelBoundaryMax.z) {
		penDist1 = paramsH.straightChannelBoundaryMax.z - posRad.z;
	}
	if (posRad.z < paramsH.straightChannelBoundaryMin.z) {
		penDist1 = posRad.z - paramsH.straightChannelBoundaryMin.z;
	}

	if (posRad.y < paramsH.straightChannelBoundaryMin.y) {
		penDist2 = posRad.y - paramsH.straightChannelBoundaryMin.y;
	}
	if (posRad.y > paramsH.straightChannelBoundaryMax.y) {
		penDist2 = paramsH.straightChannelBoundaryMax.y - posRad.y;
	}
	if (penDist1 < 0 && penDist2 < 0) {
		return Min(penDist1, penDist2);
	}
	if (penDist1 < 0)
		return penDist1;
	if (penDist2 < 0)
		return penDist2;
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideStraightChannel_XZ(real3 posRad) {
	const real_ sphR = paramsH.HSML;
	real_ penDist1 = 0;
	real_ largePenet = -5 * sphR; //like a large number. Should be negative (assume large initial penetration)

	if (posRad.z > paramsH.straightChannelBoundaryMax.z) {
		penDist1 = paramsH.straightChannelBoundaryMax.z - posRad.z;
	}
	if (posRad.z < paramsH.straightChannelBoundaryMin.z) {
		penDist1 = posRad.z - paramsH.straightChannelBoundaryMin.z;
	}
	if (penDist1 < 0)
		return penDist1;
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideTube(real3 posRad) {
	real_ penDist1 = 0;
	real_ largePenet = -5 * paramsH.HSML; //like a large number. Should be negative (assume large initial penetration)
	real2 centerLine = R2(channelCenterYZ.x, channelCenterYZ.y);
	real_ r = length(R2(posRad.y, posRad.z) - centerLine);

	if (r > channelRadius) {
		penDist1 = channelRadius - r;
	}
	if (penDist1 < 0)
		return penDist1;
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ IsInsideStepTube(real3 posRad) {
	const real_ sphR = paramsH.HSML;
	real_ penDist1 = 0;
	real_ penDist2 = 0;
	real_ largePenet = -5 * sphR; //like a large number. Should be negative (assume large initial penetration)
	real2 centerLine = R2(channelCenterYZ.x, channelCenterYZ.y);
	real_ r = length(R2(posRad.y, posRad.z) - centerLine);

	real_ tubeLength = paramsH.cMax.x - paramsH.cMin.x;
	real_ r1 = channelRadius;
	real_ r2 = 0.5 * channelRadius;
	real_ d1 = paramsH.cMin.x + .25 * tubeLength;
	real_ d2 = paramsH.cMin.x + .75 * tubeLength;
	real_ d3 = paramsH.cMin.x + tubeLength;
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
	if (r < r1 + paramsH.toleranceZone) {
		if (posRad.x < d1) {
			penDist2 = posRad.x - d1;
		}
		if (posRad.x > d2) {
			penDist2 = d2 - posRad.x;
		}
	}

	if (penDist1 < 0 && penDist2 < 0)
		return max(penDist1, penDist2); //note: both are negative. max is closer to boundary
	if (penDist1 < 0)
		return penDist1; //penDist2 is always either zero or negative
	//printf("hey \n");
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int2 CreateFluidMarkers(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		thrust::host_vector<real3> & mPosRadBoundary,
		thrust::host_vector<real4> & mVelMasBoundary,
		thrust::host_vector<real4> & mRhoPresMuBoundary,
		real_ & sphMarkerMass,
		const thrust::host_vector<real3> & rigidPos,
		const thrust::host_vector<Rotation> rigidRotMatrix,
		const thrust::host_vector<real3> & ellipsoidRadii,
		const thrust::host_vector<CylinderGeometry> & cylinderGeom,
		const thrust::host_vector<real3> & ANCF_Nodes,
		const thrust::host_vector<int2> & ANCF_ReferenceArrayNodesOnBeams,
		const ANCF_Params & flexParams) {
//	printf("\n\n\nStraightChannelBoundaries: \n   min: %f %f %f\n   max %f %f %f\n\n\n", paramsH.straightChannelBoundaryMin.x, paramsH.straightChannelBoundaryMin.y, paramsH.straightChannelBoundaryMin.z,
//			paramsH.straightChannelBoundaryMax.x, paramsH.straightChannelBoundaryMax.y, paramsH.straightChannelBoundaryMax.z);
//	printf("\n\n\nPipeSpecs: \n   channelRadius: %f \n   channelCenterYZ %f %f\n\n\n", channelRadius, channelCenterYZ.x, channelCenterYZ.y);

	//real2 rad2 = .5 * R2(paramsH.cMax.y - paramsH.cMin.y, paramsH.cMax.z - paramsH.cMin.z);
	//channelRadius = (rad2.x < rad2.y) ? rad2.x : rad2.y;
	int num_FluidMarkers = 0;
	int num_BoundaryMarkers = 0;
	srand(964);
	real_ multInitSpace = paramsH.MULT_INITSPACE; //0.9;//0.9;
	real_ initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML;
	int nFX = ceil((paramsH.cMax.x - paramsH.cMin.x) / (initSpace0));
	real_ initSpaceX = (paramsH.cMax.x - paramsH.cMin.x) / nFX;
	//printf("orig nFx and nFx %f %f\n", (paramsH.cMax.x - paramsH.cMin.x) / initSpace, ceil ((paramsH.cMax.x - paramsH.cMin.x) / (initSpace)));
	int nFY = ceil((paramsH.cMax.y - paramsH.cMin.y) / (initSpace0));
	real_ initSpaceY = (paramsH.cMax.y - paramsH.cMin.y) / nFY;
	int nFZ = ceil((paramsH.cMax.z - paramsH.cMin.z) / (initSpace0));
	real_ initSpaceZ = (paramsH.cMax.z - paramsH.cMin.z) / nFZ;
	//printf("&&&&& %f   %f %f %f \n", 1.1 * sphR, initSpaceX, initSpaceY, initSpaceZ);
	printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ,
			(nFY - 1) * initSpaceY, initSpaceY);
	sphMarkerMass = (initSpaceX * initSpaceY * initSpaceZ) * paramsH.rho0;
//	printf("sphMarkerMass *%e\n", (initSpaceX * initSpaceY * initSpaceZ) * paramsH.rho0);

	//printf("** nFX, nFX, nFX %d %d %d\n", nFX, nFY, nFZ);
	//printf("paramsH.cMin.z + initSpaceZ * (nFZ - 1) %f paramsH.cMax.z %f initSpaceZ %f initSpace0 %f\n", paramsH.cMin.z + initSpaceZ * (nFZ - 1), paramsH.cMax.z, initSpaceZ, initSpace0);
	//printf("dist&*&*&* %f %f\n", (paramsH.cMax.x - paramsH.cMin.x) - initSpace * (nFX-1) - 2 * sphR, sphR);

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
				posRad =
						paramsH.cMin
								+ R3(i * initSpaceX, j * initSpaceY, k * initSpaceZ)
								+ R3(.5 * initSpace0)/* + R3(sphR) + initSpace * .05 * (real_(rand()) / RAND_MAX)*/;
				real_ penDist = 0;
				bool flag = true;
				///penDist = IsInsideCurveOfSerpentineBeta(posRad);
				///penDist = IsInsideSerpentine(posRad);
				//*** paramsH.straightChannelBoundaryMin   should be taken care of
				//*** paramsH.straightChannelBoundaryMax   should be taken care of
				///penDist = IsInsideStraightChannel(posRad);
				penDist = IsInsideStraightChannel_XZ(posRad);
				///penDist = IsInsideTube(posRad);
				///penDist = IsInsideStepTube(posRad);

				if (penDist < -paramsH.toleranceZone)
					flag = false;
				real_ fluidDistanceFromSolidSurface = paramsH.MULT_INITSPACE * paramsH.HSML - paramsH.solidSurfaceAdjust;
				if (flag) {
					for (int rigidBodyIdx = 0; rigidBodyIdx < rigidPos.size(); rigidBodyIdx++) {
						//** Ellipsoid
						if (IsInsideEllipsoid(posRad, rigidPos[rigidBodyIdx], rigidRotMatrix[rigidBodyIdx], ellipsoidRadii[rigidBodyIdx], fluidDistanceFromSolidSurface)) {
							flag = false;
						}
						//** Cylinder_XZ
//						if ( IsInsideCylinder_XZ(posRad, rigidPos[rigidBodyIdx], ellipsoidRadii[rigidBodyIdx], fluidDistanceFromSolidSurface ) ) { flag = false;}
						//** cylinder3D
//						if (IsInsideCylinder_3D(posRad, cylinderGeom[rigidBodyIdx].pa3, cylinderGeom[rigidBodyIdx].pb3, cylinderGeom[rigidBodyIdx].r, fluidDistanceFromSolidSurface)) { flag = false;}
					}
					for (int flexID = 0;
							flexID < ANCF_ReferenceArrayNodesOnBeams.size();
							flexID++) {
						int2 nodesStartEnd2 =
								ANCF_ReferenceArrayNodesOnBeams[flexID];
						real3 pa3 = ANCF_Nodes[nodesStartEnd2.x];
						real3 pb3 = ANCF_Nodes[nodesStartEnd2.y - 1];
						if (IsInsideStraightFlex(posRad, pa3, pb3, flexParams.r, fluidDistanceFromSolidSurface)) { flag = false; }
						//if (IsInsideStraightFlexWithBob(posRad, pa3, pb3, flexParams.r, flexParams.bobRad, fluidDistanceFromSolidSurface)) { flag = false; }
					}
				}
				if (flag) {
					if (penDist > 0) {
						if (k < nFZ) {
							num_FluidMarkers++;
							mPosRad.push_back(posRad);
							real3 v3 = R3(0);
							//**************** straight channel initialliation
							if (initParabolic) {
								real_ vx = CalcChannelParabolicVel(posRad);
								v3 = R3(vx, 0, 0);
							}
							//************************************************
							mVelMas.push_back(R4(v3, sphMarkerMass));
							mRhoPresMu.push_back(
									R4(paramsH.rho0, paramsH.BASEPRES,
											paramsH.mu0, -1)); //rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
							//just note that the type, i.e. mRhoPresMu.w is real_.
							//viscosity of the water is .0018

						}
					} else {
						num_BoundaryMarkers++;
						mPosRadBoundary.push_back(posRad);
						mVelMasBoundary.push_back(R4(0, 0, 0, sphMarkerMass));
						if (-penDist / (paramsH.MULT_INITSPACE * paramsH.HSML) <= 2 ) {
							mRhoPresMuBoundary.push_back(
									R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0,
											0)); //rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
							//just note that the type, i.e. mRhoPresMu.w is real_.
							//viscosity of the water is .0018
						} else {
							mRhoPresMuBoundary.push_back(
									R4(paramsH.rho0, paramsH.LARGE_PRES, paramsH.mu0,
																		0));
						}
					}
				}
			}
		}
	}
	return I2(num_FluidMarkers, num_BoundaryMarkers);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int CreateEllipsoidMarkers(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu, real3 rigidPos,
		Rotation rigidRotMatrix, real3 ellipsoidRadii, real4 sphereVelMas,
		real3 rigidBodyOmega, real_ sphMarkerMass, int type) {
	int num_RigidBodyMarkers = 0;
	//real_ spacing = .9 * sphR;
	real_ spacing = paramsH.MULT_INITSPACE * paramsH.HSML;
	//printf("initSpaceEllipsoid = %f * sphR\n", multInitSpace);
	for (int k = 0; k < paramsH.NUM_BCE_LAYERS; k++) {
		real3 r3 = ellipsoidRadii - R3(k * spacing + paramsH.solidSurfaceAdjust);
		//printf("r, rigidR, k*spacing %f %f %f\n", r * 1000000, spherePosRad.w * 1000000, k * spacing * 1000000);
		real_ minR = r3.x;
		minR = (minR < r3.y) ? minR : r3.y;
		minR = (minR < r3.z) ? minR : r3.z;
		if (minR > 0) {
			real_ deltaTeta0 = spacing / r3.z;
			real_ teta = 0.1 * deltaTeta0;
			while (teta < PI - .1 * deltaTeta0) {
				real_ deltaPhi0 = spacing / (r3.z * sin(teta));
				real_ phi = 0.1 * deltaPhi0;
				real_ currentR = 1
						/ length(
								R3(sin(teta) * cos(phi), sin(teta) * sin(phi),
										cos(teta)) / r3);
				real_ nextR = 1
						/ length(
								R3(sin(teta + spacing / currentR) * cos(phi),
										sin(teta + spacing / currentR)
												* sin(phi),
										cos(teta + spacing / currentR)) / r3);
				real_ deltaTeta = spacing / max(currentR, nextR);
				while (phi < 2 * PI + deltaPhi0) {
					real3 mult3 = R3(sin(teta) * cos(phi), sin(teta) * sin(phi),
							cos(teta)) / r3;
					real_ r = 1 / length(mult3);
					real3 posRadRigid_sphMarkerLocal = R3(
							r * sin(teta) * cos(phi), r * sin(teta) * sin(phi),
							r * cos(teta));
					real3 posRadRigid_sphMarkerLocalRotate =
							posRadRigid_sphMarkerLocal.x
									* R3(rigidRotMatrix.a00, rigidRotMatrix.a10,
											rigidRotMatrix.a20)
									+ posRadRigid_sphMarkerLocal.y
											* R3(rigidRotMatrix.a01,
													rigidRotMatrix.a11,
													rigidRotMatrix.a21)
									+ posRadRigid_sphMarkerLocal.z
											* R3(rigidRotMatrix.a02,
													rigidRotMatrix.a12,
													rigidRotMatrix.a22);

					real3 posRadRigid_sphMarker =
							posRadRigid_sphMarkerLocalRotate + rigidPos;

					mPosRad.push_back(posRadRigid_sphMarker);
					real_ deltaPhiDum = spacing / (r * sin(teta));
					real_ rNewDum = 1
							/ length(
									R3(sin(teta) * cos(phi + deltaPhiDum),
											sin(teta) * sin(phi + deltaPhiDum),
											cos(teta)) / r3);
					real_ maxR = max(rNewDum, r);
					phi += spacing / (maxR * sin(teta));

					real3 vel = R3(sphereVelMas)
							+ cross(rigidBodyOmega,
									posRadRigid_sphMarker - rigidPos); //assuming LRF is the same as GRF at time zero (rigibodyOmega is in LRF, the second term of the cross is in GRF)
							//printf("veloc %f %f %f\n", vel.x, vel.y, vel.z);
					mVelMas.push_back(R4(vel, sphMarkerMass));
					real_ representedArea = spacing * spacing;
					mRhoPresMu.push_back(
							R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0,
									type)); // for rigid body particle, rho represents the represented area
					//																						// for the rigid particles, the fluid properties are those of the contacting fluid particles
					num_RigidBodyMarkers++;
					//printf("num_RigidBodyMarkers %d\n", num_RigidBodyMarkers);
					//printf("y %f\n", y);

					//////reCalc deltaTeta and deltaPhi
				}
				teta += deltaTeta;
			}
		}
	}
	//printf("num_RigidBodyMarkers %f\n", num_RigidBodyMarkers);
	return num_RigidBodyMarkers;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int Create3D_CylinderMarkers(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,

		thrust::host_vector<real_> & flexParametricDist,

		real3 pa3, //inital point
		real3 pb3, //end point
		real_ l, //beam length			//thrust::host_vector<real_> &  ANCF_Beam_Length
		real_ rad,
		real_ sphMarkerMass, int type) {
	int num_FlexMarkers = 0;
	real_ spacing = paramsH.MULT_INITSPACE * paramsH.HSML;

	real3 n3 = normalize(pb3 - pa3);
	real3 axis3 = cross(R3(1, 0, 0), n3);
	real_ angle;
	if (fabs(length(axis3)) < .000001) {
		angle = 0;
		axis3 = R3(1, 0, 0); //whatever
	} else {
		axis3 /= length(axis3);
		angle = acos(n3.x); //acos(dot(R3(1, 0, 0) , n3));
	}
	real4 q4 = R4(cos(0.5 * angle), axis3.x * sin(0.5 * angle),
			axis3.y * sin(0.5 * angle), axis3.z * sin(0.5 * angle));

//	printf("type %d: pa %f %f %f, pb %f %f %f, n3 %f %f %f, axis %f %f %f, angle %f, q %f %f %f %f\n",
//			type, pa3.x, pa3.y, pa3.z, pb3.x, pb3.y, pb3.z, n3.x, n3.y, n3.z,
//			axis3.x, axis3.y, axis3.z, angle, q4.x, q4.y, q4.z, q4.w);

	for (real_ s = 0; s <= l; s += spacing) {
		real3 centerPoint = pa3 + s * n3;
		mPosRad.push_back(centerPoint);
		mVelMas.push_back(R4(0, 0, 0, sphMarkerMass));
		mRhoPresMu.push_back(
				R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type)); //take care of type			 /// type needs to be unique, to differentiate flex from other flex as well as other rigids
		flexParametricDist.push_back(s);
		num_FlexMarkers++;
//	** 	for (real_ r = spacing; r < rad + .1 * spacing; r += spacing) {
		for (int k = 0; k < paramsH.NUM_BCE_LAYERS; k++) {
			real_ r = rad - (k * spacing + paramsH.solidSurfaceAdjust);
			if (r > 0) {
				real_ deltaTeta = spacing / r;
				for (real_ teta = .1 * deltaTeta; teta < 2 * PI - .1 * deltaTeta;
						teta += deltaTeta) {
					real3 BCE_Pos_local = R3(0, r * cos(teta), r * sin(teta));

	//				///ff1
	//				real_ hh = paramsH.HSML;
	//				printf("dd dist %f \n", length(BCE_Pos_local) / hh);

					real3 BCE_Pos_Global = Rotate_By_Quaternion(q4, BCE_Pos_local)
							+ centerPoint;
					mPosRad.push_back(BCE_Pos_Global);
					mVelMas.push_back(R4(0, 0, 0, sphMarkerMass));
					mRhoPresMu.push_back(
							R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type)); //take care of type
					flexParametricDist.push_back(s);
					num_FlexMarkers++;
				}
			}
		}
	}
	return num_FlexMarkers;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int Create3D_CylinderMarkersRigid(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,

		thrust::host_vector<real_> & flexParametricDist,

		real3 pa3, //inital point
		real3 pb3, //end point
		real_ l, //beam length			//thrust::host_vector<real_> &  ANCF_Beam_Length
		real_ rad,
		real4 q4,
		real_ sphMarkerMass, int type) {
	int num_BCEMarkers = 0;
	real_ multInitSpace = paramsH.MULT_INITSPACE; //0.9;//1.0;//0.9;
	real_ spacing = multInitSpace * paramsH.HSML;

	real3 n3 = normalize(pb3 - pa3);

//	real3 axis3 = cross(R3(1, 0, 0), n3);
//	real_ angle;
//	if (fabs(length(axis3)) < .000001) {
//		angle = 0;
//		axis3 = R3(1, 0, 0); //whatever
//	} else {
//		axis3 /= length(axis3);
//		angle = acos(n3.x); //acos(dot(R3(1, 0, 0) , n3));
//	}
//	real4 q4 = R4(cos(0.5 * angle), axis3.x * sin(0.5 * angle),
//			axis3.y * sin(0.5 * angle), axis3.z * sin(0.5 * angle));

//	printf("type %d: pa %f %f %f, pb %f %f %f, n3 %f %f %f, axis %f %f %f, angle %f, q %f %f %f %f\n",
//			type, pa3.x, pa3.y, pa3.z, pb3.x, pb3.y, pb3.z, n3.x, n3.y, n3.z,
//			axis3.x, axis3.y, axis3.z, angle, q4.x, q4.y, q4.z, q4.w);

	for (real_ s = 0; s <= l; s += spacing) {
		real3 centerPoint = pa3 + s * n3;
		mPosRad.push_back(centerPoint);
		mVelMas.push_back(R4(0, 0, 0, sphMarkerMass));
		mRhoPresMu.push_back(
				R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type)); //take care of type			 /// type needs to be unique, to differentiate flex from other flex as well as other rigids
		flexParametricDist.push_back(s);
		num_BCEMarkers++;
//	**	for (real_ r = spacing; r < rad + .1 * spacing; r += spacing) {
		for (int k = 0; k < paramsH.NUM_BCE_LAYERS; k++) { //check this in the future
			real_ r = rad - (k * spacing + paramsH.solidSurfaceAdjust);
			if (r > 0) {
				real_ deltaTeta = spacing / r;
				for (real_ teta = .1 * deltaTeta; teta < 2 * PI - .1 * deltaTeta;
						teta += deltaTeta) {
					real3 BCE_Pos_local = R3(r * cos(teta), r * sin(teta), 0);
					real3 BCE_Pos_Global = Rotate_By_Quaternion(q4, BCE_Pos_local)
							+ centerPoint;
					mPosRad.push_back(BCE_Pos_Global);
					mVelMas.push_back(R4(0, 0, 0, sphMarkerMass));
					mRhoPresMu.push_back(
							R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, type)); //take care of type
					flexParametricDist.push_back(s);
					num_BCEMarkers++;
				}
			}
		}
	}
	return num_BCEMarkers;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int CreateBobMarkersOnBeam(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,

		thrust::host_vector<real_> & flexParametricDist,

		real3 pb3, //end point
		real_ l, //beam length			//thrust::host_vector<real_> &  ANCF_Beam_Length
		real_ bobRad,
		real_ sphMarkerMass, int type) {
	Rotation myRotation = UnitMatrix();
	int numBobMarkers = CreateEllipsoidMarkers(mPosRad, mVelMas, mRhoPresMu,
			pb3, myRotation, R3(bobRad), R4(0), R3(0),
			sphMarkerMass, type); //as type
	int numFlexParams = flexParametricDist.size();
	flexParametricDist.resize(numFlexParams + numBobMarkers);
	thrust::fill(flexParametricDist.begin() + numFlexParams, flexParametricDist.end(), l);
	return numBobMarkers;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int CreateFlexMarkers(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,

		thrust::host_vector<real_> & flexParametricDist,

		real3 pa3, //inital point
		real3 pb3, //end point
		real_ l, //beam length			//thrust::host_vector<real_> &  ANCF_Beam_Length
		ANCF_Params flexParams,
		real_ sphMarkerMass, int type) {
	return Create3D_CylinderMarkers(
			mPosRad, mVelMas, mRhoPresMu, flexParametricDist,
			pa3, pb3, l, flexParams.r, sphMarkerMass,  type);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int CreateFlexMarkersWithBob(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,

		thrust::host_vector<real_> & flexParametricDist,

		real3 pa3, //inital point
		real3 pb3, //end point
		real_ l, //beam length			//thrust::host_vector<real_> &  ANCF_Beam_Length
		ANCF_Params flexParams,
		real_ sphMarkerMass, int type) {
	//** create cylinder
	int numCylinderMarkers = Create3D_CylinderMarkers(
			mPosRad, mVelMas, mRhoPresMu, flexParametricDist,
			pa3, pb3, l, flexParams.r, sphMarkerMass,  type);
	//** create bob
	int numBobMarkers = CreateBobMarkersOnBeam(
			mPosRad, mVelMas, mRhoPresMu, flexParametricDist,
			pb3, l, flexParams.bobRad, sphMarkerMass,  type);

	return numCylinderMarkers + numBobMarkers;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int CreateCylinderMarkers_XZ(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu, real3 rigidPos,
		real3 ellipsoidRadii, real4 sphereVelMas, real3 rigidBodyOmega,
		real_ sphMarkerMass, int type) {
	int num_RigidBodyMarkers = 0;
	real_ multInitSpace = paramsH.MULT_INITSPACE; //0.9;
	real_ spacing = multInitSpace * paramsH.HSML;
	printf("initSpaceCylinder = %f * sphR\n", multInitSpace);
	for (int k = 0; k < paramsH.NUM_BCE_LAYERS; k++) {
		real_ r = ellipsoidRadii.x -(k * spacing + paramsH.solidSurfaceAdjust);
		if (r > 0) {
			real_ deltaTeta = spacing / r;
			for (real_ teta = .1 * deltaTeta; teta < 2 * PI - .1 * deltaTeta;
					teta += deltaTeta) {
				//printf("gher %f, %f\n", deltaTeta / 2 / PI, deltaTeta);
				for (real_ y = paramsH.cMin.y;
						y < paramsH.cMax.y - .1 * spacing; y += spacing) {
					//printf("aha\n");
					real3 posRadRigid_sphMarker = R3(r * cos(teta), y,
							r * sin(teta)) + R3(rigidPos.x, 0, rigidPos.z);

					mPosRad.push_back(posRadRigid_sphMarker);
					real3 vel = R3(sphereVelMas)
							+ cross(rigidBodyOmega,
									posRadRigid_sphMarker - rigidPos); //assuming LRF is the same as GRF at time zero (rigibodyOmega is in LRF, the second term of the cross is in GRF)
					mVelMas.push_back(R4(vel, sphMarkerMass));
					real_ representedArea = spacing * spacing;
					mRhoPresMu.push_back(
							R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0,
									type)); // for rigid body particle, rho represents the represented area
					//																						// for the rigid particles, the fluid properties are those of the contacting fluid particles 
					num_RigidBodyMarkers++;
					//printf("num_RigidBodyMarkers %d\n", num_RigidBodyMarkers);
					//printf("y %f\n", y);
				}

			}
		}
	}
	//printf("num_RigidBodyMarkers %f\n", num_RigidBodyMarkers);
	return num_RigidBodyMarkers;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int main() {
	//****************************************************************************************
	time_t rawtime;
	struct tm * timeinfo;

	//(void) cudaSetDevice(0);

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	printf("Job was submittet at date/time is: %s\n", asctime(timeinfo));

	fstream inp("dist.txt", ios::in);
	real_ distance;
	inp >> distance;
	inp.close();
	printf("distance, %f\n", distance);

	bool readFromFile = false; //true;		//true: initializes from file. False: initializes inside the code

	//*** Arrays definition
	thrust::host_vector<int3> referenceArray;
//	thrust::host_vector<int3> referenceArray_Types; //this is not written to checkPoint
	thrust::host_vector<real3> mPosRad; //do not set the size here since you are using push back later
	thrust::host_vector<real4> mVelMas;
	thrust::host_vector<real4> mRhoPresMu;
	thrust::host_vector<uint> bodyIndex;

	//*** rigid bodies
	//thrust::host_vector<real4> spheresPosRad;
	thrust::host_vector<real3> rigidPos;
	thrust::host_vector<real4> mQuatRot;
	thrust::host_vector<real4> velMassRigidH;
	thrust::host_vector<real3> rigidBodyOmega;
	thrust::host_vector<real3> rigidBody_J1;
	thrust::host_vector<real3> rigidBody_J2;
	thrust::host_vector<real3> rigidBody_InvJ1;
	thrust::host_vector<real3> rigidBody_InvJ2;

	//*** flex bodies
	thrust::host_vector<real3> ANCF_Nodes;
	thrust::host_vector<real3> ANCF_Slopes;
	thrust::host_vector<real3> ANCF_NodesVel;
	thrust::host_vector<real3> ANCF_SlopesVel;
	thrust::host_vector<real_> ANCF_Beam_Length;
	thrust::host_vector<int2> ANCF_ReferenceArrayNodesOnBeams;
	thrust::host_vector<bool> ANCF_IsCantilever;
	//*** flex markers
	thrust::host_vector<real_> flexParametricDist;
	ANCF_Params flexParams;
	NumberOfObjects numObjects;
	//** default num markers
	int numAllMarkers = 0;
	//*******************************************************
	if (readFromFile) {
		ReadEverythingFromFile(readFromFile, mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray,
					rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2,
					ANCF_Nodes, ANCF_Slopes, ANCF_NodesVel, ANCF_SlopesVel, ANCF_Beam_Length, ANCF_IsCantilever,
					ANCF_ReferenceArrayNodesOnBeams, flexParametricDist,
					channelRadius, channelCenterYZ, paramsH, flexParams, numObjects);
	} else {
	//****   Initializatiobn  ****************************************************************
			//***		These fields are initialized in collideSphereSphere.cu
			//***		paramsH.gridSize;
			//***		paramsH.worldOrigin;
			//***		paramsH.cellSize;
			//***		paramsH.numBodies;
			//***		paramsH.boxDims;
		paramsH.sizeScale = 1; //don't change it.
		paramsH.HSML = 0.000013;
		paramsH.MULT_INITSPACE = 1.0;
			paramsH.NUM_BOUNDARY_LAYERS = 3;
			paramsH.toleranceZone = paramsH.NUM_BOUNDARY_LAYERS * (paramsH.HSML * paramsH.MULT_INITSPACE);
			paramsH.NUM_BCE_LAYERS = 2;
			paramsH.solidSurfaceAdjust = 0;//.6 * (paramsH.HSML * paramsH.MULT_INITSPACE); // 0.6 for bouyant, under gravity
		paramsH.BASEPRES = 0;//10;
			paramsH.LARGE_PRES = paramsH.BASEPRES;//10000;
			paramsH.deltaPress; //** modified below
			paramsH.nPeriod = 7;
		paramsH.gravity = R3(0, 0, 0);//R3(0);//R3(0, -9.81, 0);
		paramsH.bodyForce3 = R3(0.1,0,0);//R4(3.2e-3,0,0,0);// R4(0);;// /*Re = 100 */ //R4(3.2e-4, 0, 0, 0);/*Re = 100 */
			paramsH.rho0 = 1000;
			paramsH.mu0 = .001;
		paramsH.v_Max = 12e-3;//18e-3;//1.5;//2e-1; /*0.2 for Re = 100 */ //2e-3;
			paramsH.EPS_XSPH = .5f;
			paramsH.multViscosity_FSI = 1; //or 5
		paramsH.dT = 1e-5;//.001; //sph alone: .01 for Re 10;
			paramsH.tFinal = 1000;//20 * paramsH.dT; //400
			paramsH.timePause = 0;//.0005 * paramsH.tFinal;//.0001 * paramsH.tFinal; // keep it as small as possible. the time step will be 1/10 * dT
			paramsH.timePauseRigidFlex = 0;//.05 * paramsH.tFinal;
			paramsH.kdT = 5;
			paramsH.gammaBB = 0.5;
		// *** cMax cMin, see below
//		paramsH.cMin = R3(0, -.1, -.1) * paramsH.sizeScale;
//		paramsH.cMax = R3(paramsH.nPeriod * distance + 0, 1 + .1, 1 + .1)* paramsH.sizeScale;
		// ************
			paramsH.binSize0; // will be changed
			paramsH.rigidRadius; //will be changed
		paramsH.densityReinit = 1; //0: no re-initialization, 1: with initialization
		paramsH.contactBoundary = 0; // 0: straight channel, 1: serpentine

		flexParams.E = 2.0e5;
		flexParams.r = paramsH.HSML * paramsH.MULT_INITSPACE * (paramsH.NUM_BCE_LAYERS - 1);
		flexParams.rho = 1050;//7200;
		flexParams.ne = 4;
		flexParams.A = PI * pow(flexParams.r, 2.0f);
		flexParams.I = .25 * PI * pow(flexParams.r, 4.0f);
		flexParams.gravity = paramsH.gravity;
		flexParams.bobRad = .1;

		//3D cylinder params:
		real_ rhoRigid = flexParams.rho;//1.05 * paramsH.rho0;//7200; //1.0 * paramsH.rho0;

		// note: for 3D pipe Poiseuille: f = 32*Re*mu^2/(rho^2 * D^3), where f: body force, Re = rho * u_ave * D / mu --> V=rho * D^2 f / (32 mu)
		// note: for 2D pipe Poiseuille: f = 12*Re*mu^2/(rho^2 * W^3), where f: body force, Re = rho * u_ave * W / mu --> V=rho * W^2 f / (12 mu)
		//****************************************************************************************
		//*** initialize pipe
		channelRadius = 0.5 * paramsH.sizeScale; //5.6 * paramsH.sizeScale; //1.0 * paramsH.sizeScale; //tube
		channelCenterYZ = R2(0.5, 0.5);
		//*** some other definitions for boundary and such
		//****************************************************************************************
		//*** initialize straight channel
//		paramsH.straightChannelBoundaryMin = R3(0, 0, 0) * paramsH.sizeScale;
//		paramsH.straightChannelBoundaryMax = R3(paramsH.nPeriod * distance + 0, 1, 1) * paramsH.sizeScale;
		paramsH.straightChannelBoundaryMin = R3(0, 0, 0); //2D channel
		paramsH.straightChannelBoundaryMax = R3(1 * mm, .2 * mm, 1 * mm) * paramsH.sizeScale;
//		paramsH.straightChannelBoundaryMin = R3(0, 0, 0); //3D channel
//		paramsH.straightChannelBoundaryMax = R3(2 * mm, 1 * mm, 3 * mm) * paramsH.sizeScale;
		//********************************************************************************************************
		//**  reminiscent of the past******************************************************************************
		//paramsH.cMax = R3( paramsH.nPeriod * 4.6 + 0, 1.5,  4.0) * paramsH.sizeScale;  //for only CurvedSerpentine (w/out straight part)
		//**  end of reminiscent of the past   ******************************************************************
//		paramsH.cMin = R3(0, -2 * paramsH.toleranceZone, -4.5 * mm); 							//for serpentine
//		paramsH.cMax = R3( paramsH.nPeriod * sPeriod + r3_2.x + 2 * r4_2.x + r6_2.x + x_FirstChannel + 2 * x_SecondChannel, 1.5 * mm,  r6_2.y + 2 * paramsH.toleranceZone);  //for serpentine

		paramsH.cMin = R3(0, 0, -2 * paramsH.toleranceZone);						// 2D channel
		paramsH.cMax = R3( 1 * mm, 0.2 * mm,  1 * mm + 2 * paramsH.toleranceZone);

//		paramsH.cMin = R3(0, -2 * paramsH.toleranceZone, -2 * paramsH.toleranceZone);						// 3D channel
//		paramsH.cMax = R3( 2 * mm, 1 * mm + 2 * paramsH.toleranceZone,  3 * mm + 2 * paramsH.toleranceZone);

		//	paramsH.cMax = R3(paramsH.nPeriod * 1.0 + 0, .5,  3.5) * paramsH.sizeScale;  //for straight channel, sphere
		//	paramsH.cMin = R3(0, -0.1, 0.5) * paramsH.sizeScale;

		//	paramsH.cMax = R3(paramsH.nPeriod * 1.0 + 0, 1.5, 1.5) * paramsH.sizeScale;  //for tube channel, sphere
		//	paramsH.cMin = R3(0, -0.5, -0.5) * paramsH.sizeScale;

		//	paramsH.cMax = R3( paramsH.nPeriod * 20.0 + 0, 11.2 + 2,  11.2 + 2) * paramsH.sizeScale;  //for  Segre Silberberg tube
		//	paramsH.cMin = R3(0, -2, -2) * paramsH.sizeScale;							//for tube
		//****************************************************************************************
		//printf("a1  paramsH.cMax.x, y, z %f %f %f,  binSize %f\n", paramsH.cMax.x, paramsH.cMax.y, paramsH.cMax.z, 2 * paramsH.HSML);
		int3 side0 = I3(
				floor((paramsH.cMax.x - paramsH.cMin.x) / (2 * paramsH.HSML)),
				floor((paramsH.cMax.y - paramsH.cMin.y) / (2 * paramsH.HSML)),
				floor((paramsH.cMax.z - paramsH.cMin.z) / (2 * paramsH.HSML)));
		real3 binSize3 = R3((paramsH.cMax.x - paramsH.cMin.x) / side0.x,
				(paramsH.cMax.y - paramsH.cMin.y) / side0.y,
				(paramsH.cMax.z - paramsH.cMin.z) / side0.z);
		paramsH.binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
	//	paramsH.binSize0 = (paramsH.binSize0 > binSize3.z) ? paramsH.binSize0 : binSize3.z;
		paramsH.binSize0 = binSize3.y; //for effect of distance. Periodic BC in x direction. we do not care about paramsH.cMax y and z.
		paramsH.cMax = paramsH.cMin + paramsH.binSize0 * R3(side0);
		paramsH.boxDims = paramsH.cMax - paramsH.cMin;
		printf("sizes *** nP * sP + xFistChannel + xSecondChannel %f \n ", 7 * sPeriod + (r3_2.x + 2 * r4_2.x + r6_2.x) + x_FirstChannel + x_SecondChannel);

		//printf("side0 %d %d %d \n", side0.x, side0.y, side0.z);

		//************************** modify pressure ***************************
//		paramsH.deltaPress = paramsH.rho0 * paramsH.boxDims * paramsH.bodyForce3;  //did not work as I expected
		paramsH.deltaPress = 1 * paramsH.boxDims * paramsH.bodyForce3;


		printf("***** deltaP %f %f %f\n", paramsH.deltaPress.x, paramsH.deltaPress.y, paramsH.deltaPress.z);



		//**********************************************************************
		//1---------------------------------------------- Initialization ----------------------------------------
		//2------------------------------------------- Generating Random Data -----------------------------------
		// This section can be used to generate ellipsoids' specifications and save them into data.txt
		//num_FluidMarkers = 40000;//5000;//10000;//4096;//16384;//65536;//8192;//32768;;//262144;//1024;//262144;//1024; //262144;//1000000;//16 * 1024;//262144;//1024;//262144;//1024;//262144;//1024 * 16;//262144;//4194304;//262144;//1024;//4194304; //2097152;//262144;//262144;//1024;//65536;//262144;	// The number of bodies

		//--------------------buffer initialization ---------------------------
		real_ pipeRadius;
		real_ pipeLength;
		real3 pipeInPoint3 = R3(paramsH.cMin.x, channelCenterYZ.x,
				channelCenterYZ.y);

		thrust::host_vector<real3> ellipsoidRadii;
		thrust::host_vector<CylinderGeometry> cylinderGeom;
		////***** here: define rigid bodies
		string fileNameRigids("spheresPos.dat");

		//real_ rr = .4 * (real_(rand()) / RAND_MAX + 1);
		paramsH.rigidRadius = R3(1.5 * paramsH.HSML, 1.5 * paramsH.HSML, 2 * paramsH.HSML);
		//**
//		int3 stride = I3(1, 1, 1);
//		CreateRigidBodiesPattern(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, paramsH.rigidRadius, rhoRigid, stride);

		//**
	//	int numRigids = 3000;
	//	CreateRigidBodiesChannelRandom(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, paramsH.rigidRadius, rhoRigid, numRigids);
		//**
//		int3 stride = I3(1, 1, 1);
//		CreateRigidBodiesPatternWithinBeams(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, paramsH.rigidRadius, rhoRigid, stride, flexParams);

		//**
		CreateRigidBodiesFromFile(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, fileNameRigids, rhoRigid, paramsH);
		//**
//		CreateRigidBodiesFromFileSerpentinePeriodic(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, fileNameRigids, rhoRigid);
		//**
	//	real2 cylinderR_H = R2(flexParams.r, .2);
	//	CreateOne3DRigidCylinder(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega,
	//			rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, cylinderGeom, fileNameRigids, rhoRigid, cylinderR_H);
		//**
	//	//channelRadius = 1.0 * paramsH.sizeScale;
	//	CreateRigidBodiesPatternPipe(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, paramsH.rigidRadius, rhoRigid);
	//	CreateRigidBodiesPatternStepPipe(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, paramsH.rigidRadius, rhoRigid);

		//**
	//	CreateRigidBodiesRandom(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, paramsH.rigidRadius, rhoRigid, 2); //changed 2 to 4
	//	CreateRigidBodiesPatternPipe_KindaRandom(rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, paramsH.rigidRadius, rhoRigid, 128);
		//**
	//	real_ beamLength = 0.7;
	//	CreateOneFlexBody(ANCF_Nodes, ANCF_Slopes, ANCF_NodesVel, ANCF_SlopesVel,
	//			ANCF_Beam_Length, ANCF_ReferenceArrayNodesOnBeams, ANCF_IsCantilever,
	//			channelRadius,
	//			paramsH.cMax.x - paramsH.cMin.x, pipeInPoint3, beamLength, flexParams);
		//**
	//	CreateSomeFlexBodies(ANCF_Nodes, ANCF_Slopes, ANCF_NodesVel, ANCF_SlopesVel,
	//			ANCF_Beam_Length, ANCF_ReferenceArrayNodesOnBeams, ANCF_IsCantilever,
	//			channelRadius,
	//			paramsH.cMax.x - paramsH.cMin.x, pipeInPoint3, flexParams);
		//**
	//	real_ beamLength = 10 * paramsH.HSML;
	//	CreateManyFlexBodiesPipe(ANCF_Nodes, ANCF_Slopes, ANCF_NodesVel, ANCF_SlopesVel,
	//			ANCF_Beam_Length, ANCF_ReferenceArrayNodesOnBeams, ANCF_IsCantilever,
	//			channelRadius,
	//			paramsH.cMax.x - paramsH.cMin.x, pipeInPoint3, 0.5 * beamLength, flexParams);
		//**
		//*** paramsH.straightChannelBoundaryMin   should be taken care of
		//*** paramsH.straightChannelBoundaryMax   should be taken care of
//		CreateManyFlexBodiesChannelGoodWithRigids(ANCF_Nodes, ANCF_Slopes, ANCF_NodesVel, ANCF_SlopesVel,
//				ANCF_Beam_Length, ANCF_ReferenceArrayNodesOnBeams, ANCF_IsCantilever, flexParams);
		//**

		thrust::host_vector<Rotation> rigidRotMatrix(mQuatRot.size());
		ConvertQuatArray2RotArray(rigidRotMatrix, mQuatRot);

		// ---------------------------------------------------------------------
		// initialize fluid particles

		thrust::host_vector<real3> mPosRadBoundary; //do not set the size here since you are using push back later
		thrust::host_vector<real4> mVelMasBoundary;
		thrust::host_vector<real4> mRhoPresMuBoundary;
		real_ sphMarkerMass; // To be initialized in CreateFluidMarkers, and used in other places

		int2 num_fluidOrBoundaryMarkers = CreateFluidMarkers(mPosRad, mVelMas,
				mRhoPresMu, mPosRadBoundary, mVelMasBoundary,
				mRhoPresMuBoundary, sphMarkerMass, rigidPos, rigidRotMatrix,
				ellipsoidRadii, cylinderGeom, ANCF_Nodes, ANCF_ReferenceArrayNodesOnBeams,
				flexParams);
		referenceArray.push_back(I3(0, num_fluidOrBoundaryMarkers.x, -1)); //map fluid -1
	//		referenceArray_Types.push_back((I3(-1, 0, 0)));
		numAllMarkers += num_fluidOrBoundaryMarkers.x;

		mPosRad.resize(
				num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
		mVelMas.resize(
				num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
		mRhoPresMu.resize(
				num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
		////boundary: type = 0
		int num_BoundaryMarkers = 0;
		//printf("size1 %d, %d , numpart %d, numFluid %d \n", mPosRadBoundary.end() - mPosRadBoundary.begin(), mPosRadBoundary.size(), num_fluidOrBoundaryMarkers.y, num_fluidOrBoundaryMarkers.x);
		thrust::copy(mPosRadBoundary.begin(), mPosRadBoundary.end(),
				mPosRad.begin() + num_fluidOrBoundaryMarkers.x);
		thrust::copy(mVelMasBoundary.begin(), mVelMasBoundary.end(),
				mVelMas.begin() + num_fluidOrBoundaryMarkers.x);
		thrust::copy(mRhoPresMuBoundary.begin(), mRhoPresMuBoundary.end(),
				mRhoPresMu.begin() + num_fluidOrBoundaryMarkers.x);
		mPosRadBoundary.clear();
		mVelMasBoundary.clear();
		mRhoPresMuBoundary.clear();

		referenceArray.push_back(
				I3(numAllMarkers, numAllMarkers + num_fluidOrBoundaryMarkers.y,
						0)); //map bc 0
	//		referenceArray_Types.push_back(I3(0, 0, 0));
		numAllMarkers += num_fluidOrBoundaryMarkers.y;

		//rigid body: type = 1, 2, 3, ...
	//		printf("num_RigidBodyMarkers: \n");
		int totalNumRigidMarkers = 0;
		for (int rigidBodyIdx = 0; rigidBodyIdx < rigidPos.size();
				rigidBodyIdx++) {
			//** Ellipsoid
			int num_RigidBodyMarkers = CreateEllipsoidMarkers(mPosRad, mVelMas,
					mRhoPresMu, rigidPos[rigidBodyIdx],
					rigidRotMatrix[rigidBodyIdx], ellipsoidRadii[rigidBodyIdx],
					velMassRigidH[rigidBodyIdx], rigidBodyOmega[rigidBodyIdx],
					sphMarkerMass, rigidBodyIdx + 1); //as type
			//** Cylinder_XZ
	//			int num_RigidBodyMarkers = CreateCylinderMarkers_XZ(mPosRad, mVelMas, mRhoPresMu,
	//													rigidPos[rigidBodyIdx], ellipsoidRadii[rigidBodyIdx],
	//													 velMassRigidH[rigidBodyIdx],
	//													 rigidBodyOmega[rigidBodyIdx],
	//													sphMarkerMass,
	//													 rigidBodyIdx + 1);		//as type
			//** Cylinder 3D
	//			thrust::host_vector<real_> dummyParamDist;
	//			int num_RigidBodyMarkers = Create3D_CylinderMarkersRigid(mPosRad, mVelMas, mRhoPresMu,
	//					dummyParamDist,
	//					cylinderGeom[rigidBodyIdx].pa3, cylinderGeom[rigidBodyIdx].pb3, cylinderGeom[rigidBodyIdx].h, cylinderGeom[rigidBodyIdx].r, mQuatRot[rigidBodyIdx],
	//					sphMarkerMass,
	//					rigidBodyIdx + 1); 		//as type
	//			dummyParamDist.clear();
			//******************************
			referenceArray.push_back(
					I3(numAllMarkers, numAllMarkers + num_RigidBodyMarkers, 1)); //rigid type: 1
	//			referenceArray_Types.push_back(I3(1, rigidBodyIdx, 0));
			numAllMarkers += num_RigidBodyMarkers;
	//			printf(" %d \n", num_RigidBodyMarkers);
			totalNumRigidMarkers += num_RigidBodyMarkers;
		}
		int totalNumFlexMarkers = 0;
		for (int flexBodyIdx = 0;
				flexBodyIdx < ANCF_ReferenceArrayNodesOnBeams.size();
				flexBodyIdx++) {
			real3 pa3 =
					ANCF_Nodes[ANCF_ReferenceArrayNodesOnBeams[flexBodyIdx].x];
			real3 pb3 =
					ANCF_Nodes[ANCF_ReferenceArrayNodesOnBeams[flexBodyIdx].y
							- 1];

	//			//**** create single flexible beam
			int num_FlexMarkers = CreateFlexMarkers(mPosRad, mVelMas,
					mRhoPresMu, flexParametricDist, pa3, //inital point
					pb3, //end point
					ANCF_Beam_Length[flexBodyIdx], //beam length			//thrust::host_vector<real_> &  ANCF_Beam_Length
					flexParams,
					sphMarkerMass, flexBodyIdx + rigidPos.size() + 1);
			//**** create single flexible beam with bob
	//			int num_FlexMarkers = CreateFlexMarkersWithBob(mPosRad, mVelMas,
	//					mRhoPresMu, flexParametricDist, pa3, //inital point
	//					pb3, //end point
	//					ANCF_Beam_Length[flexBodyIdx], //beam length			//thrust::host_vector<real_> &  ANCF_Beam_Length
	//					flexParams,
	//					sphMarkerMass, flexBodyIdx + rigidPos.size() + 1);

			referenceArray.push_back(
					I3(numAllMarkers, numAllMarkers + num_FlexMarkers, 2)); //map bc : rigidBodyIdx + 1
	//			referenceArray_Types.push_back(I3(1, rigidBodyIdx, 0));
			numAllMarkers += num_FlexMarkers;
			totalNumFlexMarkers += num_FlexMarkers;
			//printf(" %d \n", num_RigidBodyMarkers);
		}
		//**************************************


		//@@@@@@@@@@@@@@@@@@@@@@@@@ set number of objects once for all @@@@@@@@@@@@@@@@@@@@@@22
		numObjects.numFlexBodies = ANCF_Beam_Length.size();
		numObjects.numRigidBodies = rigidPos.size();
		numObjects.numFlBcRigid = 2 + numObjects.numRigidBodies;
		numObjects.numFluidMarkers = (referenceArray[0]).y - (referenceArray[0]).x;
		numObjects.numBoundaryMarkers = (referenceArray[1]).y - (referenceArray[1]).x;
		numObjects.startRigidMarkers = (referenceArray[1]).y;
		numObjects.numRigid_SphMarkers = referenceArray[2 + numObjects.numRigidBodies - 1].y - numObjects.startRigidMarkers;
		numObjects.startFlexMarkers = (referenceArray[numObjects.numFlBcRigid-1]).y;
		numObjects.numFlex_SphMarkers = referenceArray[numObjects.numFlBcRigid + numObjects.numFlexBodies - 1].y - numObjects.startFlexMarkers;
		numObjects.numAllMarkers = numAllMarkers;


		//@@@@@@@@ rigid body
		bodyIndex.resize(numAllMarkers);
		thrust::fill(bodyIndex.begin(), bodyIndex.end(), 1);
		thrust::exclusive_scan(bodyIndex.begin(), bodyIndex.end(),
				bodyIndex.begin());
		//**** clear temporary data
		cylinderGeom.clear();
		ellipsoidRadii.clear();
		rigidRotMatrix.clear();
		printf("********************\n rigid Radii: %f %f %f\n", paramsH.rigidRadius.x, paramsH.rigidRadius.y, paramsH.rigidRadius.z);

//		//******** write data to file  ******************************************************************************************
//		WriteEverythingToFile(mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray,
//				rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2,
//				ANCF_Nodes, ANCF_Slopes, ANCF_NodesVel, ANCF_SlopesVel, ANCF_Beam_Length, ANCF_IsCantilever,
//				ANCF_ReferenceArrayNodesOnBeams, flexParametricDist,
//				channelRadius, channelCenterYZ, paramsH, flexParams, numObjects, 0);
//		//***********************************************************************************************************************
	}

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

	printf("********************\n flexParams.E: %e\n flexParams.rho: %e\n flexParams.ne: %d\n", flexParams.E, flexParams.rho, flexParams.ne);
	printf("********************\n numFlexBodies: %d\n numRigidBodies: %d\n numFluidMarkers: %d\n "
			"numBoundaryMarkers: %d\n numRigid_SphMarkers: %d\n numFlex_SphMarkers: %d\n numAllMarkers: %d\n",
			numObjects.numFlexBodies, numObjects.numRigidBodies, numObjects.numFluidMarkers, numObjects.numBoundaryMarkers,
			numObjects.numRigid_SphMarkers, numObjects.numFlex_SphMarkers, numObjects.numAllMarkers);
	printf("********************\n");
	DOUBLEPRECISION ? printf("Double Precision\n") : printf("Single Precision\n");
	printf("********************\n");

	if (numObjects.numAllMarkers != 0) {
		cudaCollisions(mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray,
				rigidPos, mQuatRot, velMassRigidH, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2,
				ANCF_Nodes, ANCF_Slopes, ANCF_NodesVel, ANCF_SlopesVel, ANCF_Beam_Length, ANCF_IsCantilever,
				ANCF_ReferenceArrayNodesOnBeams, flexParametricDist,
				channelRadius, channelCenterYZ, paramsH, flexParams, numObjects);
	}
	mPosRad.clear();
	mVelMas.clear();
	mRhoPresMu.clear();
	bodyIndex.clear();
	referenceArray.clear();
//	referenceArray_Types.clear();
	flexParametricDist.clear();
	rigidPos.clear();
	mQuatRot.clear();
	velMassRigidH.clear();
	rigidBodyOmega.clear();
	rigidBody_J1.clear();
	rigidBody_J2.clear();
	rigidBody_InvJ1.clear();
	rigidBody_InvJ2.clear();

	ANCF_Nodes.clear();
	ANCF_Slopes.clear();
	ANCF_NodesVel.clear();
	ANCF_SlopesVel.clear();
	ANCF_ReferenceArrayNodesOnBeams.clear();
	ANCF_IsCantilever.clear();
	ANCF_Beam_Length.clear();
	return 0;
}
