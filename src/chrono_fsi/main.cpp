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

//for memory leak detection, apparently does not work in conjunction with cuda
//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>//just for min

#include <cutil_inline.h>
#include <cutil_math.h>
#include <thrust/host_vector.h>
#include <thrust/scan.h>
#include "collideSphereSphere.cuh"
#include "SDKCollisionSystem.cuh" //just for HSML and BASEPRES
#include <algorithm>

#define F4 make_float4
#define F3 make_float3
#define F2 make_float2

using namespace std;
//typedef unsigned int uint;

const float sPeriod = 4.6 * sizeScale;		//serpentine period
const float toleranceZone = 3 * HSML;
float3 straightChannelMin;
float3 straightChannelMax;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
float Min(float a, float b) {
	return (a < b) ? a : b;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateMassMomentEllipsoid(
		float & mass,
		float3 & j1,
		float3 & j2,
		float r1, float r2, float r3,
		const float rhoRigid) {
	mass = 4.0 / 3 * PI * r1 * r2 * r3 * rhoRigid;			//for sphere
	j1 = .2 * mass * F3(r2 * r2 + r3 * r3, 0.0f, 0.0f);
	j2 = .2 * mass * F3(r1 * r1 + r3 * r3, 0.0f, r1 * r1 + r2 * r2);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateMassMomentCylinder(
		float & mass,
		float3 & j1,
		float3 & j2,
		float r1,
		const float3 cMin,
		const float3 cMax,
		const float rhoRigid) {
		mass = PI * pow(r1, 2) * (cMax.y - cMin.y) * rhoRigid;	//for cylinder
		j1 = F3(1.0 / 12.0  * mass * (3 * pow(r1, 2) + pow(cMax.y - cMin.y, 2)), 0, 0);
		j2 = F3(.5 * mass * pow(r1, 2), 0, 1.0 / 12.0  * mass * (3 * pow(r1, 2) + pow(cMax.y - cMin.y, 2)));
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CalcInvJ(const float3 j1, const float3 j2, float3 & invJ1, float3 & invJ2) {
	//normalize J to deal with machine precision
	float3 maxJ3 = fmaxf(j1, j2);
	float maxComp = max(maxJ3.x, maxJ3.y);
	maxComp = max(maxComp, maxJ3.z);
	//********************************************
	float3 nJ1 = j1 / maxComp;	//nJ1 is normalJ1
	float3 nJ2 = j2 / maxComp;	//nJ2 is normalJ2

	float detJ = 2 * nJ1.z * nJ1.y * nJ2.y - nJ1.z * nJ1.z * nJ2.x - nJ1.y * nJ1.y * nJ2.z + nJ1.x * nJ2.x * nJ2.z - nJ1.x * nJ2.y * nJ2.y;
	invJ1 = F3(nJ2.x * nJ2.z - nJ2.y * nJ2.y, -nJ1.y * nJ2.z + nJ1.z * nJ2.y, nJ1.y * nJ2.y - nJ1.z * nJ2.x);
	invJ2 = F3(-nJ1.z * nJ1.z + nJ1.x * nJ2.z, -nJ1.x * nJ2.y + nJ1.z * nJ1.y, -nJ1.y * nJ1.y + nJ1.x * nJ2.x);

	//printf("invJ %f %f %f %f %f %f\n", aa.x, aa.y, aa.z, bb.x, bb.y, bb.z);
	//printf("invJ %f %f %f %f %f %f\n", 1e12 * j1.x, 1e12 *  j1.y, 1e12 *  j1.z,  1e12 * j2.x,  1e12 * j2.y, 1e12 *  j2.z);
	//printf("detJ %e\n", detJ * maxComp);
	// j = maxComp * nJ, therefore, jInv = maxComp * nJ_Inv
	invJ1 = invJ1 / detJ / maxComp;
	invJ2 = invJ2 / detJ / maxComp;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void CreateRigidBodiesFromFile(
		thrust::host_vector<float3> & rigidPos,
		thrust::host_vector<float4> & spheresVelMas,
		thrust::host_vector<float3> & rigidBodyOmega,
		thrust::host_vector<float3> & rigidBody_J1,
		thrust::host_vector<float3> & rigidBody_J2,
		thrust::host_vector<float3> & rigidBody_InvJ1,
		thrust::host_vector<float3> & rigidBody_InvJ2,
		thrust::host_vector<float3> & ellipsoidRadii,
		const float3 cMin,
		const float3 cMax,
		const string fileNameRigids,
		const float rhoRigid,
		float & channelRadius) {

	fstream ifileSpheres(fileNameRigids.c_str(), ios::in);
	//rRigidBody = .08 * sizeScale;//.06 * sizeScale;//.125 * sizeScale; // .25 * sizeScale; //.06 * sizeScale;//.08 * sizeScale;//.179 * sizeScale;

	float x, y, z;
	char ch;

	float dumRRigidBody1, dumRRigidBody2, dumRRigidBody3;
	ifileSpheres >> x >> ch >> y >> ch >> z >> ch >> dumRRigidBody1 >> ch >> dumRRigidBody2 >> ch >> dumRRigidBody3;
	int counterRigid = 0;
	while (!ifileSpheres.eof()) {
		//float r = rRigidBody * (.75 + .75 * float(rand())/RAND_MAX);
		for (int period = 0; period < nPeriod; period++) {
			rigidPos.push_back(F3(x * sizeScale + period * sPeriod, y * sizeScale, z * sizeScale));
			//float r1 = .8 * rRigidBody, r2 = 1.2 * rRigidBody, r3 = 3 * rRigidBody;
			//float r1 = rRigidBody, r2 = rRigidBody, r3 = rRigidBody;
			float r1 = dumRRigidBody1 * sizeScale;
			float r2 = dumRRigidBody2 * sizeScale;
			float r3 = dumRRigidBody3 * sizeScale;
			ellipsoidRadii.push_back(F3(r1, r2, r3));
			float mass;
			float3 j1, j2;

			CreateMassMomentEllipsoid(mass, j1, j2, r1, r2, r3, rhoRigid);						//create Ellipsoid
			//CreateMassMomentCylinder(mass, j1, j2, r1, cMin, cMax, rhoRigid);			//create Cylinder

			spheresVelMas.push_back(F4(0, 0, 0, float(mass)));
			rigidBodyOmega.push_back(F3(0, 0, 0));
			rigidBody_J1.push_back(j1);
			rigidBody_J2.push_back(j2);

			float3 invJ1, invJ2;
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
		thrust::host_vector<float3> & rigidPos,
		thrust::host_vector<float4> & spheresVelMas,
		thrust::host_vector<float3> & rigidBodyOmega,
		thrust::host_vector<float3> & rigidBody_J1,
		thrust::host_vector<float3> & rigidBody_J2,
		thrust::host_vector<float3> & rigidBody_InvJ1,
		thrust::host_vector<float3> & rigidBody_InvJ2,
		thrust::host_vector<float3> & ellipsoidRadii,
		const float3 referenceR,
		const float rhoRigid,
		float & channelRadius) {

	printf("referenceR %f %f %f \n", referenceR.x, referenceR.y, referenceR.z);
	//printf("cMin %f %f %f, cMax %f %f %f\n", straightChannelMin.x, straightChannelMin.y, straightChannelMin.z, straightChannelMax.x, straightChannelMax.y, straightChannelMax.z);
	float3 spaceRigids = 2 * (referenceR + 2 * F3(HSML));
	float3 n3Rigids = (straightChannelMax - straightChannelMin) / spaceRigids;
	for (int i = 1; i < n3Rigids.x - 1; i++) {
		for  (int j = 1; j < n3Rigids.y - 1; j++) {
			 for (int k = 1; k < n3Rigids.z - 1; k++) {
				 float3 pos = straightChannelMin + F3(i, j, k) * spaceRigids;
				 //printf("rigidPos %f %f %f\n", pos.x, pos.y, pos.z);
				 rigidPos.push_back(pos);
				 ellipsoidRadii.push_back(referenceR);
				 float mass;
				 float3 j1, j2;
				 CreateMassMomentEllipsoid(mass, j1, j2, referenceR.x, referenceR.y, referenceR.z, rhoRigid);						//create Ellipsoid

				spheresVelMas.push_back(F4(0, 0, 0, float(mass)));
				rigidBodyOmega.push_back(F3(0, 0, 0));
				rigidBody_J1.push_back(j1);
				rigidBody_J2.push_back(j2);

				float3 invJ1, invJ2;
				CalcInvJ(j1, j2, invJ1, invJ2);
				rigidBody_InvJ1.push_back(invJ1);
				rigidBody_InvJ2.push_back(invJ2);
			 }
		}
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideSphere(float4 sphParPos, float4 spherePosRad, float clearance) {
	float3 dist3 = F3(sphParPos - spherePosRad);
	if (length(dist3) < spherePosRad.w + clearance) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//the ellipsoids do not have any rotation
bool IsInsideEllipsoid(float4 sphParPos, float3 rigidPos, float3 radii, float clearance) {
	float3 dist3 = F3(sphParPos) - rigidPos;
	float3 mappedDist = dist3 / (radii + F3(clearance));
	if (length(mappedDist) < 1) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideCylinder_XZ(float4 sphParPos, float3 rigidPos, float3 radii, float clearance) {
	float3 dist3 = F3(sphParPos) - rigidPos;
	dist3.y = 0;
	//if (length(dist3) < spherePosRad.w + 2 * sphParPos.w) {
	if (length(dist3) < radii.x + clearance) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
float IsOutBoundaryCircle(float2 coord, float2 cent2, float r) {
		float cDist = length(coord - cent2);
		return cDist - r;
}
//--------------------------------------------------------------------------------------------------------------------------------
float IsInBoundaryCircle(float2 coord, float2 cent2, float r) {
		float cDist = length(coord - cent2);
		return r - cDist;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
float IsInsideSerpentine(float4 posRad) {
	const float sphR = posRad.w;
	float x, y;
	float distFromWall = 0;//2 * sphR;//0;//2 * sphR;
	float penDist = 0;
	float largePenet = -5*sphR;//like a large number. Should be negative (assume large initial penetration)
	float penDist2 = 0;
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
	if (posRad.x < nPeriod * sPeriod - toleranceZone) {
		float r1 = 1.3 * sizeScale, r2 = 1.0 * sizeScale, r3=2.0 * sizeScale, r4 = 0.3 * sizeScale;
		x = fmod(posRad.x, sPeriod); //posRad.x - int(posRad.x / sPeriod) * sPeriod; //fmod
		y = posRad.z;
		if (x >= 0 && x < 1.3 * sizeScale) {
			if (y < -3 * toleranceZone) return largePenet;
			if (y < 0) return (x - 1.3 * sizeScale);
			penDist = IsOutBoundaryCircle(F2(x, y), F2(0, 0), r1); if (penDist < 0) return penDist;
			penDist = IsInBoundaryCircle(F2(x, y), F2(0, 1.0 * sizeScale), r3); if (penDist < 0) return penDist;
		} else if (x >= 1.3 * sizeScale && x < 2.0 * sizeScale) {
			if (y > 1.0 * sizeScale) { penDist = IsInBoundaryCircle(F2(x, y), F2(0, 1.0 * sizeScale), r3); if (penDist < 0) return penDist; }
			else if (y < 0) { penDist = IsInBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist; }
		} else if (x >= 2.0 * sizeScale && x < 2.6 * sizeScale) {
			if (y < .55 * sizeScale) {
				penDist = IsInBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist;
				penDist = IsOutBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, .55 * sizeScale), r4); if (penDist < 0) return penDist; }
			else if (y < 2 * sizeScale) { penDist = IsOutBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, y), r4); if (penDist < 0) return penDist; }
			else return largePenet;
		} else if (x >= 2.6 * sizeScale && x < 3.3 * sizeScale) {
			if (y > 1.0 * sizeScale) { penDist = IsInBoundaryCircle(F2(x, y), F2(4.6 * sizeScale, 1.0 * sizeScale), r3); if (penDist < 0) return penDist; }
			else if (y < 0) { penDist = IsInBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist; }
		} else if (x >= 3.3 * sizeScale && x < 4.6 * sizeScale) {
			if (y < -3 * toleranceZone) return largePenet;
			if (y < 0) return 3.3 * sizeScale - x;
			penDist = IsOutBoundaryCircle(F2(x, y), F2(4.6 * sizeScale, 0), r1); if (penDist < 0) return penDist;
			penDist = IsInBoundaryCircle(F2(x, y), F2(4.6 * sizeScale, 1.0 * sizeScale), r3); if (penDist < 0) return penDist;
		}
		if (!isOut)
			return -largePenet;
		return penDist2;
	}

	//straight channel
	x = posRad.x - nPeriod * sPeriod;
	y = posRad.z;

	if (x < 0) {
		if (y < .55 * sizeScale - toleranceZone || y > 3.55 * sizeScale + toleranceZone) return largePenet;
		if ((y < 1.3 * sizeScale) || (y > 3 * sizeScale))
			return x;
	}
	//horizontal walls
	if (x > 0) {
		if (x < 5 * sizeScale) {
			penDist = y - (.55 * sizeScale); if (penDist < 0) return penDist;
			penDist = (3.55 * sizeScale) - y; if (penDist < 0) return penDist;
		} else if (x > 5 * sizeScale + toleranceZone && x < 6.5 * sizeScale - toleranceZone) {
			penDist = y - (1.55 * sizeScale); if (penDist < 0) return penDist;
			penDist = (2.55 * sizeScale) - y; if (penDist < 0) return penDist;
		} else if (x >= 6.5 * sizeScale) {
			penDist = y - (1.302 * sizeScale); if (penDist < 0) return penDist;
			penDist = (3 * sizeScale) - y; if (penDist < 0) return penDist;
		}
	}

	//vertical walls
	if (y < 1.302 * sizeScale || y > 3 * sizeScale) {penDist = x; if (penDist < 0) return penDist;}
	if (x > 5 * sizeScale && x < 5 * sizeScale + toleranceZone) {
		if (y < .55 * sizeScale - toleranceZone || y > 3.55 * sizeScale + toleranceZone) return largePenet;
		if (y < 1.55 * sizeScale || y > 2.55 * sizeScale) {penDist = (5 * sizeScale) - x; if (penDist < 0) return penDist; }
	}
	if (x > 6.5 * sizeScale - toleranceZone && x < 6.5 * sizeScale) {
		if (y < 1.3 * sizeScale - toleranceZone || y > 3 * sizeScale + toleranceZone) return largePenet;
		if (y < 1.55 * sizeScale || y > 2.55 * sizeScale) {penDist = x - (6.5 * sizeScale); if (penDist < 0) return penDist; }
	}
	if (!isOut)
		return -largePenet;
	return penDist2;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
float IsInsideCurveOfSerpentine(float4 posRad) {
	const float sphR = posRad.w;
	float x, y;
	float distFromWall = 0;//2 * sphR;//0;//2 * sphR;
	float penDist = 0;
	float largePenet = -5*sphR;//like a large number. Should be negative (assume large initial penetration)
	float penDist2 = 0;
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
	float r1 = 1.3 * sizeScale, r2 = 1.0 * sizeScale, r3=2.0 * sizeScale, r4 = 0.3 * sizeScale;
	x = fmod(posRad.x, sPeriod); //posRad.x - int(posRad.x / sPeriod) * sPeriod; //fmod
	y = posRad.z;
	if (x >= 0 && x < 1.3 * sizeScale) {
		if (y < -3 * toleranceZone) return largePenet;
		if (y < 0) return (x - 1.3 * sizeScale);
		penDist = IsOutBoundaryCircle(F2(x, y), F2(0, 0), r1); if (penDist < 0) return penDist;
		penDist = IsInBoundaryCircle(F2(x, y), F2(0, 1.0 * sizeScale), r3); if (penDist < 0) return penDist;
	} else if (x >= 1.3 * sizeScale && x < 2.0 * sizeScale) {
		if (y > 1.0 * sizeScale) { penDist = IsInBoundaryCircle(F2(x, y), F2(0, 1.0 * sizeScale), r3); if (penDist < 0) return penDist; }
		else if (y < 0) { penDist = IsInBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist; }
	} else if (x >= 2.0 * sizeScale && x < 2.6 * sizeScale) {
		if (y < .55 * sizeScale) {
			penDist = IsInBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist;
			penDist = IsOutBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, .55 * sizeScale), r4); if (penDist < 0) return penDist; }
		else if (y < 2 * sizeScale) { penDist = IsOutBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, y), r4); if (penDist < 0) return penDist; }
		else return largePenet;
	} else if (x >= 2.6 * sizeScale && x < 3.3 * sizeScale) {
		if (y > 1.0 * sizeScale) { penDist = IsInBoundaryCircle(F2(x, y), F2(4.6 * sizeScale, 1.0 * sizeScale), r3); if (penDist < 0) return penDist; }
		else if (y < 0) { penDist = IsInBoundaryCircle(F2(x, y), F2(2.3 * sizeScale, 0), r2); if (penDist < 0) return penDist; }
	} else if (x >= 3.3 * sizeScale && x < 4.6 * sizeScale) {
		if (y < -3 * toleranceZone) return largePenet;
		if (y < 0) return 3.3 * sizeScale - x;
		penDist = IsOutBoundaryCircle(F2(x, y), F2(4.6 * sizeScale, 0), r1); if (penDist < 0) return penDist;
		penDist = IsInBoundaryCircle(F2(x, y), F2(4.6 * sizeScale, 1.0 * sizeScale), r3); if (penDist < 0) return penDist;
	}
	if (!isOut)
		return -largePenet;
	return penDist2;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
float IsInsideStraightChannel(float4 posRad) {
	const float sphR = posRad.w;
	float penDist1 = 0;
	float penDist2 = 0;
	//const float toleranceZone = 2 * sphR;
	float largePenet = -5 * sphR;		//like a large number. Should be negative (assume large initial penetration)

	if (posRad.z > straightChannelMax.z) {
		penDist1 = straightChannelMax.z - posRad.z;
	}
	if (posRad.z < straightChannelMin.z) {
		penDist1 = posRad.z - straightChannelMin.z;
	}

	if (posRad.y < straightChannelMin.y) {
		penDist2 = posRad.y - straightChannelMin.y;
	}
	if (posRad.y > straightChannelMax.y) {
		penDist2 = straightChannelMax.y - posRad.y;
	}
	if (penDist1 < 0 && penDist2 < 0) {
		return Min(penDist1, penDist2);
	}
	if (penDist1 < 0) return penDist1;
	if (penDist2 < 0) return penDist2;
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
float IsInsideStraightChannel_XZ(float4 posRad) {
	const float sphR = posRad.w;
	float penDist1 = 0;
	//const float toleranceZone = 2 * sphR;
	float largePenet = -5 * sphR;		//like a large number. Should be negative (assume large initial penetration)

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
float IsInsideTube(float4 posRad, float3 cMax, float3 cMin, float channelRadius) {
	const float sphR = posRad.w;
	float penDist1 = 0;
	//const float toleranceZone = 2 * sphR;
	float largePenet = -5 * sphR; //like a large number. Should be negative (assume large initial penetration)
	float2 centerLine = F2(.5 * (cMin.y + cMax.y), .5 * (cMin.z + cMax.z));
	float r = length(F2(posRad.y, posRad.z) - centerLine);
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
int2 CreateFluidParticles(
		thrust::host_vector<float4> & mPosRad,
		thrust::host_vector<float4> & mVelMas,
		thrust::host_vector<float4> & mRhoPresMu,
		thrust::host_vector<float4> & mPosRadBoundary,
		thrust::host_vector<float4> & mVelMasBoundary,
		thrust::host_vector<float4> & mRhoPresMuBoundary,
		const thrust::host_vector<float3> & rigidPos,
		const thrust::host_vector<float3> & ellipsoidRadii,
		float sphR,
		float3 cMax,
		float3 cMin,
		float rho,
		float pres,
		float mu,
		float & channelRadius) {
	float2 rad2 = .5 * F2(cMax.y - cMin.y, cMax.z - cMin.z);
	channelRadius = (rad2.x < rad2.y) ? rad2.x : rad2.y;
	channelRadius = 1.0 * sizeScale; //tube
	int num_FluidParticles = 0;
	int num_BoundaryParticles = 0;
	srand(964);
	//float initSpace0 = 0.9 * sphR; //1.1 * sphR;//1.1 * sphR;//pow(4.0 / 3 * PI, 1.0 / 3) * sphR;
	float initSpace0 = 1.0 * sphR;
	int nFX = ceil((cMax.x - cMin.x) / (initSpace0));
	float initSpaceX = (cMax.x - cMin.x) / nFX;
	//printf("orig nFx and nFx %f %f\n", (cMax.x - cMin.x) / initSpace, ceil ((cMax.x - cMin.x) / (initSpace)));
	int nFY = ceil((cMax.y - cMin.y) / (initSpace0));
	float initSpaceY = (cMax.y - cMin.y) / nFY;
	int nFZ = ceil((cMax.z - cMin.z) / (initSpace0));
	float initSpaceZ = (cMax.z - cMin.z) / nFZ;
	//printf("&&&&& %f   %f %f %f \n", 1.1 * sphR, initSpaceX, initSpaceY, initSpaceZ);
	printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ, (nFY - 1) * initSpaceY, initSpaceY);
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
				float4 posRad;
//					printf("initSpace X, Y, Z %f %f %f \n", initSpaceX, initSpaceY, initSpaceZ);
				posRad = F4(
						cMin + make_float3(i * initSpaceX, j * initSpaceY, k * initSpaceZ)
								+ make_float3(.5 * initSpace0)/* + make_float3(sphR) + initSpace * .05 * (float(rand()) / RAND_MAX)*/, sphR);
				float penDist = 0;
				bool flag = true;
				///penDist = IsInsideCurveOfSerpentine(posRad);
				///penDist = IsInsideSerpentine(posRad);
				penDist = IsInsideStraightChannel(posRad);
				///penDist = IsInsideStraightChannel_XZ(posRad);
				///penDist = IsInsideTube(posRad, cMax, cMin, channelRadius);
				if (penDist < -toleranceZone) flag = false;
				if (flag) {
					for (int rigidSpheres = 0; rigidSpheres < rigidPos.size(); rigidSpheres++) {
						if (IsInsideEllipsoid(posRad, rigidPos[rigidSpheres], ellipsoidRadii[rigidSpheres], initSpace0)) { flag = false; }
//						if ( IsInsideCylinder_XZ(posRad, rigidPos[rigidSpheres], ellipsoidRadii[rigidSpheres], initSpace0 ) ) { flag = false;}
					}
				}
				if (flag) {
					if (penDist > 0) {
						num_FluidParticles++;
						mPosRad.push_back(posRad);
						mVelMas.push_back(F4(0, 0, 0, (initSpaceX * initSpaceY * initSpaceZ) * rho));
						mRhoPresMu.push_back(F4(rho, pres, mu, -1)); //rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
																	//just note that the type, i.e. mRhoPresMu.w is float.
																	//viscosity of the water is .0018

																	//if (posRad.x < .98 * cMax.x && posRad.x > .96 * cMax.x && posRad.y > .48 * sizeScale &&  posRad.y < .52 * sizeScale
																	//	&& posRad.z > 1.7 * sizeScale &&  posRad.y < 1.75 * sizeScale) {
																	//	if (num_FluidParticles < 1) {
																	//		num_FluidParticles ++;
																	//		mPosRad.push_back(posRad);
																	//		mVelMas.push_back( F4(0, 0, 0, pow(initSpace, 3) * rho) );
																	//		mRhoPresMu.push_back(F4(rho, pres, mu, -1));		//rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
																	//															//just note that the type, i.e. mRhoPresMu.w is float.
																	//															//viscosity of the water is .0018
																	//	}
																	//}
					} else {
						num_BoundaryParticles++;
						mPosRadBoundary.push_back(posRad);
						mVelMasBoundary.push_back(F4(0, 0, 0, (initSpaceX * initSpaceY * initSpaceZ) * rho));
						mRhoPresMuBoundary.push_back(F4(rho, pres, mu, 0));	//rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
						//just note that the type, i.e. mRhoPresMu.w is float.
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
		thrust::host_vector<float4> & mPosRad,
		thrust::host_vector<float4> & mVelMas,
		thrust::host_vector<float4> & mRhoPresMu,
		float3 rigidPos,
		float3 ellipsoidRadii,
		float4 sphereVelMas,
		float3 rigidBodyOmega,
		float sphR,
		float rho,
		float pres,
		float mu,
		float3 cMin,
		float3 cMax,
		int type) {
	int num_rigidBodyParticles = 0;
	//float spacing = .9 * sphR;
	float spacing = 1.0 * sphR;
	for (int k = 0; k < 3; k++) {
		float3 r3 = ellipsoidRadii - F3(k * spacing);
		//printf("r, rigidR, k*spacing %f %f %f\n", r * 1000000, spherePosRad.w * 1000000, k * spacing * 1000000);
		float minR = r3.x;
		minR = (minR < r3.y) ? minR : r3.y;
		minR = (minR < r3.z) ? minR : r3.z;
		if ( minR > 0) {
			float deltaTeta0 = spacing / r3.z;
			float teta = 0.1 * deltaTeta0;
			while (teta < PI - .1 * deltaTeta0) {
				float deltaPhi0 = spacing / (r3.z * sin(teta));
				float phi = 0.1 * deltaPhi0;
				float currentR = 1 / length(F3(sin(teta) * cos(phi), sin(teta) * sin(phi), cos(teta)) / r3);
				float nextR = 1 / length(F3(sin(teta + spacing / currentR) * cos(phi), sin(teta + spacing / currentR) * sin(phi), cos(teta + spacing / currentR)) / r3);
				float deltaTeta = spacing / max(currentR, nextR);
				while (phi < 2 * PI + deltaPhi0) {
					float3 mult3 = F3(sin(teta) * cos(phi), sin(teta) * sin(phi), cos(teta)) / r3;
					float r = 1 / length(mult3);
					float4 posRadRigid_sphParticle = F4(r * sin(teta) * cos(phi), r * sin(teta) * sin(phi), r * cos(teta), sphR)
							+ F4(rigidPos, 0);
					mPosRad.push_back(posRadRigid_sphParticle);
					float deltaPhiDum = spacing / (r * sin(teta));
					float rNewDum = 1 / length(F3(sin(teta) * cos(phi + deltaPhiDum), sin(teta) * sin(phi + deltaPhiDum), cos(teta)) / r3);
					float maxR = max(rNewDum, r);
					phi += spacing / (maxR * sin(teta));

					float3 vel = F3(sphereVelMas) + cross(rigidBodyOmega, F3(posRadRigid_sphParticle) - rigidPos); //assuming LRF is the same as GRF at time zero (rigibodyOmega is in LRF, the second term of the cross is in GRF)
					//printf("veloc %f %f %f\n", vel.x, vel.y, vel.z);
					mVelMas.push_back(F4(vel, pow(spacing, 3) * rho));
					float representedArea = spacing * spacing;
					mRhoPresMu.push_back(F4(rho, pres, mu, type));					// for rigid body particle, rho represents the represented area
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
		thrust::host_vector<float4> & mPosRad,
		thrust::host_vector<float4> & mVelMas,
		thrust::host_vector<float4> & mRhoPresMu,
		float3 rigidPos,
		float3 ellipsoidRadii,
		float4 sphereVelMas,
		float3 rigidBodyOmega,
		float sphR,
		float rho,
		float pres,
		float mu,
		float3 cMin,
		float3 cMax,
		int type) {
	int num_rigidBodyParticles = 0;
	float spacing = .9 * sphR;
	for (int k = 0; k < 3; k++) {
		float r = ellipsoidRadii.x - k * spacing;
		if (r > 0) {
			float deltaTeta = spacing / r;
			for (float teta = .1 * deltaTeta; teta < 2 * PI - .1 * deltaTeta; teta += deltaTeta) {
				//printf("gher %f, %f\n", deltaTeta / 2 / PI, deltaTeta);
				for (float y = cMin.y; y < cMax.y - .1 * spacing; y += spacing) {
					//printf("aha\n");
					float4 posRadRigid_sphParticle = F4(r * cos(teta), y, r * sin(teta), sphR) + F4(rigidPos.x, 0, rigidPos.z, 0);

					mPosRad.push_back(posRadRigid_sphParticle);
					float3 vel = F3(sphereVelMas) + cross(rigidBodyOmega, F3(posRadRigid_sphParticle) - rigidPos); //assuming LRF is the same as GRF at time zero (rigibodyOmega is in LRF, the second term of the cross is in GRF)
					mVelMas.push_back(F4(vel, pow(spacing, 3) * rho));
					float representedArea = spacing * spacing;
					mRhoPresMu.push_back(F4(rho, pres, mu, type));					// for rigid body particle, rho represents the represented area
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
	printf("chaghal\n");
	//(void) cudaSetDevice(0);
	int numAllParticles = 0;
	//********************************************************************************************************
	//** Initialization
	float r = HSML;	//.02;

	float3 cMin = make_float3(0, -0.2, -1.2) * sizeScale; 							//for channel and serpentine
	//float3 cMin = make_float3(0, -0.2, -0.2) * sizeScale;							//for tube

	//float3 cMax = make_float3( nPeriod * 4.6 + 0, 1.5,  4.0) * sizeScale;  //for only CurvedSerpentine (w/out straight part)
	//float3 cMax = make_float3( nPeriod * 4.6 + 7, 1.5,  4.0) * sizeScale;  //for serpentine
	float3 cMax = make_float3( nPeriod * 2.0 + 0, 1.5,  4.0) * sizeScale;  //for  straight channel
	//float3 cMax = make_float3( nPeriod * 2.0 + 0, 2.2,  2.2) * sizeScale;  //for  tube

//	float3 cMax = make_float3(nPeriod * 1.0 + 0, .5,  3.5) * sizeScale;  //for straight channel, sphere
//	float3 cMin = make_float3(0, -0.1, 0.5) * sizeScale;
//	float3 cMax = make_float3(nPeriod * 1.0 + 0, 1.5, 1.5) * sizeScale;  //for tube channel, sphere
//	float3 cMin = make_float3(0, -0.5, -0.5) * sizeScale;

	straightChannelMin = F3(cMin.x, 0.0 * sizeScale, 1.0 * sizeScale);
	straightChannelMax = F3(cMax.x, 1.0 * sizeScale, 3.0 * sizeScale);
	//printf("a1  cMax.x, y, z %f %f %f,  binSize %f\n", cMax.x, cMax.y, cMax.z, 2 * HSML);
	int3 side0 = I3(floor((cMax.x - cMin.x) / (2 * HSML)), floor((cMax.y - cMin.y) / (2 * HSML)), floor((cMax.z - cMin.z) / (2 * HSML)));
	float3 binSize3 = make_float3((cMax.x - cMin.x) / side0.x, (cMax.y - cMin.y) / side0.y, (cMax.z - cMin.z) / side0.z);
	float binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
	binSize0 = (binSize0 > binSize3.z) ? binSize0 : binSize3.z;
	cMax = cMin + binSize0 * make_float3(side0);
	printf("a2  cMax.x, y, z %f %f %f,  binSize %f\n", cMax.x, cMax.y, cMax.z, binSize0);
	//printf("side0 %d %d %d \n", side0.x, side0.y, side0.z);

	float delT = .02 * sizeScale;
//	float delT = .001 * sizeScale;

	bool readFromFile = false;  //true;		//true: initializes from file. False: initializes inside the code

	float pres = BASEPRES;  //0;//100000;
	float mu = mu0;
	float rhoRigid;
	//1---------------------------------------------- Initialization ----------------------------------------
	//2------------------------------------------- Generating Random Data -----------------------------------
	// This section can be used to generate ellipsoids' specifications and save them into data.txt	
	//num_FluidParticles = 40000;//5000;//10000;//4096;//16384;//65536;//8192;//32768;;//262144;//1024;//262144;//1024; //262144;//1000000;//16 * 1024;//262144;//1024;//262144;//1024;//262144;//1024 * 16;//262144;//4194304;//262144;//1024;//4194304; //2097152;//262144;//262144;//1024;//65536;//262144;	// The number of bodies

	//--------------------buffer initialization ---------------------------
	thrust::host_vector<int3> referenceArray;
	thrust::host_vector<float4> mPosRad;			//do not set the size here since you are using push back later
	thrust::host_vector<float4> mVelMas;
	thrust::host_vector<float4> mRhoPresMu;

	//thrust::host_vector<float4> spheresPosRad;
	thrust::host_vector<float3> rigidPos;
	thrust::host_vector<float3> ellipsoidRadii;
	thrust::host_vector<float4> spheresVelMas;
	thrust::host_vector<float3> rigidBodyOmega;
	thrust::host_vector<float3> rigidBody_J1;
	thrust::host_vector<float3> rigidBody_J2;
	thrust::host_vector<float3> rigidBody_InvJ1;
	thrust::host_vector<float3> rigidBody_InvJ2;

	////***** here: define rigid bodies
	string fileNameRigids("spheresPos.dat");
	rhoRigid = 1000; //1050; //originally .079 //.179 for cylinder
	float channelRadius;
	//CreateRigidBodiesFromFile(rigidPos, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, cMin, cMax, fileNameRigids, rhoRigid, channelRadius);
	float3 r3Ellipsoid = F3(.03 * sizeScale);
	CreateRigidBodiesPattern(rigidPos, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, ellipsoidRadii, r3Ellipsoid, rhoRigid, channelRadius);

	printf("size rigids %d\n", rigidPos.size());
	//---------------------------------------------------------------------
	// initialize fluid particles
	if (readFromFile) {
		int num_FluidParticles = 0;
		fstream inp("initializer.txt", ios::in);
		inp >> num_FluidParticles;
		for (int i = 0; i < num_FluidParticles; i++) {
			char dummyCh;
			float4 posRad = make_float4(0);
			float4 velMas = make_float4(0);
			float4 rhoPresMu = make_float4(0);
			inp >> posRad.x >> dummyCh >> posRad.y >> dummyCh >> posRad.z >> dummyCh >> posRad.w >> dummyCh >> velMas.x >> dummyCh >> velMas.y
					>> dummyCh >> velMas.z >> dummyCh >> velMas.w >> dummyCh >> rhoPresMu.x >> dummyCh >> rhoPresMu.y >> dummyCh >> rhoPresMu.z
					>> dummyCh >> rhoPresMu.w;

			rhoPresMu.z = mu;

			mPosRad.push_back(posRad);
			mVelMas.push_back(velMas);
			mRhoPresMu.push_back(rhoPresMu);
		}
		//num_FluidParticles *= 2;
		referenceArray.push_back(I3(0, num_FluidParticles, -1));
	} else {
		thrust::host_vector<float4> mPosRadBoundary;			//do not set the size here since you are using push back later
		thrust::host_vector<float4> mVelMasBoundary;
		thrust::host_vector<float4> mRhoPresMuBoundary;

		int2 num_fluidOrBoundaryParticles = CreateFluidParticles(mPosRad, mVelMas, mRhoPresMu, mPosRadBoundary, mVelMasBoundary, mRhoPresMuBoundary,
				rigidPos, ellipsoidRadii, r, cMax, cMin, rho0, pres, mu, channelRadius);
		referenceArray.push_back(I3(0, num_fluidOrBoundaryParticles.x, -1));
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
		mPosRadBoundary.clear();
		mVelMasBoundary.clear();
		mRhoPresMuBoundary.clear();

		referenceArray.push_back(I3(numAllParticles, numAllParticles + num_fluidOrBoundaryParticles.y, 0));
		numAllParticles += num_fluidOrBoundaryParticles.y;
		printf("num_BoundaryParticles: %d\n", num_fluidOrBoundaryParticles.y);

		//rigid body: type = 1, 2, 3, ...
		printf("num_RigidBodyParticles: \n");
		for (int rigidSpheres = 0; rigidSpheres < rigidPos.size(); rigidSpheres++) {
			int num_RigidBodyParticles = CreateEllipsoidParticles(mPosRad, mVelMas, mRhoPresMu, rigidPos[rigidSpheres], ellipsoidRadii[rigidSpheres], spheresVelMas[rigidSpheres],
					rigidBodyOmega[rigidSpheres], r, rho0, pres, mu, cMin, cMax, rigidSpheres + 1);		//as type
//			int num_RigidBodyParticles = CreateCylinderParticles_XZ(mPosRad, mVelMas, mRhoPresMu,
//													rigidPos[rigidSpheres], ellipsoidRadii[rigidSpheres],
//													 spheresVelMas[rigidSpheres],
//													 rigidBodyOmega[rigidSpheres],
//													 r,
//													 rho0, pres, mu,
//													 cMin, cMax,
//													 rigidSpheres + 1);		//as type
			referenceArray.push_back(I3(numAllParticles, numAllParticles + num_RigidBodyParticles, rigidSpheres + 1));
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
		cudaCollisions(mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray, numAllParticles, cMax, cMin, delT, rigidPos, spheresVelMas,
				rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, binSize0, channelRadius);
	}
	mPosRad.clear();
	mVelMas.clear();
	mRhoPresMu.clear();
	bodyIndex.clear();
	referenceArray.clear();
	rigidPos.clear();
	ellipsoidRadii.clear();
	spheresVelMas.clear();
	rigidBodyOmega.clear();
	rigidBody_J1.clear();
	rigidBody_J2.clear();
	rigidBody_InvJ1.clear();
	rigidBody_InvJ2.clear();
	return 0;
}
