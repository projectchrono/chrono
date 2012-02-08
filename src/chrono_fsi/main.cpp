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
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
float Min(float a, float b) {
	return (a < b) ? a : b;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideSphere(float4 sphParPos, float4 spherePosRad, float clearance) {
	float3 dist3 = make_float3(sphParPos - spherePosRad);
	if (length(dist3) < spherePosRad.w + clearance) {
		return true;
	} else {
		return false;
	}
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool IsInsideCylinder_XZ(float4 sphParPos, float4 spherePosRad, float clearance) {
	float3 dist3 = make_float3(sphParPos) - make_float3(spherePosRad.x, sphParPos.y, spherePosRad.z);
	//if (length(dist3) < spherePosRad.w + 2 * sphParPos.w) {
	if (length(dist3) < spherePosRad.w + clearance) {
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
float IsInsideStraightChannel(float4 posRad) {
	const float sphR = posRad.w;
	float penDist1 = 0; 
	float penDist2 = 0; 
	//const float toleranceZone = 2 * sphR;
	float largePenet = -5*sphR;//like a large number. Should be negative (assume large initial penetration)

	if (posRad.z > 3.0 * sizeScale) {penDist1 = 3.0 * sizeScale - posRad.z;}
	if (posRad.z < 1.0 * sizeScale) {penDist1 = posRad.z - 1.0 * sizeScale;}

	if (posRad.y < 0) {penDist2 = posRad.y;}
	if (posRad.y > 1.0 * sizeScale) {penDist2 = 1.0 * sizeScale - posRad.y;}
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
	float penDist2 = 0; 
	//const float toleranceZone = 2 * sphR;
	float largePenet = -5*sphR;//like a large number. Should be negative (assume large initial penetration)

	//if (posRad.z > 3.0 * sizeScale) {penDist1 = 3.0 * sizeScale - posRad.z;}
	if (posRad.z > 2.0 * sizeScale) {penDist1 = 2.0 * sizeScale - posRad.z;}
	if (posRad.z < 1.0 * sizeScale) {penDist1 = posRad.z - 1.0 * sizeScale;}

	if (penDist1 < 0) return penDist1;
	//printf("hey \n");
	return -largePenet;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int2 CreateFluidParticles(	thrust::host_vector<float4> & mPosRad,
							thrust::host_vector<float4> & mVelMas,
							thrust::host_vector<float4> & mRhoPresMu,
							thrust::host_vector<float4> & mPosRadBoundary,
							thrust::host_vector<float4> & mVelMasBoundary,
							thrust::host_vector<float4> & mRhoPresMuBoundary,
							const thrust::host_vector<float4> & spheresPosRad,
							float sphR,
							float3 cMax, float3 cMin,
							float rho, float pres, float mu
							) {
		int num_FluidParticles = 0;
		int num_BoundaryParticles = 0;
		srand(964);
		float initSpace0 = 0.9 * sphR;//1.1 * sphR;//1.1 * sphR;//pow(4.0 / 3 * PI, 1.0 / 3) * sphR;
		int nFX = ceil ((cMax.x - cMin.x) / (initSpace0)) ;
		float initSpaceX = (cMax.x - cMin.x) / nFX;
		//printf("orig nFx and nFx %f %f\n", (cMax.x - cMin.x) / initSpace, ceil ((cMax.x - cMin.x) / (initSpace)));
		int nFY = ceil ((cMax.y - cMin.y) / (initSpace0)) ;
		float initSpaceY = (cMax.y - cMin.y) / nFY;
		int nFZ = ceil ((cMax.z - cMin.z) / (initSpace0)) ;
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

		for(int i=0; i < nFX; i++) {
			for (int j=0; j < nFY; j++) {
				for (int k=0; k < nFZ; k++) {
					float4 posRad;
//					printf("initSpace X, Y, Z %f %f %f \n", initSpaceX, initSpaceY, initSpaceZ);
					posRad = F4(cMin + make_float3(i * initSpaceX, j * initSpaceY, k * initSpaceZ) + make_float3(.5 * initSpace0)/* + make_float3(sphR) + initSpace * .05 * (float(rand()) / RAND_MAX)*/, sphR);
					float penDist = 0;
					bool flag = true;
					///penDist = IsInsideSerpentine(posRad); if (penDist < -toleranceZone) flag= false;
					///penDist = IsInsideStraightChannel(posRad); if (penDist < -toleranceZone) flag= false;
					penDist = IsInsideStraightChannel_XZ(posRad); if (penDist < -toleranceZone) flag= false;
					if (flag) {
							for (int rigidSpheres = 0; rigidSpheres < spheresPosRad.size(); rigidSpheres ++) {
//								if ( IsInsideSphere(posRad, spheresPosRad[rigidSpheres], initSpace0 ) ) { flag = false;}
								if ( IsInsideCylinder_XZ(posRad, spheresPosRad[rigidSpheres], initSpace0 ) ) { flag = false;}
						}
					}
					if (flag) {	
						if (penDist > 0) {					
							num_FluidParticles ++;
							mPosRad.push_back(posRad);
							mVelMas.push_back( F4(0, 0, 0, (initSpaceX * initSpaceY * initSpaceZ) * rho) );						
							mRhoPresMu.push_back(F4(rho, pres, mu, -1));		//rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
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
							num_BoundaryParticles ++;
							mPosRadBoundary.push_back(posRad);
							mVelMasBoundary.push_back( F4(0, 0, 0, (initSpaceX * initSpaceY * initSpaceZ) * rho) );						
							mRhoPresMuBoundary.push_back(F4(rho, pres, mu, 0));			//rho, pressure, viscosity for water at standard condition, last component is the particle type: -1: fluid, 0: boundary, 1, 2, 3, .... rigid bodies.
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
int CreateSphereParticles(thrust::host_vector<float4> & mPosRad,
				 thrust::host_vector<float4> & mVelMas,
				 thrust::host_vector<float4> & mRhoPresMu,
				 float4 spherePosRad,
				 float4 sphereVelMas,
				 float3 rigidBodyOmega,
				 float sphR,
				 float rho, float pres, float mu,
				 float3 cMin, float3 cMax,
				 int type) {
	int num_rigidBodyParticles = 0;
	float spacing = .9 * sphR;
	for (int k = 0; k < 3; k++) {
		float r = spherePosRad.w - k * spacing;
		//printf("r, rigidR, k*spacing %f %f %f\n", r * 1000000, spherePosRad.w * 1000000, k * spacing * 1000000);
		if (r > 0) {
			float deltaTeta = spacing / r;
			for (float teta = .1 * deltaTeta; teta < PI - .1 * deltaTeta; teta += deltaTeta) {
				float deltaPhi = spacing / (r * sin(teta));
				for (float phi = 0; phi < 2 * PI + deltaPhi; phi += deltaPhi) {
					float4 posRadRigid_sphParticle = F4(r * sin(teta) * cos(phi), r * sin(teta) * sin(phi), r * cos(teta), sphR)
											+ F4( F3(spherePosRad), 0);

					mPosRad.push_back(posRadRigid_sphParticle);
					float3 vel = F3(sphereVelMas) + cross( rigidBodyOmega, F3(posRadRigid_sphParticle - spherePosRad) ); //assuming LRF is the same as GRF at time zero (rigibodyOmega is in LRF, the second term of the cross is in GRF)
					mVelMas.push_back( F4(vel, pow(spacing, 3) * rho) );
					float representedArea = spacing * spacing;
					mRhoPresMu.push_back(F4(rho, pres, mu, type));						// for rigid body particle, rho represents the represented area
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
int CreateCylinderParticles_XZ(thrust::host_vector<float4> & mPosRad,
				 thrust::host_vector<float4> & mVelMas,
				 thrust::host_vector<float4> & mRhoPresMu,
				 float4 spherePosRad, 
				 float4 sphereVelMas,
				 float3 rigidBodyOmega,
				 float sphR,
				 float rho, float pres, float mu,
				 float3 cMin, float3 cMax,
				 int type) {
	int num_rigidBodyParticles = 0;
	float spacing = .9 * sphR;
	for (int k = 0; k < 3; k++) {
		float r = spherePosRad.w - k * spacing;
		//printf("r, rigidR, k*spacing %f %f %f\n", r * 1000000, spherePosRad.w * 1000000, k * spacing * 1000000);
		if (r > 0) {
			float deltaTeta = spacing / r;
			for (float teta = .1 * deltaTeta; teta < 2 * PI - .1 * deltaTeta; teta += deltaTeta) {
				//printf("gher %f, %f\n", deltaTeta / 2 / PI, deltaTeta);
				for (float y = cMin.y; y < cMax.y - .1 * spacing; y += spacing) {
					//printf("aha\n");
					float4 posRadRigid_sphParticle = F4(r * cos(teta), y, r * sin(teta), sphR)
						+ F4(spherePosRad.x, 0, spherePosRad.z, 0);

					mPosRad.push_back(posRadRigid_sphParticle);
					float3 vel = F3(sphereVelMas) + cross( rigidBodyOmega, F3(posRadRigid_sphParticle - spherePosRad) ); //assuming LRF is the same as GRF at time zero (rigibodyOmega is in LRF, the second term of the cross is in GRF)
					mVelMas.push_back( F4(vel, pow(spacing, 3) * rho) );
					float representedArea = spacing * spacing;
					mRhoPresMu.push_back(F4(rho, pres, mu, type));						// for rigid body particle, rho represents the represented area
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
	float r = HSML;//.02;
	float rRigidBody;
	//float3 cMax = make_float3( nPeriod * 4.6 + 7, 1.0,  4.0) * sizeScale;  //for serpentine
	//float3 cMax = make_float3( nPeriod * 4.6 + 0, 1.0,  4.0) * sizeScale;  //for straight channel
	//float3 cMax = make_float3( nPeriod * 4.6 + 0, .4,  4.0) * sizeScale;  //for straight channel, cylinders
	//float3 cMin = make_float3(0, -0.1, -1.2) * sizeScale;
	//float3 cMax = make_float3(nPeriod * 1.0 + 0, .3,  3.5) * sizeScale;  //for straight channel, cylinders
	float3 cMax = make_float3(nPeriod * 1.0 + 0, .5,  3.5) * sizeScale;  //for straight channel, sphere
	float3 cMin = make_float3(0, -0.1, 0.5) * sizeScale;


	//printf("a1  cMax.x, y, z %f %f %f,  binSize %f\n", cMax.x, cMax.y, cMax.z, 2 * HSML);
	int3 side0 = I3(floor( (cMax.x - cMin.x) / (2 * HSML) ), floor( (cMax.y - cMin.y) / (2 * HSML) ), floor( (cMax.z - cMin.z) / (2 * HSML) ));
	float3 binSize3 = make_float3( (cMax.x - cMin.x) / side0.x, (cMax.y - cMin.y) / side0.y, (cMax.z - cMin.z) / side0.z );
	float binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
	binSize0 = (binSize0 > binSize3.z)? binSize0 : binSize3.z;
	cMax = cMin + binSize0 * make_float3(side0);
	printf("a2  cMax.x, y, z %f %f %f,  binSize %f\n", cMax.x, cMax.y, cMax.z, binSize0);
	//printf("side0 %d %d %d \n", side0.x, side0.y, side0.z);


	//float delT = .02 * sizeScale;
	float delT = .02* sizeScale;
	//float delT = .00001;
	bool readFromFile = false;//true;		//true: initializes from file. False: initializes inside the code
	
	float pres = BASEPRES;//0;//100000;
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

	thrust::host_vector<float4> spheresPosRad;
	thrust::host_vector<float4> spheresVelMas;
	thrust::host_vector<float3> rigidBodyOmega;
	thrust::host_vector<float3> rigidBody_J1;
	thrust::host_vector<float3> rigidBody_J2;
	thrust::host_vector<float3> rigidBody_InvJ1;
	thrust::host_vector<float3> rigidBody_InvJ2;

	////***** here: define rigid bodies
	string fileNameRigids("spheresPos.dat");
	fstream ifileSpheres(fileNameRigids.c_str(), ios::in);
	rRigidBody = .125 * sizeScale; // .25 * sizeScale; //.06 * sizeScale;//.08 * sizeScale;//.179 * sizeScale;
	rhoRigid = 1000;//1050; //originally .079 //.179 for cylinder
	float x, y, z;
	char ch;
	ifileSpheres>>x>>ch>>y>>ch>>z;
	while (!ifileSpheres.eof()) {
	//float r = rRigidBody * (.75 + .75 * float(rand())/RAND_MAX);
		for (int period = 0; period < nPeriod; period++) {		
			spheresPosRad.push_back(F4(x * sizeScale + period * sPeriod, y * sizeScale, z * sizeScale, rRigidBody));
			//********** intitialization of rigid bodies: Spheres
//			float mass = 4.0 / 3 * PI * pow(rRigidBody, 3) * rhoRigid;			//for sphere
//			float3 j1, j2;
//			j1 = F3(.4 * mass * pow(rRigidBody, 2), 0, 0);
//			j2 = F3(.4 * mass * pow(rRigidBody, 2), 0, .4 * mass * pow(rRigidBody, 2));
			//****************************************************
			//********** intitialization of rigid bodies: Cylinders
			float mass = PI * pow(rRigidBody, 2) * (cMax.y - cMin.y) * rhoRigid;	//for cylinder
			float3 j1, j2;
			j1 = F3(1.0 / 12.0  * mass * (3 * pow(rRigidBody, 2) + pow(cMax.y - cMin.y, 2)), 0, 0);
			j2 = F3(.5 * mass * pow(rRigidBody, 2), 0, 1.0 / 12.0  * mass * (3 * pow(rRigidBody, 2) + pow(cMax.y - cMin.y, 2)));
//			//****************************************************
			spheresVelMas.push_back( F4(0, 0, 0, mass) );
			rigidBodyOmega.push_back( F3(0, 0, 0) );
			rigidBody_J1.push_back(j1);
			rigidBody_J2.push_back(j2);
			float detJ = 2 * j1.z * j1.y * j2.y - j1.z * j1.z * j2.x - j1.y * j1.y * j2.z + j1.x * j2.x * j2.z - j1.x * j2.y * j2.y;
			float3 invJ1 = F3(j2.x * j2.z - j2.y * j2.y, -j1.y * j2.z + j1.z * j2.y,  j1.y * j2.y - j1.z * j2.x);
			float3 invJ2 = F3(-j1.z * j1.z + j1.x * j2.z, -j1.x * j2.y + j1.z * j1.y, -j1.y * j1.y + j1.x * j2.x);
			rigidBody_InvJ1.push_back(invJ1 / detJ);
			rigidBody_InvJ2.push_back(invJ2 / detJ);
		}
		ifileSpheres>>x>>ch>>y>>ch>>z;
	}
//	printf("*********************************** J/Me6 %f \n",  .5  * pow(rRigidBody, 2) * 1e6);
	ifileSpheres.close();
	printf("size rigids %d\n", spheresPosRad.size());
	//---------------------------------------------------------------------
	// initialize fluid particles
	if (readFromFile) {
		int num_FluidParticles = 0;
		fstream inp("initializer.txt", ios::in);
		inp>>num_FluidParticles;
		for(int i=0; i < num_FluidParticles; i++) { 
			char dummyCh;
			float4 posRad = make_float4(0);
			float4 velMas = make_float4(0);
			float4 rhoPresMu = make_float4(0);
			inp	>> posRad.x >> dummyCh >> posRad.y >> dummyCh >> posRad.z >> dummyCh >> posRad.w >> dummyCh
				>> velMas.x >> dummyCh >> velMas.y >> dummyCh >> velMas.z >> dummyCh >> velMas.w >> dummyCh
				>> rhoPresMu.x >> dummyCh >> rhoPresMu.y >> dummyCh >> rhoPresMu.z >> dummyCh >> rhoPresMu.w;

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

		int2 num_fluidOrBoundaryParticles = CreateFluidParticles(mPosRad, mVelMas, mRhoPresMu, 
										mPosRadBoundary, mVelMasBoundary, mRhoPresMuBoundary,
										spheresPosRad,
										r, cMax, cMin,
										rho0, pres, mu);
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
		for (int rigidSpheres = 0; rigidSpheres < spheresPosRad.size(); rigidSpheres ++) {
//			int num_RigidBodyParticles = CreateSphereParticles(mPosRad, mVelMas, mRhoPresMu,
//													 spheresPosRad[rigidSpheres],
//													 spheresVelMas[rigidSpheres],
//													 rigidBodyOmega[rigidSpheres],
//													 r,
//													 rho0, pres, mu,
//													 cMin, cMax,
//													 rigidSpheres + 1);		//as type
			int num_RigidBodyParticles = CreateCylinderParticles_XZ(mPosRad, mVelMas, mRhoPresMu,
													 spheresPosRad[rigidSpheres],
													 spheresVelMas[rigidSpheres],
													 rigidBodyOmega[rigidSpheres],
													 r,
													 rho0, pres, mu,
													 cMin, cMax,
													 rigidSpheres + 1);		//as type
			referenceArray.push_back(I3(numAllParticles, numAllParticles + num_RigidBodyParticles, rigidSpheres + 1));
			numAllParticles += num_RigidBodyParticles;
			printf(" %d \n", num_RigidBodyParticles);
		}
		printf("\n");
	}
	
	

	//@@@@@@@@ rigid body

	thrust::host_vector<uint> bodyIndex(numAllParticles);
	thrust::fill(bodyIndex.begin(), bodyIndex.end(), 1);
	thrust::exclusive_scan(bodyIndex.begin(), bodyIndex.end(), bodyIndex.begin());
	printf("numAllParticles %d\n", numAllParticles);

	if(numAllParticles != 0) {
		cudaCollisions(mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray, numAllParticles, cMax, cMin, delT, spheresPosRad, spheresVelMas, rigidBodyOmega, rigidBody_J1, rigidBody_J2, rigidBody_InvJ1, rigidBody_InvJ2, binSize0);
	}
	mPosRad.clear();
	mVelMas.clear();
	mRhoPresMu.clear();
	bodyIndex.clear();
	referenceArray.clear();
	spheresPosRad.clear();
	spheresVelMas.clear();
	rigidBodyOmega.clear();
	rigidBody_J1.clear();
	rigidBody_J2.clear();
	rigidBody_InvJ1.clear();
	rigidBody_InvJ2.clear();
	return 0;
}
