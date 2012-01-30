#include <cutil_math.h>					
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/reduce.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "collideSphereSphere.cuh"
#include "SDKCollisionSystem.cuh"
#include <string.h>
#include <stdio.h>
#include <math.h>
//#include <cudpp/cudpp.h> //you have to copy the cudpp.dll as well , this is just for sorting

//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>//just for min

//#####################################################################################
#define B_SIZE 128

#define I1CAST(x) (int*)thrust::raw_pointer_cast(&x[0])
#define U1CAST(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define F1CAST(x) (float*)thrust::raw_pointer_cast(&x[0])
#define F3CAST(x) (float3*)thrust::raw_pointer_cast(&x[0])
#define F4CAST(x) (float4*)thrust::raw_pointer_cast(&x[0])
#define TCAST(x) thrust::raw_pointer_cast(x.data())

#define LARGE_NUMBER 99999999
#define SMALL_NUMBER -99999999
//#####################################################################################
__constant__ int		mNumSpheresD;
__constant__ float		dTD;
__constant__ int2		updatePortionD;
__constant__ float3		cMinD;
__constant__ float3		cMaxD;
__constant__ int2		portionD;
__constant__ int		flagD;
__constant__ int		numRigidBodiesD;
__constant__ int		startRigidParticleD;
__constant__ int		numRigid_SphParticlesD;

int maxblock = 65535;
//--------------------------------------------------------------------------------------------------------------------------------
//computes dV/dt and dRho/dt, i.e. force terms. First
__device__ inline float4 DifVelocityRho_BoundaryDEM(	const float4 & posRadA, const float4 & posRadB,
											const float4 & velMasA, const float4 & velMasB) {
	//float3 dist3 = F3(posRadA - posRadB);
	////periodic BC on x
	//dist3.x -= ( (dist3.x > 0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0 );
	//dist3.x += ( (dist3.x < -0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0 );
	////
	//float d = length(dist3);//sqrt( dot(dist3, dist3) );
	
	float	kS = 39240.0; //392400.0;	//spring
	float	kD = 4200.0;	//420.0;				//damper
	float3 dist3 = F3(posRadA - posRadB);
	float l = posRadA.w + posRadB.w - length(dist3);
	float3 n = dist3 / length(dist3);					//unit vector B to A
	float m_eff = (velMasA.w * velMasB.w) / (velMasA.w + velMasB.w);
	float3 force = (l < 0) ? F3(0) : ( 
					pow(sizeScale, 3) * kS * l * n - 
					kD * m_eff * dot( F3 (velMasA - velMasB) , n ) * n		//relative velocity at contact is simply assumed as the relative vel of the centers. If you are updating the rotation, this should be modified.
					);
	return F4(force / velMasA.w, 0);
}
//--------------------------------------------------------------------------------------------------------------------------------
//Force terms for the particles close to the boundary
//here, d is center distance not penetration distance (distance of particle's center from rigid wall)
__device__ inline float4 DifVelocityRhoBoundary(	const float3 & n3, float d, 
													const float4 & posRad, const float4 & velMas, const float4 & rhoPresMu) {
	//****** SPH version
	float wB = W3(d, posRad.w) / W2(0, posRad.w);
	float wBMax = W3(0, posRad.w) / W2(0, posRad.w);
	float normV = sqrt(dot(F3(velMas), F3(velMas)));
	float pB = rhoPresMu.y;
	float3 derivV = F3(0);
	float derivRho = 0;
	float mult = (d > 0) ? wB : (2* wBMax - wB);
	if (mult < .000001 /*epsilon*/) {return F4(0);}
	derivV = n3 * ( 2.0 * pB / rhoPresMu.x * mult							//pressure force from wall
		- 3000 / sizeScale * dot(n3, F3(velMas)))									//damping force from wall
		- 500 / sizeScale * F3(velMas);												//damping in all directions
	derivRho = -rhoPresMu.x * dot(n3, F3(velMas)) * mult; 
	return F4(derivV, derivRho);
}
//--------------------------------------------------------------------------------------------------------------------------------
//Force terms for the particles close to the boundary
//here, d is center distance not penetration distance (distance of particle's center from rigid wall)
__device__ inline float4 DifVelocityRhoBoundary_Beta(	const float3 & n3, float d, 
													const float4 & posRad, const float4 & velMas, const float4 & rhoPresMu) {
	//****** SPH version
	if (fabs(rhoPresMu.w) < .1) {return F4(0);}
	float wB = W3(d, posRad.w) / W2(0, posRad.w);
	float wBMax = W3(0, posRad.w) / W2(0, posRad.w);
	float normV = sqrt(dot(F3(velMas), F3(velMas)));
	float pB = rhoPresMu.y;
	float3 derivV = F3(0);
	float derivRho = 0;
	float mult = (d > 0) ? wB : (2* wBMax - wB);
	if (mult < .000001 /*epsilon*/) {return F4(0);}
	derivV = - 500 / sizeScale * F3(velMas);												//damping in all directions
	derivRho = 0; 
	//rhoPresMu.w = 0;
	return F4(derivV, derivRho);
}
//--------------------------------------------------------------------------------------------------------------------------------
//Force terms for the particles close to the boundary
//d is center distance not penetration distance
__device__ inline float4 DifVelocityRhoBoundary_DEM_RigidParticles(	const float3 & n3, float d, 
													const float4 & posRad, const float4 & velMas, const float4 & rhoPresMu) {
	//****** DEM version
	if (d > 0 && W3(d, posRad.w) < .000001 /*epsilon*/) {return F4(0);}
	d = fabs(d - 2 * posRad.w);
	float	kS = 39240.0f * sizeScale; //392400.0;	//spring
	float	kD = 4200.0f * sizeScale;	//420.0;				//damper
	//	float	kS = 39240.0; //392400.0;	//spring
	//float	kD = 4200.0;	//420.0;				//damper
	float	m_eff = 0.5f * velMas.w;
	float3 force = (d < 0) ? F3(0) : ( //d is always positive in here
					kS * d * n3 - 
					kD * m_eff * dot( F3 (velMas) , n3 ) * n3		//relative velocity at contact is simply assumed as the relative vel of the centers. If you are updating the rotation, this should be modified.
					);
	return F4(force / velMas.w, 0);
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline void ForceOutBoundaryCircle(float2 & n2, float & d, float2 coord, float2 cent2, float serpR, float sphR) {
		float BC_Margine = 0;//2 * sphR;//0;//2 * sphR;
		float cDist = length(coord - cent2);
		if (cDist < serpR + BC_Margine) {
			n2 = coord - cent2;
			n2 /= cDist;
			//d = serpR + sphR - cDist;	//penetration distance (positive), 
			d = cDist - serpR;			//distance of particle's center from serpentine wall
		}
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline void ForceInBoundaryCircle(float2 & n2, float & d, float2 coord, float2 cent2, float serpR, float sphR) { 
		float BC_Margine = 0;//2 * sphR;//0;//2 * sphR;
		float cDist = length(coord - cent2);
		if (cDist > serpR - BC_Margine) {
			n2 = (coord - cent2) * -1;
			n2 /= cDist;
			//d = cDist - (serpR - sphR); 	//penetration distance (positive)
			d = serpR - cDist;				//distance of particle's center from serpentine wall
		}
}
////--------------------------------------------------------------------------------------------------------------------------------
////rigid body boundary condtion, 2nd version, periodic BC along x. A basic implementation. 
//__device__ inline void ApplyBoundaryRigid2(float4 & posRadRigidD, float4 & velMassRigidD, float3 & deltaPos, float3 & deltaVel) {
//	float rRigidBody = posRadRigidD.w;
//	if (posRadRigidD.x < cMinD.x) {
//		posRadRigidD.x += cMaxD.x - cMinD.x;
//	}
//	if (posRadRigidD.y < cMinD.y + rRigidBody) {
//		posRadRigidD.y = cMinD.y + rRigidBody;
//		deltaPos.y = cMinD.y + rRigidBody - posRadRigidD.y;
//		velMassRigidD.y += -1.2 * velMassRigidD.y;
//		deltaVel.y += -1.2 * velMassRigidD.y;
//	}
//	if (posRadRigidD.z < cMinD.z + rRigidBody) {
//		posRadRigidD.z = cMinD.z + rRigidBody;
//		deltaPos.z = cMinD.z + rRigidBody - posRadRigidD.z;
//		velMassRigidD.z += -1.2 * velMassRigidD.z;
//		deltaVel.z += -1.2 * velMassRigidD.z;
//	}
//
//	if (posRadRigidD.x > cMaxD.x) {
//		posRadRigidD.x -= cMaxD.x - cMinD.x;
//	}
//	if (posRadRigidD.y > cMaxD.y - rRigidBody) {
//		posRadRigidD.y = cMaxD.y - rRigidBody;
//		deltaPos.y = cMaxD.y - rRigidBody - posRadRigidD.y;
//		velMassRigidD.y += -1.2 * velMassRigidD.y;
//		deltaVel.y += -1.2 * velMassRigidD.y;
//	}
//	if (posRadRigidD.z > cMaxD.z - rRigidBody) {
//		posRadRigidD.z = cMaxD.z - rRigidBody;
//		deltaPos.z = cMaxD.z - rRigidBody - posRadRigidD.z; 
//		velMassRigidD.z += -1.2 * velMassRigidD.z;
//		deltaVel.z += -1.2 * velMassRigidD.z;
//	}
//}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void BoundarySerpentine (float4 * posRadD, float4 * velMasD, float4 * rhoPresMuD, float4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += portionD.x;								// updatePortionD = [start, end] index of the update portion
	if (index >= portionD.y) {return;}	
		
	float4 velMas = velMasD[index];
	float4 posRad = posRadD[index];
	float4 rhoPresMu = rhoPresMuD[index];
	//if (fabs(rhoPresMu.w) < .1) {return;} //ie boundary
	float3 n3;
	float d;
	float4 wallBoundaryForce = F4(0);
	
	const float sphR = posRad.w;
	float BC_Margine = 0;//2 * sphR;//0;//2 * sphR;

	if (posRad.y < cMinD.y + BC_Margine) {
		n3 = F3(0, 1, 0); 
		d = posRad.y - cMinD.y;
		//d = cMinD.y + sphR - posRad.y;
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(n3, d, posRad, velMas, rhoPresMu);
		}
	} 

	if (posRad.y > cMaxD.y - BC_Margine) { 
		n3 = F3(0, -1, 0);
		d = cMaxD.y - posRad.y;
		//d = posRad.y - cMaxD.y + sphR;
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(n3, d, posRad, velMas, rhoPresMu);
		}
	}

	//serpentine
	const float sPeriod = 4.6 * sizeScale;
	n3 = F3(1, 0, 0);
	float2 n2 = F2(1, 0);
	d = 5 * sphR;		//as a large number, greater than the kernel support length
	const float toleranceZone = 2 * sphR;
	float x, y;
	if (posRad.x < nPeriod * sPeriod) {
		float r1 = 1.3 * sizeScale, r2 = 1.0 * sizeScale, r3 = 2.0 * sizeScale, r4 = 0.3 * sizeScale;
		x = fmod(posRad.x, sPeriod); //posRad.x - int(posRad.x / sPeriod) * sPeriod; //fmod
		y = posRad.z;
		if (x >= 0 && x < 1.3 * sizeScale + BC_Margine) {
			ForceOutBoundaryCircle(n2, d, F2(x, y), F2(0, 0), r1, sphR); 
			ForceInBoundaryCircle(n2, d, F2(x, y), F2(0, 1.0 * sizeScale), r3, sphR); 
		} else if (x >= 1.3 * sizeScale + BC_Margine && x < 2.0 * sizeScale - BC_Margine) {		
			if (y > 1.0 * sizeScale) { ForceInBoundaryCircle(n2, d, F2(x, y), F2(0, 1.0 * sizeScale), r3, sphR); }
			else if (y < 0) { ForceInBoundaryCircle(n2, d, F2(x, y), F2(2.3 * sizeScale, 0), r2, sphR); }
		} else if (x >= 2.0 * sizeScale - BC_Margine && x < 2.6 * sizeScale + BC_Margine) {
			ForceInBoundaryCircle(n2, d, F2(x, y), F2(2.3 * sizeScale, 0), r2, sphR);
			if (y < .55 * sizeScale) { ForceOutBoundaryCircle(n2, d, F2(x, y), F2(2.3 * sizeScale, .55 * sizeScale), r4, sphR); }
			else if (y > .55 * sizeScale) { ForceOutBoundaryCircle(n2, d, F2(x, y), F2(2.3 * sizeScale, y), r4, sphR); }		
		} else if (x >= 2.6 * sizeScale + BC_Margine && x < 3.3 * sizeScale - BC_Margine) {
			if (y > 1.0 * sizeScale) { ForceInBoundaryCircle(n2, d, F2(x, y), F2(4.6 * sizeScale, 1.0 * sizeScale), r3, sphR); }
			else if (y < 0) { ForceInBoundaryCircle(n2, d, F2(x, y), F2(2.3 * sizeScale, 0), r2, sphR); }
		} else if (x >= 3.3 * sizeScale - BC_Margine && x < 4.6 * sizeScale) {
			ForceOutBoundaryCircle(n2, d, F2(x, y), F2(4.6 * sizeScale, 0), r1, sphR); 
			ForceInBoundaryCircle(n2, d, F2(x, y), F2(4.6 * sizeScale, 1.0 * sizeScale), r3, sphR); 
		}
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(F3(n2.x, 0, n2.y), d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(F3(n2.x, 0, n2.y), d, posRad, velMas, rhoPresMu);
		}
	} 
	//straight channel
	if (posRad.x > nPeriod * sPeriod - toleranceZone) {
		x = posRad.x - nPeriod * sPeriod;
		y = posRad.z;
		d = 5 * sphR;		//as a large number, greater than the kernel support length

		//horizontal walls
		if (x > 0) {
			if (x < 5 * sizeScale) {
				if (y < .55 * sizeScale + BC_Margine) {
					n3 = F3(0, 0, 1);
					d = y - .55 * sizeScale;
				} else if (y > 3.55 * sizeScale - BC_Margine) {
					n3 = F3(0, 0, -1);
					d = 3.55 * sizeScale - y;
				}
			} else if (x < 6.5 * sizeScale) {
				if (y < 1.55 * sizeScale + BC_Margine) {
					n3 = F3(0, 0, 1);
					d = y - 1.55 * sizeScale;
				} else if (y > 2.55 * sizeScale - BC_Margine) {
					n3 = F3(0, 0, -1);
					d = 2.55 * sizeScale - y;
				}
			} else if (x >= 6.5 * sizeScale) {
				if (y < 1.302 * sizeScale + BC_Margine) {
					n3 = F3(0, 0, 1);
					d = y - 1.302 * sizeScale;
				} else if (y > 3 * sizeScale - BC_Margine) {
					n3 = F3(0, 0, -1);
					d = 3 * sizeScale - y;
				}
			}
		}
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(n3, d, posRad, velMas, rhoPresMu);
		}

		//vertical walls
		d = 5 * sphR;		//as a large number, greater than the kernel support length
		if (x < BC_Margine && (y < 1.3 * sizeScale || y > 3 * sizeScale)) {
			n3 = F3(1, 0, 0);
			d = x;
		} 
		if ( (x > 5 * sizeScale - BC_Margine && x < 5 * sizeScale + toleranceZone) && (y < 1.55 * sizeScale || y > 2.55 * sizeScale) ) {
			n3 = F3(-1, 0, 0);
			d = 5 * sizeScale - x;
		}
		if ( (x > 6.5 * sizeScale - toleranceZone && x < 6.5 * sizeScale + BC_Margine) && (y < 1.55 * sizeScale || y > 2.55 * sizeScale) ) {
			n3 = F3(1, 0, 0);
			d = x - 6.5 * sizeScale;			
		}
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(n3, d, posRad, velMas, rhoPresMu);
		}
	}

	derivVelRhoD[index] += wallBoundaryForce;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void BoundaryStraightChannel (float4 * posRadD, float4 * velMasD, float4 * rhoPresMuD, float4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += portionD.x;								// updatePortionD = [start, end] index of the update portion
	if (index >= portionD.y) {return;}	
		
	float4 velMas = velMasD[index];
	float4 posRad = posRadD[index];
	float4 rhoPresMu = rhoPresMuD[index];
	//if (fabs(rhoPresMu.w) < .1) {return;} //ie boundary
	float3 n3;
	float d;
	float4 wallBoundaryForce = F4(0);
	
	const float sphR = posRad.w;
	float BC_Margine = 0;//2 * sphR;//0;//2 * sphR;

	if (posRad.y < cMinD.y + BC_Margine) {
		n3 = F3(0, 1, 0); 
		d = posRad.y - cMinD.y;
		//d = cMinD.y + sphR - posRad.y;
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(n3, d, posRad, velMas, rhoPresMu);
		}
	} 

	if (posRad.y > cMaxD.y - BC_Margine) { 
		n3 = F3(0, -1, 0);
		d = cMaxD.y - posRad.y;
		//d = posRad.y - cMaxD.y + sphR;
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(n3, d, posRad, velMas, rhoPresMu);
		}
	}

	if (posRad.z < 1 * sizeScale + BC_Margine) {
		n3 = F3(0, 0, 1); 
		d = posRad.z - 1 * sizeScale;
		//d = cMinD.y + sphR - posRad.y;
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(n3, d, posRad, velMas, rhoPresMu);
		}
	} 

	if (posRad.z > 3 * sizeScale - BC_Margine) { 
		n3 = F3(0, 0, -1);
		d = 3 * sizeScale - posRad.z;
		//d = posRad.y - cMaxD.y + sphR;
		if (flagD == 0) {
			wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
		} else if (flagD == 1) {
			wallBoundaryForce += DifVelocityRhoBoundary_DEM_RigidParticles(n3, d, posRad, velMas, rhoPresMu);
		}
	}

	derivVelRhoD[index] += wallBoundaryForce;
}
//--------------------------------------------------------------------------------------------------------------------------------
//Applying boundary condition through force. Second version: periodic BC along x
//used when boundary is not implemented through descritized mesh
__global__ void BoundaryForceKernelFluid (float4 * posRadD, float4 * velMasD, float4 * rhoPresMuD, float4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += portionD.x;								// updatePortionD = [start, end] index of the update portion
	if (index >= portionD.y) {return;}	
		
	float4 velMas = velMasD[index];
	float4 posRad = posRadD[index];
	float4 rhoPresMu = rhoPresMuD[index];
	float3 n3;
	float d;
	float4 wallBoundaryForce = F4(0);

	if (posRad.y < cMinD.y + posRad.w) {
		n3 = F3(0, 1, 0); 
		d = posRad.y - cMinD.y;
		wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
	} 
	if (posRad.z < cMinD.z + posRad.w) { 
		n3 = F3(0, 0, 1);
		d = posRad.z - cMinD.z;
		wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
	}

	if (posRad.y > cMaxD.y - posRad.w) { 
		n3 = F3(0, -1, 0);
		d = cMaxD.y - posRad.y;
		wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
	}
	if (posRad.z > cMaxD.z - posRad.w) { 
		n3 = F3(0, 0, -1);
		d = cMaxD.z - posRad.z;
		wallBoundaryForce += DifVelocityRhoBoundary_Beta(n3, d, posRad, velMas, rhoPresMu);
	} 
	derivVelRhoD[index] += wallBoundaryForce;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateKernelFluid(float4 * posRadD, float4 * velMasD, float3 * vel_XSPH_D, float4 * rhoPresMuD, float4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortionD.x;								// updatePortionD = [start, end] index of the update portion
	if (index >= updatePortionD.y) {return;}				 
	
	float4 velMas = velMasD[index];
	float3 vel_XSPH = vel_XSPH_D[index];
	float4 posRad = posRadD[index];
	float3 updatedPositon = F3(posRad) + vel_XSPH * dTD;
	posRadD[index] = F4(updatedPositon, posRad.w);							//posRadD updated

	float4 derivVelRho = derivVelRhoD[index];
	float4 rhoPresMu = rhoPresMuD[index];
	float rho2 = rhoPresMu.x + derivVelRho.w * dTD;							//rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	float3 updatedVelocity = F3(velMas + derivVelRho * dTD);
	velMasD[index] = F4(updatedVelocity, /*rho2 / rhoPresMu.x * */velMas.w);	//velMasD updated

	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu;											//rhoPresMuD updated 
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateKernelBoundary(float4 * posRadD, float4 * velMasD, float4 * rhoPresMuD, float4 * derivVelRhoD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortionD.x;								// updatePortionD = [start, end] index of the update portion
	if (index >= updatePortionD.y) {return;}				 

	float4 derivVelRho = derivVelRhoD[index];
	float4 rhoPresMu = rhoPresMuD[index];
	float rho2 = rhoPresMu.x + derivVelRho.w * dTD;							//rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu;											//rhoPresMuD updated 
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x
__global__ void ApplyPeriodicBoundaryXKernel(float4 * posRadD, float4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= mNumSpheresD) { return; }
	float4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {return;} //no need to do anything if it is a boundary particle
	float4 posRad = posRadD[index];
	if (posRad.x > cMaxD.x) {
		posRad.x -= (cMaxD.x - cMinD.x);
		posRadD[index] = posRad;
		return;
	} 
	if (posRad.x < cMinD.x) {
		posRad.x += (cMaxD.x - cMinD.x);
		posRadD[index] = posRad;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along y
__global__ void ApplyPeriodicBoundaryYKernel(float4 * posRadD, float4 * rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= mNumSpheresD) { return; }
	float4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {return;} //no need to do anything if it is a boundary particle
	float4 posRad = posRadD[index];
	if (posRad.y > cMaxD.y) {
		posRad.y -= (cMaxD.y - cMinD.y);
		posRadD[index] = posRad;
		return;
	} 
	if (posRad.y < cMinD.y) {
		posRad.y += (cMaxD.y - cMinD.y);
		posRadD[index] = posRad;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryXKernel_RigidBodies(float4 * posRadRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) { return; }
	float4 posRadRigid = posRadRigidD[index];
	if (posRadRigid.x > cMaxD.x) {
		posRadRigid.x -= (cMaxD.x - cMinD.x);
		posRadRigidD[index] = posRadRigid;
		return;
	}
	if (posRadRigid.x < cMinD.x) {
		posRadRigid.x += (cMaxD.x - cMinD.x);
		posRadRigidD[index] = posRadRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
//applies periodic BC along x, for ridid bodies
__global__ void ApplyPeriodicBoundaryYKernel_RigidBodies(float4 * posRadRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numRigidBodiesD) { return; }
	float4 posRadRigid = posRadRigidD[index];
	if (posRadRigid.y > cMaxD.y) {
		posRadRigid.y -= (cMaxD.y - cMinD.y);
		posRadRigidD[index] = posRadRigid;
		return;
	}
	if (posRadRigid.y < cMinD.y) {
		posRadRigid.y += (cMaxD.y - cMinD.y);
		posRadRigidD[index] = posRadRigid;
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void	CalcTorqueShare(float4* torqueParticlesD, float4* derivVelRhoD, float4* posRadD, int* rigidIdentifierD, float4* posRadRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidParticleIndex = index + startRigidParticleD;								// updatePortionD = [start, end] index of the update portion
	if (index >= numRigid_SphParticlesD) {return;}
	float3 dist3 = Distance(posRadD[rigidParticleIndex], posRadRigidD[rigidIdentifierD[index]]);
	torqueParticlesD[index] = F4(cross(dist3, F3(derivVelRhoD[rigidParticleIndex])) , 0);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateKernelRigid(
								  float4 * totalForces4, 
								  float4 * posRadRigidD, 
								  float4 * velMassRigidD, 
								  float3 * deltaPositionD, 
								  float3 * deltaVelocityD, 
								  float rigid_SPH_mass) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA > numRigidBodiesD) {return;}
	
	float4 dummyPosRad = posRadRigidD[rigidSphereA];
	float4 dummyVelMas = velMassRigidD[rigidSphereA];

	float4 totalForce4 = totalForces4[rigidSphereA];
	float3 derivV_SPH = rigid_SPH_mass * F3(totalForce4) / dummyVelMas.w;		//gravity is applied in the force kernel

	float3 deltaPos = F3(dummyVelMas) * dTD;
	dummyPosRad += F4(deltaPos, 0);

	float3 deltaVel = derivV_SPH * dTD;
	dummyVelMas += F4(deltaVel, 0);

	//float3 boundaryDeltaPos = F3(0);
	//float3 boundaryDeltaVel = F3(0);
	//ApplyBoundaryRigid2(dummyPosRad, dummyVelMas, boundaryDeltaPos, boundaryDeltaVel);
	//deltaPos += boundaryDeltaPos;
	//deltaVel += boundaryDeltaVel;

	deltaPositionD[rigidSphereA] = deltaPos;
	deltaVelocityD[rigidSphereA] = deltaVel;
	posRadRigidD[rigidSphereA] = dummyPosRad;
	velMassRigidD[rigidSphereA] = dummyVelMas;
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateKernelRigid_XZ_Motion(
									 float4 * totalForces4, 
									 float4 * totalTorque4,
									 float4 * posRadRigidD, 
									 float4 * velMassRigidD, 
									 float4 * cylinderRotOmegaJD, 
									 float3 * deltaPositionD, 
									 float3 * deltaVelocityD, 
									 float3 * deltaXZTetaOmegaD,
									 float rigid_SPH_mass) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA > numRigidBodiesD) {return;}
	
	float4 dummyPosRad = posRadRigidD[rigidSphereA];
	float4 dummyVelMas = velMassRigidD[rigidSphereA];
	float4 dummyTetaOmegaJ = cylinderRotOmegaJD[rigidSphereA];

	float4 totalForce4 = totalForces4[rigidSphereA];
	totalForce4.y = 0;		//the only different line from UpdateKernelRigid
	float3 derivV_SPH = rigid_SPH_mass * F3(totalForce4) / dummyVelMas.w;		//in fact, totalForce4 is originially sum of dV/dt of sph particles and should be multiplied by m to produce force. gravity is applied in the force kernel

	float3 deltaPos = F3(dummyVelMas) * dTD;
	deltaPos.y = 0;
		//deltaPos = F3(0);//
	dummyPosRad += F4(deltaPos, 0);

	float3 deltaVel = derivV_SPH * dTD;
	deltaVel.y = 0;
		//deltaVel = F3(0);//
	dummyVelMas += F4(deltaVel, 0);
//------------- torque
	float totalTorque = totalTorque4[rigidSphereA].y;					//for the XZ motion, only y component of the torque is necessary
	float derivOmega = rigid_SPH_mass * totalTorque / dummyTetaOmegaJ.z;
	float deltaTeta = dummyTetaOmegaJ.y * dTD;
		//deltaTeta = 0;
	float deltaOmega = derivOmega * dTD;
		//deltaOmega = 0;
	dummyTetaOmegaJ += F4(deltaTeta, deltaOmega, 0, 0);
	dummyTetaOmegaJ.x -= (dummyTetaOmegaJ.x > 2 * PI) ? 2 * PI : 0;
	dummyTetaOmegaJ.x += (dummyTetaOmegaJ.x < 0) ? 2 * PI : 0;

	//float3 boundaryDeltaPos = F3(0);
	//float3 boundaryDeltaVel = F3(0);
	//ApplyBoundaryRigid2(dummyPosRad, dummyVelMas, boundaryDeltaPos, boundaryDeltaVel);
	//deltaPos += boundaryDeltaPos;
	//deltaVel += boundaryDeltaVel;

	deltaPositionD[rigidSphereA] = deltaPos;
	deltaVelocityD[rigidSphereA] = deltaVel;
	deltaXZTetaOmegaD[rigidSphereA] = F3(deltaTeta, deltaOmega, 0);

	posRadRigidD[rigidSphereA] = dummyPosRad;
	velMassRigidD[rigidSphereA] = dummyVelMas;
	cylinderRotOmegaJD[rigidSphereA] = dummyTetaOmegaJ;
//	printf("totalTorque %f, dummyTetaOmegaJ.z %f, rigid_SPH_mass/dummyTetaOmegaJ.z %f\n", totalTorque*1e6, dummyTetaOmegaJ.z *1e12, rigid_SPH_mass/dummyTetaOmegaJ.z);
//	printf("totalTorque %f\n", totalTorque );
//	printf("derivOmega %f\n", derivOmega );
//	printf("deltaOmega %f\n", deltaOmega );
//	printf("deltaTetae6 %f\n", deltaTeta * 1e6 );
//	printf("totalForce4 %f\n", length(F3(totalForce4)) );
//	printf("derivV_SPH %f\n", length(derivV_SPH) );
//	printf("deltaVel %f\n", length(deltaVel) );
//	printf("deltaPos e6 %f\n", length(deltaPos) *1e6);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateRigidParticlesPosition_Original(float4 * posRadD, float4 * velMasD, float3 * deltaPositionD, float3 * deltaVelocityD, int * rigidIdentifierD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidParticleIndex = index + startRigidParticleD;								// updatePortionD = [start, end] index of the update portion
	if (index >= startRigidParticleD + numRigid_SphParticlesD) {return;}
	int rigidBodyIndex = rigidIdentifierD[index];
	posRadD[rigidParticleIndex] += F4(deltaPositionD[rigidBodyIndex], 0);
	velMasD[rigidParticleIndex] += F4(deltaVelocityD[rigidBodyIndex], 0);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the rigid body particles
__global__ void UpdateRigidParticlesPosition_XZ_Motion(
	float4 * posRadD, 
	float4 * velMasD, 
	float3 * deltaPositionD, 
	float3 * deltaXZTetaOmegaD,
	int * rigidIdentifierD, 
	float4 * posRadRigidD,
	float4 * velMassRigidD,
	float4 * cylinderRotOmegaJD) 
{
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	uint rigidParticleIndex = index + startRigidParticleD;								// updatePortionD = [start, end] index of the update portion
	if (index >= numRigid_SphParticlesD) {return;}
	int rigidBodyIndex = rigidIdentifierD[index];
	float4 posRadRigid = posRadRigidD[rigidBodyIndex];

	float4 posRad = posRadD[rigidParticleIndex];
	float3 deltaPosRigid = deltaPositionD[rigidBodyIndex];
			//float3 r3_0 = F3(posRad) - (F3(posRadRigid) - deltaPosRigid);
	float3 r3 = Distance(F3(posRad), F3(posRadRigid) - deltaPosRigid);  //old rigid body position is (posRadRigid - deltaPosRigid)
	r3.y = 0; //for the XZ motion
	
	float3 deltaXZTetaOmega = deltaXZTetaOmegaD[rigidBodyIndex];
	float3 deltaPos = deltaPosRigid + cross( F3(0, deltaXZTetaOmega.x, 0), r3 );

	posRadD[rigidParticleIndex] += F4(deltaPos, 0);
	float3 velRigid = F3(velMassRigidD[rigidBodyIndex]);
	float3 rotOmegaJRigid = F3(cylinderRotOmegaJD[rigidBodyIndex]);
	float3 velParticle = velRigid + cross( F3(0, rotOmegaJRigid.y, 0), r3);
	float mass = velMasD[rigidParticleIndex].w;
	velMasD[rigidParticleIndex] = F4( velParticle, mass );
			//r3.y = 0; if (length(r3) > .201 * sizeScale) printf("**** length r3 %f, r3 %f %f %f, r3_0 %f %f %f\n", length(r3), r3.x, r3.y, r3.z, r3_0.x, r3_0.y, r3_0.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
void MapSPH_ToGrid(
							float resolution,
							int3 & cartesianGridDims,
							thrust::host_vector<float4> & rho_Pres_CartH,
							thrust::host_vector<float4> & vel_VelMag_CartH,
							thrust::device_vector<float4> & posRadD, 
							thrust::device_vector<float4> & velMasD, 
							thrust::device_vector<float4> & rhoPresMuD, 
							int mNSpheres, 
							SimParams paramsH) {
    float4* m_dSortedPosRad;
    float4* m_dSortedVelMas;
	float4* m_dSortedRhoPreMu;
    uint*  m_dCellStart;			// index of start of each cell in sorted list
    uint*  m_dCellEnd;				// index of end of cell

	int3 SIDE = paramsH.gridSize;
	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	allocateArray((void**)&m_dSortedPosRad, mNSpheres * sizeof(float4));
    allocateArray((void**)&m_dSortedVelMas, mNSpheres * sizeof(float4));
    allocateArray((void**)&m_dSortedRhoPreMu, mNSpheres * sizeof(float4));

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);
    
	allocateArray((void**)&m_dCellStart, m_numGridCells*sizeof(uint));
    allocateArray((void**)&m_dCellEnd, m_numGridCells*sizeof(uint));

	// calculate grid hash
	calcHash(
        U1CAST(m_dGridParticleHash),
        U1CAST(m_dGridParticleIndex),
        F4CAST(posRadD),
        mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());
	
	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(
        m_dCellStart,
        m_dCellEnd,
		m_dSortedPosRad,
		m_dSortedVelMas,
		m_dSortedRhoPreMu,
        U1CAST(m_dGridParticleHash),
        U1CAST(m_dGridParticleIndex),
		TCAST(posRadD),
		F4CAST(velMasD),
		F4CAST(rhoPresMuD),
		mNSpheres,
		m_numGridCells);

	//float resolution = 8 * paramsH.particleRadius;
	cartesianGridDims = I3(paramsH.boxDims / resolution) + I3(1);
	printf("^^^ bodDim %f %f %f, GridDim %d %d %d, resolution %f \n",paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z, cartesianGridDims.x, cartesianGridDims.y, cartesianGridDims.z, resolution);
	uint cartesianGridSize = cartesianGridDims.x * cartesianGridDims.y * cartesianGridDims.z;
	thrust::device_vector<float4> rho_Pres_CartD(cartesianGridSize);
	thrust::device_vector<float4> vel_VelMag_CartD(cartesianGridSize);

	CalcCartesianData(
        F4CAST(rho_Pres_CartD),
		F4CAST(vel_VelMag_CartD),
        m_dSortedPosRad,
        m_dSortedVelMas,
		m_dSortedRhoPreMu,
        U1CAST(m_dGridParticleIndex),
        m_dCellStart,
        m_dCellEnd,
        cartesianGridSize,
		cartesianGridDims,
		resolution);

	freeArray(m_dSortedPosRad);
	freeArray(m_dSortedVelMas);
	freeArray(m_dSortedRhoPreMu);

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

	freeArray(m_dCellStart);
	freeArray(m_dCellEnd);

	rho_Pres_CartH.resize(cartesianGridSize);
	vel_VelMag_CartH.resize(cartesianGridSize);
	thrust::copy(rho_Pres_CartD.begin(), rho_Pres_CartD.end(), rho_Pres_CartH.begin());
	thrust::copy(vel_VelMag_CartD.begin(), vel_VelMag_CartD.end(), vel_VelMag_CartH.begin());

	rho_Pres_CartD.clear();
	vel_VelMag_CartD.clear();
}
//*******************************************************************************************************************************
//builds the neighbors' list of each particle and finds the force on each particle
//calculates the interaction force between 1- fluid-fluid, 2- fluid-solid, 3- solid-fluid particles
//calculates forces from other SPH or solid particles, as wall as boundaries
void PrintToFile(
			thrust::device_vector<float4> & posRadD, 
			thrust::device_vector<float4> & velMasD, 
			thrust::device_vector<float4> & rhoPresMuD, 
			const thrust::host_vector<int3> & referenceArray,
			const thrust::device_vector<int> & rigidIdentifierD,
			thrust::device_vector<float4> & posRadRigidD,
			thrust::device_vector<float4> & velMassRigidD,
			thrust::device_vector<float4> & cylinderRotOmegaJD,
			float3 cMax, 
			float3 cMin,
			SimParams paramsH,
			float delT,
			int tStep
			) {
	FILE *fileNameFluid;
	FILE *fileNameRigids;
	FILE *fileNameSlice;
	FILE *fileNameCartesianTotal;
	FILE *fileNameCartesianMidplane;
	FILE *fileVelocityProfPoiseuille;
	FILE *fileRigidParticleCenter;


	int stepSaveFluid = 100000;
	///if (tStep%100 == 0 &&  tStep > 20400) {
	////if (tStep > 12506) {
	if (tStep % stepSaveFluid == 0) {
		if (tStep / stepSaveFluid == 0) {
			fileNameFluid = fopen("dataFluid.txt", "w");
			fprintf(fileNameFluid, "variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Rho\", \"Pressure\", \"type\"\n");
		} else {
			fileNameFluid = fopen("dataFluid.txt", "a");
		}

		fprintf(fileNameFluid, "zone\n");
		for (int i = referenceArray[0].x; i < referenceArray[1].y; i++) {
			float3 pos = F3(posRadD[i]);
			float3 vel = F3(velMasD[i]);
			float4 rP = rhoPresMuD[i];
			float velMag = length(vel);
			fprintf(fileNameFluid, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, velMag, rP.x, rP.y, rP.w);
		}
		fflush(fileNameFluid);	
		fclose(fileNameFluid);
	}

	int stepSaveRigid = 5000;
	///if (tStep % 20 == 0 && tStep > 56000) {
	//if (tStep > 12506) {
	if (tStep % stepSaveRigid == 0) {
		if (tStep / stepSaveRigid == 0) {
			fileNameRigids = fopen("dataRigidParticle.txt", "w");
			fprintf(fileNameRigids, "variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Teta\", \"Omega\", \"Rho\", \"Pressure\", \"bodySize\", \"type\"\n");
		} else {
			fileNameRigids = fopen("dataRigidParticle.txt", "a");
		}
		fprintf(fileNameRigids, "zone\n");
		if (referenceArray.size() > 2) {
			const int numRigidBodies = posRadRigidD.size();
			int startRigidParticle = (I2(referenceArray[2])).x;
			for (int i = startRigidParticle; i < referenceArray[2 + numRigidBodies - 1].y; i++) {
				float3 pos = F3(posRadD[i]);
				float3 vel = F3(velMasD[i]);
				float4 rP = rhoPresMuD[i];
				float velMag = length(vel);
				int rigidID = rigidIdentifierD[i - startRigidParticle];
				float4 rigPosRad = posRadRigidD[rigidID];
				float4 rotOmega = cylinderRotOmegaJD[rigidID];
				fprintf(fileNameRigids, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, velMag, rotOmega.x, rotOmega.y, rP.x, rP.y, rigPosRad.w, rP.w);
			}
		}
		fflush(fileNameRigids);
		fclose(fileNameRigids);
	}

	
	int stepSaveFluidSlice = 20000;//1;//20000;
	//if (tStep%100 == 0 &&  tStep > 20400) {
	//if (tStep > 49100) {	
	if (tStep % stepSaveFluidSlice == 0) {		
		//if (tStep / stepSaveFluidSlice == 49101) {
		if (tStep / stepSaveFluidSlice == 0) {
			fileNameSlice = fopen("dataTotalSlice.txt", "w");
			fprintf(fileNameSlice, "variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Rho\", \"Pressure\", \"type\"\n");
		} else {
			fileNameSlice = fopen("dataTotalSlice.txt", "a");
		}
		fprintf(fileNameSlice, "zone\n");
		for (int i = referenceArray[0].x; i < referenceArray[referenceArray.size() - 1].y; i++) {
			float4 posRad = posRadD[i];
			float3 pos = F3(posRad);
			float rad = posRad.w;
			float3 vel = F3(velMasD[i]);
			float4 rP = rhoPresMuD[i];
			float velMag = length(vel);
			if ( (pos.y < cMin.y + 0.5 * (cMax.y - cMin.y) + 3 * rad) && (pos.y > cMin.y + 0.5 * (cMax.y - cMin.y) - 3 * rad) ) {
				fprintf(fileNameSlice, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, velMag, rP.x, rP.y, rP.w);
			}
		}
		fflush(fileNameSlice);
		fclose(fileNameSlice);
	}
	//----------------------------------------
	thrust::host_vector<float4> rho_Pres_CartH(1);
	thrust::host_vector<float4> vel_VelMag_CartH(1);
	float resolution = 2 * HSML;
	int3 cartesianGridDims;
	int tStepCartesianTotal = 100000;
	int tStepCartesianSlice = 1000;
	int tStepPoiseuilleProf = tStepCartesianSlice;

	int stepCalcCartesian = min(tStepCartesianTotal, tStepCartesianSlice); 
	stepCalcCartesian = min(stepCalcCartesian, tStepPoiseuilleProf);

	if (tStep % stepCalcCartesian == 0) {
		MapSPH_ToGrid(resolution, cartesianGridDims, rho_Pres_CartH, vel_VelMag_CartH, posRadD, velMasD, rhoPresMuD, referenceArray[referenceArray.size() - 1].y, paramsH);
	}
	if (tStep % tStepCartesianTotal == 0) {		
		if (tStep / tStepCartesianTotal == 0) {
			fileNameCartesianTotal = fopen("dataCartesianTotal.txt", "w");
			fprintf(fileNameCartesianTotal, "variables = \"x\", \"y\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Rho\", \"Pressure\"\n");
		} else {
			fileNameCartesianTotal = fopen("dataCartesianTotal.txt", "a");
		}
		fprintf(fileNameCartesianTotal, "zone I = %d, J = %d, K = %d\n", cartesianGridDims.x, cartesianGridDims.y, cartesianGridDims.z);
		for (int k = 0; k < cartesianGridDims.z; k++) {
			for (int j = 0; j < cartesianGridDims.y; j++) {
				for (int i = 0; i < cartesianGridDims.x; i++) {
					int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x * cartesianGridDims.y;
					float3 gridNodeLoc = resolution * F3(i, j, k) + paramsH.worldOrigin;
					fprintf(fileNameCartesianTotal, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", gridNodeLoc.x, gridNodeLoc.y, gridNodeLoc.z, vel_VelMag_CartH[index].x, vel_VelMag_CartH[index].y, vel_VelMag_CartH[index].z, vel_VelMag_CartH[index].w, rho_Pres_CartH[index].x, rho_Pres_CartH[index].y);
				}
			}
		}
		fflush(fileNameCartesianTotal);
		fclose(fileNameCartesianTotal);
	}
//--------------
	if (tStep % tStepCartesianSlice == 0) {	
		if (tStep / tStepCartesianSlice == 0) {
			fileNameCartesianMidplane = fopen("dataCartesianMidplane.txt", "w");
			fprintf(fileNameCartesianMidplane, "variables = \"x\", \"z\", \"Vx\", \"Vy\", \"Vz\", \"Velocity Magnitude\", \"Rho\", \"Pressure\"\n");
		} else {
			fileNameCartesianMidplane = fopen("dataCartesianMidplane.txt", "a");
		}
		fprintf(fileNameCartesianMidplane, "zone I = %d, J = %d\n", cartesianGridDims.x, cartesianGridDims.z);
		int j = cartesianGridDims.y / 2;
		for (int k = 0; k < cartesianGridDims.z; k++) {
			for (int i = 0; i < cartesianGridDims.x; i++) {
				int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x * cartesianGridDims.y;
				float3 gridNodeLoc = resolution * F3(i, j, k) + paramsH.worldOrigin;
				fprintf(fileNameCartesianMidplane, "%f, %f, %f, %f, %f, %f, %f, %f\n", gridNodeLoc.x, gridNodeLoc.z, vel_VelMag_CartH[index].x, vel_VelMag_CartH[index].y, vel_VelMag_CartH[index].z, vel_VelMag_CartH[index].w, rho_Pres_CartH[index].x, rho_Pres_CartH[index].y);
			}
		}
		fflush(fileNameCartesianMidplane);
		fclose(fileNameCartesianMidplane);
	}
//--------------
	if (tStep % tStepPoiseuilleProf == 0) {	
		if (tStep / tStepPoiseuilleProf == 0) {
			fileVelocityProfPoiseuille = fopen("dataVelProfile.txt", "w");
			fprintf(fileVelocityProfPoiseuille, "variables = \"Z(m)\", \"Vx(m/s)\"\n");

		} else {
			fileVelocityProfPoiseuille = fopen("dataVelProfile.txt", "a");
		}
		fprintf(fileVelocityProfPoiseuille, "zone T=\"t = %f s\"\n", delT * tStep);
		int j = cartesianGridDims.y / 2;
		int i = cartesianGridDims.x / 2;
		for (int k = 0; k < cartesianGridDims.z; k++) {
			int index = i + j * cartesianGridDims.x + k * cartesianGridDims.x * cartesianGridDims.y;
			float3 gridNodeLoc = resolution * F3(i, j, k) + paramsH.worldOrigin;
			if (gridNodeLoc.z > 1 * sizeScale && gridNodeLoc.z < 2 * sizeScale) {fprintf(fileVelocityProfPoiseuille, "%f, %f\n", gridNodeLoc.z, vel_VelMag_CartH[index].x);}
		}
		fflush(fileVelocityProfPoiseuille);
		fclose(fileVelocityProfPoiseuille);
	}
//--------------
	int tStepRigidCenterPos = 1000;
	if (tStep % tStepRigidCenterPos == 0) {	
		if (tStep / tStepRigidCenterPos == 0) {
			fileRigidParticleCenter = fopen("dataRigidCenter.txt", "w");
			fprintf(fileRigidParticleCenter, "variables = \"t(s)\", \"Z(m)\"\n");
			fprintf(fileRigidParticleCenter, "zone\n");

		} else {
			fileRigidParticleCenter = fopen("dataRigidCenter.txt", "a");
		}
		if (referenceArray.size() > 2) {
			float4 pR_rigid = posRadRigidD[0];
			fprintf(fileRigidParticleCenter, "%f, %0.10f\n", tStep * delT, pR_rigid.z);
		}
		fflush(fileRigidParticleCenter);
		fclose(fileRigidParticleCenter);
	}



			//if (tStep%10000 == 0) {
			//	char dumStr[5];
			//	int dumNumChar = sprintf(dumStr, "%d", int(tStep / 10000) + 1);
			//	char* fileNameMultipleZones;
			//	strcpy(fileNameMultipleZones, "slicesMultipleZones/dataTotalSlice");
			//	strcat(fileNameMultipleZones, dumStr);
			//	strcat(fileNameMultipleZones, ".dat");

			//	outFileMultipleZones = fopen(fileNameMultipleZones, "w");
			//	fprintf(outFileMultipleZones, "variables = \'x\', \'y\', \'z\', \'vX\', \'vY\', \'vZ\', \'velocity_magnitude\', \'rho\', \'pressure\'\nzone\n");
			//	for (int i = referenceArray[0].x; i < referenceArray[referenceArray.size() - 1].y; i++) {
			//		float4 posRad = posRadD[i];
			//		float3 pos = F3(posRad);
			//		float rad = posRad.w;
			//		float3 vel = F3(velMasD[i]);
			//		float3 rP = F3(rhoPresMuD[i]);
			//		float velMag = length(vel);
			//		if ( (pos.y < cMin.y + 0.5 * (cMax.y - cMin.y) + 4 * rad) && (pos.y > cMin.y + 0.5 * (cMax.y - cMin.y) - 4 * rad) ) {
			//			fprintf(outFileMultipleZones, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, velMag, rP.x, rP.y);
			//		}
			//	}
			//	fclose(outFileMultipleZones);
			//}
	rho_Pres_CartH.clear();
	vel_VelMag_CartH.clear();
}





//*******************************************************************************************************************************
//builds the neighbors' list of each particle and finds the force on each particle
//calculates the interaction force between 1- fluid-fluid, 2- fluid-solid, 3- solid-fluid particles
//calculates forces from other SPH or solid particles, as wall as boundaries
void ForceSPH(
		   thrust::device_vector<float4> & posRadD, 
		   thrust::device_vector<float4> & velMasD, 
		   thrust::device_vector<float3> & vel_XSPH_D,
		   thrust::device_vector<float4> & rhoPresMuD, 
		   thrust::device_vector<uint> & bodyIndexD, 
		   thrust::device_vector<float4> & derivVelRhoD, 
		   const thrust::host_vector<int3> & referenceArray,
		   int mNSpheres,
		   int3 SIDE
		   ) 
{
	// Part1: contact detection #########################################################################################################################
	//
	//&&
	//&&
	//&&
	//&&
	//&&
	//&&
	//&&

    // grid data for sorting method
    float4* m_dSortedPosRad;
    float4* m_dSortedVelMas;
	float4* m_dSortedRhoPreMu;
    uint*  m_dCellStart;			// index of start of each cell in sorted list
    uint*  m_dCellEnd;				// index of end of cell

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	allocateArray((void**)&m_dSortedPosRad, mNSpheres * sizeof(float4));
    allocateArray((void**)&m_dSortedVelMas, mNSpheres * sizeof(float4));
    allocateArray((void**)&m_dSortedRhoPreMu, mNSpheres * sizeof(float4));

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);
    
	allocateArray((void**)&m_dCellStart, m_numGridCells*sizeof(uint));
    allocateArray((void**)&m_dCellEnd, m_numGridCells*sizeof(uint));


	// calculate grid hash
	calcHash(
        U1CAST(m_dGridParticleHash),
        U1CAST(m_dGridParticleIndex),
        F4CAST(posRadD),
        mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());
	
	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(
        m_dCellStart,
        m_dCellEnd,
		m_dSortedPosRad,
		m_dSortedVelMas,
		m_dSortedRhoPreMu,
        U1CAST(m_dGridParticleHash),
        U1CAST(m_dGridParticleIndex),
		TCAST(posRadD),
		F4CAST(velMasD),
		F4CAST(rhoPresMuD),
		mNSpheres,
		m_numGridCells);

	//process collisions
	thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), F4(0));																	//initialize derivVelRhoD with zero. necessary
	thrust::fill(derivVelRhoD.begin() + referenceArray[0].x, derivVelRhoD.begin() + referenceArray[0].y, bodyForce4);				//add body force to fluid particles.

	RecalcVelocity_XSPH(
        F3CAST(vel_XSPH_D),
        m_dSortedPosRad,
        m_dSortedVelMas,
		m_dSortedRhoPreMu,
        U1CAST(m_dGridParticleIndex),
        m_dCellStart,
        m_dCellEnd,
        mNSpheres,
        m_numGridCells);

    collide(
        F4CAST(derivVelRhoD),
        m_dSortedPosRad,
        m_dSortedVelMas,
		F3CAST(vel_XSPH_D),
		m_dSortedRhoPreMu,
        U1CAST(m_dGridParticleIndex),
        m_dCellStart,
        m_dCellEnd,
        mNSpheres,
        m_numGridCells);
	
	freeArray(m_dSortedPosRad);
	freeArray(m_dSortedVelMas);
	freeArray(m_dSortedRhoPreMu);

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

	freeArray(m_dCellStart);
	freeArray(m_dCellEnd);
	//&&
	//&&
	//&&
	//&&
	//&&
	//&&
	//&&

	////is used for the boundaries that are implemented as straight walls (cMax cMin)
	////if you are implementing the boundary as spherical decomposition, no need for this kernel
	//uint blockFluidParticles, nThreads;	
	//int2 portion = make_int2(referenceArray[0]);
	//int flag = 0;

	//cudaMemcpyToSymbolAsync(flagD, &flag, sizeof(flag));
	//cudaMemcpyToSymbolAsync(portionD, &portion, sizeof(portion));
	//computeGridSize(portion.y - portion.x, 128, blockFluidParticles, nThreads);
	//BoundarySerpentine<<<blockFluidParticles,  nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	/////BoundaryStraightChannel<<<blockFluidParticles,  nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	//cudaThreadSynchronize();
	//CUT_CHECK_ERROR("Kernel execution failed: BoundarySerpentine, for fluid");

	//if (referenceArray.size() > 2) {
	//	portion = make_int2(referenceArray[2].x, referenceArray[ referenceArray.size() - 1 ].y);
	//	flag = 1;
	//	cudaMemcpyToSymbolAsync(flagD, &flag, sizeof(flag));

	//	uint blocksRigidBodyParticles;
	//	computeGridSize(portion.y - portion.x, 128, blocksRigidBodyParticles, nThreads);
	//	cudaMemcpyToSymbolAsync(portionD, &portion, sizeof(portion));
	//	//BoundarySerpentineRigidBodies<<<blocksRigidBodyParticles,  nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	//	//cudaThreadSynchronize();
	//	
	//	BoundarySerpentine<<<blocksRigidBodyParticles,  nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	//	///BoundaryStraightChannel<<<blocksRigidBodyParticles,  nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	//	cudaThreadSynchronize();
	//	CUT_CHECK_ERROR("Kernel execution failed: BoundarySerpentine, for solid");
	//}
}
//--------------------------------------------------------------------------------------------------------------------------------
void DensityReinitialization(
							 thrust::device_vector<float4> & posRadD, 
							 thrust::device_vector<float4> & velMasD, 
							 thrust::device_vector<float4> & rhoPresMuD, 
							 int mNSpheres, 
							 int3 SIDE) {
    float4* m_dSortedPosRad;
    float4* m_dSortedVelMas;
	float4* m_dSortedRhoPreMu;
    uint*  m_dCellStart;			// index of start of each cell in sorted list
    uint*  m_dCellEnd;				// index of end of cell

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	allocateArray((void**)&m_dSortedPosRad, mNSpheres * sizeof(float4));
    allocateArray((void**)&m_dSortedVelMas, mNSpheres * sizeof(float4));
    allocateArray((void**)&m_dSortedRhoPreMu, mNSpheres * sizeof(float4));

	thrust::device_vector<uint> m_dGridParticleHash(mNSpheres);
	thrust::device_vector<uint> m_dGridParticleIndex(mNSpheres);
    
	allocateArray((void**)&m_dCellStart, m_numGridCells*sizeof(uint));
    allocateArray((void**)&m_dCellEnd, m_numGridCells*sizeof(uint));

	// calculate grid hash
	calcHash(
        U1CAST(m_dGridParticleHash),
        U1CAST(m_dGridParticleIndex),
        F4CAST(posRadD),
        mNSpheres);

	thrust::sort_by_key(m_dGridParticleHash.begin(), m_dGridParticleHash.end(), m_dGridParticleIndex.begin());
	
	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(
        m_dCellStart,
        m_dCellEnd,
		m_dSortedPosRad,
		m_dSortedVelMas,
		m_dSortedRhoPreMu,
        U1CAST(m_dGridParticleHash),
        U1CAST(m_dGridParticleIndex),
		TCAST(posRadD),
		F4CAST(velMasD),
		F4CAST(rhoPresMuD),
		mNSpheres,
		m_numGridCells);

	ReCalcDensity(
        F4CAST(posRadD),
		F4CAST(velMasD),
		F4CAST(rhoPresMuD),
        m_dSortedPosRad,
        m_dSortedVelMas,
		m_dSortedRhoPreMu,
        U1CAST(m_dGridParticleIndex),
        m_dCellStart,
        m_dCellEnd,
        mNSpheres,
        m_numGridCells);

	freeArray(m_dSortedPosRad);
	freeArray(m_dSortedVelMas);
	freeArray(m_dSortedRhoPreMu);

	m_dGridParticleHash.clear();
	m_dGridParticleIndex.clear();

	freeArray(m_dCellStart);
	freeArray(m_dCellEnd);
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles by calling UpdateKernelFluid 
void UpdateFluid(
				 thrust::device_vector<float4> & posRadD, 
				 thrust::device_vector<float4> & velMasD, 
				 thrust::device_vector<float3> & vel_XSPH_D,
				 thrust::device_vector<float4> & rhoPresMuD, 
				 thrust::device_vector<float4> & derivVelRhoD, 
				 const thrust::host_vector<int3> & referenceArray, 
				 float dT) {
	int2 updatePortion = I2(referenceArray[0]);	
	//int2 updatePortion = I2(referenceArray[0].x, referenceArray[0].y);
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
	cudaMemcpyToSymbolAsync(updatePortionD, &updatePortion, sizeof(updatePortion));

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid, nThreads);
	UpdateKernelFluid<<<nBlock_UpdateFluid, nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F3CAST(vel_XSPH_D), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelFluid");
}
//--------------------------------------------------------------------------------------------------------------------------------
//updates the fluid particles by calling UpdateKernelFluid 
void UpdateBoundary(
				 thrust::device_vector<float4> & posRadD, 
				 thrust::device_vector<float4> & velMasD, 
				 thrust::device_vector<float4> & rhoPresMuD, 
				 thrust::device_vector<float4> & derivVelRhoD, 
				 const thrust::host_vector<int3> & referenceArray, 
				 float dT) {
	int2 updatePortion = I2(referenceArray[1]);	
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));
	cudaMemcpyToSymbolAsync(updatePortionD, &updatePortion, sizeof(updatePortion));

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid, nThreads);
	UpdateKernelBoundary<<<nBlock_UpdateFluid, nThreads>>>(F4CAST(posRadD), F4CAST(velMasD), F4CAST(rhoPresMuD), F4CAST(derivVelRhoD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelFluid");
}
//--------------------------------------------------------------------------------------------------------------------------------
//rigid body boundary condtion. A basic implementation. 
float4 ApplyBoundaryRigid(float4 & posRadRigidH, float4 & velMassRigidH, float3 cMax, float3 cMin, float rRigidBody) {
	float4 delta4 = F4(0);
	if (posRadRigidH.x < cMin.x + rRigidBody) {
		posRadRigidH.x = cMin.x + rRigidBody;
		velMassRigidH.x *= -.2;
		delta4.x = cMin.x + rRigidBody - posRadRigidH.x;
	}
	if (posRadRigidH.y < cMin.y + rRigidBody) {
		posRadRigidH.y = cMin.y + rRigidBody;
		velMassRigidH.y *= -.2;
		delta4.y = cMin.y + rRigidBody - posRadRigidH.y;
	}
	if (posRadRigidH.z < cMin.z + rRigidBody) {
		posRadRigidH.z = cMin.z + rRigidBody;
		velMassRigidH.z *= -.2;
		delta4.z = cMin.z + rRigidBody - posRadRigidH.z;
	}

	if (posRadRigidH.x > cMax.x - rRigidBody) {
		posRadRigidH.x = cMax.x - rRigidBody;
		velMassRigidH.x *= -.2;
		delta4.x = cMax.x - rRigidBody - posRadRigidH.x;
	}
	if (posRadRigidH.y > cMax.y - rRigidBody) {
		posRadRigidH.y = cMax.y - rRigidBody;
		velMassRigidH.y *= -.2;
		delta4.y = cMax.y - rRigidBody - posRadRigidH.y;
	}
	if (posRadRigidH.z > cMax.z - rRigidBody) {
		posRadRigidH.z = cMax.z - rRigidBody;
		velMassRigidH.z *= -.2;
		delta4.z = cMax.z - rRigidBody - posRadRigidH.z; 
	}
	return delta4;
}
//--------------------------------------------------------------------------------------------------------------------------------
void ApplyBoundary(
						thrust::device_vector<float4> & posRadD, 
						thrust::device_vector<float4> & rhoPresMuD, 
						int mNSpheres,
						thrust::device_vector<float4> & posRadRigidD, 
						thrust::device_vector<float4> & velMassRigidD, 	
						int numRigidBodies
					) {
	uint nBlock_NumSpheres, nThreads_SphParticles;
	computeGridSize(mNSpheres, 256, nBlock_NumSpheres, nThreads_SphParticles);
	ApplyPeriodicBoundaryXKernel<<<nBlock_NumSpheres, nThreads_SphParticles>>>(F4CAST(posRadD), F4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryXKernel");
	ApplyPeriodicBoundaryYKernel<<<nBlock_NumSpheres, nThreads_SphParticles>>>(F4CAST(posRadD), F4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: ApplyPeriodicBoundaryXKernel");
//////////////
	uint nBlock_NumRigids, nThreads_RigidBodies;
	computeGridSize(numRigidBodies, 128, nBlock_NumRigids, nThreads_RigidBodies);
	
	cudaMemcpyToSymbolAsync(numRigidBodiesD, &numRigidBodies, sizeof(numRigidBodies));		//can be defined outside of the kernel, and only once
	ApplyPeriodicBoundaryXKernel_RigidBodies<<<nBlock_NumRigids, nThreads_RigidBodies>>>(F4CAST(posRadRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
	ApplyPeriodicBoundaryYKernel_RigidBodies<<<nBlock_NumRigids, nThreads_RigidBodies>>>(F4CAST(posRadRigidD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
}
//--------------------------------------------------------------------------------------------------------------------------------
void UpdateRigidBody(
					 thrust::device_vector<float4> & posRadD,
					 thrust::device_vector<float4> & velMasD,
					 thrust::device_vector<float4> & derivVelRhoD,
					 const thrust::device_vector<int> & rigidIdentifierD,
					 const thrust::host_vector<int3> & referenceArray,
					 thrust::device_vector<float4> & posRadRigidD,
					 thrust::device_vector<float4> & velMassRigidD,
					 thrust::device_vector<float4> & cylinderRotOmegaJD,
					 SimParams paramsH,
					 float dT) 
{	
	if (referenceArray.size() < 3) {return;}
	const int numRigidBodies = posRadRigidD.size();
	float4 typicalRigidSPH = velMasD[referenceArray[2].x];
	float rigid_SPH_mass = typicalRigidSPH.w;
	//printf("rigid_SPH_mass %f\n", 1000000*rigid_SPH_mass);
	float4 typicalRigidVelMas = velMassRigidD[0];
	//printf("sph mass and total mass: %f %f\n", rigid_SPH_mass, typicalRigidVelMas.w);
	cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(dT));

	int numRigid_SphParticles = rigidIdentifierD.size();
	int startRigidParticle = (I2(referenceArray[2])).x;
	cudaMemcpyToSymbolAsync(startRigidParticleD, &startRigidParticle, sizeof(startRigidParticle));		//can be defined outside of the kernel, and only once
	cudaMemcpyToSymbolAsync(numRigid_SphParticlesD, &numRigid_SphParticles, sizeof(numRigid_SphParticles));		//can be defined outside of the kernel, and only once

//g	
	
	thrust::device_vector<float4> totalForces4(numRigidBodies);
	thrust::device_vector<float4> totalTorque4(numRigidBodies);
	thrust::fill(totalForces4.begin(), totalForces4.end(), F4(0));
	thrust::device_vector<int> dummyIdentify(numRigidBodies);
	thrust::equal_to<int> binary_pred;
	//printf("&&&&  %d %d %d %d\n", rigidIdentifierD.size(), derivVelRhoD.end() - derivVelRhoD.begin() - startRigidParticle, derivVelRhoD.size() - startRigidParticle, startRigidParticle);
	//printf("numRigidBodies %d\n", numRigidBodies);
	(void)thrust::reduce_by_key(rigidIdentifierD.begin(), rigidIdentifierD.end(), derivVelRhoD.begin() + startRigidParticle, dummyIdentify.begin(), totalForces4.begin(), binary_pred, thrust::plus<float4>());

	thrust::device_vector<float4> torqueParticlesD(numRigid_SphParticles);
	uint nBlocks_numRigid_SphParticles;
	uint nThreads_SphParticles;
	computeGridSize(numRigid_SphParticles, 256, nBlocks_numRigid_SphParticles, nThreads_SphParticles);
	//printf("numRigid_SphParticles %d %d %d\n", numRigid_SphParticles, nBlocks_numRigid_SphParticles, nThreads_SphParticles);
	CalcTorqueShare<<<nBlocks_numRigid_SphParticles, nThreads_SphParticles>>>(F4CAST(torqueParticlesD), F4CAST(derivVelRhoD), F4CAST(posRadD), I1CAST(rigidIdentifierD), F4CAST(posRadRigidD)); 
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: CalcTorqueShare");
	(void)thrust::reduce_by_key(rigidIdentifierD.begin(), rigidIdentifierD.end(), torqueParticlesD.begin(), dummyIdentify.begin(), totalTorque4.begin(), binary_pred, thrust::plus<float4>());

	torqueParticlesD.clear();
	dummyIdentify.clear();

	//add gravity
	thrust::device_vector<float4> gravityForces4(numRigidBodies);
	thrust::fill(gravityForces4.begin(), gravityForces4.end(), F4(paramsH.gravity));
	thrust::transform(totalForces4.begin(), totalForces4.end(), gravityForces4.begin(), totalForces4.begin(), thrust::plus<float4>());
	gravityForces4.clear();

	thrust::device_vector<float3> deltaPositionD(numRigidBodies);
	thrust::device_vector<float3> deltaVelocityD(numRigidBodies);
	thrust::device_vector<float3> deltaXZTetaOmegaD(numRigidBodies);
	thrust::fill(deltaPositionD.begin(), deltaPositionD.end(), F3(0));
	thrust::fill(deltaVelocityD.begin(), deltaVelocityD.end(), F3(0));
	thrust::fill(deltaXZTetaOmegaD.begin(), deltaXZTetaOmegaD.end(), F3(0));

	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);
	cudaMemcpyToSymbolAsync(numRigidBodiesD, &numRigidBodies, sizeof(numRigidBodies));		//can be defined outside of the kernel, and only once
//	UpdateKernelRigid<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(F4CAST(totalForces4), F4CAST(posRadRigidD), F4CAST(velMassRigidD), F3CAST(deltaPositionD), F3CAST(deltaVelocityD), rigid_SPH_mass); 
	// copy rigid_SPH_mass to symbol -constant memory
	UpdateKernelRigid_XZ_Motion<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(F4CAST(totalForces4), F4CAST(totalTorque4), F4CAST(posRadRigidD), F4CAST(velMassRigidD), F4CAST(cylinderRotOmegaJD), F3CAST(deltaPositionD), F3CAST(deltaVelocityD), F3CAST(deltaXZTetaOmegaD), rigid_SPH_mass); 
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
	totalForces4.clear();
	totalTorque4.clear();


	//int numRigid_SphParticles = referenceArray[numRigidBodies + 2 - 1].y - referenceArray[2].x;
	UpdateRigidParticlesPosition_XZ_Motion<<<nBlocks_numRigid_SphParticles, nThreads_SphParticles>>>(F4CAST(posRadD), F4CAST(velMasD), F3CAST(deltaPositionD), F3CAST(deltaXZTetaOmegaD), I1CAST(rigidIdentifierD), F4CAST(posRadRigidD), F4CAST(velMassRigidD), F4CAST(cylinderRotOmegaJD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: UpdateKernelRigid");
	deltaPositionD.clear();
	deltaVelocityD.clear();
	deltaXZTetaOmegaD.clear();

//g

}
//##############################################################################################################################################
// the main function, which updates the particles and implements BC
void cudaCollisions(thrust::host_vector<float4> & mPosRad, 
					thrust::host_vector<float4> & mVelMas, 
					thrust::host_vector<float4> & mRhoPresMu, 
					const thrust::host_vector<uint> & bodyIndex, 
					const thrust::host_vector<int3> & referenceArray, 
					int & mNSpheres, 
					float3 cMax, 
					float3 cMin, 
					float delT, 
					thrust::host_vector<float4> & posRadRigidH, 
					thrust::host_vector<float4> & velMassRigidH,
					thrust::host_vector<float4> & cylinderRotOmegaJH,
					float binSize0) {
	//--------- initialization ---------------
	//cudaError_t dumDevErr = cudaSetDevice(2);
	cudaEvent_t start, stop; 
	cudaEventCreate(&start);
	cudaEventCreate(&stop); 
	cudaEventRecord( start, 0 );

	//printf("cMin.x, y, z, CMAx.x, y, z, binSize %f %f %f , %f %f %f, %f\n", cMin.x, cMin.y, cMin.z, cMax.x, cMax.y, cMax.z, binSize0); 

	cudaMemcpyToSymbolAsync(cMinD, &cMin, sizeof(cMin));
	cudaMemcpyToSymbolAsync(cMaxD, &cMax, sizeof(cMax));
	cudaMemcpyToSymbolAsync(mNumSpheresD, &mNSpheres, sizeof(mNSpheres));


	//?$ edit this
	int numRigidBodies = posRadRigidH.size();
	thrust::device_vector<float4> posRadD(mNSpheres);
	thrust::copy(mPosRad.begin(), mPosRad.end(), posRadD.begin());
	thrust::device_vector<float4> velMasD(mNSpheres);
	thrust::copy(mVelMas.begin(), mVelMas.end(), velMasD.begin());
	thrust::device_vector<float4> rhoPresMuD(mNSpheres);
	thrust::copy(mRhoPresMu.begin(), mRhoPresMu.end(), rhoPresMuD.begin());

	thrust::device_vector<float4> posRadRigidD(numRigidBodies);
	thrust::copy(posRadRigidH.begin(), posRadRigidH.end(), posRadRigidD.begin());
	thrust::device_vector<float4> velMassRigidD(numRigidBodies);
	thrust::copy(velMassRigidH.begin(), velMassRigidH.end(), velMassRigidD.begin());
	thrust::device_vector<float4> cylinderRotOmegaJD(numRigidBodies);
	thrust::copy(cylinderRotOmegaJH.begin(), cylinderRotOmegaJH.end(), cylinderRotOmegaJD.begin());


	thrust::device_vector<uint> bodyIndexD(mNSpheres);
	thrust::copy(bodyIndex.begin(), bodyIndex.end(), bodyIndexD.begin());
	thrust::device_vector<float4> derivVelRhoD(mNSpheres);
	
	int startRigidParticle = (I2(referenceArray[1])).y;
	thrust::device_vector<int> rigidIdentifierD(0);
	//printf("referenceArray.size() %d\n", referenceArray.size());
	if (referenceArray.size() > 2) {
		startRigidParticle = (I2(referenceArray[2])).x;
		int numRigid_SphParticles = referenceArray[2 + numRigidBodies - 1].y - startRigidParticle;	
		rigidIdentifierD.resize(numRigid_SphParticles);
		for (int rigidSphereA = 0; rigidSphereA < numRigidBodies; rigidSphereA ++) {
			int2 updatePortion = I2(referenceArray[2 + rigidSphereA]);						//first two component of the referenceArray denote to the fluid and boundary particles
			thrust::fill(rigidIdentifierD.begin() + (updatePortion.x - startRigidParticle), rigidIdentifierD.begin() + (updatePortion.y - startRigidParticle), rigidSphereA);
		}
	}
	//int i =  rigidIdentifierD[429];
	//printf("rigid body coord %d %f %f\n", i, posRadRigidH[i].x, posRadRigidH[i].z);
	//printf("length %f\n", length(F2(posRadRigidH[i].x - .003474, posRadRigidH[i].z - .000673)));

	//****************************** bin size adjustement and contact detection stuff *****************************
		//float mBinSize0 = (mNSpheres == 0) ? mBinSize0 : 2 * HSML;
		//float3 cMinOffset = cMin - 3 * F3(0, mBinSize0, mBinSize0);		//periodic bc in x direction
		//float3 cMaxOffset = cMax + 3 * F3(0, mBinSize0, mBinSize0);
		////float3 cMinOffset = cMin - 3 * F3(mBinSize0, mBinSize0, mBinSize0);		//periodic bc in x direction
		////float3 cMaxOffset = cMax + 3 * F3(mBinSize0, mBinSize0, mBinSize0);

		/////printf("side.x %f\n", abs(cMaxOffset.x - cMinOffset.x) / mBinSize);
		//int3 SIDE = I3(  floor( (cMaxOffset.x - cMinOffset.x) / mBinSize0 ), floor( (cMaxOffset.y - cMinOffset.y) / mBinSize0 ), floor( (cMaxOffset.z - cMinOffset.z) / mBinSize0)  );
		//float mBinSize = (cMaxOffset.x - cMinOffset.x) / SIDE.x;  //this one works when periodic BC is only on x. if it was on y as well (or on z), you would have problem. 
	float3 cMinOffset = cMin - 3 * F3(0, 0, binSize0);		//periodic bc in x direction
	float3 cMaxOffset = cMax + 3 * F3(0, 0, binSize0);
	int3 SIDE = I3(  int( (cMaxOffset.x - cMinOffset.x) / binSize0 + .1), int( (cMaxOffset.y - cMinOffset.y) / binSize0 + .1), floor( (cMaxOffset.z - cMinOffset.z) / binSize0 + .1)  );
	float mBinSize = binSize0;															//Best solution in that case may be to change cMax or cMin such that periodic sides be a multiple of binSize

	printf("SIDE: %d, %d, %d\n", SIDE.x, SIDE.y, SIDE.z);
	//*******************
	SimParams paramsH;
	paramsH.gravity = Gravity;//Gravity * sizeScale;;// F3(0, -9.8, 0) * sizeScale; //F3(0, -9800, 0) * sizeScale;
	paramsH.particleRadius = HSML;
	paramsH.gridSize = SIDE;
	//paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = cMinOffset;
	paramsH.cellSize = F3(mBinSize, mBinSize, mBinSize);
	paramsH.boxDims = cMaxOffset - cMinOffset;
		printf("boxDims: %f, %f, %f\n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z);

	setParameters(&paramsH);
	    cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, &paramsH, sizeof(SimParams)) );

	//********************************************************************************

	FILE *outFileMultipleZones;

	int povRayCounter = 0;
	int stepEnd = 1e6; //200000;//10000;//50000;//100000;


	//for (int tStep = 0; tStep < 0; tStep ++) {
	for (int tStep = 0; tStep < stepEnd + 1; tStep ++) {
//		if (tStep > 10000) delT = .2;
		cudaEvent_t start2, stop2; 
		cudaEventCreate(&start2);
		cudaEventCreate(&stop2); 
		cudaEventRecord( start2, 0 );
		
		//computations
		thrust::device_vector<float4> posRadD2 = posRadD;
		thrust::device_vector<float4> velMasD2 = velMasD;
		thrust::device_vector<float4> rhoPresMuD2 = rhoPresMuD;
		thrust::device_vector<float4> posRadRigidD2 = posRadRigidD;
		thrust::device_vector<float4> velMassRigidD2 = velMassRigidD;
		thrust::device_vector<float4> cylinderRotOmegaJD2 = cylinderRotOmegaJD;
		thrust::device_vector<float3> vel_XSPH_D(mNSpheres);

		ForceSPH(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD, derivVelRhoD, referenceArray, mNSpheres, SIDE);		//?$ right now, it does not consider gravity or other stuff on rigid bodies. they should be applied at rigid body solver		
		UpdateFluid(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * delT);		//assumes ...D2 is a copy of ...D
		//UpdateBoundary(posRadD2, velMasD2, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * delT);		//assumes ...D2 is a copy of ...D
		UpdateRigidBody(posRadD2, velMasD2, derivVelRhoD, rigidIdentifierD, referenceArray, posRadRigidD2, velMassRigidD2, cylinderRotOmegaJD2, paramsH, 0.5 * delT);
		ApplyBoundary(posRadD2, rhoPresMuD2, mNSpheres, posRadRigidD2, velMassRigidD2, numRigidBodies);
		
		ForceSPH(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, mNSpheres, SIDE);
		UpdateFluid(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, derivVelRhoD, referenceArray, delT);
		//UpdateBoundary(posRadD, velMasD, rhoPresMuD, derivVelRhoD, referenceArray, delT);
		UpdateRigidBody(posRadD, velMasD, derivVelRhoD, rigidIdentifierD, referenceArray, posRadRigidD, velMassRigidD, cylinderRotOmegaJD, paramsH, delT);
		ApplyBoundary(posRadD, rhoPresMuD, mNSpheres, posRadRigidD, velMassRigidD, numRigidBodies);

		posRadD2.clear();
		velMasD2.clear();
		rhoPresMuD2.clear();
		posRadRigidD2.clear();
		velMassRigidD2.clear();
		cylinderRotOmegaJD2.clear();
		vel_XSPH_D.clear();

		//density re-initialization
		if (tStep%10 == 0) {
			DensityReinitialization(posRadD, velMasD, rhoPresMuD, mNSpheres, SIDE); //does not work for analytical boundaries (non-meshed) and free surfaces
		}

		PrintToFile(posRadD, velMasD, rhoPresMuD, referenceArray, rigidIdentifierD, posRadRigidD, velMassRigidD, cylinderRotOmegaJD, cMax, cMin, paramsH, delT, tStep);

		float time2;
		cudaEventRecord( stop2, 0 ); 
		cudaEventSynchronize( stop2 ); 
		cudaEventElapsedTime( &time2, start2, stop2 );
		cudaEventDestroy( start2 );
		cudaEventDestroy( stop2 );
		if (tStep%10 == 0) { printf("step: %d, step Time: %f\n ", tStep, time2); }
		
		//_CrtDumpMemoryLeaks(); //for memory leak detection (msdn suggestion for VS) apparently does not work in conjunction with cuda

	}

	//you may copy back to host
	posRadD.clear();
	velMasD.clear();
	rhoPresMuD.clear();
	posRadRigidD.clear();
	velMassRigidD.clear();
	cylinderRotOmegaJD.clear();
	bodyIndexD.clear();
	derivVelRhoD.clear();
	rigidIdentifierD.clear();

	float time;
	cudaEventRecord( stop, 0 ); 
	cudaEventSynchronize( stop ); 
	cudaEventElapsedTime( &time, start, stop );
	cudaEventDestroy( start );
	cudaEventDestroy( stop );
	printf("total Time: %f\n ", time);
}
