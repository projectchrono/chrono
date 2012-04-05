#ifndef SDKCOLLISIONSYSTEM_CUH
#define SDKCOLLISIONSYSTEM_CUH

#ifdef __CDT_PARSER__
#define __host__
#define __device__
#define __global__
#define __constant__
#define __shared__
#define CUDA_KERNEL_DIM(...) ()
#else
#define CUDA_KERNEL_DIM(...)  <<< __VA_ARGS__ >>>
#endif


typedef unsigned int uint;

//#if USE_TEX
#if 0
#define FETCH(t, i) tex1Dfetch(t##Tex, i)
#else
#define FETCH(t, i) t[i]
#endif

//#define sizeScale .001
#define sizeScale .001

#define HSML 0.06f*sizeScale
#define BASEPRES 0
#define nPeriod 7
#define Gravity make_float3(0, 0, 0)
//#define bodyForce4 make_float4(.005, 0, 0, 0)
#define bodyForce4 make_float4(.5, 0, 0, 0)
//#define bodyForce4 make_float4(.1, 0, 0, 0)
//#define bodyForce4 make_float4(.0004, 0, 0, 0) //segre. size Scale 1
#define rho0 1000
#define mu0 .001f
//#define mu0 1.0f
#define v_Max .1f
//#define v_Max .014f //estimated maximum velocity of fluid
//#define v_Max .0005f //estimated maximum velocity of fluid
#define EPS_XSPH .5f

#define F4	make_float4
#define F3	make_float3
#define F2	make_float2
#define I2	make_int2
#define I3	make_int3
#define PI 3.14159265f

struct SimParams {
		float3 gravity;
		float globalDamping;
		float particleRadius;

		int3 gridSize;
		float3 worldOrigin;
		float3 cellSize;

		uint numBodies;
		float3 boxDims;
};
struct fluidData {
		float rho;
		float pressure;
		float velocityMag;
		float3 velocity;
};
__constant__ SimParams paramsD;
__constant__ int3 cartesianGridDimsD;
__constant__ float resolutionD;

//--------------------------------------------------------------------------------------------------------------------------------
//3D SPH kernel function, W3_SplineA
__device__ inline float W3_Spline(float d, float h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = fabs(d) / h;
	if (q < 1) {
		return (0.25f / (PI * h * h * h) * (pow(2 - q, 3) - 4 * pow(1 - q, 3)));
	}
	if (q < 2) {
		return (0.25f / (PI * h * h * h) * pow(2 - q, 3));
	}
	return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
//2D SPH kernel function, W2_SplineA
__device__ inline float W2_Spline(float d, float h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = fabs(d) / h;
	if (q < 1) {
		return (5 / (14 * PI * h * h) * (pow(2 - q, 3) - 4 * pow(1 - q, 3)));
	}
	if (q < 2) {
		return (5 / (14 * PI * h * h) * pow(2 - q, 3));
	}
	return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
//3D SPH kernel function, W3_QuadraticA
__device__ inline float W3_Quadratic(float d, float h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = fabs(d) / h;
	if (q < 2) {
		return (1.25f / (PI * h * h * h) * .75f * (pow(.5f * q, 2) - q + 1));
	}
	return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
//2D SPH kernel function, W2_QuadraticA
__device__ inline float W2_Quadratic(float d, float h) { // d is positive. h is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = fabs(d) / h;
	if (q < 2) {
		return (2.0f / (PI * h * h) * .75f * (pow(.5f * q, 2) - q + 1));
	}
	return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
//Gradient of the kernel function
// d: magnitude of the distance of the two particles
// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a = pos_a - pos_b
__device__ inline float3 GradW_Spline(float3 d, float h) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = length(d) / h;
	if (q < 1) {
		return .75f / (PI * powf(h, 5)) * (3 * q - 4) * d;
	}
	if (q < 2) {
		return .75f / (PI * powf(h, 5)) * (-q + 4.0f - 4.0f / q) * d;
	}
	return F3(0);
}
//--------------------------------------------------------------------------------------------------------------------------------
//Gradient of the kernel function
// d: magnitude of the distance of the two particles
// dW * dist3 gives the gradiant of W3_Quadratic, where dist3 is the distance vector of the two particles, (dist3)a = pos_a - pos_b
__device__ inline float3 GradW_Quadratic(float3 d, float h) { // d is positive. r is the sph particle radius (i.e. h in the document) d is the distance of 2 particles
	float q = length(d) / h;
	if (q < 2) {
		return 1.25f / (PI * powf(h, 5)) * .75f * (.5f - 1.0f / q) * d;
	}
	return F3(0);
}
//--------------------------------------------------------------------------------------------------------------------------------
#define W3 W3_Spline
#define W2 W2_Spline
#define GradW GradW_Spline
//--------------------------------------------------------------------------------------------------------------------------------
//Eos is also defined in SDKCollisionSystem.cu
//fluid equation of state
__device__ inline float Eos(float rho, float type) {
	////******************************
	//int gama = 1;
	//if (type < -.1) {
	//	return 1 * (100000 * (pow(rho / rho0, gama) - 1) + BASEPRES);
	//	//return 100 * rho;
	//} 
	//////else {
	//////	return 1e9;
	//////}

	//******************************	
	int gama = 7;
	float B = 100 * rho0 * v_Max * v_Max / gama; //200;//314e6; //c^2 * rho0 / gama where c = 1484 m/s for water
	if (type < +.1f) {
		return B * (pow(rho / rho0, gama) - 1); //1 * (B * (pow(rho / rho0, gama) - 1) + BASEPRES);
	} else return BASEPRES;
}
//--------------------------------------------------------------------------------------------------------------------------------
//distance between two particles, considering the periodic boundary condition
__device__ inline float3 Distance(float3 a, float3 b) {
	float3 dist3 = a - b;
	dist3.x -= ((dist3.x > 0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);
	dist3.x += ((dist3.x < -0.5f * paramsD.boxDims.x) ? paramsD.boxDims.x : 0);

	dist3.y -= ((dist3.y > 0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);
	dist3.y += ((dist3.y < -0.5f * paramsD.boxDims.y) ? paramsD.boxDims.y : 0);

	dist3.z -= ((dist3.z > 0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);
	dist3.z += ((dist3.z < -0.5f * paramsD.boxDims.z) ? paramsD.boxDims.z : 0);
	return dist3;
}
//--------------------------------------------------------------------------------------------------------------------------------
//distance between two particles, considering the periodic boundary condition
__device__ inline float3 Distance(float4 posRadA, float4 posRadB) {
	return Distance(F3(posRadA), F3(posRadB));
}
//--------------------------------------------------------------------------------------------------------------------------------
//distance between two particles, considering the periodic boundary condition
__device__ inline float3 Distance(float4 posRadA, float3 posRadB) {
	return Distance(F3(posRadA), posRadB);
}
//--------------------------------------------------------------------------------------------------------------------------------
void allocateArray(void **devPtr, size_t size);
void freeArray(void *devPtr);
void setParameters(SimParams *hostParams);

void computeGridSize(uint n, uint blockSize, uint &numBlocks, uint &numThreads);

void calcHash(uint* gridParticleHash, uint* gridParticleIndex, float4 * pos, int numParticles);

void reorderDataAndFindCellStart(
		uint* cellStart,
		uint* cellEnd,
		float4* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleHash,
		uint* gridParticleIndex,
		float4* oldPosRad,
		float4* oldVelMas,
		float4* oldRhoPreMu,
		uint numParticles,
		uint numCells);

void RecalcVelocity_XSPH(
		float3* vel_XSPH_D,
		float4* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numParticles,
		uint numCells);

void collide(
		float4* derivVelRhoD,
		float4* sortedPosRad,
		float4* sortedVelMas,
		float3* vel_XSPH_D,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint numParticles,
		uint numCells);

void ReCalcDensity(
		float4* posRadD,
		float4* velMasD,
		float4* rhoPresMuD,
		float4* m_dSortedPosRad,
		float4* m_dSortedVelMas,
		float4* m_dSortedRhoPreMu,
		uint* m_dGridParticleIndex,
		uint* m_dCellStart,
		uint* m_dCellEnd,
		uint numParticles,
		uint numCells);

void CalcCartesianData(
		float4* rho_Pres_CartD,
		float4* vel_VelMag_CartD,
		float4* sortedPosRad,
		float4* sortedVelMas,
		float4* sortedRhoPreMu,
		uint* gridParticleIndex,
		uint* cellStart,
		uint* cellEnd,
		uint cartesianGridSize,
		int3 cartesianGridDims,
		float resolution);

#endif
