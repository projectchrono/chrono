/*
 * SDKCollisionSystem.cu
 *
 *  Created on: Mar 2, 2013
 *      Author: Arman Pazouki
 */
#include <thrust/sort.h>
#include "chrono_fsi/SDKCollisionSystem.cuh"


//#include "extraOptionalFunctions.cuh"
//#include "SDKCollisionSystemAdditional.cuh"

/**
 * @brief calcGridHash
 * @details  See SDKCollisionSystem.cuh
 */
__device__ int3 calcGridPos(Real3 p) {
	int3 gridPos;
	gridPos.x = floor((p.x - paramsD.worldOrigin.x) / paramsD.cellSize.x);
	gridPos.y = floor((p.y - paramsD.worldOrigin.y) / paramsD.cellSize.y);
	gridPos.z = floor((p.z - paramsD.worldOrigin.z) / paramsD.cellSize.z);
	return gridPos;
}

/**
 * @brief calcGridHash
 * @details  See SDKCollisionSystem.cuh
 */
__device__ uint calcGridHash(int3 gridPos) {
	gridPos.x -= ((gridPos.x >= paramsD.gridSize.x) ? paramsD.gridSize.x : 0);
	gridPos.y -= ((gridPos.y >= paramsD.gridSize.y) ? paramsD.gridSize.y : 0);
	gridPos.z -= ((gridPos.z >= paramsD.gridSize.z) ? paramsD.gridSize.z : 0);

	gridPos.x += ((gridPos.x < 0) ? paramsD.gridSize.x : 0);
	gridPos.y += ((gridPos.y < 0) ? paramsD.gridSize.y : 0);
	gridPos.z += ((gridPos.z < 0) ? paramsD.gridSize.z : 0);

	return __umul24(__umul24(gridPos.z, paramsD.gridSize.y), paramsD.gridSize.x)
			+ __umul24(gridPos.y, paramsD.gridSize.x) + gridPos.x;
}

/**
 * @brief calcGridHash
 * @details  See SDKCollisionSystem.cuh
 */
__device__ inline Real4 DifVelocityRho(const Real3& dist3, const Real& d,
		const Real3& velMasA, const Real3& vel_XSPH_A, const Real3& velMasB,
		const Real3& vel_XSPH_B, const Real4& rhoPresMuA,
		const Real4& rhoPresMuB, Real multViscosity) {
	Real epsilonMutualDistance = .01f;
	Real3 gradW = GradW(dist3);

	// Real vAB_Dot_rAB = dot(velMasA - velMasB, dist3);

	//	//*** Artificial viscosity type 1.1
	//	Real alpha = .001;
	//	Real c_ab = 10 * paramsD.v_Max; //Ma = .1;//sqrt(7.0f * 10000 / ((rhoPresMuA.x + rhoPresMuB.x) / 2.0f));
	//	//Real h = paramsD.HSML;
	//	Real rho = .5f * (rhoPresMuA.x + rhoPresMuB.x);
	//	Real nu = alpha * paramsD.HSML * c_ab / rho;

	//	//*** Artificial viscosity type 1.2
	//	Real nu = 22.8f * paramsD.mu0 / 2.0f / (rhoPresMuA.x * rhoPresMuB.x);
	//	Real3 derivV = -paramsD.markerMass * (
	//		rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)
	//		- nu * vAB_Dot_rAB / ( d * d + epsilonMutualDistance * paramsD.HSML * paramsD.HSML )
	//		) * gradW;
	//	return mR4(derivV,
	//		rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));

	//*** Artificial viscosity type 2
	Real rAB_Dot_GradW = dot(dist3, gradW);
	Real rAB_Dot_GradW_OverDist = rAB_Dot_GradW
			/ (d * d + epsilonMutualDistance * paramsD.HSML * paramsD.HSML);
	Real3 derivV = -paramsD.markerMass
			* (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x)
					+ rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) * gradW
			+ paramsD.markerMass * (8.0f * multViscosity) * paramsD.mu0
					* pow(rhoPresMuA.x + rhoPresMuB.x, Real(-2))
					* rAB_Dot_GradW_OverDist * (velMasA - velMasB);
	Real derivRho = rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x
			* dot(vel_XSPH_A - vel_XSPH_B, gradW);
	//	Real zeta = 0;//.05;//.1;
	//	Real derivRho = rhoPresMuA.x * paramsD.markerMass * invrhoPresMuBx * (dot(vel_XSPH_A - vel_XSPH_B, gradW)
	//			+ zeta * paramsD.HSML * (10 * paramsD.v_Max) * 2 * (rhoPresMuB.x / rhoPresMuA.x - 1) *
	// rAB_Dot_GradW_OverDist
	//			);
	return mR4(derivV, derivRho);

	//	//*** Artificial viscosity type 1.3
	//	Real rAB_Dot_GradW = dot(dist3, gradW);
	//	Real3 derivV = -paramsD.markerMass * (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x *
	// rhoPresMuB.x)) * gradW
	//		+ paramsD.markerMass / (rhoPresMuA.x * rhoPresMuB.x) * 2.0f * paramsD.mu0 * rAB_Dot_GradW / ( d * d +
	// epsilonMutualDistance * paramsD.HSML * paramsD.HSML ) * (velMasA - velMasB);
	//	return mR4(derivV,
	//		rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real3 DifVelocity_SSI_DEM(const Real3& dist3, const Real& d,
		const Real3& velMasA, const Real3& velMasB) {
	// printf("** DifVelocity_SSI_DEM\n");
	Real l = paramsD.MULT_INITSPACE * paramsD.HSML - d;  // penetration distance
	if (l < 0) {
		return mR3(0);
	}
	Real kS = .00006; // 6;//3; //50; //1000.0; //392400.0;	//spring. 50 worked almost fine. I am using 30 to be
					  // sure!
	Real kD = 40; // 20;//40.0;//20.0; //420.0;				//damping coef. // 40 is good don't change it.
	Real3 n = dist3 / d;  // unit vector B to A
	Real m_eff = 0.5 * paramsD.markerMass; //(mA * mB) / (mA + mB);
	Real3 force = (/*pow(paramsD.sizeScale, Real(3)) * */kS * l
			- kD * m_eff * dot(velMasA - velMasB, n)) * n; // relative velocity at contact is simply assumed as the relative vel of the centers. If you are
														   // updating the rotation, this should be modified.
	return force / paramsD.markerMass;  // return dV/dT same as SPH
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real3 DifVelocity_SSI_Lubrication(const Real3& dist3,
		const Real& d, const Real3& velMasA, const Real3& velMasB) {
	// printf("** DifVelocity_SSI_Lubrication\n");
	Real Delta_c = paramsD.HSML;
	Real s = d - paramsD.MULT_INITSPACE * paramsD.HSML;
	if (s > Delta_c)
		return mR3(0);

	Real Delta_i = .1 * Delta_c;
	Real mult = 0;
	if (s > Delta_i) {
		mult = 1 / s - 1 / Delta_c;
	} else {
		mult = 1 / Delta_i - 1 / Delta_c;
	}
	Real3 n = dist3 / d;  // unit vector B to A
	Real3 force = -(mult * 1.5 * PI * paramsD.mu0 * paramsD.HSML * paramsD.HSML)
			* dot(velMasA - velMasB, n) * n;
	return force / paramsD.markerMass;
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ Real3 deltaVShare(int3 gridPos, uint index, Real3 posRadA,
		Real3 velMasA, Real4 rhoPresMuA, Real3* sortedPosRad,
		Real3* sortedVelMas, Real4* sortedRhoPreMu, uint* cellStart,
		uint* cellEnd) {
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	Real3 deltaV = mR3(0.0f);

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) {  // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) {  // check not colliding with self
				Real3 posRadB = FETCH(sortedPosRad, j);
				Real3 dist3 = Distance(posRadA, posRadB);
				Real d = length(dist3);
				if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
					continue;
				Real4 rhoPresMuB = FETCH(sortedRhoPreMu, j);
				if (rhoPresMuB.w > -.1)
					continue; //# B must be fluid (A was checked originally and it is fluid at this point), accoring to
				// colagrossi (2003), the other phase (i.e. rigid) should not be considered)
				Real multRho = 2.0f / (rhoPresMuA.x + rhoPresMuB.x);
				Real3 velMasB = FETCH(sortedVelMas, j);
				deltaV += paramsD.markerMass * (velMasB - velMasA) * W3(d)
						* multRho;
			}
		}
	}
	return deltaV;
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ void BCE_modification_Share(
		Real4& deltaVDenom,  // in and out
		Real4& deltaRP, int& isAffected, int3 gridPos, uint index,
		Real3 posRadA, Real3* sortedPosRad, Real3* sortedVelMas,
		Real4* sortedRhoPreMu, uint* cellStart, uint* cellEnd) {
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) {  // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			Real3 posRadB = FETCH(sortedPosRad, j);
			Real3 dist3 = Distance(posRadA, posRadB);
			Real d = length(dist3);
			if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
				continue;
			Real4 rhoPresMuB = FETCH(sortedRhoPreMu, j);

			Real Wd = W3(d);
			Real3 velMasB = FETCH(sortedVelMas, j);
			//			deltaVDenom += mR4(
			//					paramsD.markerMass / rhoPresMuB.x * velMasB * Wd,
			//					paramsD.markerMass / rhoPresMuB.x * Wd);
			//			deltaVDenom += mR4(
			//					velMasB * Wd,
			//					Wd);

			if (rhoPresMuB.w < -.1) { // only fluid pressure is used to update BCE pressure see Eq 27 of Adami, 2012 paper

				isAffected = (Wd > W3(1.99 * paramsD.HSML));

				deltaVDenom += mR4(velMasB * Wd, Wd);

				deltaRP += mR4(rhoPresMuB.x * dist3 * Wd, // Arman: check if dist3 or -dist3
				rhoPresMuB.y * Wd);
			}
		}
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
// modify pressure for body force
__device__ __inline__ void modifyPressure(Real4& rhoPresMuB,
		const Real3& dist3Alpha) {
	// body force in x direction
	rhoPresMuB.y =
			(dist3Alpha.x > 0.5 * paramsD.boxDims.x) ?
					(rhoPresMuB.y - paramsD.deltaPress.x) : rhoPresMuB.y;
	rhoPresMuB.y =
			(dist3Alpha.x < -0.5 * paramsD.boxDims.x) ?
					(rhoPresMuB.y + paramsD.deltaPress.x) : rhoPresMuB.y;
	// body force in x direction
	rhoPresMuB.y =
			(dist3Alpha.y > 0.5 * paramsD.boxDims.y) ?
					(rhoPresMuB.y - paramsD.deltaPress.y) : rhoPresMuB.y;
	rhoPresMuB.y =
			(dist3Alpha.y < -0.5 * paramsD.boxDims.y) ?
					(rhoPresMuB.y + paramsD.deltaPress.y) : rhoPresMuB.y;
	// body force in x direction
	rhoPresMuB.y =
			(dist3Alpha.z > 0.5 * paramsD.boxDims.z) ?
					(rhoPresMuB.y - paramsD.deltaPress.z) : rhoPresMuB.y;
	rhoPresMuB.y =
			(dist3Alpha.z < -0.5 * paramsD.boxDims.z) ?
					(rhoPresMuB.y + paramsD.deltaPress.z) : rhoPresMuB.y;
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ Real4 collideCell(int3 gridPos, uint index, Real3 posRadA,
		Real3 velMasA, Real3 vel_XSPH_A, Real4 rhoPresMuA, Real3* sortedPosRad,
		Real3* sortedVelMas, Real3* vel_XSPH_Sorted_D, Real4* sortedRhoPreMu,
		uint* cellStart, uint* cellEnd) {

	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	Real3 derivV = mR3(0.0f);
	Real derivRho = 0.0f;

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) {  // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) {  // check not colliding with self
				Real3 posRadB = FETCH(sortedPosRad, j);
				Real3 dist3Alpha = posRadA - posRadB;
				Real3 dist3 = Distance(posRadA, posRadB);
				Real d = length(dist3);
				if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
					continue;

				Real4 rhoPresMuB = FETCH(sortedRhoPreMu, j);
				if ((fabs(rhoPresMuB.w - rhoPresMuA.w) < .1)
						&& rhoPresMuA.w > -.1) {
					continue;
				}
				Real3 velMasB = FETCH(sortedVelMas, j);
				modifyPressure(rhoPresMuB, dist3Alpha);
				Real multViscosit = 1;
				Real4 derivVelRho = mR4(0.0f);
				Real3 vel_XSPH_B = FETCH(vel_XSPH_Sorted_D, j);
				derivVelRho = DifVelocityRho(dist3, d, velMasA, vel_XSPH_A,
						velMasB, vel_XSPH_B, rhoPresMuA, rhoPresMuB,
						multViscosit);
				derivV += mR3(derivVelRho);
				derivRho += derivVelRho.w;
			}
		}
	}

	// ff1
	//	if (rhoPresMuA.w > 0) printf("force value %f %f %f\n", 1e20*derivV.x, 1e20*derivV.y, 1e20*derivV.z);
	return mR4(derivV, derivRho);
} //--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ __inline__ void stressCell(Real3& devS3, Real3& volS3, int3 gridPos,
		uint index, Real3 posRadA, Real3 velMasA, Real4 rhoPresMuA,
		Real3* sortedPosRad, Real3* sortedVelMas, Real4* sortedRhoPreMu,
		uint* cellStart, uint* cellEnd) {

	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	Real3 derivV = mR3(0.0f);

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) {  // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) {  // check not colliding with self
				Real3 posRadB = FETCH(sortedPosRad, j);
				Real3 dist3Alpha = posRadA - posRadB;
				Real3 dist3 = Distance(posRadA, posRadB);
				Real d = length(dist3);
				if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
					continue;

				Real3 velMasB = FETCH(sortedVelMas, j);
				Real4 rhoPresMuB = FETCH(sortedRhoPreMu, j);

				Real3 vr = velMasB - velMasA;
				Real3 gradW = GradW(dist3);

				// Randles and Libersky, 1996
				devS3 += -paramsD.mu0 * paramsD.markerMass / rhoPresMuB.x
						*
						mR3(vr.x * gradW.y + vr.y * gradW.x,
								vr.x * gradW.z + vr.z * gradW.x,
								vr.y * gradW.z + vr.z * gradW.y);
				volS3 += -paramsD.mu0 * paramsD.markerMass / rhoPresMuB.x * 4.0
						/ 3.0
						* mR3(vr.x * gradW.x, vr.y * gradW.y, vr.z * gradW.z);
			}
		}
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ void collideCellDensityReInit(Real& densityShare, Real& denominator,
		int3 gridPos, uint index, Real3 posRadA, Real3* sortedPosRad,
		Real3* sortedVelMas, Real4* sortedRhoPreMu, uint* cellStart,
		uint* cellEnd) {

	//?c2 printf("grid pos %d %d %d \n", gridPos.x, gridPos.y, gridPos.z);
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	Real densityShare2 = 0.0f;
	Real denominator2 = 0.0f;

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) {  // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) {  // check not colliding with self
				Real3 posRadB = FETCH(sortedPosRad, j);
				Real4 rhoPreMuB = FETCH(sortedRhoPreMu, j);
				Real3 dist3 = Distance(posRadA, posRadB);
				Real d = length(dist3);
				if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
					continue;
				Real partialDensity = paramsD.markerMass * W3(d); // optimize it ?$
				densityShare2 += partialDensity;
				denominator2 += partialDensity / rhoPreMuB.x;
				// if (fabs(W3(d)) < .00000001) {printf("good evening, distance %f %f %f\n", dist3.x, dist3.y, dist3.z);
				// printf("posRadA %f %f %f, posRadB, %f %f %f\n", posRadA.x, posRadA.y, posRadA.z, posRadB.x, posRadB.y,
				// posRadB.z);
				//}
			}
		}
	}
	densityShare += densityShare2;
	denominator += denominator2;
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ void projectTheClosestFluidMarker(Real3& distRhoPress, int3 gridPos,
		uint index, Real3 posRadA, Real3* sortedPosRad, Real4* sortedRhoPreMu,
		uint* cellStart, uint* cellEnd) {

	//?c2 printf("grid pos %d %d %d \n", gridPos.x, gridPos.y, gridPos.z);
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) {  // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j == index)
				continue;
			Real4 rhoPreMuB = FETCH(sortedRhoPreMu, j);
			if (rhoPreMuB.w > -.1)
				continue;  // we don't care about the closest non-fluid marker
			Real3 posRadB = FETCH(sortedPosRad, j);
			Real3 dist3 = Distance(posRadA, posRadB);
			Real d = length(dist3);
			if (distRhoPress.x > d) {
				distRhoPress = mR3(d, rhoPreMuB.x, rhoPreMuB.y);
			}
		}
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ void calcOnCartesianShare(Real3& v_share, Real4& rp_share,
		int3 gridPos, Real4 gridNodePos4, Real3* sortedPosRad,
		Real3* sortedVelMas, Real4* sortedRhoPreMu, uint* cellStart,
		uint* cellEnd) {

	//?c2 printf("grid pos %d %d %d \n", gridPos.x, gridPos.y, gridPos.z);
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) {  // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			Real3 posRadB = FETCH(sortedPosRad, j);
			Real3 velMasB = FETCH(sortedVelMas, j);
			Real4 rhoPreMuB = FETCH(sortedRhoPreMu, j);
			Real3 dist3 = Distance(gridNodePos4, posRadB);
			Real d = length(dist3);
			Real mult = paramsD.markerMass / rhoPreMuB.x * W3(d);
			v_share += mult * velMasB;  // optimize it ?$
			rp_share += mult * mR4(rhoPreMuB.x, rhoPreMuB.y, 0, 0);
		}
	}
}

/**
 * @brief calcHashD
 * @details
 * 		 1. Get particle index. Determine by the block and thread we are in.
 * 		 2. From x,y,z position determine which bin it is in.
 * 		 3. Calculate hash from bin index.
 * 		 4. Store hash and particle index associated with it.
 *
 * @param gridMarkerHash
 * @param gridMarkerIndex
 * @param posRad
 * @param numAllMarkers
 */
__global__ void calcHashD(uint* gridMarkerHash,   // output
		uint* gridMarkerIndex,  // output
		Real3* posRad,          // input: positions
		Real4* rp, uint numAllMarkers) {

	/* Calculate the index of where the particle is stored in posRad. */
	uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numAllMarkers)
		return;

	Real3 p = posRad[index];

	if (!(isfinite(p.x) && isfinite(p.y) && isfinite(p.z))) {
		printf("Error! particle position is NAN: thrown from SDKCollisionSystem.cu, calcHashD !\n");
	}

	/* Check particle is inside the domain. */
	Real3 boxCorner = paramsD.worldOrigin;
	if (p.x < boxCorner.x || p.y < boxCorner.y || p.z < boxCorner.z) {
		printf("Out of Min Boundary\n");
		return;
	}
	boxCorner = paramsD.worldOrigin + paramsD.boxDims;
	if (p.x > boxCorner.x || p.y > boxCorner.y || p.z > boxCorner.z) {
		printf(
				"Out of max Boundary, point %f %f %f, type %f, boundary max: %f %f %f \n",
				p.x, p.y, p.z, rp->w, boxCorner.x, boxCorner.y, boxCorner.z);
		return;
	}

	/* Get x,y,z bin index in grid */
	int3 gridPos = calcGridPos(p);
	/* Calculate a hash from the bin index */
	uint hash = calcGridHash(gridPos);

	/* Store grid hash */
	gridMarkerHash[index] = hash;
	/* Store particle index associated to the hash we stored in gridMarkerHash */
	gridMarkerIndex[index] = index;
}

/**
 * @brief reorderDataAndFindCellStartD
 * @details See SDKCollisionSystem.cuh for more info
 */
__global__ void reorderDataAndFindCellStartD(uint* cellStart, // output: cell start index
		uint* cellEnd,        // output: cell end index
		Real3* sortedPosRad,  // output: sorted positions
		Real3* sortedVelMas,  // output: sorted velocities
		Real4* sortedRhoPreMu, uint* gridMarkerHash, // input: sorted grid hashes
		uint* gridMarkerIndex,      // input: sorted particle indices
		uint* mapOriginalToSorted, // mapOriginalToSorted[originalIndex] = originalIndex
		Real3* oldPosRad,           // input: sorted position array
		Real3* oldVelMas,           // input: sorted velocity array
		Real4* oldRhoPreMu, uint numAllMarkers) {
	extern __shared__ uint sharedHash[];  // blockSize + 1 elements
	/* Get the particle index the current thread is supposed to be looking at. */
	uint index = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
	uint hash;
	/* handle case when no. of particles not multiple of block size */
	if (index < numAllMarkers) {
		hash = gridMarkerHash[index];
		/* Load hash data into shared memory so that we can look at neighboring particle's hash
		 * value without loading two hash values per thread
		 */
		sharedHash[threadIdx.x + 1] = hash;

		if (index > 0 && threadIdx.x == 0) {
			/* first thread in block must load neighbor particle hash */
			sharedHash[0] = gridMarkerHash[index - 1];
		}
	}

	__syncthreads();

	if (index < numAllMarkers) {
		/* If this particle has a different cell index to the previous particle then it must be
		 * the first particle in the cell, so store the index of this particle in the cell. As it
		 * isn't the first particle, it must also be the cell end of the previous particle's cell
		 */
		if (index == 0 || hash != sharedHash[threadIdx.x]) {
			cellStart[hash] = index;
			if (index > 0)
				cellEnd[sharedHash[threadIdx.x]] = index;
		}

		if (index == numAllMarkers - 1) {
			cellEnd[hash] = index + 1;
		}

		/* Now use the sorted index to reorder the pos and vel data */
		uint originalIndex = gridMarkerIndex[index];  // map sorted to original
		mapOriginalToSorted[index] = index;	// will be sorted outside. Alternatively, you could have mapOriginalToSorted[originalIndex] = index; without need to sort. But that is not thread safe
		Real3 posRad = FETCH(oldPosRad, originalIndex); // macro does either global read or texture fetch
		Real3 velMas = FETCH(oldVelMas, originalIndex); // see particles_kernel.cuh
		Real4 rhoPreMu = FETCH(oldRhoPreMu, originalIndex);

		if (!(isfinite(posRad.x) && isfinite(posRad.y)
				&& isfinite(posRad.z))) {
			printf("Error! particle position is NAN: thrown from SDKCollisionSystem.cu, reorderDataAndFindCellStartD !\n");
		}
		if (!(isfinite(velMas.x) && isfinite(velMas.y)
				&& isfinite(velMas.z))) {
			printf("Error! particle velocity is NAN: thrown from SDKCollisionSystem.cu, reorderDataAndFindCellStartD !\n");
		}
		if (!(isfinite(rhoPreMu.x) && isfinite(rhoPreMu.y)
				&& isfinite(rhoPreMu.z) && isfinite(rhoPreMu.w))) {
			printf("Error! particle rhoPreMu is NAN: thrown from SDKCollisionSystem.cu, reorderDataAndFindCellStartD !\n");
		}
		sortedPosRad[index] = posRad;
		sortedVelMas[index] = velMas;
		sortedRhoPreMu[index] = rhoPreMu;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void newVel_XSPH_D(Real3* vel_XSPH_Sorted_D,  // output: new velocity
		Real3* sortedPosRad,       // input: sorted positions
		Real3* sortedVelMas,       // input: sorted velocities
		Real4* sortedRhoPreMu, uint* gridMarkerIndex, // input: sorted particle indices
		uint* cellStart, uint* cellEnd, uint numAllMarkers) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numAllMarkers)
		return;

	// read particle data from sorted arrays

	Real4 rhoPreMuA = FETCH(sortedRhoPreMu, index);
	Real3 velMasA = FETCH(sortedVelMas, index);
	if (rhoPreMuA.w > -0.1) { // v_XSPH is calculated only for fluid markers. Keep unchanged if not fluid.
		vel_XSPH_Sorted_D[index] = velMasA;
		return;
	}

	Real3 posRadA = FETCH(sortedPosRad, index);
	Real3 deltaV = mR3(0);

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	/// if (gridPos.x == paramsD.gridSize.x-1) printf("****aha %d %d\n", gridPos.x, paramsD.gridSize.x);

	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + mI3(x, y, z);
				deltaV += deltaVShare(neighbourPos, index, posRadA, velMasA,
						rhoPreMuA, sortedPosRad, sortedVelMas, sortedRhoPreMu,
						cellStart, cellEnd);
			}
		}
	}
	//   // write new velocity back to original unsorted location
	// sortedVel_XSPH[index] = velMasA + paramsD.EPS_XSPH * deltaV;

	// write new velocity back to original unsorted location
	// uint originalIndex = gridMarkerIndex[index];
	Real3 vXSPH = velMasA + paramsD.EPS_XSPH * deltaV;
	if (!(isfinite(vXSPH.x) && isfinite(vXSPH.y)
			&& isfinite(vXSPH.z))) {
		printf("Error! particle vXSPH is NAN: thrown from SDKCollisionSystem.cu, newVel_XSPH_D !\n");
	}
	vel_XSPH_Sorted_D[index] = vXSPH;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void new_BCE_VelocityPressure(
		Real3* sortedVelMas_ModifiedBCE,    // input: sorted velocities
		Real4* sortedRhoPreMu_ModifiedBCE,  // input: sorted velocities
		Real3* sortedPosRad,                // input: sorted positions
		Real3* sortedVelMas,                // input: sorted velocities
		Real4* sortedRhoPreMu, uint* cellStart, uint* cellEnd,
		uint numAllMarkers) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numAllMarkers)
		return;
	Real4 rhoPreMuA = FETCH(sortedRhoPreMu, index);
	if (rhoPreMuA.w < -0.1) {  // keep unchanged if fluid
		return;
	}

	// read particle data from sorted arrays
	Real3 posRadA = FETCH(sortedPosRad, index);
	Real3 velMasA = FETCH(sortedVelMas, index);
	int isAffected = 0;

	Real4 deltaVDenom = mR4(0);
	Real4 deltaRP = mR4(0);

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	/// if (gridPos.x == paramsD.gridSize.x-1) printf("****aha %d %d\n", gridPos.x, paramsD.gridSize.x);

	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + mI3(x, y, z);
				BCE_modification_Share(deltaVDenom, deltaRP, isAffected,
						neighbourPos, index, posRadA, sortedPosRad,
						sortedVelMas, sortedRhoPreMu, cellStart, cellEnd);
			}
		}
	}

	if (isAffected) {
		Real3 modifiedBCE_v = 2 * velMasA - mR3(deltaVDenom) / deltaVDenom.w;
		sortedVelMas_ModifiedBCE[index] = modifiedBCE_v;

		Real pressure = (deltaRP.w + dot(paramsD.gravity, mR3(deltaRP)))
				/ deltaVDenom.w;  //(in fact:  (paramsD.gravity -
		// aW), but aW for moving rigids
		// is hard to calc. Assume aW is
		// zero for now
		Real density = InvEos(pressure);
		sortedRhoPreMu_ModifiedBCE[index] = mR4(density, pressure, rhoPreMuA.z,
				rhoPreMuA.w);
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void collideD(Real4* sortedDerivVelRho_fsi_D,  // output: new velocity
		Real3* sortedPosRad,  // input: sorted positions
		Real3* sortedVelMas,  // input: sorted velocities
		Real3* vel_XSPH_Sorted_D, Real4* sortedRhoPreMu,
		uint* cellStart, uint* cellEnd, uint numAllMarkers) {

	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numAllMarkers)
		return;

	// read particle data from sorted arrays
	Real3 posRadA = FETCH(sortedPosRad, index);
	Real3 velMasA = FETCH(sortedVelMas, index);
	Real4 rhoPreMuA = FETCH(sortedRhoPreMu, index);

//	uint originalIndex = gridMarkerIndex[index];
	Real3 vel_XSPH_A = FETCH(vel_XSPH_Sorted_D, index);

	Real4 derivVelRho = sortedDerivVelRho_fsi_D[index];

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	// examine neighbouring cells
	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			for (int z = -1; z <= 1; z++) {
				derivVelRho += collideCell(gridPos + mI3(x, y, z), index,
						posRadA, velMasA, vel_XSPH_A, rhoPreMuA, sortedPosRad,
						sortedVelMas, vel_XSPH_Sorted_D, sortedRhoPreMu,
						cellStart, cellEnd);
			}
		}
	}

	// write new velocity back to original unsorted location
	// *** let's tweak a little bit :)
	if (!(isfinite(derivVelRho.x) && isfinite(derivVelRho.y)
			&& isfinite(derivVelRho.z) && isfinite(derivVelRho.w))) {
		printf("Error! particle derivVelRho is NAN: thrown from SDKCollisionSystem.cu, collideD !\n");
	}
	sortedDerivVelRho_fsi_D[index] = derivVelRho;
}
//--------------------------------------------------------------------------------------------------------------------------------
// calculate particles stresses
__global__ void CalcBCE_Stresses_kernel(Real3* devStressD, Real3* volStressD,
		Real3* sortedPosRad, Real3* sortedVelMas, Real4* sortedRhoPreMu,
		uint* mapOriginalToSorted, uint* cellStart, uint* cellEnd, int numBCE) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numBCE) {
		return;
	}
	// Arman take care of this
	uint BCE_Index = index
			+ min(numObjectsD.startRigidMarkers, numObjectsD.startRigidMarkers); // updatePortion = [start, end] index of the update portion
	uint originalIndex = mapOriginalToSorted[BCE_Index]; // index in the sorted array

	// read particle data from sorted arrays
	Real3 posRadA = FETCH(sortedPosRad, originalIndex);
	Real3 velMasA = FETCH(sortedVelMas, originalIndex);
	Real4 rhoPreMuA = FETCH(sortedRhoPreMu, originalIndex);

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	Real3 devS3 = mR3(0);
	Real3 volS3 = mR3(0);

	// examine neighbouring cells
	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			for (int z = -1; z <= 1; z++) {
				stressCell(devS3, volS3, gridPos + mI3(x, y, z), originalIndex,
						posRadA, velMasA, rhoPreMuA, sortedPosRad, sortedVelMas,
						sortedRhoPreMu, cellStart, cellEnd);
			}
		}
	}

	devStressD[index] = devS3;
	volStressD[index] = volS3;
}
//--------------------------------------------------------------------------------------------------------------------------------
// calculate particles stresses
__global__ void CalcBCE_MainStresses_kernel(Real4* mainStressD,
		Real3* devStressD, Real3* volStressD, int numBCE) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numBCE) {
		return;
	}

	Real3 devS3 = devStressD[index];
	Real3 volS3 = volStressD[index];
	Real4 mainS3 = mR4(0);
	mainS3.w = sqrt(
			.5
					* (pow(volS3.x - volS3.y, Real(2))
							+ pow(volS3.x - volS3.z, Real(2))
							+ pow(volS3.y - volS3.z, Real(2))
							+ 6
									* (devS3.x * devS3.x + devS3.y * devS3.y
											+ devS3.z * devS3.z)));

	mainStressD[index] = mainS3;
}
//--------------------------------------------------------------------------------------------------------------------------------
// without normalization
__global__ void ReCalcDensityD_F1(Real3* oldPosRad, Real3* oldVelMas,
		Real4* oldRhoPreMu, Real3* sortedPosRad, Real3* sortedVelMas,
		Real4* sortedRhoPreMu, uint* gridMarkerIndex, uint* cellStart,
		uint* cellEnd, uint numAllMarkers) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numAllMarkers)
		return;

	// read particle data from sorted arrays
	Real3 posRadA = FETCH(sortedPosRad, index);
	Real4 rhoPreMuA = FETCH(sortedRhoPreMu, index);

	if (rhoPreMuA.w > -.1)
		return;

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	Real densityShare = 0.0f;
	Real denominator = 0.0f;
	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + mI3(x, y, z);
				collideCellDensityReInit(densityShare, denominator,
						neighbourPos, index, posRadA, sortedPosRad,
						sortedVelMas, sortedRhoPreMu, cellStart, cellEnd);
			}
		}
	}
	// write new velocity back to original unsorted location
	uint originalIndex = gridMarkerIndex[index];

	Real newDensity = densityShare + paramsD.markerMass * W3(0); //?$ include the particle in its summation as well
	Real newDenominator = denominator
			+ paramsD.markerMass * W3(0) / rhoPreMuA.x;
	if (rhoPreMuA.w < 0) {
		//		rhoPreMuA.x = newDensity; // old version
		rhoPreMuA.x = newDensity / newDenominator;  // correct version
	}
	rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
	oldRhoPreMu[originalIndex] = rhoPreMuA;
}
//--------------------------------------------------------------------------------------------------------------------------------
// without normalization
__global__ void ProjectDensityPressureToBCandBCE_D(Real4* oldRhoPreMu,
		Real3* sortedPosRad, Real4* sortedRhoPreMu, uint* gridMarkerIndex,
		uint* cellStart, uint* cellEnd, uint numAllMarkers) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numAllMarkers)
		return;

	// read particle data from sorted arrays
	Real3 posRadA = FETCH(sortedPosRad, index);
	Real4 rhoPreMuA = FETCH(sortedRhoPreMu, index);

	if (rhoPreMuA.w < -.1)
		return;

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	Real3 distRhoPress =
	mR3((RESOLUTION_LENGTH_MULT + 2) * paramsD.HSML, rhoPreMuA.x, rhoPreMuA.y); //(large distance, rhoA, pA)
	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + mI3(x, y, z);
				projectTheClosestFluidMarker(distRhoPress, neighbourPos, index,
						posRadA, sortedPosRad, sortedRhoPreMu, cellStart,
						cellEnd);
			}
		}
	}
	// write new velocity back to original unsorted location
	uint originalIndex = gridMarkerIndex[index];
	rhoPreMuA.x = distRhoPress.y;
	rhoPreMuA.y = distRhoPress.z;
	oldRhoPreMu[originalIndex] = rhoPreMuA;
}
//--------------------------------------------------------------------------------------------------------------------------------
// without normalization
__global__ void CalcCartesianDataD(Real4* rho_Pres_CartD,
		Real4* vel_VelMag_CartD, Real3* sortedPosRad, Real3* sortedVelMas,
		Real4* sortedRhoPreMu, uint* gridMarkerIndex, uint* cellStart,
		uint* cellEnd, int3 cartesianGridDims, Real resolution) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index
			>= cartesianGridDims.x * cartesianGridDims.y * cartesianGridDims.z)
		return;

	int3 gridLoc;
	gridLoc.z = index / (cartesianGridDims.x * cartesianGridDims.y);
	gridLoc.y = (index % (cartesianGridDims.x * cartesianGridDims.y))
			/ cartesianGridDims.x;
	gridLoc.x = (index % (cartesianGridDims.x * cartesianGridDims.y))
			% cartesianGridDims.x;
	// alias cartesianGridDims = Dim,  you can say:   "index = (Dim.x * Dim.y) * gridLoc.z + Dim.x * gridLoc.y +
	// gridLoc.x"

	// get address in grid
	Real3 gridNodePos3 = mR3(gridLoc) * resolution + paramsD.worldOrigin;
	int3 gridPos = calcGridPos(gridNodePos3);

	Real3 vel_share = mR3(0.0f);
	Real4 rho_pres_share = mR4(0.0f);
	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + mI3(x, y, z);
				calcOnCartesianShare(vel_share, rho_pres_share, neighbourPos,
				mR4(gridNodePos3), sortedPosRad, sortedVelMas, sortedRhoPreMu,
						cellStart, cellEnd);
			}
		}
	}
	// write new velocity back to original unsorted location
	//  uint originalIndex = gridMarkerIndex[index];

	// Real newDensity = densityShare + paramsD.markerMass * W3(0); //?$ include the particle in its summation as well
	// if (rhoPreMuA.w < -.1) { rhoPreMuA.x = newDensity; }
	// rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
	//   oldRhoPreMu[originalIndex] = rhoPreMuA;
	/////printf("density %f\n", rhoPreMuA.x);
	/////printf("densityshare %f\n", densityShare);
	/////printf("gridPos x y z %d %d %d %f\n", gridPos.x, gridPos.y, gridPos.z, densityShare);
	rho_Pres_CartD[index] = rho_pres_share;
	vel_VelMag_CartD[index] = mR4(vel_share, length(vel_share));
}

//%%%%%%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateFluidD(Real3* posRadD, Real3* velMasD, Real3* vel_XSPH_D,
		Real4* rhoPresMuD, Real4* derivVelRhoD, int2 updatePortion, Real dT) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortion.x; // updatePortion = [start, end] index of the update portion
	if (index >= updatePortion.y) {
		return;
	}
	Real4 derivVelRho = derivVelRhoD[index];
	Real4 rhoPresMu = rhoPresMuD[index];

	if (rhoPresMu.w < 0) {
		Real3 vel_XSPH = vel_XSPH_D[index];
		// 0** if you have rigid BCE, make sure to apply same tweaks to them, to satify action/reaction. Or apply tweak to
		// force in advance
		// 1*** let's tweak a little bit :)
		if (length(vel_XSPH) > paramsD.tweakMultV * paramsD.HSML / paramsD.dT
				&& paramsD.enableTweak) {
			vel_XSPH *= (paramsD.tweakMultV * paramsD.HSML / paramsD.dT)
					/ length(vel_XSPH);
			if (length(vel_XSPH)
					> 1.001 * paramsD.tweakMultV * paramsD.HSML / paramsD.dT) { // infinity
				if (paramsD.enableAggressiveTweak) {
					vel_XSPH = mR3(0);
				} else {
					printf("Error! Infinite vel_XSPH detected!\n");
				}
			}
		}
		// 1*** end tweak

		Real3 posRad = posRadD[index];
		Real3 updatedPositon = posRad + vel_XSPH * dT;
		posRadD[index] = updatedPositon;  // posRadD updated

		Real3 velMas = velMasD[index];
		Real3 updatedVelocity = velMas + mR3(derivVelRho) * dT;
		// 2*** let's tweak a little bit :)
		if (length(updatedVelocity)
				> paramsD.tweakMultV * paramsD.HSML / paramsD.dT
				&& paramsD.enableTweak) {
			updatedVelocity *= (paramsD.tweakMultV * paramsD.HSML / paramsD.dT)
					/ length(updatedVelocity);
			if (length(updatedVelocity)
					> 1.001 * paramsD.tweakMultV * paramsD.HSML / paramsD.dT) { // infinity
				if (paramsD.enableAggressiveTweak) {
					updatedVelocity = mR3(0);
				} else {
					printf("Error! Infinite updatedVelocity detected!\n");
				}
			}
		}
		// 2*** end tweak
		velMasD[index] = updatedVelocity;

	}
	// 3*** let's tweak a little bit :)
	if (fabs(derivVelRho.w) > paramsD.tweakMultRho * paramsD.rho0 / paramsD.dT
			&& paramsD.enableTweak) {
		derivVelRho.w *= (paramsD.tweakMultRho * paramsD.rho0 / paramsD.dT)
				/ fabs(derivVelRho.w);  // to take care of the sign as well
		if (fabs(derivVelRho.w)
				> 1.001 * paramsD.tweakMultRho * paramsD.rho0 / paramsD.dT) {
			if (paramsD.enableAggressiveTweak) {
				derivVelRho.w = 0;
			} else {
				printf("Error! Infinite derivRho detected!\n");
			}
		}
	}
	// 2*** end tweak
	Real rho2 = rhoPresMu.x + derivVelRho.w * dT; // rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu;  // rhoPresMuD updated
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateFluidD_init_LF(Real3* posRadD, Real3* velMasD_half,
		Real4* rhoPresMuD_half, Real4* derivVelRhoD, int2 updatePortion,
		Real dT) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortion.x; // updatePortion = [start, end] index of the update portion
	if (index >= updatePortion.y) {
		return;
	}

	Real4 derivVelRho = derivVelRhoD[index];
	Real3 velMas = velMasD_half[index];
	Real3 updatedVelocity = velMas + mR3(derivVelRho) * (0.5 * dT);
	velMasD_half[index] = updatedVelocity;  // velMasD_half updated

	Real3 posRad = posRadD[index];
	Real3 updatedPositon = posRad + updatedVelocity * dT;
	posRadD[index] = updatedPositon;  // posRadD updated

	Real4 rhoPresMu = rhoPresMuD_half[index];
	Real rho2 = rhoPresMu.x + derivVelRho.w * (0.5 * dT); // rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD_half[index] = rhoPresMu;  // rhoPresMuD_half updated
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateFluidD_rho_vel_LF(Real3* velMasD, Real4* rhoPresMuD,
		Real3* velMasD_old, Real4* rhoPresMuD_old, Real4* derivVelRhoD,
		int2 updatePortion, Real dT) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortion.x; // updatePortion = [start, end] index of the update portion
	if (index >= updatePortion.y) {
		return;
	}

	Real4 derivVelRho = derivVelRhoD[index];
	Real3 velMas = velMasD_old[index];
	Real3 updatedVelocity = velMas + mR3(derivVelRho) * dT;
	// 2*** let's tweak a little bit :)
	//	if (length(updatedVelocity) > .1 * paramsD.HSML / dT  && paramsD.enableTweak) {
	//		updatedVelocity *= ( .1 * paramsD.HSML / dT ) / length(updatedVelocity);
	//		if (length(updatedVelocity) > 1.001) { // infinity
	//			if (paramsD.enableAggressiveTweak) {
	//				updatedVelocity = mR3(0);
	//			} else {
	//				printf("Error! Infinite updatedVelocity detected!\n");
	//			}
	//		}
	//	}
	// 2*** end tweak
	velMasD[index] = updatedVelocity; // velMasD_half updated

	Real4 rhoPresMu = rhoPresMuD_old[index];

	// 3*** let's tweak a little bit :)
	//	if (fabs(derivVelRho.w) > .002 * paramsD.rho0 / dT  && paramsD.enableTweak) {
	//		derivVelRho.w *= (.002 * paramsD.rho0 / dT) / fabs(derivVelRho.w); //to take care of the sign as well
	//		if (fabs(derivVelRho.w) > 00201 * paramsD.rho0 / dT) {
	//			if (paramsD.enableAggressiveTweak) {
	//				derivVelRho.w = 0;
	//			} else {
	//				printf("Error! Infinite derivRho detected!\n");
	//			}
	//		}
	//	}
	// 2*** end tweak
	Real rho2 = rhoPresMu.x + derivVelRho.w * dT; // rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu;  // rhoPresMuD_half updated
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateFluidD_EveryThing_LF(Real3* posRadD, Real3* velMasD_half,
		Real4* rhoPresMuD_half, Real4* derivVelRhoD, int2 updatePortion,
		Real dT) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortion.x; // updatePortion = [start, end] index of the update portion
	if (index >= updatePortion.y) {
		return;
	}

	Real4 derivVelRho = derivVelRhoD[index];
	Real3 velMas = velMasD_half[index];
	Real3 updatedVelocity = velMas + mR3(derivVelRho) * dT;
	// 2*** let's tweak a little bit :)
	//	if (length(updatedVelocity) > .1 * paramsD.HSML / dT  && paramsD.enableTweak) {
	//		updatedVelocity *= ( .1 * paramsD.HSML / dT ) / length(updatedVelocity);
	//		if (length(updatedVelocity) > 1.001) { // infinity
	//			if (paramsD.enableAggressiveTweak) {
	//				updatedVelocity = mR3(0);
	//			} else {
	//				printf("Error! Infinite updatedVelocity detected!\n");
	//			}
	//		}
	//	}
	// 2*** end tweak
	velMasD_half[index] = updatedVelocity;  // velMasD_half updated

	posRadD[index] += updatedVelocity * dT;  // posRadD updated

	Real4 rhoPresMu = rhoPresMuD_half[index];

	// 3*** let's tweak a little bit :)
	//	if (fabs(derivVelRho.w) > .002 * paramsD.rho0 / dT  && paramsD.enableTweak) {
	//		derivVelRho.w *= (.002 * paramsD.rho0 / dT) / fabs(derivVelRho.w); //to take care of the sign as well
	//		if (fabs(derivVelRho.w) > 00201 * paramsD.rho0 / dT) {
	//			if (paramsD.enableAggressiveTweak) {
	//				derivVelRho.w = 0;
	//			} else {
	//				printf("Error! Infinite derivRho detected!\n");
	//			}
	//		}
	//	}
	// 2*** end tweak
	Real rho2 = rhoPresMu.x + derivVelRho.w * dT; // rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD_half[index] = rhoPresMu;  // rhoPresMuD_half updated
}

/**
 * @brief Copies the sortedVelXSPH to velXSPH according to indexing
 * @details [long description]
 *
 * @param vel_XSPH_D
 * @param vel_XSPH_Sorted_D Pointer to new sorted vel_XSPH vector
 * @param m_dGridMarkerIndex List of indeces used to sort vel_XSPH_D
 */

__global__ void CopySorted_vXSPH_dVdRho_to_original_kernel(Real3* vel_XSPH_D,
		Real4* derivVelRhoD,
		Real3* vel_XSPH_Sorted_D, Real4* sortedDerivVelRho_fsi_D,
		uint* mapOriginalToSorted) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numObjectsD.numAllMarkers)
		return;
	vel_XSPH_D[index] = vel_XSPH_Sorted_D[mapOriginalToSorted[index]];
	derivVelRhoD[index] = sortedDerivVelRho_fsi_D[mapOriginalToSorted[index]];
}

//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles' properties, i.e. velocity, density, pressure, position
__global__ void UpdateKernelBoundary(Real3* posRadD, Real3* velMasD,
		Real4* rhoPresMuD, Real4* derivVelRhoD, int2 updatePortion, Real dT) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	index += updatePortion.x; // updatePortion = [start, end] index of the update portion
	if (index >= updatePortion.y) {
		return;
	}

	Real4 derivVelRho = derivVelRhoD[index];
	Real4 rhoPresMu = rhoPresMuD[index];
	Real rho2 = rhoPresMu.x + derivVelRho.w * dT; // rho update. (i.e. rhoPresMu.x), still not wriiten to global matrix
	rhoPresMu.y = Eos(rho2, rhoPresMu.w);
	rhoPresMu.x = rho2;
	rhoPresMuD[index] = rhoPresMu;  // rhoPresMuD updated
}

//--------------------------------------------------------------------------------------------------------------------------------
// applies periodic BC along x
__global__ void ApplyPeriodicBoundaryXKernel(Real3* posRadD,
		Real4* rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numAllMarkers) {
		return;
	}
	Real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	}  // no need to do anything if it is a boundary particle
	Real3 posRad = posRadD[index];
	if (posRad.x > paramsD.cMax.x) {
		posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.x;
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
	if (posRad.x < paramsD.cMin.x) {
		posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.x;
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
// applies periodic BC along y
__global__ void ApplyPeriodicBoundaryYKernel(Real3* posRadD,
		Real4* rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numAllMarkers) {
		return;
	}
	Real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	}  // no need to do anything if it is a boundary particle
	Real3 posRad = posRadD[index];
	if (posRad.y > paramsD.cMax.y) {
		posRad.y -= (paramsD.cMax.y - paramsD.cMin.y);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.y;
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
	if (posRad.y < paramsD.cMin.y) {
		posRad.y += (paramsD.cMax.y - paramsD.cMin.y);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.y;
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------
// applies periodic BC along z
__global__ void ApplyPeriodicBoundaryZKernel(Real3* posRadD,
		Real4* rhoPresMuD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numAllMarkers) {
		return;
	}
	Real4 rhoPresMu = rhoPresMuD[index];
	if (fabs(rhoPresMu.w) < .1) {
		return;
	}  // no need to do anything if it is a boundary particle
	Real3 posRad = posRadD[index];
	if (posRad.z > paramsD.cMax.z) {
		posRad.z -= (paramsD.cMax.z - paramsD.cMin.z);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.z;
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
	if (posRad.z < paramsD.cMin.z) {
		posRad.z += (paramsD.cMax.z - paramsD.cMin.z);
		posRadD[index] = posRad;
		if (rhoPresMu.w < -.1) {
			rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.z;
			rhoPresMuD[index] = rhoPresMu;
		}
		return;
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------------------------------------------------------------------
void allocateArray(void** devPtr, size_t size) {
	cudaMalloc(devPtr, size);
}
//--------------------------------------------------------------------------------------------------------------------------------
void freeArray(void* devPtr) {
	cudaFree(devPtr);
}

/**
 * @brief iDivUp
 * @details Round a / b to nearest higher integer value
 *
 * @param a numerator
 * @param b denominator
 *
 * @return ceil(a/b)
 */
uint iDivUp(uint a, uint b) {
	return (a % b != 0) ? (a / b + 1) : (a / b);
}

/**
 * @brief computeGridSize
 * @details Compute grid and thread block size for a given number of elements
 *
 * @param n Total number of elements. Each elements needs a thread to be computed
 * @param blockSize Number of threads per block.
 * @param numBlocks output
 * @param numThreads Output: number of threads per block
 */
void computeGridSize(uint n, uint blockSize, uint& numBlocks,
		uint& numThreads) {
	uint n2 = (n == 0) ? 1 : n;
	numThreads = min(blockSize, n2);
	numBlocks = iDivUp(n2, numThreads);
}

/**
 * @brief [brief description]
 * @details [long description]
 *
 * @param hostParams [description]
 * @param numObjects [description]
 */
void setParameters(SimParams* hostParams, NumberOfObjects* numObjects) {
	// copy parameters to constant memory
	cudaMemcpyToSymbolAsync(paramsD, hostParams, sizeof(SimParams));
	cudaMemcpyToSymbolAsync(numObjectsD, numObjects, sizeof(NumberOfObjects));
}

/**
 * @brief Wrapper function for calcHashD
 * @details See SDKCollisionSystem.cuh for more info
 */
void calcHash(thrust::device_vector<uint>& gridMarkerHash,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<Real3>& posRad,
		thrust::device_vector<Real4>& rhoPreMu, int numAllMarkers) {
	/* Is there a need to optimize the number of threads used at once? */
	uint numThreads, numBlocks;

	computeGridSize(numAllMarkers, 256, numBlocks, numThreads);

	/* Execute Kernel */
	calcHashD<<<numBlocks, numThreads>>>(U1CAST(gridMarkerHash),
			U1CAST(gridMarkerIndex), mR3CAST(posRad), mR4CAST(rhoPreMu),
			numAllMarkers);

	/* Check for errors in kernel execution */
	cudaThreadSynchronize();
	cudaCheckError()
	;
}

/**
 * @brief Wrapper function for reorderDataAndFindCellStartD
 * @details
 * 		See SDKCollisionSystem.cuh for brief.
 */
void reorderDataAndFindCellStart(thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,

		thrust::device_vector<uint>& gridMarkerHash,
		thrust::device_vector<uint>& gridMarkerIndex,

		thrust::device_vector<uint>& mapOriginalToSorted,

		thrust::device_vector<Real3>& oldPosRad,
		thrust::device_vector<Real3>& oldVelMas,
		thrust::device_vector<Real4>& oldRhoPreMu, uint numAllMarkers,
		uint numCells) {
	uint numThreads, numBlocks;
	computeGridSize(numAllMarkers, 256, numBlocks, numThreads); //?$ 256 is blockSize

	/* Set all cells to empty */
	cudaMemset(U1CAST(cellStart), 0xffffffff, numCells * sizeof(uint));

	//#if USE_TEX
	//#if 0
	//    cutilSafeCall(cudaBindTexture(0, oldPosTex, oldPosRad, numAllMarkers*sizeof(Real4)));
	//    cutilSafeCall(cudaBindTexture(0, oldVelTex, oldVelMas, numAllMarkers*sizeof(Real4)));
	//#endif

	uint smemSize = sizeof(uint) * (numThreads + 1);
	reorderDataAndFindCellStartD<<<numBlocks, numThreads, smemSize>>>(
			U1CAST(cellStart), U1CAST(cellEnd), mR3CAST(sortedPosRad),
			mR3CAST(sortedVelMas), mR4CAST(sortedRhoPreMu),
			U1CAST(gridMarkerHash), U1CAST(gridMarkerIndex),
			U1CAST(mapOriginalToSorted), mR3CAST(oldPosRad), mR3CAST(oldVelMas),
			mR4CAST(oldRhoPreMu), numAllMarkers);
	cudaThreadSynchronize();
	cudaCheckError()
	;

	// unroll sorted index to have the location of original particles in the sorted arrays
	thrust::device_vector<uint> dummyIndex = gridMarkerIndex;
	thrust::sort_by_key(dummyIndex.begin(), dummyIndex.end(),
			mapOriginalToSorted.begin());
	dummyIndex.clear();
	//#if USE_TEX
	//#if 0
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//#endif
}

/**
 * @brief Wrapper function for newVel_XSPH_D
 */
void RecalcVelocity_XSPH(thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers,
		uint numCells) {
	/* thread per particle */
	uint numThreads, numBlocks;
	computeGridSize(numAllMarkers, 64, numBlocks, numThreads);

	/* Execute the kernel */
	newVel_XSPH_D<<<numBlocks, numThreads>>>(mR3CAST(vel_XSPH_Sorted_D),
			mR3CAST(sortedPosRad), mR3CAST(sortedVelMas),
			mR4CAST(sortedRhoPreMu), U1CAST(gridMarkerIndex), U1CAST(cellStart),
			U1CAST(cellEnd), numAllMarkers);

	cudaThreadSynchronize();
	cudaCheckError()
	;
}
//--------------------------------------------------------------------------------------------------------------------------------
void RecalcSortedVelocityPressure_BCE(
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers) {
	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numAllMarkers, 64, numBlocks, numThreads);

	// Arman modified BCE velocity version
	thrust::device_vector<Real3> sortedVelMas_ModifiedBCE = sortedVelMas;
	thrust::device_vector<Real4> sortedRhoPreMu_ModifiedBCE = sortedRhoPreMu;

	new_BCE_VelocityPressure<<<numBlocks, numThreads>>>(
			mR3CAST(sortedVelMas_ModifiedBCE),
			mR4CAST(sortedRhoPreMu_ModifiedBCE),  // input: sorted velocities
			mR3CAST(sortedPosRad), mR3CAST(sortedVelMas),
			mR4CAST(sortedRhoPreMu), U1CAST(cellStart), U1CAST(cellEnd),
			numAllMarkers);
	cudaThreadSynchronize();
	cudaCheckError()
	;

	thrust::copy(sortedVelMas_ModifiedBCE.begin(),
			sortedVelMas_ModifiedBCE.end(), sortedVelMas.begin());
	thrust::copy(sortedRhoPreMu_ModifiedBCE.begin(),
			sortedRhoPreMu_ModifiedBCE.end(), sortedRhoPreMu.begin());

	sortedVelMas_ModifiedBCE.clear();
	sortedRhoPreMu_ModifiedBCE.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
void CalcBCE_Stresses(thrust::device_vector<Real3>& devStressD,
		thrust::device_vector<Real3>& volStressD,
		thrust::device_vector<Real4>& mainStressD,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& mapOriginalToSorted,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, int numBCE) {
	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numBCE, 128, numBlocks, numThreads);
	CalcBCE_Stresses_kernel<<<numBlocks, numThreads>>>(mR3CAST(devStressD),
			mR3CAST(volStressD), mR3CAST(sortedPosRad), mR3CAST(sortedVelMas),
			mR4CAST(sortedRhoPreMu), U1CAST(mapOriginalToSorted),
			U1CAST(cellStart), U1CAST(cellEnd), numBCE);

	cudaThreadSynchronize();
	cudaCheckError()
	;

	CalcBCE_MainStresses_kernel<<<numBlocks, numThreads>>>(mR4CAST(mainStressD),
			mR3CAST(devStressD), mR3CAST(volStressD), numBCE);

	cudaThreadSynchronize();
	cudaCheckError()
	;
}

/**
 * @brief Wrapper function for collideD
 * @details
 * 		See SDKCollisionSystem.cuh for informaton on collide
 */
void collide(thrust::device_vector<Real4>& sortedDerivVelRho_fsi_D,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real4>& sortedRhoPreMu,

		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers, uint numCells,
		Real dT) {
	//#if USE_TEX
	//    cutilSafeCall(cudaBindTexture(0, oldPosTex, sortedPosRad, numAllMarkers*sizeof(Real4)));
	//    cutilSafeCall(cudaBindTexture(0, oldVelTex, sortedVelMas, numAllMarkers*sizeof(Real4)));
	//    cutilSafeCall(cudaBindTexture(0, cellStartTex, cellStart, numCells*sizeof(uint)));
	//    cutilSafeCall(cudaBindTexture(0, cellEndTex, cellEnd, numCells*sizeof(uint)));
	//#endif

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numAllMarkers, 64, numBlocks, numThreads);

	// execute the kernel
	collideD<<<numBlocks, numThreads>>>(mR4CAST(sortedDerivVelRho_fsi_D),
			mR3CAST(sortedPosRad), mR3CAST(sortedVelMas),
			mR3CAST(vel_XSPH_Sorted_D), mR4CAST(sortedRhoPreMu),
			U1CAST(cellStart), U1CAST(cellEnd),
			numAllMarkers);

	cudaThreadSynchronize();
	cudaCheckError();

//					// unroll sorted index to have the location of original particles in the sorted arrays
//					thrust::device_vector<uint> dummyIndex = gridMarkerIndex;
//					thrust::sort_by_key(dummyIndex.begin(), dummyIndex.end(),
//							derivVelRhoD.begin());
//					dummyIndex.clear();


	//#if USE_TEX
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//    cutilSafeCall(cudaUnbindTexture(cellStartTex));
	//    cutilSafeCall(cudaUnbindTexture(cellEndTex));
	//#endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void ReCalcDensity(thrust::device_vector<Real3>& oldPosRad,
		thrust::device_vector<Real3>& oldVelMas,
		thrust::device_vector<Real4>& oldRhoPreMu,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers) {
	//#if USE_TEX
	//    cutilSafeCall(cudaBindTexture(0, oldPosTex, sortedPosRad, numAllMarkers*sizeof(Real4)));
	//    cutilSafeCall(cudaBindTexture(0, oldVelTex, sortedVelMas, numAllMarkers*sizeof(Real4)));
	//    cutilSafeCall(cudaBindTexture(0, cellStartTex, cellStart, numCells*sizeof(uint)));
	//    cutilSafeCall(cudaBindTexture(0, cellEndTex, cellEnd, numCells*sizeof(uint)));
	//#endif

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numAllMarkers, 64, numBlocks, numThreads);

	// execute the kernel
	ReCalcDensityD_F1<<<numBlocks, numThreads>>>(mR3CAST(oldPosRad),
			mR3CAST(oldVelMas), mR4CAST(oldRhoPreMu), mR3CAST(sortedPosRad),
			mR3CAST(sortedVelMas), mR4CAST(sortedRhoPreMu),
			U1CAST(gridMarkerIndex), U1CAST(cellStart), U1CAST(cellEnd),
			numAllMarkers);

	cudaThreadSynchronize();
	cudaCheckError()
	;

	//#if USE_TEX
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//    cutilSafeCall(cudaUnbindTexture(cellStartTex));
	//    cutilSafeCall(cudaUnbindTexture(cellEndTex));
	//#endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void ProjectDensityPressureToBCandBCE(thrust::device_vector<Real4>& oldRhoPreMu,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers) {
	//#if USE_TEX
	//    cutilSafeCall(cudaBindTexture(0, oldPosTex, sortedPosRad, numAllMarkers*sizeof(Real4)));
	//    cutilSafeCall(cudaBindTexture(0, oldVelTex, sortedVelMas, numAllMarkers*sizeof(Real4)));
	//    cutilSafeCall(cudaBindTexture(0, cellStartTex, cellStart, numCells*sizeof(uint)));
	//    cutilSafeCall(cudaBindTexture(0, cellEndTex, cellEnd, numCells*sizeof(uint)));
	//#endif

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numAllMarkers, 64, numBlocks, numThreads);

	// execute the kernel
	ProjectDensityPressureToBCandBCE_D<<<numBlocks, numThreads>>>(
			mR4CAST(oldRhoPreMu), mR3CAST(sortedPosRad),
			mR4CAST(sortedRhoPreMu), U1CAST(gridMarkerIndex), U1CAST(cellStart),
			U1CAST(cellEnd), numAllMarkers);

	cudaThreadSynchronize();
	cudaCheckError()
	;

	//#if USE_TEX
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//    cutilSafeCall(cudaUnbindTexture(cellStartTex));
	//    cutilSafeCall(cudaUnbindTexture(cellEndTex));
	//#endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void CalcCartesianData(thrust::device_vector<Real4>& rho_Pres_CartD,
		thrust::device_vector<Real4>& vel_VelMag_CartD,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint cartesianGridSize,
		int3 cartesianGridDims, Real resolution) {

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(cartesianGridSize, 64, numBlocks, numThreads);

	// execute the kernel
	CalcCartesianDataD<<<numBlocks, numThreads>>>(mR4CAST(rho_Pres_CartD),
			mR4CAST(vel_VelMag_CartD), mR3CAST(sortedPosRad),
			mR3CAST(sortedVelMas), mR4CAST(sortedRhoPreMu),
			U1CAST(gridMarkerIndex), U1CAST(cellStart), U1CAST(cellEnd),
			cartesianGridDims, resolution);

	cudaThreadSynchronize();
	cudaCheckError()
	;

	//#if USE_TEX
	//    cutilSafeCall(cudaUnbindTexture(oldPosTex));
	//    cutilSafeCall(cudaUnbindTexture(oldVelTex));
	//    cutilSafeCall(cudaUnbindTexture(cellStartTex));
	//    cutilSafeCall(cudaUnbindTexture(cellEndTex));
	//#endif
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles by calling UpdateFluidD
void UpdateFluid(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real3>& vel_XSPH_D,
		thrust::device_vector<Real4>& rhoPresMuD,
		thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT) {

//	int4 referencePortion = referenceArray[0];
//	if (referencePortion.z != -1) {
//		printf("error in UpdateFluid, accessing non fluid\n");
//		return;
//	}
//	int2 updatePortion = mI2(referencePortion);
	int2 updatePortion = mI2(0, referenceArray[referenceArray.size() - 1].y);
	// int2 updatePortion = mI2(referenceArray[0].x, referenceArray[0].y);

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid,
			nThreads);
	UpdateFluidD<<<nBlock_UpdateFluid, nThreads>>>(mR3CAST(posRadD),
			mR3CAST(velMasD), mR3CAST(vel_XSPH_D), mR4CAST(rhoPresMuD),
			mR4CAST(derivVelRhoD), updatePortion, dT);
	cudaThreadSynchronize();
	cudaCheckError()
	;
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles by calling UpdateFluid_init_LF
void UpdateFluid_init_LF(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD_half,
		thrust::device_vector<Real4>& rhoPresMuD_half,
		const thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT) {
	int4 referencePortion = referenceArray[0];
	if (referencePortion.z != -1) {
		printf("error in UpdateFluid, accessing non fluid\n");
		return;
	}
	int2 updatePortion = mI2(referencePortion);
	// int2 updatePortion = mI2(referenceArray[0].x, referenceArray[0].y);

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid,
			nThreads);
	UpdateFluidD_init_LF<<<nBlock_UpdateFluid, nThreads>>>(mR3CAST(posRadD),
			mR3CAST(velMasD_half), mR4CAST(rhoPresMuD_half),
			mR4CAST(derivVelRhoD), updatePortion, dT);
	cudaThreadSynchronize();
	cudaCheckError()
	;
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles by calling UpdateFluid_rho_vel_LF
void UpdateFluid_rho_vel_LF(thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real4>& rhoPresMuD,
		const thrust::device_vector<Real3>& velMasD_old,
		const thrust::device_vector<Real4>& rhoPresMuD_old,
		const thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT) {
	int4 referencePortion = referenceArray[0];
	if (referencePortion.z != -1) {
		printf("error in UpdateFluid, accessing non fluid\n");
		return;
	}
	int2 updatePortion = mI2(referencePortion);
	// int2 updatePortion = mI2(referenceArray[0].x, referenceArray[0].y);

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid,
			nThreads);
	UpdateFluidD_rho_vel_LF<<<nBlock_UpdateFluid, nThreads>>>(mR3CAST(velMasD),
			mR4CAST(rhoPresMuD), mR3CAST(velMasD_old), mR4CAST(rhoPresMuD_old),
			mR4CAST(derivVelRhoD), updatePortion, dT);
	cudaThreadSynchronize();
	cudaCheckError()
	;
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles by calling UpdateFluid_EveryThing_LF
void UpdateFluid_EveryThing_LF(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD_half,
		thrust::device_vector<Real4>& rhoPresMuD_half,
		const thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT) {
	int4 referencePortion = referenceArray[0];
	if (referencePortion.z != -1) {
		printf("error in UpdateFluid, accessing non fluid\n");
		return;
	}
	int2 updatePortion = mI2(referencePortion);
	// int2 updatePortion = mI2(referenceArray[0].x, referenceArray[0].y);

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid,
			nThreads);
	UpdateFluidD_EveryThing_LF<<<nBlock_UpdateFluid, nThreads>>>(
			mR3CAST(posRadD), mR3CAST(velMasD_half), mR4CAST(rhoPresMuD_half),
			mR4CAST(derivVelRhoD), updatePortion, dT);
	cudaThreadSynchronize();
	cudaCheckError()
	;
}

//--------------------------------------------------------------------------------------------------------------------------------
void CopySorted_vXSPH_dVdRho_to_original(thrust::device_vector<Real3>& vel_XSPH_D,
		thrust::device_vector<Real4>& derivVelRhoD,
		thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real4>& sortedDerivVelRho_fsi_D,
		thrust::device_vector<uint>& mapOriginalToSorted, int numAllMarkers) {
	uint nBlock_NumSpheres, nThreads_SphMarkers;
	computeGridSize(numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);
	CopySorted_vXSPH_dVdRho_to_original_kernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
			mR3CAST(vel_XSPH_D), mR4CAST(derivVelRhoD),
			mR3CAST(vel_XSPH_Sorted_D),mR4CAST(sortedDerivVelRho_fsi_D),
			U1CAST(mapOriginalToSorted));
	cudaThreadSynchronize();
	cudaCheckError()
	;
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the fluid particles by calling UpdateBoundary
void UpdateBoundary(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real4>& rhoPresMuD,
		thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray, Real dT) {
	int4 referencePortion = referenceArray[1];
	if (referencePortion.z != 0) {
		printf("error in UpdateBoundary, accessing non boundary\n");
		return;
	}
	int2 updatePortion = mI2(referencePortion);

	uint nBlock_UpdateFluid, nThreads;
	computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid,
			nThreads);
	UpdateKernelBoundary<<<nBlock_UpdateFluid, nThreads>>>(mR3CAST(posRadD),
			mR3CAST(velMasD), mR4CAST(rhoPresMuD), mR4CAST(derivVelRhoD),
			updatePortion, dT);
	cudaThreadSynchronize();
	cudaCheckError()
	;
}

/**
 * @brief ApplyBoundarySPH_Markers
 * @details
 * 		See SDKCollisionSystem.cuh for more info
 */
void ApplyBoundarySPH_Markers(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real4>& rhoPresMuD, int numAllMarkers) {
	uint nBlock_NumSpheres, nThreads_SphMarkers;
	computeGridSize(numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);
	ApplyPeriodicBoundaryXKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
			mR3CAST(posRadD), mR4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	cudaCheckError()
	;
	// these are useful anyway for out of bound particles
	ApplyPeriodicBoundaryYKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
			mR3CAST(posRadD), mR4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	cudaCheckError()
	;
	ApplyPeriodicBoundaryZKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
			mR3CAST(posRadD), mR4CAST(rhoPresMuD));
	cudaThreadSynchronize();
	cudaCheckError()
	;

	//	SetOutputPressureToZero_X<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(mR3CAST(posRadD), mR4CAST(rhoPresMuD));
	//    cudaThreadSynchronize();
	//    cudaCheckError();
}
