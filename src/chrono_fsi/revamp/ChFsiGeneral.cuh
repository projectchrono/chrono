// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki
// =============================================================================
//
// Class for fsi properties and functions.//
// =============================================================================

#ifndef CH_FSIGENERAL_H_
#define CH_FSIGENERAL_H_

//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
__device__ __host__ inline void RotationMatirixFromQuaternion(Real3& AD1,
		Real3& AD2, Real3& AD3, const Real4& q) {
	AD1 = 2
			* mR3(0.5f - q.z * q.z - q.w * q.w, q.y * q.z - q.x * q.w,
					q.y * q.w + q.x * q.z);
	AD2 = 2
			* mR3(q.y * q.z + q.x * q.w, 0.5f - q.y * q.y - q.w * q.w,
					q.z * q.w - q.x * q.y);
	AD3 = 2
			* mR3(q.y * q.w - q.x * q.z, q.z * q.w + q.x * q.y,
					0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ __host__ inline Real3 InverseRotate_By_RotationMatrix_DeviceHost(
		const Real3& A1, const Real3& A2, const Real3& A3, const Real3& r3) {
	return mR3(A1.x * r3.x + A2.x * r3.y + A3.x * r3.z,
			A1.y * r3.x + A2.y * r3.y + A3.y * r3.z,
			A1.z * r3.x + A2.z * r3.y + A3.z * r3.z);
}


namespace fsi {
	class CH_FSI_API ChFsiGeneral {
public:
	/**
 * @brief computeGridSize
 * @details Compute grid and thread block size for a given number of elements
 *
 * @param n Total number of elements. Each elements needs a thread to be computed
 * @param blockSize Number of threads per block.
 * @param numBlocks output
 * @param numThreads Output: number of threads per block
 */
	void computeGridSize(uint n, uint blockSize, uint& numBlocks, uint& numThreads);

private:
	uint iDivUp(uint a, uint b);



}


}

#endif
