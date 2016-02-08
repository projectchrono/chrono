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
// Base class for changing device arrays in non-cuda files
// =============================================================================

#ifndef CH_DEVICEUTILS_H_
#define CH_DEVICEUTILS_H_

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "chrono_fsi/custom_cutil_math.h"

namespace chrono {
namespace fsi {

class CH_FSI_API ChDeviceUtils : public ChFsiGeneral {
	public:

	static void ResizeMyThrust3(thrust::device_vector<Real3>& mThrustVec, int mSize);
	static void ResizeMyThrust4(thrust::device_vector<Real4>& mThrustVec, int mSize);
	static void FillMyThrust4(thrust::device_vector<Real4>& mThrustVec, Real4 v);
	static void ClearMyThrustR3(thrust::device_vector<Real3>& mThrustVec);
	static void ClearMyThrustR4(thrust::device_vector<Real4>& mThrustVec);
	static void ClearMyThrustU1(thrust::device_vector<uint>& mThrustVec);
	static void PushBackR3(thrust::device_vector<Real3>& mThrustVec, Real3 a3);
	static void PushBackR4(thrust::device_vector<Real4>& mThrustVec, Real4 a4);
	static void ResizeR3(thrust::device_vector<Real3>& mThrustVec, int size);
	static void ResizeR4(thrust::device_vector<Real4>& mThrustVec, int size);
	static void ResizeU1(thrust::device_vector<uint>& mThrustVec, int size);

	private:
};
} // end namespace fsi
} // end namespace chrono

#endif