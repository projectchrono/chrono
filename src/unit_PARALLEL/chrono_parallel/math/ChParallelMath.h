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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: definition of some convenience functions for math operations
// =============================================================================

#ifndef CHPARALLELMATH_H
#define CHPARALLELMATH_H

#include "real.h"
#include "real2.h"
#include "real3.h"
#include "real4.h"
#include "mat33.h"
#include "other_types.h"


//Computes the nearest power of two to the given value and returns it
static inline uint nearest_pow(const uint &num) {
   uint n = num > 0 ? num - 1 : 0;
   n |= n >> 1;
   n |= n >> 2;
   n |= n >> 4;
   n |= n >> 8;
   n |= n >> 16;
   n++;
   return n;
}

//Swaps two objects of any type using a temporary
template<class T>
static inline void Swap(T &a,
                        T &b) {
   T tmp = a;
   a = b;
   b = tmp;
}

// Given a frame with origin 'p' and orientation 'q', transform the
// position vector 'rl' expressed in the local frame into the parent
// frame:  rp = p + A * rl
static inline real3 TransformLocalToParent(const real3& p,
                                           const quaternion& q,
                                           const real3& rl) {
   return p + quatRotate(rl, q);
}

// Given a frame with origin 'p' and orientation 'q', transform the
// position vector 'rp' expressed in the parent frame into the local
// frame:  rl = A^T * (rp - p)
static inline real3 TransformParentToLocal(const real3& p,
                                           const quaternion& q,
                                           const real3& rp) {
   return quatRotateT(rp - p, q);
}

#endif

