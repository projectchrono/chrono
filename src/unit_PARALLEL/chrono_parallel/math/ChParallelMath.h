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

//#define I4  int4
#define I3  _make_int3
#define I2  _make_int2
#define U3  make_uint3

typedef unsigned int uint;

struct bool2 {
   bool x, y;
   bool2() {
   }
   bool2(bool a,
         bool b)
         : x(a), y(b) {
   }
};

static inline int3 _make_int3(const int &a,
                              const int &b,
                              const int &c) {
   int3 t;
   t.x = a;
   t.y = b;
   t.z = c;
   return t;
}

static inline int2 _make_int2(const int &a,
                              const int &b) {
   int2 t;
   t.x = a;
   t.y = b;
   return t;
}

static inline int3 _make_int3(const real3 &a) {
   int3 t;
   t.x = a.x;
   t.y = a.y;
   t.z = a.z;
   return t;
}

////////Other Operations
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

////////Generic Operations
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

