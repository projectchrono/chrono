// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHC_COMPILE_H
#define CHC_COMPILE_H

#include <cmath>

namespace chrono {
namespace collision {

// prevents compiler warnings when PQP_REAL is float
/*
inline float sqrt(float x) { return (float)sqrt((double)x); }
inline float cos(float x) { return (float)cos((double)x); }
inline float sin(float x) { return (float)sin((double)x); }
inline float fabs(float x) { return (float)fabs((double)x); }
*/

//-------------------------------------------------------------------------
//
// PQP_REAL
//
// This is the floating point type used throughout PQP.  doubles are
// recommended, both for their precision and because the software has
// mainly been tested using them.  However, floats appear to be faster
// (by 60% on some machines).
//
//-------------------------------------------------------------------------

typedef double PQP_REAL;

}  // end namespace collision
}  // end namespace chrono

#endif
