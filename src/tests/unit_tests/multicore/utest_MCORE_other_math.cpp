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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Chrono::Multicore unit test for MPR collision detection
// =============================================================================

#include "unit_testing.h"

using namespace chrono;

TEST(ChronoMulticore, other_math) {
    uvec4 a = _make_uvec4(1, 2, 3, 4);
    uvec4 r = Sort(a);
    Assert_eq(a, r);

    uvec4 b;
    b = _make_uvec4(4, 3, 2, 1);
    r = Sort(b);
    Assert_eq(a, r);

    b = _make_uvec4(4, 2, 3, 1);
    r = Sort(b);
    Assert_eq(a, r);

    b = _make_uvec4(3, 4, 2, 1);
    r = Sort(b);
    Assert_eq(a, r);

    b = _make_uvec4(1, 3, 2, 4);
    r = Sort(b);
    Assert_eq(a, r);
}
