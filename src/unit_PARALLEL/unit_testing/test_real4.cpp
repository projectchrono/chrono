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
// ChronoParallel unit test for real class
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>
#include "unit_testing.h"
#include "chrono_parallel/math/real4.h"


int main(
   int argc,
   char* argv[]) {
// =============================================================================
 { //real4 inverse 
   real4 a(11,-2,0,-2);
   real4 b = inv(a);
   StrictEqual(b.w,11.0/129.0);
   StrictEqual(b.x,2.0/129.0);
   StrictEqual(b.y,0.0);
   StrictEqual(b.z,2.0/129.0);
}
// =============================================================================
{ //real4 normalize 
   real4 a(11,-2,0,-2);
   real4 b = normalize(a);
   WeakEqual(b.w,11.0/sqrt(129.0));
   WeakEqual(b.x,-2.0/sqrt(129.0));
   WeakEqual(b.y,0.0);
   WeakEqual(b.z,-2.0/sqrt(129.0));
}
// =============================================================================
{ //real4 multiply 
   real4 a(11/2.0,-2,-3,-2);
   real4 b(11,-2,0,-2);
   real4 c = mult(a,b);
   WeakEqual(c.w,105/2.0);
   WeakEqual(c.x,-27.0);
   WeakEqual(c.y,-33.0);
   WeakEqual(c.z,-39.0);
}
// =============================================================================
{ //real4 rotate 
   real3 a(1.0,2.0,-3.0);
   real4 b(2.0,-2,-2,-2);
   b=normalize(b);


   real3 c = quatRotate(a,b);
   WeakEqual(c.x,2);
   WeakEqual(c.y,-3);
   WeakEqual(c.z,1);
}
// =============================================================================
{ //real4 rotate 
   real4 b(11/2.0,-2,-3,-2);
   real4 c=~b;
   WeakEqual(c.w,5.5);
   WeakEqual(c.x,2.0);
   WeakEqual(c.y,3.0);
   WeakEqual(c.z,2.0);
}

// =============================================================================
{ //real4 rotate 
   real3 a(1.0,2.0,-3.0);
   real4 b(2.0,-2,-2,-2);
   b=normalize(b);
   real3 c = quatRotateMatT(a,b);
   WeakEqual(c.x,-3);
   WeakEqual(c.y,1);
   WeakEqual(c.z,2);
}

// =============================================================================

return 0;
}

