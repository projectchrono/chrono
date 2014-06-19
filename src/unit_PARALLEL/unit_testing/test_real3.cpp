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
#include "chrono_parallel/math/real3.h"


int main(
   int argc,
   char* argv[]) {
// =============================================================================
   { // zero constructor
      real3 zero;
      StrictEqual(zero.x,0.0);  StrictEqual(zero.y,0.0);  StrictEqual(zero.z,0.0);
   }

   { // 1 float constructor
      real3 value(1.5);
      StrictEqual(value.x,1.5);  StrictEqual(value.y,1.5);  StrictEqual(value.z,1.5);
   }

   { // 3 float constructor
      real3 value(1.5, 2.6, 3.7);
      StrictEqual(value.x,1.5);  StrictEqual(value.y,2.6);  StrictEqual(value.z,3.7);
   }
// =============================================================================
   { // float 3 add
      real3 a(1.0, 2.0, 3.0); real3 b(3.0, 2.0, 1.0);
      real3 c = a+b;
      StrictEqual(c.x,4.0);   StrictEqual(c.y,4.0);   StrictEqual(c.z,4.0);
   }

   { // float 3 subtract
      real3 a(1.0, 2.0, 3.0);
      real3 b(3.0, 2.0, 1.0);
      real3 c = a-b;
      StrictEqual(c.x,-2.0);  StrictEqual(c.y,0.0);   StrictEqual(c.z,2.0);
   }

   { // float 3 multiply
      real3 a(1.0, 2.0, 3.0); real3 b(3.0, 2.0, 1.0);
      real3 c = a*b;
      StrictEqual(c.x,3.0);   StrictEqual(c.y,4.0);   StrictEqual(c.z,3.0);
   }

   { // float 3 divide
      real3 a(1.0, 2.0, 3.0); real3 b(3.0, 2.0, 1.0);
      real3 c = a/b;
      StrictEqual(c.x,1.0/3.0);  StrictEqual(c.y,2.0/2.0);  StrictEqual(c.z,3.0/1.0);
   }
   { // float 3 negate
      real3 a(1.0, 2.0, 3.0);
      real3 c = -a;
      StrictEqual(c.x,-1.0);  StrictEqual(c.y,-2.0);  StrictEqual(c.z,-3.0);
   }
// =============================================================================
   { // float 3 add
      real3 a(1.0, 2.0, 3.0); real3 b(3.0, 2.0, 1.0);
      a+=b;
      StrictEqual(a.x,4.0);   StrictEqual(a.y,4.0);   StrictEqual(a.z,4.0);
   }

   { // float 3 subtract
      real3 a(1.0, 2.0, 3.0); real3 b(3.0, 2.0, 1.0);
      a-=b;
      StrictEqual(a.x,-2.0);  StrictEqual(a.y,0.0);   StrictEqual(a.z,2.0);
   }

   { // float 3 multiply
      real3 a(1.0, 2.0, 3.0); real3 b(3.0, 2.0, 1.0);
      a*=b;
      StrictEqual(a.x,3.0);   StrictEqual(a.y,4.0);   StrictEqual(a.z,3.0);
   }

   { // float 3 divide
      real3 a(1.0, 2.0, 3.0); real3 b(3.0, 2.0, 1.0);
      a/=b;
      StrictEqual(a.x,1.0/3.0);  StrictEqual(a.y,2.0/2.0); StrictEqual(a.z,3.0/1.0);
   }
// =============================================================================


   { // float 3 add
      real3 a(1.0, 2.0, 3.0); real b(2.0);
      real3 c = a+b;
      StrictEqual(c.x,3.0);   StrictEqual(c.y,4.0);   StrictEqual(c.z,5.0);
   }

   { // float 3 subtract
      real3 a(1.0, 2.0, 3.0); real b(2.0);
      real3 c = a-b;
      StrictEqual(c.x,-1.0);  StrictEqual(c.y,0.0);   StrictEqual(c.z,1.0);
   }

   { // float 3 multiply
      real3 a(1.0, 2.0, 3.0); real b(2.0);
      real3 c = a*b;
      StrictEqual(c.x,2.0);   StrictEqual(c.y,4.0);   StrictEqual(c.z,6.0);
   }

   { // float 3 divide
      real3 a(1.0, 2.0, 3.0); real b(2.0);
      real3 c = a/b;
      StrictEqual(c.x,1.0/2.0);  StrictEqual(c.y,2.0/2.0); StrictEqual(c.z,3.0/2.0);
   }
// =============================================================================


   { // float 3 add
      real3 a(1.0, 2.0, 3.0); real b(2.0);
      a+=b;
      StrictEqual(a.x,3.0);   StrictEqual(a.y,4.0);   StrictEqual(a.z,5.0);
   }

   { // float 3 subtract
      real3 a(1.0, 2.0, 3.0); real b(2.0);
      a-=b;
      StrictEqual(a.x,-1.0);  StrictEqual(a.y,0.0);   StrictEqual(a.z,1.0);
   }

   { // float 3 multiply
      real3 a(1.0, 2.0, 3.0); real b(2.0);
      a*=b;
      StrictEqual(a.x,2.0);   StrictEqual(a.y,4.0);   StrictEqual(a.z,6.0);
   }

   { // float 3 divide
      real3 a(1.0, 2.0, 3.0); real b(2.0);
      a/=b;
      StrictEqual(a.x,1.0/2.0);  StrictEqual(a.y,2.0/2.0); StrictEqual(a.z,3.0/2.0);
   }
// =============================================================================

   { // float 3 dot
      real3 a(1.0, 2.0, 3.0); real3 b(2.0, 1.0, 3.0);
      real c = dot(a,b);
      StrictEqual(c,13.0);
   }
   { // float 3 cross
      real3 a(1.0, 2.0, 3.0); real3 b(2.0, 1.0, 3.0);
      real3 c = cross(a,b);
      StrictEqual(c.x,3.0);  StrictEqual(c.y,3.0); StrictEqual(c.z,-3.0);
   }
   { // float 3 cross
      real3 a = normalize(real3(rand(), rand(), rand()));
      real3 b = normalize(real3(rand(), rand(), rand()));
      real3 ans1 = cross(a,b);
      ChVector<float> ans2;
      ans2.Cross(ToChVector(a),ToChVector(b));
      StrictEqual(ans1,ToReal3(ans2));
   }

   { // float 3 length
      real3 a(1.0, 2.0, -3.0);
      real c = length(a);
      StrictEqual(c,sqrt(14));
   }
   { // float 3 normalize
      real3 a(1.0, 2.0, -3.0);
      real3 c = normalize(a);
      WeakEqual(c.x,1.0/sqrt(14));
      WeakEqual(c.y,2.0/sqrt(14));
      WeakEqual(c.z,-3.0/sqrt(14));
   }



   return 0;
}

