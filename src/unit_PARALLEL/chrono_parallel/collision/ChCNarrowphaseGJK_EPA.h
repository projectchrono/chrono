/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
GJK-EPA collision solver by Nathanael Presson, 2008
*/


#ifndef CHC_NARROWPHASE_GJK_EPA_H
#define CHC_NARROWPHASE_GJK_EPA_H

#include "chrono_parallel/collision/ChCNarrowphase.h"

namespace chrono {
namespace collision {

struct sResults {
   enum eStatus {
      Separated, /* Shapes does not penetrate                                   */
      Penetrating, /* Shapes are penetrating                                  */
      GJK_Failed, /* GJK phase fail, no big issue, shapes are probably just 'touching' */
      EPA_Failed /* EPA phase fail, bigger problem, need to save parameters, and debug */
   } status;
   real3 witnesses[2];
   real3 normal;
   real distance;
};

CH_PARALLEL_API
bool GJKDistance(const ConvexShape& shape0,
                 const ConvexShape& shape1,
                 const real3& guess,
                 sResults& results);
CH_PARALLEL_API
bool GJKPenetration(const ConvexShape& shape0,
                    const ConvexShape& shape1,
                    const real3& guess,
                    sResults& results);

CH_PARALLEL_API
bool GJKCollide(const ConvexShape& shape0,
                    const ConvexShape& shape1,
                    sResults& results);

CH_PARALLEL_API
bool GJKFindPenetration(const ConvexShape& shape0,
                const ConvexShape& shape1,
                sResults& results);


}  // end namespace collision
}  // end namespace chrono

#endif

