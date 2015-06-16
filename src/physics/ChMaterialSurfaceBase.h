//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHMATERIALSURFACEBASE_H
#define CHMATERIALSURFACEBASE_H

#include "core/ChShared.h"

namespace chrono {
///
/// Base class for specifying material properties for contact force
/// generation.
///
class ChApi ChMaterialSurfaceBase : public ChShared {
  public:
    virtual void StreamOUT(ChStreamOutAscii& mstream) = 0;
    virtual void StreamOUT(ChStreamOutBinary& mstream) = 0;
    virtual void StreamIN(ChStreamInBinary& mstream) = 0;
};

}  // END_OF_NAMESPACE____

#endif
