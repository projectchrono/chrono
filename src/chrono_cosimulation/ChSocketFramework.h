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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHSOCKETFRAMEWORK_H
#define CHSOCKETFRAMEWORK_H

#include "chrono_cosimulation/ChSocket.h"

namespace chrono {
namespace cosimul {

/// A single object of this class must be instantiated before using
/// all classes related to sockets, because it initializes some platform-specific
/// settings.
/// Delete it after you do not need sockets anymore.

class ChApiCosimulation ChSocketFramework {
  public:
    ChSocketFramework();
    ~ChSocketFramework();
};

}  // end namespace cosimul
}  // end namespace chrono

#endif
