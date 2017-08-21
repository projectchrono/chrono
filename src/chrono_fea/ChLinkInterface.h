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

#ifndef CHLINKINTERFACE_H
#define CHLINKINTERFACE_H

#include "chrono/physics/ChLinkBase.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_constraints
/// @{

/// Class for grouping all those constraints that can interface FEA elements
/// of different types.
/// Concrete classes will be inherited from this class.

class ChApiFea ChLinkInterface : public ChLinkBase {

  private:
  public:
    ChLinkInterface(){};
    ~ChLinkInterface(){};
};

/// @} fea_constraints

}  // end namespace fea
}  // end namespace chrono

#endif
