// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Base class for a FEA element with hexahedral shape.
// =============================================================================

#ifndef CH_HEXAHEDRON_H
#define CH_HEXAHEDRON_H

#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Base class for a FEA element with hexahedral shape.
class ChApi ChElementHexahedron {
  public:
    ChElementHexahedron() {}
    virtual ~ChElementHexahedron() {}

    /// Return the specified hexahedron node (0 <= n <= 7).
    virtual std::shared_ptr<ChNodeFEAxyz> GetHexahedronNode(int n) = 0;
};

/// @} fea_elements

}  // namespace fea
}  // namespace chrono

#endif
